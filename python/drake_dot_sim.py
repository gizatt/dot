import numpy as np
import os
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    BasicVector,
    Box,
    CoulombFriction,
    FirstOrderLowPassFilter,
    LeafSystem,
    MultibodyForces,
    MultibodyPlant,
    Parser,
    RigidTransform,
    SceneGraph,
    Simulator,
    SpatialInertia,
    UnitInertia,
    VectorSystem,
    ZeroOrderHold
)

def sigmoid(x):
  return 1 / (1 + np.exp(-x))

class ServoController(LeafSystem):
    ''' Simulates control of a set of servos: translates from PWM signals
    for the servos to joint angles for each servo. Then takes produced servo
    torques corresponding to each link and translates them into actual joint
    torques for the joints, taking the fourbar linkage in Dot's legs into account.

    The config dict should contain root-level entries for
    "left_front", "right_front", "left_back", "right_back" legs, each containing
    "hip_roll", "hip_pitch", and "knee_pitch" servo info dicts. '''
    def __init__(self, plant, config_dict):
        LeafSystem.__init__(self)
        self.config = config_dict
        self.plant = plant
        self.plant_context = plant.CreateDefaultContext()
        self.nu = plant.num_actuated_dofs()
        self.nq = plant.num_positions()
        self.nv = plant.num_velocities()

        # Build up a vectorized servo processing pipeline.
        # From a PWM input vector, transform to an angle using the calibration
        # info by taking PWM * scale + center.
        pwm_to_angle_center = [] # will be N_servos np vector
        pwm_to_angle_scale = [] # will be N_servo np vector
        self.pwm_ranges = []
        self.servo_names = []

        # Record servo speed and torque limits. (I believe the fourbar
        # impacts required servo speed but not torque -- torque gets transmitted
        # through the linkage directly to the knee.)
        speed_limits = [] # will be N_servos np vector
        torque_limits = [] # will be N_servos np vector

        leg_names = ["left_front", "left_back", "right_front", "right_back"]
        for leg_name in leg_names:
            # assert leg_name in self.config.keys()
            if leg_name in self.config.keys():
                leg_info = self.config[leg_name]
                for joint_name in ["hip_roll", "hip_pitch", "knee_pitch"]:
                    assert joint_name in leg_info.keys()
                    servo_info = leg_info[joint_name]
                    # Build range mapping.
                    pwm_range = np.array(servo_info["pwm_range"])
                    angle_range = np.array(servo_info["angle_range"]) * np.pi / 180.
                    angle_per_us = (angle_range[1] - angle_range[0]) / (pwm_range[1] - pwm_range[0])
                    pwm_to_angle_scale.append(angle_per_us)
                    pwm_to_angle_center.append(angle_range[0] - angle_per_us * pwm_range[0])
                    # Write down speed and torque limits, converting to Nm and rad/sec
                    # from kg*cm and sec/60deg.
                    speed_limits.append(60./servo_info["speed_limit"] * np.pi/180.)
                    torque_limits.append(servo_info["torque_limit"]*0.0980665)
                    self.pwm_ranges.append(pwm_range)
                    self.servo_names.append(servo_info["joint_name"])

        self.pwm_to_angle_center = np.array(pwm_to_angle_center)
        self.pwm_to_angle_scale = np.array(pwm_to_angle_scale)
        self.speed_limits = np.array(speed_limits)
        self.torque_limits = np.array(torque_limits)
        self.n_servos = len(self.pwm_to_angle_center)

        # Servo simulator PD gains.
        self.Kp_for_vel_target = 100.
        self.Kp_for_torque = 100.
        # In this deadzone, direction position PD is applied, which is more
        # stable for contacts / holding position.
        self.angle_deadzone = 25.0 * np.pi/180.
        self.deadzone_Kp = 10.
        self.deadzone_Kd = 1.
        
        

        # Build linear map from the current robot position and velocity vectors to the
        # corresponding servo position. (This takes the four bar linkage
        # into account.)
        self.q_to_servo_angles_A = np.zeros((self.n_servos, self.nq))
        self.v_to_servo_velocities_A = np.zeros((self.n_servos, self.nv))
        self.servos_to_actuator_map = np.zeros((self.nu, self.n_servos))
        # no bias term since velocity will be directly proportional
        for k, leg_name in enumerate(leg_names):
            # assert leg_name in self.config.keys()
            if leg_name in self.config.keys():
                leg_offset = k*3 + 0
                leg_info = self.config[leg_name]
                # Hip joints is directly connected to their joints
                hip_r_actuator = plant.GetJointActuatorByName(leg_info["hip_roll"]["joint_name"])
                hip_r_joint = hip_r_actuator.joint()
                self.q_to_servo_angles_A[leg_offset + 0,
                                    hip_r_joint.position_start()] = 1.
                self.v_to_servo_velocities_A[leg_offset + 0,
                                        hip_r_joint.velocity_start()] = 1.
                self.servos_to_actuator_map[int(hip_r_actuator.index()), leg_offset + 0] = 1.
                hip_p_actuator = plant.GetJointActuatorByName(leg_info["hip_pitch"]["joint_name"])
                hip_p_joint = hip_p_actuator.joint()
                self.q_to_servo_angles_A[leg_offset + 1,
                                    hip_p_joint.position_start()] = 1.
                self.v_to_servo_velocities_A[leg_offset + 1,
                                        hip_p_joint.velocity_start()] = 1.
                self.servos_to_actuator_map[int(hip_p_actuator.index()), leg_offset + 1] = 1.
                # Knee is connected by fourbar to hip;
                # knee angle = knee servo angle - hip servo angle
                # so knee servo angle = knee angle - hip angle
                knee_p_actuator = plant.GetJointActuatorByName(leg_info["knee_pitch"]["joint_name"])
                knee_p_joint = knee_p_actuator.joint()
                self.q_to_servo_angles_A[leg_offset + 2,
                                    hip_p_joint.position_start()] = 1.
                self.q_to_servo_angles_A[leg_offset + 2,
                                    knee_p_joint.position_start()] = 1.
                self.v_to_servo_velocities_A[leg_offset + 2,
                                    hip_p_joint.velocity_start()] = 1.
                self.v_to_servo_velocities_A[leg_offset + 2,
                                    knee_p_joint.velocity_start()] = 1.
                self.servos_to_actuator_map[int(knee_p_actuator.index()), leg_offset + 2] = 1.

        self.DeclareVectorInputPort("pwm_setpoints", BasicVector(self.n_servos))
        self.DeclareVectorInputPort("x", BasicVector(self.nq + self.nv))

        # Produce a driving torque on the joint to strictly enforce
        # underlying servo positions, down to torque limits.
        self.DeclareVectorOutputPort("torque", BasicVector(self.nu),
                                     self.CalcOutput)

    def CalcOutput(self, context, output):
        x_curr = self.get_input_port(1).Eval(context)
        q_curr = x_curr[:self.nq]
        v_curr = x_curr[-self.nv:]
        self.plant.SetPositionsAndVelocities(self.plant_context, x_curr)

        servo_setpoints = self.get_input_port(0).Eval(context)
        servo_angle_setpoints = self.pwm_to_angle_scale * servo_setpoints + self.pwm_to_angle_center
        
        servo_angles_curr = self.q_to_servo_angles_A.dot(q_curr)
        servo_velocities_curr = self.v_to_servo_velocities_A.dot(v_curr)

        # Weird proportional velocity control
        velocity_target = self.Kp_for_vel_target * (servo_angle_setpoints - servo_angles_curr)
        velocity_target = np.clip(velocity_target, -self.speed_limits, self.speed_limits)
        servo_torques = self.Kp_for_torque * (velocity_target - servo_velocities_curr)
        
        # Deadzone control
        servo_torques_deadzone = self.deadzone_Kp * (servo_angle_setpoints - servo_angles_curr) + \
                        self.deadzone_Kd * (-servo_velocities_curr)
        deadzone_mask = np.abs(servo_angle_setpoints - servo_angles_curr) <= self.angle_deadzone
        servo_torques[deadzone_mask] = servo_torques_deadzone[deadzone_mask]

        # Limit and map torques back to actuation inputs.
        servo_torques = np.clip(servo_torques, -self.torque_limits, self.torque_limits)
        out_actuation = self.servos_to_actuator_map.dot(servo_torques)
        output.SetFromVector(out_actuation)

class ServoSliders(VectorSystem):
    """
    Provides a simple tcl/tk gui with one slider per servo to provide input
    for a ServoController
    """

    def __init__(self, servo_controller, resolution=-1, length=200,
                 update_period_sec=0.005, window=None):
        """"
        Based on https://github.com/RobotLocomotion/drake/blob/master/bindings/pydrake/manipulation/simple_ui.py.
        Args:
            servo_controller: A ServoController instance.
            resolution:  A scalar or vector of length robot.num_positions()
                         that specifies the discretization of the slider.  Use
                         -1 (the default) to disable any rounding.
            length:      The length of the sliders (passed as an argument to
                         tk.Scale).
            update_period_sec: Specifies how often the window update() method
                         gets called.
            window:      Optionally pass in a tkinter.Tk() object to add these
                         widgets to.  Default behavior is to create a new
                         window.
            title:       The string that appears as the title of the gui
                         window.  Use None to generate a default title.  This
                         parameter is only used if a window==None.
        """
        VectorSystem.__init__(self, 0, servo_controller.n_servos)

        def _reshape(x, num):
            x = np.array(x)
            assert len(x.shape) <= 1
            return np.array(x) * np.ones(num)
        resolution = _reshape(resolution, servo_controller.n_servos)

        title = "Servo Controls"

        if window is None:
            self.window = tk.Tk()
            self.window.title(title)
        else:
            self.window = window

        # Schedule window updates in either case (new or existing window):
        self.DeclarePeriodicPublish(update_period_sec, 0.0)

        self._slider = []
        self._default_position = np.array([(r[1] + r[0]) / 2. for r in servo_controller.pwm_ranges])

        k = 0
        for servo_k in range(servo_controller.n_servos):
            pwm_range = servo_controller.pwm_ranges[servo_k]
            name = servo_controller.servo_names[servo_k]
            self._slider.append(tk.Scale(self.window,
                                         from_=pwm_range[0],
                                         to=pwm_range[1],
                                         resolution=resolution[k],
                                         label=name,
                                         length=length,
                                         orient=tk.HORIZONTAL))
            self._slider[k].pack()
            k += 1

    def set_position(self, q):
        """
        Set the slider positions to the values in q.  
        """
        assert(len(q) == len(self._slider))
        for i in range(len(self._slider)):
            self._slider[i].set(q[i])

    def DoPublish(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        output[:] = self._default_position
        for i in range(0, len(self._slider)):
            output[i] = self._slider[i].get()

def add_ground(plant):
    ground_model = plant.AddModelInstance("ground_model")
    ground_box = plant.AddRigidBody(
        "ground", ground_model,
        SpatialInertia(1, np.array([0, 0, 0]), UnitInertia(1, 1, 1)))
    X_WG = RigidTransform([0, 0, -0.05])
    ground_geometry_id = plant.RegisterCollisionGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        CoulombFriction(0.3, 0.25))
    plant.RegisterVisualGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        [0.5, 0.5, 0.5, 1.])
    plant.WeldFrames(plant.world_frame(), ground_box.body_frame(), X_WG)

def setup_argparse_for_setup_dot_diagram(parser):
    parser.add_argument("--yaml_path", help="path to yaml config", default="../models/dot_servo_config.yaml")
    parser.add_argument("--welded", action='store_true')

def setup_dot_diagram(builder, args):
    ''' Using an existing DiagramBuilder, adds a sim of the Dot
    robot. Args comes from argparse.

    The returned controller will need its first port connected to
    a setpoint source.'''

    with open(args.yaml_path, "r") as f:
        config_dict = yaml.load(f, Loader=Loader)
    sdf_path = os.path.join(os.path.dirname(args.yaml_path), config_dict["sdf_path"])
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0005)
    parser = Parser(plant)
    model = parser.AddModelFromFile(sdf_path)
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("body"), RigidTransform(p=[0., 0., 0.25]))
    if args.welded:
        plant.WeldFrames(plant.world_frame(), plant.GetBodyByName("body").body_frame())
    else:
        add_ground(plant)
    plant.Finalize()

    controller = builder.AddSystem(ServoController(plant, config_dict))
    # Fixed control-rate controller with a low pass filter on its torque output.
    zoh = builder.AddSystem(ZeroOrderHold(period_sec=0.001, vector_size=controller.n_servos))
    filter = builder.AddSystem(FirstOrderLowPassFilter(time_constant=0.02, size=controller.n_servos))
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(1))
    builder.Connect(controller.get_output_port(0), zoh.get_input_port(0))
    builder.Connect(zoh.get_output_port(0), filter.get_input_port(0))
    builder.Connect(filter.get_output_port(0), plant.get_actuation_input_port())
    return plant, scene_graph, controller