"""
Modified from https://github.com/RobotLocomotion/drake/blob/4751fd6f0b61313faceccb7c6081bc077b35448b/bindings/pydrake/multibody/examples/jupyter_widgets_examples.ipynb
Runs simple visualization of the supplied SDF with sliders for moving joints.
"""

import argparse
import sys

import numpy as np

from pydrake.geometry import DrakeVisualizer
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import FirstOrderLowPassFilter, SignalLogger
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    MultibodyPlant, SceneGraph, Parser, FindResourceOrThrow, ConnectMeshcatVisualizer, JointSliders,
    LeafSystem, BasicVector, ConstantVectorSource, MultibodyForces, RigidTransform, SpatialInertia, UnitInertia, Box, CoulombFriction
)

class ServoController(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self.plant = plant
        self.plant_context = plant.CreateDefaultContext()
        self.torque_limit = 10 # Nm, TODO get from MBP
        self.speed_limit = 4*np.pi # rad/sec, TODO get from MBP
        self.accel_limit = 100.*np.pi/2. # rad/sec/sec, TODO get from MBP?
        self.Kp = 10.
        self.Kd = self.Kp * 0.01
        self.Bpinv = plant.MakeActuationMatrix().T # np.linalg.pinv(plant.MakeActuationMatrix())

        self.nu = plant.num_actuated_dofs()
        self.nq = plant.num_positions()
        self.nv = plant.num_velocities()
        
        self.DeclareVectorInputPort("u", BasicVector(self.nq))

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

        q_targ = self.get_input_port(0).Eval(context)
        v_targ = np.zeros(self.nv)
        p_term = self.Kp * (q_targ - q_curr)
        p_term =self.plant.MapQDotToVelocity(self.plant_context, p_term)
        a_targ = p_term + self.Kd * (v_targ - v_curr)

        out_torques = a_targ
        # Apply torque limit
        out_torques = np.clip(out_torques, -self.torque_limit, self.torque_limit)
        out_actuation = self.Bpinv.dot(out_torques)
        output.SetFromVector(out_actuation)


def add_ground(plant):
    ground_model = plant.AddModelInstance("ground_model")
    ground_box = plant.AddRigidBody(
        "ground", ground_model,
        SpatialInertia(1, np.array([0, 0, 0]), UnitInertia(1, 1, 1)))
    X_WG = RigidTransform([0, 0, -0.05])
    ground_geometry_id = plant.RegisterCollisionGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        CoulombFriction(0.1, 0.05))
    plant.RegisterVisualGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        [0.5, 0.5, 0.5, 1.])
    plant.WeldFrames(plant.world_frame(), ground_box.body_frame(), X_WG)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf_path", help="path to sdf")
    parser.add_argument("--interactive", action='store_true')
    parser.add_argument("--welded", action='store_true')
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

    q_init = np.array([0.25, np.pi/4, -np.pi/2.,
             -0.25, np.pi/4, -np.pi/2.,
             0.25, np.pi/4, -np.pi/2.,
             -0.25, np.pi/4, -np.pi/2.])

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0005)
    parser = Parser(plant)
    model = parser.AddModelFromFile(args.sdf_path)
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("body"), RigidTransform(p=[0., 0., 0.5]))
    if args.welded:
        plant.WeldFrames(plant.world_frame(), plant.GetBodyByName("body").body_frame())
    else:
        add_ground(plant)
    plant.Finalize()

    if args.meshcat:
        meshcat = ConnectMeshcatVisualizer(
            builder, output_port=scene_graph.get_query_output_port(),
            zmq_url=args.meshcat, open_browser=args.open_browser)

    controller = builder.AddSystem(ServoController(plant))
    if args.interactive:
        # Add sliders to set positions of the joints.
        sliders = builder.AddSystem(JointSliders(robot=plant))
        sliders.set_joint_position(q_init)
        builder.Connect(sliders.get_output_port(0), controller.get_input_port(0))
    else:
        source = builder.AddSystem(ConstantVectorSource(np.zeros(controller.nu)))
        builder.Connect(source.get_output_port(0), controller.get_input_port(0))

    builder.Connect(plant.get_state_output_port(), controller.get_input_port(1))
    force = np.zeros(plant.num_actuated_dofs())
    #force[2] = -1.
    #force_source = builder.AddSystem(ConstantVectorSource(force))
    #builder.Connect(force_source.get_output_port(0), plant.get_actuation_input_port())
    builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(1E6)

if __name__ == '__main__':
    main()