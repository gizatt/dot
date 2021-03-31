"""
Modified from https://github.com/RobotLocomotion/drake/blob/4751fd6f0b61313faceccb7c6081bc077b35448b/bindings/pydrake/multibody/examples/jupyter_widgets_examples.ipynb
Runs simple visualization of the supplied SDF with sliders for moving joints.
"""

import argparse
import sys

import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    BasicVector,
    ConnectMeshcatVisualizer,
    ConstantVectorSource,
    DiagramBuilder,
    JointSliders,
    LeafSystem,
    MeshcatVisualizer,
    MultibodyForces,
    MultibodyPlant,
    Parser,
    RigidTransform,
    SceneGraph,
    Simulator
)

from drake_dot_sim import setup_dot_diagram, setup_argparse_for_setup_dot_diagram

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    setup_argparse_for_setup_dot_diagram(parser)
    parser.add_argument("--interactive", action='store_true')
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

    q_init = np.array([0.25, np.pi/4, -np.pi/2.,
             -0.25, np.pi/4, -np.pi/2.,
             0.25, np.pi/4, -np.pi/2.,
             -0.25, np.pi/4, -np.pi/2.])

    builder = DiagramBuilder()
    plant, scene_graph, servo_controller = setup_dot_diagram(
        builder, args)
    
    if args.interactive:
        # Add sliders to set positions of the joints.
        sliders = builder.AddSystem(JointSliders(robot=plant))
        sliders.set_joint_position(q_init)
        builder.Connect(sliders.get_output_port(0), servo_controller.get_input_port(0))
    else:
        source = builder.AddSystem(ConstantVectorSource(np.zeros(servo_controller.nu)))
        builder.Connect(source.get_output_port(0), servo_controller.get_input_port(0))

    if args.meshcat:
        meshcat = ConnectMeshcatVisualizer(
            builder, output_port=scene_graph.get_query_output_port(),
            zmq_url=args.meshcat, open_browser=args.open_browser)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(1E6)

if __name__ == '__main__':
    main()