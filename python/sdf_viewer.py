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
from pydrake.all import MultibodyPlant, SceneGraph, Parser, FindResourceOrThrow, ConnectMeshcatVisualizer, JointSliders
from pydrake.systems.rendering import MultibodyPositionToGeometryPose

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf_path", help="path to sdf")
    parser.add_argument("--interactive", action='store_true')
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    plant = MultibodyPlant(time_step=0.0)
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant)
    model = parser.AddModelFromFile(args.sdf_path)
    plant.Finalize()

    if args.meshcat:
        meshcat = ConnectMeshcatVisualizer(
            builder, output_port=scene_graph.get_query_output_port(),
            zmq_url=args.meshcat, open_browser=args.open_browser)

    if args.interactive:
        # Add sliders to set positions of the joints.
        sliders = builder.AddSystem(JointSliders(robot=plant))
        to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
        builder.Connect(sliders.get_output_port(0), to_pose.get_input_port())
        builder.Connect(
            to_pose.get_output_port(),
            scene_graph.get_source_pose_port(plant.get_source_id()))


    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(1E6)

if __name__ == '__main__':
    main()