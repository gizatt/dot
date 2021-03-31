"""
Modified from https://github.com/RobotLocomotion/drake/blob/4751fd6f0b61313faceccb7c6081bc077b35448b/bindings/pydrake/multibody/examples/jupyter_widgets_examples.ipynb
Runs simple visualization of the supplied SDF with sliders for moving joints.
"""

import argparse
import sys

import numpy as np

from pydrake.all import (
    ConnectMeshcatVisualizer,
    DiagramBuilder,
    PiecewisePolynomial,
    MeshcatVisualizer,
    Simulator
)

from drake_dot_sim import setup_dot_diagram, setup_argparse_for_setup_dot_diagram, ServoSliders, TrajectoryLooper

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-t1", default=0.01, help="Extend time")
    parser.add_argument("-t2", default=0.1, help="Hold time")
    setup_argparse_for_setup_dot_diagram(parser)
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

    t1 = float(args.t1)
    t2 = float(args.t2)

    q_stance_1 = np.array([1600, 2250, 1600,
                         1600, 2100, 2000,
                         1400, 2100, 2000,
                         1400, 2250, 1600])


    q_stance_2 = np.array([1600, 2100, 2000,
                         1600, 2250, 1600,
                         1400, 2250, 1600,
                         1400, 2100, 2000])

    breaks = np.cumsum([0., t1, t2, t1, t2])
    samples = np.stack([q_stance_1, q_stance_2, q_stance_2, q_stance_1, q_stance_1]).T
    trajectory = PiecewisePolynomial.FirstOrderHold(breaks, samples)

    builder = DiagramBuilder()
    plant, scene_graph, servo_controller = setup_dot_diagram(
        builder, args)
    

    trajectory_source = builder.AddSystem(TrajectoryLooper(trajectory))
    builder.Connect(trajectory_source.get_output_port(0), servo_controller.get_input_port(0))

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