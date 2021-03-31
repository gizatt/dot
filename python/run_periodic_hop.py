"""
Modified from https://github.com/RobotLocomotion/drake/blob/4751fd6f0b61313faceccb7c6081bc077b35448b/bindings/pydrake/multibody/examples/jupyter_widgets_examples.ipynb
Runs simple visualization of the supplied SDF with sliders for moving joints.
"""

import argparse
import sys

import numpy as np

from pydrake.all import (
    BasicVector,
    ConnectMeshcatVisualizer,
    DiagramBuilder,
    LeafSystem,
    PiecewisePolynomial,
    MeshcatVisualizer,
    Simulator
)

from drake_dot_sim import setup_dot_diagram, setup_argparse_for_setup_dot_diagram, ServoSliders


class TrajectoryLooper(LeafSystem):
    def __init__(self, trajectory):
        LeafSystem.__init__(self)
        self.trajectory = trajectory
        self.DeclareVectorOutputPort(
            "setpoint",
            BasicVector(self.trajectory.rows()),
            self.CalcOutput
        )

    def CalcOutput(self, context, output):
        t = context.get_time()
        looped_t = np.mod(t, self.trajectory.end_time())
        output.SetFromVector(self.trajectory.value(looped_t))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-t1", default=0.05, help="Extend leg")
    parser.add_argument("-t2", default=0.5, help="Dwell at top")
    parser.add_argument("-t3", default=0.5, help="Contract leg")
    parser.add_argument("-t4", default=0.1, help="Wait at bottom")
    setup_argparse_for_setup_dot_diagram(parser)
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()
    t1 = float(args.t1)
    t2 = float(args.t2)
    t3 = float(args.t3)
    t4 = float(args.t4)
    
    q_crouch = np.array([1600, 2000, 2000,
                         1600, 2000, 2000,
                         1400, 2000, 2000,
                         1400, 2000, 2000])

    q_extend = np.array([1600, 1600, 2400,
                         1600, 1600, 2400,
                         1400, 1600, 2400,
                         1400, 1600, 2400])
    breaks = np.cumsum([0., t1, t2, t3, t4])
    samples = np.stack([q_crouch, q_extend, q_extend, q_crouch, q_crouch]).T
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