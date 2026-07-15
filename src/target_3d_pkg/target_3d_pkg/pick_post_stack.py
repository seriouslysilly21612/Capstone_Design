"""Post-detection stack: pick_logic + pick_target_3d + pick_target_base
in ONE process (2026-07-14 CPU-reduction phase 2 — TRIED AND REJECTED).

Measured result: the merged process cost MORE CPU than the three separate
ones (36.5% vs 31.1% sum; whole-pipeline 45.3% vs 42.9%). Cause: rclpy's
SingleThreadedExecutor rebuilds its wait-set over ALL entities on every
iteration, so combining three nodes (subscriptions + parameter services)
raised every callback's dispatch cost by more than the saved per-process /
DDS-participant overhead. The launch file therefore still starts the three
separate executables. Kept for reference; a real win here would require
rclcpp composition (intra-process, C++ executor), not an rclpy merge.
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor

from pick_logic_pkg.pick_logic import PickLogicNode

from target_3d_pkg.pick_target_3d_node import PickTarget3DNode
from target_3d_pkg.pick_target_base_node import PickTargetBaseNode


def main(args=None):
    rclpy.init(args=args)
    nodes = [PickLogicNode(), PickTarget3DNode(), PickTargetBaseNode()]
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
