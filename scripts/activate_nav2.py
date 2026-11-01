#!/usr/bin/env python3
"""Programmatic replacement for the "Startup" button of the Nav2 panel in RViz.

For each robot: ask /<robot>/lifecycle_manager_<stage>/is_active; if not active,
send ManageLifecycleNodes STARTUP (exactly what the RViz panel does). Then poll
is_active until every robot reports active. Idempotent / safe to re-run.

  activate_nav2.py --stage localization robot1 robot2
  activate_nav2.py --stage navigation   robot1 robot2
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Trigger


def call(node, client, req, timeout):
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result() if fut.done() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage', choices=['localization', 'navigation'], required=True)
    ap.add_argument('--timeout', type=float, default=240.0)
    ap.add_argument('robots', nargs='+')
    args = ap.parse_args()

    rclpy.init()
    node = Node('demo_nav2_activator_' + args.stage)
    log = node.get_logger()
    deadline = time.time() + args.timeout
    mgr = {r: f'/{r}/lifecycle_manager_{args.stage}' for r in args.robots}
    active_cli = {r: node.create_client(Trigger, m + '/is_active') for r, m in mgr.items()}
    manage_cli = {r: node.create_client(ManageLifecycleNodes, m + '/manage_nodes') for r, m in mgr.items()}

    for r in args.robots:
        for cli in (active_cli[r], manage_cli[r]):
            while not cli.wait_for_service(timeout_sec=1.0):
                if time.time() > deadline:
                    log.error(f'{cli.srv_name} not available'); sys.exit(1)

    def is_active(r):
        res = call(node, active_cli[r], Trigger.Request(), 5.0)
        return bool(res and res.success)

    pending_startup = {}
    for r in args.robots:
        if is_active(r):
            log.info(f'{r}: {args.stage} already active')
            continue
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP
        pending_startup[r] = manage_cli[r].call_async(req)   # don't block: startup can take a while
        log.info(f'{r}: STARTUP sent to {mgr[r]}/manage_nodes')

    remaining = set(args.robots)
    while remaining and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        for r in sorted(remaining):
            if is_active(r):
                log.info(f'{r}: {args.stage} stack ACTIVE')
                remaining.discard(r)
    for r in remaining:
        log.error(f'{r}: {args.stage} stack NOT active after {args.timeout:.0f}s')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if remaining else 0)


if __name__ == '__main__':
    main()
