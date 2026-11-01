import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from amazon_robot_msg.action import FollowTargets
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

'''
JdeMultirobot 

The goal of this exercise is to create a plan which describes the pickup and drop-off locations of pallets, and then
executes it. A plan for each robot consists of two arrays describing locations to drive to (PoseStamped)
and action to be performed (Load /unload/ do nothing).

In our use case, task is described as 
[drive to pickup -> picking up -> drive to drop-off -> unload] 

Following are some of the mandatory (!) , optional (/) and feature(*) goals. For feature goals, you will also have 
to implement the logic code by yourself. 

1. (!) Create a collision Free plan. No robots should collide with each other.
2. (!) Move at least 2 pallets from pallet locations to storage location
3. (/) No replanning / Recovery behaviour used while driving
4. (/) Fastest moving of all pallets from pickup to drop-off
5. (/) Fastest moving of all pallets from pickup to drop-off and back
6. (/) Lowest average time spent per pallet per task. One task is drive -> pickup -> drop-off
7. (/) Avoiding cross paths
8. (/) Multiple stops before reaching goal
9. (/) Dynamic replanning based on current conditions 
10. (*) Energy of the robot required to drive to the pallet
11. (*) High priority pallet first.

and so on .. 

You can consider following parameters while making the plan:
1. Distance from robot to the nearest pallet
2. Distance of pallet from the storage location
3. Current waypoints, remaining waypoints ..
4. State of the lift joint  

To access these parameters, you will have to use various ros concepts such as subscribing to a topic or 
creating an action client (just like this one!)

For example, to get a path between current position of the robot and another position (in our case, a pallet
at positon (x,y,z)), you can call the action server /<robot_name>/compute_path_to_pose which will return you 
an array of points of the path.

You can also get the current robot position using robot odometry topic (/<robot_name>/odom), which you can use for
different calculations. You can also experiment with different path planners, costmaps, behaviour trees and planners.


Following topics might be helpful:
/<robot_name>/amcl_pose
/<robot_name>/scan
/<robot_name>/imu
/<robot_name>/odom
/<robot_name>/initialpose
/<robot_name>/cmd_vel
/<robot_name>/map


Following action servers might be useful:
/<robot_name>/FollowTargets
/<robot_name>/backup
/<robot_name>/compute_path_to_pose
/<robot_name>/follow_path
/<robot_name>/navigate_to_pose
/<robot_name>/spin
/<robot_name>/wait


Have fun!

'''


pallets = {
    'pallet_1': {'x_pose': 3.58, 'y_pose': 0.49, 'z_pose': 0.01},
    'pallet_2': {'x_pose': 3.60, 'y_pose': -1.56, 'z_pose': 0.01},
    'pallet_3': {'x_pose': 3.61, 'y_pose': -3.29, 'z_pose': 0.01},
    'pallet_4': {'x_pose': 3.58, 'y_pose': -5.07, 'z_pose': 0.01},
    'pallet_5': {'x_pose': 3.61, 'y_pose': -6.91, 'z_pose': 0.01},
    'pallet_6': {'x_pose': 3.72, 'y_pose': -8.88, 'z_pose': 0.01},
}

storage_locations = {
    'storage_location_1': {'x_pose': -5.84, 'y_pose': -3.35, 'z_pose': 0.01},
    'storage_location_2': {'x_pose': -5.84, 'y_pose': 1.12, 'z_pose': 0.01},
    'storage_location_3': {'x_pose': -5.84, 'y_pose': -7.76, 'z_pose': 0.01},
}



free_area = {
    'right_end_of_corridor': {'x_pose': 1.35, 'y_pose': -6.78, 'z_pose': 0.01},
    'left_end_of_corridor': {'x_pose': 0.92, 'y_pose': 6.45, 'z_pose': 0.01},
}

lift_stages = {'load': 2, 'unload': -2, 'half_load': 1, 'half_unload': -1, 'unchanged': 0}


def get_pose_stamped(x_pose, y_pose, z_pose, yaw=None):
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = x_pose
    pose_stamped.pose.position.y = y_pose
    pose_stamped.pose.position.z = z_pose
    if yaw is not None:     # otherwise the quaternion stays all-zero (heading not enforced)
        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
    return pose_stamped


class WarehouseController:

    def __init__(self):
        self.action_client1 = FollowTargetActionClient('/robot1/FollowTargets', 'robot1_follow_target_action_client')
        self.action_client2 = FollowTargetActionClient('/robot2/FollowTargets', 'robot2_follow_target_action_client')
        r1 = self.robot1 = Sequencer('robot1', self.action_client1)
        r2 = self.robot2 = Sequencer('robot2', self.action_client2)
        # 3 beds only, in every other slot (3.7 m apart, so a carried bed never hits a neighbour):
        #   bed 1 = pallet_1 slot (y 0.49), bed 2 = pallet_3 slot (y -3.29), bed 3 = pallet_5 slot (y -6.91)
        #  1) robot1 walks into the passage beside the beds (next to bed 3) and waits,
        #  2) once robot1 is parked there, robot2 takes bed 2 to storage_location_1,
        #  3) once bed 2 is down, robot1 takes bed 1 to storage_location_2,
        #  4) once robot1 has lifted bed 1 and is back out in the aisle (heading west at y~1),
        #     robot2 takes bed 3 to storage_location_3.
        # Every pickup first stops in the aisle facing the slot, then drives straight in.
        bed1, bed2, bed3 = bed1_exact, pallets['pallet_3'], pallets['pallet_5']
        r1.add_move(passage_wait)                                            # r1 job 0
        r2.add_delivery(bed2, storage_locations['storage_location_1'],
                        start_when=lambda: r1.job_index >= 1,                # r1 parked in the passage
                        approach=facing_slot(bed2))
        r1.add_delivery(bed1, storage_locations['storage_location_2'],
                        start_when=lambda: r2.delivered >= 1,                # r1 jobs 1-4
                        approach=bed1_approach)
        r2.add_delivery(bed3, storage_locations['storage_location_3'],
                        start_when=lambda: r1.job_index >= 4,                # r1 lifted + in aisle
                        approach=facing_slot(bed3))

    # TODO: Use this function to create your plan
    def create_plan(self):
        pass

    def create_test_plan(self):
        print("Plan (3 beds, every other slot): robot1 -> passage | robot2 bed2 -> storage_1 | robot1 bed1 -> storage_2 | robot2 bed3 -> storage_3")

    def execute_plan(self):
        executor = MultiThreadedExecutor()
        for node in (self.action_client1, self.action_client2, self.robot1, self.robot2):
            executor.add_node(node)
        executor.spin()


def aisle(pallet):
    """Point in the aisle straight out from a pallet slot. After lifting, the robot first
    drives here so the carried pallet doesn't sweep along (and shove) the neighbouring pallets."""
    return {'x_pose': 1.6, 'y_pose': pallet['y_pose'], 'z_pose': 0.01}


def facing_slot(pallet):
    """Aisle point in front of a bed slot, facing it (yaw 0), so the robot drives straight in."""
    return dict(aisle(pallet), yaw=0.0)


# Where robot1 waits: in the passage beside the beds, next to bed 3 (its legs keep robot1's
# localization exact while it waits), > 3 m from everything robot2 does for bed 2.
passage_wait = {'x_pose': 1.4, 'y_pose': -6.0, 'z_pose': 0.01}

# Bed 1's exact pose on the robots' map: centre of its 4 legs in map_3beds (x legs 2.65 / 4.555,
# y legs 0.03..0.83) and its real heading (Gazebo world: yaw 1.588 rad = 0.017 rad off square).
# Robot1 lines up with the bed in the aisle, then drives straight in along the bed's long axis.
BED1_YAW = 1.588 - math.pi / 2
bed1_exact = {'x_pose': 3.60, 'y_pose': 0.43, 'z_pose': 0.01, 'yaw': BED1_YAW}
bed1_approach = {'x_pose': 1.6, 'y_pose': 0.43 - 2.0 * math.tan(BED1_YAW), 'z_pose': 0.01,
                 'yaw': BED1_YAW}


class Sequencer(Node):
    """Sends one robot its goals one at a time. Each delivery = lift at the pallet, back out
    to the aisle, put down at the storage spot; it starts once its `start_when()` is true.
    A failed goal is retried (then skipped)."""

    def __init__(self, name, client, max_retries=5):
        super().__init__(name + '_sequencer')
        self.name, self.client, self.max_retries = name, client, max_retries
        self.jobs = []                  # (pose, load, start_when or None, delivery number)
        self.job_index = self.retries = self.delivered = 0
        self.busy = self.finished = False
        # Lift efforts sent to Gazebo add up (each lasts 2000 s). Track the sum so that a
        # load leaves +2 (lift up) and an unload leaves -1: with exactly 0 the light lift
        # plate stays touching the pallet and drags it away when the robot leaves.
        self.lift_net = 0
        self.pending_effort = 0
        self.pause_until = 0.0
        self.create_timer(1.0, self.tick)

    def add_move(self, pose, start_when=None):
        self.jobs.append((pose, lift_stages['unchanged'], start_when, 0))

    def add_delivery(self, pallet, storage, start_when=None, approach=None):
        """approach: optional point to drive to first (straight in front of the bed), so the
        robot enters the bed slot from the aisle and not diagonally past other racks."""
        n = self.deliveries = getattr(self, 'deliveries', 0) + 1
        if approach is not None:
            self.jobs.append((approach, lift_stages['unchanged'], start_when, n))
            start_when = None
        self.jobs += [(pallet, lift_stages['load'], start_when, n),
                      (aisle(pallet), lift_stages['unchanged'], None, n),
                      (storage, lift_stages['unload'], None, n)]

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self):
        if self.busy or self.finished or self.now() < self.pause_until:
            return
        if self.job_index >= len(self.jobs):
            self.finished = True
            self.get_logger().info('%s finished all deliveries' % self.name)
            return
        pose, load, start_when, n = self.jobs[self.job_index]
        if start_when is not None and not start_when():
            return                      # waiting for the other robot's turn
        if load == lift_stages['load'] and self.retries == 0:
            self.get_logger().info('%s: delivery %d started' % (self.name, n))
        if load == lift_stages['load']:
            effort = 2 - self.lift_net
        elif load == lift_stages['unload']:
            effort = -1 - self.lift_net
        else:
            effort = 0
        self.pending_effort = effort
        self.busy = True
        # The controller ignores the load list when it has only 1 entry (it then just toggles
        # lift up/down), so send the pose twice: [load, unchanged].
        self.client.send_targets([get_pose_stamped(**pose)] * 2, [effort, lift_stages['unchanged']],
                                 done_cb=self.on_done)

    def on_done(self, ok):
        if ok:
            self.lift_net += self.pending_effort
            if self.pending_effort != 0:
                self.pause_until = self.now() + 3.0   # let the lift finish moving before driving
        if ok or self.retries >= self.max_retries:
            if not ok:
                self.get_logger().warn('%s job %d FAILED, skipped' % (self.name, self.job_index))
            if self.jobs[self.job_index][1] == lift_stages['unload']:
                self.delivered += 1
                self.get_logger().info('%s: delivery %d done' % (self.name, self.delivered))
            self.job_index += 1
            self.retries = 0
        else:
            self.retries += 1
            self.get_logger().warn('%s job %d failed, retry %d' % (self.name, self.job_index, self.retries))
        self.busy = False


class FollowTargetActionClient(Node):

    def __init__(self, action_name='/robot1/FollowTargets', action_client_name='follow_target_action_client'):
        super().__init__(action_client_name)
        self._action_client = ActionClient(self, FollowTargets, action_name)
        self.current_waypoint = 0


    def get_action_client(self):
        return self._action_client

    def send_targets(self, poses, loads, done_cb=None):
        self.get_logger().info('Received Goal poses: {0}'.format(len(poses)))
        self.get_logger().info('Received Load instructions: {0}'.format(len(loads)))
        if len(poses) == len(loads):
            goal_msg = FollowTargets.Goal()
            goal_msg.poses = poses
            goal_msg.load = loads
            self.goal_length = len(poses)
            self._action_client.wait_for_server()
            future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            if done_cb is not None:
                future.add_done_callback(lambda f: self._on_accepted(f, done_cb))
        else:
            self.get_logger().warn('Unequal amount of Poses and loads!')


    def _on_accepted(self, future, done_cb):
        handle = future.result()
        if not handle.accepted:
            done_cb(False)
            return
        handle.get_result_async().add_done_callback(
            lambda f: done_cb(f.result().status == GoalStatus.STATUS_SUCCEEDED))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_waypoint = feedback.current_waypoint
        # self.get_logger().info('Currently Executing: {0} out of {0} targets'.format(feedback.current_waypoint, self.goal_length))



def main(args=None):
    rclpy.init(args=args)
    warehouse_controller = WarehouseController()
    warehouse_controller.create_test_plan()
    warehouse_controller.execute_plan()


if __name__ == '__main__':
    main()
