import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
from arm_genesis_simulation.arm_genesis_simulation.genesis_bridge import GenesisInterface
import time
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        self.joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
            "finger_joint1", "finger_joint2"
        ]
        self.genesis_interface = GenesisInterface()
        self.genesis_interface.init_genesis()
        self.genesis_interface.build_scene()

         # Callback Groups for multithreading
        self.callback_group = ReentrantCallbackGroup()

        # Publishers
        self.joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', 10,callback_group = self.callback_group)
        self.joint_controller_state_publisher = self.create_publisher(
            JointTrajectoryControllerState, '/joint_trajectory_controller/state', 10,callback_group = self.callback_group)

         # Action Server
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.joint_command_callback,callback_group = self.callback_group)


        # Service Server
        self.reset_service = self.create_service(Trigger, '/reset_simulation', self.reset_callback,callback_group = self.callback_group)


        # Timers
        self.update_timer = self.create_timer(0.01, self.update_callback,callback_group = self.callback_group)

        # Initialize the simulation
        self.genesis_interface.reset_simulation()
        self.simulation_running = True
        self.simulation_thread = threading.Thread(target=self.simulation_loop)
        self.simulation_thread.start()
        self.target_positions = []
        self.current_target_index = 0
        self.get_logger().info("Arm controller node initialized.")

    def simulation_loop(self):
       while self.simulation_running:
           self.genesis_interface.step_simulation()
           time.sleep(0.001)


    def reset_callback(self, request, response):
        self.get_logger().info("Resetting simulation.")
        self.genesis_interface.reset_simulation()
        self.current_target_index=0
        response.success = True
        response.message = "Simulation reset."
        return response

    def joint_command_callback(self, goal_handle):
      self.get_logger().info("Received joint command.")
      if goal_handle.request.trajectory.points:
          target_positions = []
          for point in goal_handle.request.trajectory.points:
            target_positions.append(point.positions)
          self.target_positions = target_positions
          self.current_target_index = 0
          self.go_to_next_target()
      goal_handle.succeed()
      result = FollowJointTrajectory.Result()
      return result

    def go_to_next_target(self):
        if self.current_target_index < len(self.target_positions):
            target_position = self.target_positions[self.current_target_index]
            self.genesis_interface.control_joint_position(target_position)
            self.current_target_index += 1
            self.get_logger().info(f"Moving to target {self.current_target_index}")

        else:
            self.get_logger().info("Reached all the positions.")
    def update_callback(self):
        self.publish_joint_state()
        self.publish_joint_controller_state()
        self.go_to_next_target()

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names

        joint_state_msg.position = self.genesis_interface.get_joint_positions()
        joint_state_msg.velocity = self.genesis_interface.get_joint_velocities()
        joint_state_msg.effort = self.genesis_interface.get_joint_forces()

        self.joint_state_publisher.publish(joint_state_msg)

    def publish_joint_controller_state(self):
       controller_state_msg = JointTrajectoryControllerState()
       controller_state_msg.header.stamp = self.get_clock().now().to_msg()
       controller_state_msg.joint_names = self.joint_names

       controller_state_msg.actual.positions = self.genesis_interface.get_joint_positions()
       controller_state_msg.actual.velocities = self.genesis_interface.get_joint_velocities()
       controller_state_msg.actual.time_from_start = rclpy.duration.Duration(seconds=self.genesis_interface.get_current_time()).to_msg()
       controller_state_msg.desired.positions = self.genesis_interface.get_joint_positions()
       controller_state_msg.desired.velocities = self.genesis_interface.get_joint_velocities()
       controller_state_msg.desired.time_from_start = rclpy.duration.Duration(seconds = self.genesis_interface.get_current_time()).to_msg()
       controller_state_msg.error.positions =  [0]*len(self.joint_names)
       controller_state_msg.error.velocities = [0]*len(self.joint_names)
       controller_state_msg.error.time_from_start = rclpy.duration.Duration(seconds = 0).to_msg()

       self.joint_controller_state_publisher.publish(controller_state_msg)

    def destroy_node(self):
        self.simulation_running = False
        self.simulation_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    arm_controller_node = ArmControllerNode()
    rclpy.spin(arm_controller_node)
    arm_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()