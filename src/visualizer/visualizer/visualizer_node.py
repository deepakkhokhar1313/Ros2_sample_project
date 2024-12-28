import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rviz2.bindings import rviz2_display


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.rviz = rviz2_display.RvizDisplay()
        self.declare_parameter('frame_id', 'world',ParameterDescriptor(description = "reference frame"))
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )
        self.rviz.initialize(self)


    def joint_state_callback(self, msg):
        self.rviz.update_robot_state(msg,frame_id = self.frame_id)

def main(args=None):
    rclpy.init(args=args)
    vis_node = VisualizationNode()
    rclpy.spin(vis_node)
    vis_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()