import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class PentaI2CActuators(Node):
    def __init__(self):
        super().__init__('penta_i2c_actuators')
        self.get_logger().info('Starting I2C actuators control')

        # Declare and load parameters
        self.declare_params()
        self.load_params()

        # Initialize joint states names and position vector
        self.joints_states_names = []
        self.q = []
        self.actuator_setpoint_degree = []

        for i in range(self.limbs_num):
            for j in range(self.joints_per_limb):
                joint_name = f'limb{i}/joint{j}'
                self.joints_states_names.append(joint_name)

        # Initialize the joint states position vector
        self.q = [0.0] * self.joints_count
        self.actuator_setpoint_degree = [0.0] * self.joints_count

        # Publisher to publish aggregated joint states
        self.joint_state_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.setpoint_publisher_ = self.create_publisher(JointState, '/actuator_setpoint_degree', 10)

        # Subscribers for each limb's joint state topic
        self.limb_joints_subscriber_ = []
        for i in range(self.limbs_num):
            topic_string = f'/limb{i}/joint_state'
            subscriber = self.create_subscription(
                JointState,
                topic_string,
                lambda msg, limb_index=i: self.on_joint_state_callback_limb(limb_index, msg),
                10  # Set QoS to 10
            )
            self.limb_joints_subscriber_.append(subscriber)
        
        # Timer to call the publishing function at fixed intervals
        timer_interval_sec = self.update_interval_millis / 1000.0  # Convert millis to seconds
        self.timer = self.create_timer(timer_interval_sec, self.timer_callback)

    def timer_callback(self):
        # Publish aggregated joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joints_states_names
        msg.position = self.q
        self.joint_state_publisher_.publish(msg)

        # Publish actuator setpoints in degrees
        for i in range(self.joints_count):
            self.actuator_setpoint_degree[i] = self.dir[i] * (self.q[i] * 180.0 / math.pi - self.initial_joints_bias_degree[i])

        msg_setpoint = JointState()
        msg_setpoint.header.stamp = self.get_clock().now().to_msg()
        msg_setpoint.name = self.joints_states_names
        msg_setpoint.position = self.actuator_setpoint_degree
        self.setpoint_publisher_.publish(msg_setpoint)
    
    def declare_params(self):
        # Declare robot geometry parameters
        self.declare_parameter('i2c_actuators_params.limbs_num', 5)  # Default value 5
        self.declare_parameter('i2c_actuators_params.joints_per_limb', 3)  # Default value 3
        self.declare_parameter('i2c_actuators_params.update_interval_millis', 100)  # Default 100 ms
        self.declare_parameter('i2c_actuators_params.actuator_angle_bias_at_joint_zero_degree', [0.0] * 15)  # Default bias
        self.declare_parameter('i2c_actuators_params.dir', [1.0] * 15)  # Default direction (1.0 for no inversion)

    def load_params(self):
        # Load parameters and handle errors
        self.update_interval_millis = self.get_parameter('i2c_actuators_params.update_interval_millis').get_parameter_value().integer_value
        self.limbs_num = self.get_parameter('i2c_actuators_params.limbs_num').get_parameter_value().integer_value
        self.joints_per_limb = self.get_parameter('i2c_actuators_params.joints_per_limb').get_parameter_value().integer_value
        self.joints_count = self.limbs_num * self.joints_per_limb
        self.initial_joints_bias_degree = self.get_parameter('i2c_actuators_params.actuator_angle_bias_at_joint_zero_degree').get_parameter_value().double_array_value
        self.dir = self.get_parameter('i2c_actuators_params.dir').get_parameter_value().double_array_value

        if len(self.initial_joints_bias_degree) != self.joints_count:
            self.get_logger().error('ERROR, actuator_angle_bias_at_joint_zero_degree parameter size mismatch!')
        if len(self.dir) != self.joints_count:
            self.get_logger().error('ERROR, dir parameter size mismatch!')

        if self.limbs_num is not None:
            self.get_logger().info(f'(limbs_num) parameter loaded and equal to: {self.limbs_num}')
        else:
            self.get_logger().error('ERROR, cannot load (limbs_num) parameter')
        
        if self.joints_per_limb is not None:
            self.get_logger().info(f'(joints_per_limb) parameter loaded and equal to: {self.joints_per_limb}')
        else:
            self.get_logger().error('ERROR, cannot load (joints_per_limb) parameter')

    def on_joint_state_callback_limb(self, limb_index, joint_state):
        # Update the q vector with the received joint state positions for this limb
        index_start = limb_index * self.joints_per_limb
        self.q[index_start:index_start+self.joints_per_limb] = joint_state.position[:self.joints_per_limb]  # Assuming joint state size matches joints_per_limb


def main(args=None):
    rclpy.init(args=args)
    node = PentaI2CActuators()

    # Keep the node alive to receive and process messages
    rclpy.spin(node)

    # Clean up on shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
