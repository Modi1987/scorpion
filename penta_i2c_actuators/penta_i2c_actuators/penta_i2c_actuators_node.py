import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import sys

class PentaI2CActuators(Node):
    def __init__(self, mode):
        super().__init__('penta_i2c_actuators')
        self.get_logger().info(f'Starting I2C actuators control in {mode} mode')

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
        self.servo_actuation_range_degree = [0.0] * self.joints_count
        self.servo_min_pulse_width_microsec = [0.0] * self.joints_count
        self.servo_max_pulse_width_microsec = [0.0] * self.joints_count
        
        # Set mode (real or virtual)
        self.real_mode_flag = True if mode == 'real' else False
        if self.real_mode_flag:
            from adafruit_servokit import ServoKit
            self.kit = ServoKit(channels=16)  # Use 16-channel board
            for i in range(self.joints_count):
                self.kit.servo[i].set_pulse_width_range(self.servo_min_pulse_width_microsec[i], self.servo_max_pulse_width_microsec[i]) 
                self.kit.servo[i].actuation_range = self.servo_angle_range_degree[i]

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
        self.joints_pos_received = False

    def timer_callback(self):
        if not self.joints_pos_received:
            self.get_logger().warn("Joints angles not received yet, will not send commands on the I2C bus.")
            return
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
        # command the motors over i2c bus
        if self.real_mode_flag:
            # Actuate the motors
            for i in range(self.joints_count):
                self.kit.servo[i].angle = self.actuator_setpoint_degree[i]
    
    def declare_params(self):
        # Declare robot geometry parameters
        self.declare_parameter('limbs_num', 5)  # Default value 5
        self.declare_parameter('joints_per_limb', 3)  # Default value 3
        self.declare_parameter('i2c_actuators_params.update_interval_millis', 100)  # Default 100 ms
        self.declare_parameter('i2c_actuators_params.actuator_angle_bias_at_joint_zero_degree', [0.0] * 15)  # Default bias
        self.declare_parameter('i2c_actuators_params.dir', [1.0] * 15)  # Default direction (1.0 for no inversion)
        self.declare_parameter('servo_parameters.servo_angle_range_degree', [180.0] * 15) # angular range degree
        self.declare_parameter('servo_parameters.servo_min_pulse_width_microsec', [500.0] * 15) # microseconds
        self.declare_parameter('servo_parameters.servo_max_pulse_width_microsec', [2500.0] * 15) # microseconds

    def load_params(self):
        # Load parameters and handle errors
        self.limbs_num = self.get_parameter('limbs_num').get_parameter_value().integer_value
        self.joints_per_limb = self.get_parameter('joints_per_limb').get_parameter_value().integer_value
        self.get_logger().info(f'Loaded limbs_num: {self.limbs_num}, joints_per_limb: {self.joints_per_limb}')
        self.joints_count = self.limbs_num * self.joints_per_limb
        self.update_interval_millis = self.get_parameter('i2c_actuators_params.update_interval_millis').get_parameter_value().integer_value
        self.get_logger().info(f'Loaded update_interval_millis for I2C bus: {self.update_interval_millis} ms')
        self.initial_joints_bias_degree = self.get_parameter('i2c_actuators_params.actuator_angle_bias_at_joint_zero_degree').get_parameter_value().double_array_value
        self.dir = self.get_parameter('i2c_actuators_params.dir').get_parameter_value().double_array_value
        self.servo_angle_range_degree = self.get_parameter('servo_parameters.servo_angle_range_degree').get_parameter_value().double_array_value
        self.servo_min_pulse_width_microsec = self.get_parameter('servo_parameters.servo_min_pulse_width_microsec').get_parameter_value().double_array_value
        self.servo_max_pulse_width_microsec = self.get_parameter('servo_parameters.servo_max_pulse_width_microsec').get_parameter_value().double_array_value

        # Helper to format array for logging
        def format_array_to_string(x_list):
            return '[' + ', '.join(map(str, x_list)) + ']'

        if len(self.initial_joints_bias_degree) != self.joints_count:
            self.get_logger().error('ERROR: actuator_angle_bias_at_joint_zero_degree parameter size mismatch!')
        else:
            self.get_logger().info(f'Initial joints bias in degree is loaded: {format_array_to_string(self.initial_joints_bias_degree)}')

        if len(self.dir) != self.joints_count:
            self.get_logger().error('ERROR: Direction array size mismatch with joints count!')
        else:
            self.get_logger().info(f'Direction array is loaded: {format_array_to_string(self.dir)}')

        if len(self.servo_angle_range_degree) != self.joints_count:
            self.get_logger().error('ERROR: Servo angle range array size mismatch with joints count!')
        else: 
            self.get_logger().info(f'Servo joints angle range is loaded: {format_array_to_string(self.servo_angle_range_degree)}')

        if len(self.servo_min_pulse_width_microsec) != self.joints_count:
            self.get_logger().error('ERROR: Servo minimum pulse width array size mismatch!')
        else: 
            self.get_logger().info(f'Servos minimum pulse width is loaded: {format_array_to_string(self.servo_min_pulse_width_microsec)}')

        if len(self.servo_max_pulse_width_microsec) != self.joints_count:
            self.get_logger().error('ERROR: Servo maximum pulse width array size mismatch!')
        else:
            self.get_logger().info(f'Servos maximum pulse width is loaded: {format_array_to_string(self.servo_max_pulse_width_microsec)}')

    def on_joint_state_callback_limb(self, limb_index, joint_state):
        # Update the q vector with the received joint state positions for this limb
        index_start = limb_index * self.joints_per_limb
        self.q[index_start:index_start+self.joints_per_limb] = joint_state.position[:self.joints_per_limb]
        if not self.joints_pos_received:
            self.joints_pos_received = True

def main(args=None):
    rclpy.init(args=args)

    # Retrieve the mode from arguments
    mode = 'virtual'
    if len(sys.argv) > 1:
        mode = sys.argv[1]

    # Create the node with the mode passed
    node = PentaI2CActuators(mode=mode)

    # Keep the node alive to receive and process messages
    rclpy.spin(node)

    # Clean up on shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
