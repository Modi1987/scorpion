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
        self.initialize_properties()
                
        # Set mode (real or virtual)
        self.real_mode_flag = True
        if self.real_mode_flag:
            from adafruit_servokit import ServoKit
            self.kit = ServoKit(channels=16)  # Use 16-channel board
            for i in range(self.joints_count):
                self.kit.servo[i].set_pulse_width_range(self.servo_min_pulse_width_microsec[i], self.servo_max_pulse_width_microsec[i]) 
                self.kit.servo[i].actuation_range = self.servo_actuation_range_degree[i]

        # Publisher actuators setpoint from joint states
        self.setpoint_publisher_ = self.create_publisher(JointState, '/actuator_setpoint_degree', 10)
        self.joint_states_subscriber = self.create_subscription(
                JointState,
                "/joint_states",
                lambda msg : self.on_joint_states_callback(msg),
                10  # Set QoS to 10
            )
    
    def initialize_properties(self):
        # Initialize i2c related joint states names and servo position vector (degrees)
        self.joints_states_names = []
        self.q = [0.0] * self.joints_count # geometrical joint angle rads
        self.actuator_setpoint_degree = [0.0] * self.joints_count # servo motor angle degree
        for i in range(self.limbs_num):
            for j in range(self.joints_per_limb[i]):
                joint_name = f'limb{i}/joint{j}'
                self.joints_states_names.append(joint_name)

    def on_joint_states_callback(self, msg):
        # Create dictionary of joints stats from received /joint_states message
        joint_states_dict = {name: {'position': pos, 'velocity': vel, 'effort': eff}
                        for name, pos, vel, eff in zip(
                            msg.name,
                            msg.position if msg.position else [None] * len(msg.name),
                            msg.velocity if msg.velocity else [None] * len(msg.name),
                            msg.effort if msg.effort else [None] * len(msg.name)
                        )}
        # Update received joints states and log errors
        for i, name in enumerate(self.joints_states_names):
            if name in joint_states_dict:
                joint_state = joint_states_dict[name]
                self.q[i] = joint_state['position']
            else:
                self.get_logger().error(f"Joint {name} not found in the current message.")
        # Calculate actuator setpoints in degrees
        for i in range(self.joints_count):
            servo_setpoint = self.dir[i] * (self.q[i] * 180.0 / math.pi) + self.initial_joints_bias_degree[i]
            max_val = self.servo_actuation_range_degree[i]
            self.actuator_setpoint_degree[i] = self.clamp(servo_setpoint, i, 0.0, max_val)
        # Publish actuators setpoint
        self.publish_actuators_setpoint()
    
    def clamp(self, value, index, min_val, max_val, margin=1.0):
        min_safe_limit = min_val + margin
        if value < (min_safe_limit):
            self.get_logger().error(f'ERROR: Servo[{index}] calculated setpoint is {value} degrees, however its minimum permissible angle is {min_safe_limit}, clamping value to {min_safe_limit}!')
            return min_safe_limit
        max_safe_limit = max_val - margin
        if value > (max_val - margin):
            self.get_logger().error(f'ERROR: Servo[{index}] calculated setpoint is {value} degrees, however maximum servo angle is {max_safe_limit}, clamping value to {max_safe_limit}!')            
            return max_safe_limit
        return value


    def publish_actuators_setpoint(self):
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
        self.declare_parameter('joints_per_limb', [3]*5)  # Default value 3
        self.declare_parameter('i2c_actuators_params.update_interval_millis', 100)  # Default 100 ms
        self.declare_parameter('i2c_actuators_params.actuator_angle_bias_at_joint_zero_degree', [0.0] * 15)  # Default bias
        self.declare_parameter('i2c_actuators_params.dir', [1.0] * 15)  # Default direction (1.0 for no inversion)
        self.declare_parameter('servo_parameters.servo_actuation_range_degree', [180.0] * 15) # angular range degree
        self.declare_parameter('servo_parameters.servo_min_pulse_width_microsec', [500.0] * 15) # microseconds
        self.declare_parameter('servo_parameters.servo_max_pulse_width_microsec', [2500.0] * 15) # microseconds

    def load_params(self):
        # Load parameters and handle errors
        self.limbs_num = self.get_parameter('limbs_num').get_parameter_value().integer_value
        # Helper to format array for logging
        def format_array_to_string(x_list):
            return '[' + ', '.join(map(str, x_list)) + ']'
        self.joints_per_limb = self.get_parameter('joints_per_limb').get_parameter_value().integer_array_value
        if (len(self.joints_per_limb) != self.limbs_num):
            self.get_logger().error(f" Error, limbs_num paramters {self.limbs_num} is not equal to the size of the vector joints_per_limb {len(self.joints_per_limb)}")
        self.get_logger().info(f'Loaded limbs_num: {self.limbs_num}, joints_per_limb: {format_array_to_string(self.joints_per_limb)}')
        self.joints_count = sum(self.joints_per_limb)
        self.get_logger().info(f'Total limbs joints count is: {self.joints_count}') 
        self.update_interval_millis = self.get_parameter('i2c_actuators_params.update_interval_millis').get_parameter_value().integer_value
        self.get_logger().info(f'Loaded update_interval_millis for I2C bus: {self.update_interval_millis} ms')
        self.initial_joints_bias_degree = self.get_parameter('i2c_actuators_params.actuator_angle_bias_at_joint_zero_degree').get_parameter_value().double_array_value
        self.dir = self.get_parameter('i2c_actuators_params.dir').get_parameter_value().double_array_value
        self.servo_actuation_range_degree = self.get_parameter('servo_parameters.servo_actuation_range_degree').get_parameter_value().double_array_value
        self.servo_min_pulse_width_microsec = self.get_parameter('servo_parameters.servo_min_pulse_width_microsec').get_parameter_value().double_array_value
        self.servo_max_pulse_width_microsec = self.get_parameter('servo_parameters.servo_max_pulse_width_microsec').get_parameter_value().double_array_value

        if len(self.initial_joints_bias_degree) != self.joints_count:
            self.get_logger().error('ERROR: actuator_angle_bias_at_joint_zero_degree parameter size mismatch!')
        else:
            self.get_logger().info(f'Initial joints bias in degree is loaded: {format_array_to_string(self.initial_joints_bias_degree)}')

        if len(self.dir) != self.joints_count:
            self.get_logger().error('ERROR: Direction array size mismatch with joints count!')
        else:
            self.get_logger().info(f'Direction array is loaded: {format_array_to_string(self.dir)}')

        if len(self.servo_actuation_range_degree) != self.joints_count:
            self.get_logger().error('ERROR: Servo angle range array size mismatch with joints count!')
        else: 
            self.get_logger().info(f'Servo joints angle range is loaded: {format_array_to_string(self.servo_actuation_range_degree)}')

        if len(self.servo_min_pulse_width_microsec) != self.joints_count:
            self.get_logger().error('ERROR: Servo minimum pulse width array size mismatch!')
        else: 
            self.get_logger().info(f'Servos minimum pulse width is loaded: {format_array_to_string(self.servo_min_pulse_width_microsec)}')

        if len(self.servo_max_pulse_width_microsec) != self.joints_count:
            self.get_logger().error('ERROR: Servo maximum pulse width array size mismatch!')
        else:
            self.get_logger().info(f'Servos maximum pulse width is loaded: {format_array_to_string(self.servo_max_pulse_width_microsec)}')


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
