import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import serial
import time
import math
import sys

# Serial settings
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
UPDATE_INTERVAL = 0.1  # seconds

# --- LX-824 Communication Constants ---
HEADER = [0x55, 0x55]

class PentaHiwonderActuators(Node):
    def __init__(self, mode):
        super().__init__('penta_hiwonder_actuators')
        if mode not in ['real', 'virtual']:
            self.get_logger().error(f'Invalid mode "{mode}" specified. Defaulting to "virtual" mode.')
            mode = 'virtual'

        if mode == 'real':
            self.real_mode_flag = True
        else:
            self.real_mode_flag = False

        self.get_logger().info(f'Starting hiwonder actuators control in {mode} mode')

        # Declare and load parameters
        self.declare_params()
        self.load_params()
        self.initialize_properties()

        if self.real_mode_flag:
            # Open serial connection
            is_successful = self.open_serial_connection()
            if not is_successful:
                self.get_logger().error('Failed to open serial port. Exiting...')
                sys.exit(1)

        # Publisher actuators setpoint from joint states
        self.setpoint_publisher_ = self.create_publisher(JointState, 'actuator_setpoint_degree', 1)
        self.ticks_publisher_ = self.create_publisher(JointState, 'actuator_ticks', 1)
        sub_callback_group = MutuallyExclusiveCallbackGroup()
        self.joint_states_subscriber = self.create_subscription(
                JointState,
                "joint_states",
                lambda msg : self.on_joint_states_callback(msg),
                1,
                callback_group=sub_callback_group
            )
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.motor_update_timer = self.create_timer(UPDATE_INTERVAL, self.update_motors_callback, callback_group=timer_callback_group)

    def open_serial_connection(self):
        try:
            self.serial = serial.Serial(PORT, BAUDRATE, timeout=1)
            self.get_logger().info(f'Serial port {PORT} opened successfully at {BAUDRATE} baudrate.')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port {PORT}: {e}')
            return False

    def update_motors_callback(self):
        if self.real_mode_flag:
            # Actuate the motors
            for i in range(self.joints_count):
                position_degree = self.actuator_setpoint_degree[i]
                actuation_range_degree = self.servo_actuation_range_degree[i]
                ticks_span = self.servo_max_ticks[i] - self.servo_min_ticks[i]
                position_ticks = self.servo_min_ticks[i] + (position_degree * ticks_span) / actuation_range_degree
                if (self.last_position_ticks[i] == position_ticks):
                    continue  # No change in position, skip
                self.last_position_ticks[i] = position_ticks
                servo_id = i + 1  # Servo IDs start from 1
                self.move_one_servo(servo_id, position_ticks)
        self.publish_ticks_message()

    def initialize_properties(self):
        self.index_cache = None
        self.joints_states_names = []
        self.q = [0.0] * self.joints_count # geometrical joint angle rads
        self.last_position_ticks = [-1] * self.joints_count # to track last sent position
        self.actuator_setpoint_degree = [0.0] * self.joints_count # servo motor angle degree
        for i in range(self.limbs_num):
            for j in range(self.joints_per_limb[i]):
                joint_name = f'limb{i}/joint{j}'
                self.joints_states_names.append(joint_name)

    def on_joint_states_callback(self, msg):
        pos_list = msg.position
        # Pre-map joint names to indices for O(1) lookup
        if self.index_cache is None:
            self.index_cache = []
            names = msg.name
            for i, name in enumerate(self.joints_states_names):
                if name in names:
                    index = names.index(name)
                    self.index_cache.append(index)
                else:
                    self.node.get_logger().error(f"Joint {name} not found in message.")
            return
        # Compute actuator setpoints (vectorized-like)
        deg_per_rad = 180.0 / math.pi
        for i in range(self.joints_count):
            self.q[i] = pos_list[self.index_cache[i]]
            servo_setpoint = self.dir[i] * (self.q[i] * deg_per_rad) + self.initial_joints_bias_degree[i]
            self.actuator_setpoint_degree[i] = self.clamp(
                servo_setpoint, i, 0.0, self.servo_actuation_range_degree[i]
            )
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

    def send_serial_command(self, servo_id, cmd, params=None):
        """Send a packet to LX-824 servo"""
        if params is None:
            params = []
        length = len(params) + 3  # Length = params + CMD + checksum + length itself
        packet = HEADER + [servo_id, length, cmd] + params
        checksum = (~(sum(packet[2:])) & 0xFF)
        packet.append(checksum)
        self.serial.write(bytearray(packet))
        time.sleep(0.005)

    def move_one_servo(self, servo_id, position_ticks):
        """
        Move LX-824 servo to position (0-1000).
        Default center ≈ 500.
        """
        time_ms = int(UPDATE_INTERVAL * 1000 * 0.8)  # time in ms
        position_ticks = int(position_ticks)
        pos_l = position_ticks & 0xFF
        pos_h = (position_ticks >> 8) & 0xFF
        time_l = time_ms & 0xFF
        time_h = (time_ms >> 8) & 0xFF
        self.send_serial_command(servo_id, 1, [pos_l, pos_h, time_l, time_h])

    def publish_actuators_setpoint(self):
        msg_setpoint = JointState()
        msg_setpoint.header.stamp = self.get_clock().now().to_msg()
        msg_setpoint.name = self.joints_states_names
        msg_setpoint.position = self.actuator_setpoint_degree
        self.setpoint_publisher_.publish(msg_setpoint)
    
    def publish_ticks_message(self):
        msg_ticks = JointState()
        msg_ticks.header.stamp = self.get_clock().now().to_msg()
        msg_ticks.name = self.joints_states_names
        msg_ticks.position = [float(tick) for tick in self.last_position_ticks]
        self.ticks_publisher_.publish(msg_ticks)

    def declare_params(self):
        # Declare robot geometry parameters
        self.declare_parameter('limbs_num', 5)  # Default value 5
        self.declare_parameter('joints_per_limb', [3]*5)  # Default value 3
        self.declare_parameter('i2c_actuators_params.actuator_angle_bias_at_joint_zero_degree', [0.0] * 15)  # Default bias
        self.declare_parameter('i2c_actuators_params.dir', [1.0] * 15)  # Default direction (1.0 for no inversion)
        self.declare_parameter('servo_parameters.servo_actuation_range_degree', [270.0] * 15) # angular range degree
        self.declare_parameter('servo_parameters.servo_min_ticks', [0] * 15) # ticks
        self.declare_parameter('servo_parameters.servo_max_ticks', [1000] * 15) # ticks

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
        self.initial_joints_bias_degree = self.get_parameter('i2c_actuators_params.actuator_angle_bias_at_joint_zero_degree').get_parameter_value().double_array_value
        self.dir = self.get_parameter('i2c_actuators_params.dir').get_parameter_value().double_array_value
        self.servo_actuation_range_degree = self.get_parameter('servo_parameters.servo_actuation_range_degree').get_parameter_value().double_array_value
        self.servo_min_ticks = self.get_parameter('servo_parameters.servo_min_ticks').get_parameter_value().integer_array_value
        self.servo_max_ticks = self.get_parameter('servo_parameters.servo_max_ticks').get_parameter_value().integer_array_value

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

        if len(self.servo_min_ticks) != self.joints_count:
            self.get_logger().error('ERROR: Servo minimum pulse width array size mismatch!')
        else: 
            self.get_logger().info(f'Servos minimum pulse width is loaded: {format_array_to_string(self.servo_min_ticks)}')

        if len(self.servo_max_ticks) != self.joints_count:
            self.get_logger().error('ERROR: Servo maximum pulse width array size mismatch!')
        else:
            self.get_logger().info(f'Servos maximum pulse width is loaded: {format_array_to_string(self.servo_max_ticks)}')


def main(args=None):
    rclpy.init(args=args)

    # Retrieve the mode from arguments
    mode = 'virtual'
    if len(sys.argv) > 1:
        mode = sys.argv[1]

    # Create the node with the mode passed
    node = PentaHiwonderActuators(mode=mode)

    # Keep the node alive to receive and process messages
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning Hiwonder servo control, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
