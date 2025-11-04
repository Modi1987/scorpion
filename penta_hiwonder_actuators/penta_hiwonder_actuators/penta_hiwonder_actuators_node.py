import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import serial
import time
import math
import sys
from dataclasses import dataclass
from std_msgs.msg import Float32MultiArray

# Serial default settings, overridden from config.yaml
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_UPDATE_INTERVAL = 0.1  # seconds

# --- LX-824 Communication Constants ---
HEADER = [0x55, 0x55]

@dataclass
class TimeStamp:
    last_serial_update: float
    serial_update_interval: float
    serial_update_hz: float


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
                rclpy.shutdown() 
                sys.exit(1)

        # Publisher actuators setpoint from joint states
        self.setpoint_publisher_ = self.create_publisher(JointState, 'actuator_setpoint_degree', 1)
        self.ticks_publisher_ = self.create_publisher(JointState, 'actuator_ticks', 1)
        self.serial_hz_publisher_ = self.create_publisher(Float32MultiArray, "actuator_update_rate_hz", 1)
        sub_callback_group = MutuallyExclusiveCallbackGroup()
        self.joint_states_subscriber = self.create_subscription(
                JointState,
                "joint_states",
                lambda msg : self.on_joint_states_callback(msg),
                1,
                callback_group=sub_callback_group
            )
        timer_callback_group = ReentrantCallbackGroup()
        self.motor_update_timer = self.create_timer(self.update_interval_sec, self.update_motors_callback, callback_group=timer_callback_group)

    def open_serial_connection(self):
        try:
            port = self.serial_port
            baudrate = self.baudrate
            self.serial = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Serial port {port} opened successfully at {baudrate} baudrate.')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port {port}: {e}')
            return False

    def update_motors_callback(self):
        if self.real_mode_flag:
            # Actuate the motors
            for i in range(self.joints_count):
                position_degree = self.actuator_setpoint_degree[i]
                actuation_range_degree = self.servo_actuation_range_degree[i]
                position_ticks = self.servo_min_ticks[i] + float(position_degree * self.ticks_spans[i]) / actuation_range_degree
                position_ticks = int(position_ticks)
                if (self.last_position_ticks[i] == position_ticks):
                    continue  # No change in position, skip
                self.last_position_ticks[i] = position_ticks
                servo_id = i + 1  # Servo IDs start from 1
                self.move_one_servo(servo_id, position_ticks)
        self.publish_ticks_message()

    def initialize_properties(self):
        # Pre-compute conversion factors
        self.deg_per_rad = 180.0 / math.pi
        self.ticks_spans = [max_tick - min_tick for max_tick, min_tick in 
                           zip(self.servo_max_ticks, self.servo_min_ticks)]
        self.index_cache = None
        self.joints_states_names = []
        self.last_position_ticks = [-1] * self.joints_count # to track last sent position
        self.actuator_setpoint_degree = [0.0] * self.joints_count
        self.actuator_update_stamps = [TimeStamp(-1.0, -1.0, -1.0) for i in range(self.joints_count)]
        self.motor_update_hz = [0.0 for i in range(self.joints_count)]
        # messages
        self.motors_update_msg_hz = Float32MultiArray()
        self.actuator_msg_ticks = JointState()
        self.actuator_msg_setpoint = JointState()
        for i in range(self.joints_count):
            self.actuator_setpoint_degree[i] = self.initial_joints_bias_degree[i]
        for i in range(self.limbs_num):
            for j in range(self.joints_per_limb[i]):
                joint_name = f'limb{i}/joint{j}'
                self.joints_states_names.append(joint_name)

    def on_joint_states_callback(self, msg):
        pos_list = msg.position
        # Sanity checks
        if pos_list is None:
            self.get_logger().error('Received joints setpoint positions are None, ignoring!')
            return
        if len(pos_list) < self.joints_count:
            self.get_logger().error(
                f'Received joints setpoint positions size {len(pos_list)} is less than {self.joints_count}, ignoring!'
            )
            return
        # Pre-map joint names to indices for O(1) lookup
        if self.index_cache is None:
            self.index_cache = []
            names = msg.name
            for i, name in enumerate(self.joints_states_names):
                if name in names:
                    index = names.index(name)
                    self.index_cache.append(index)
                else:
                    self.get_logger().error(f"Joint {name} not found in message.")
            n = len(self.index_cache)
            if n < self.joints_count:
                self.get_logger().error(
                    f'Joints index cache size {n} is less than {self.joints_count}, clearing index cache!'
                )
                self.index_cache = None
            return
        # Compute actuator setpoints (vectorized-like)
        for i in range(self.joints_count):
            q_rad = pos_list[self.index_cache[i]] # geometrical joint angle rads
            setpoint_degree = self.dir[i] * (q_rad * self.deg_per_rad) + self.initial_joints_bias_degree[i]
            self.actuator_setpoint_degree[i] = self.clamp(
                setpoint_degree, i, 0.0, self.servo_actuation_range_degree[i]
            )
        # Publish actuators setpoint
        self.publish_actuators_setpoint()

    def clamp(self, value, index, min_val, max_val, margin=1.0):
        min_safe_limit = min_val + margin
        if value < min_safe_limit:
            self.get_logger().error(f'ERROR: Servo[{index}] calculated setpoint is {value} degrees, however its minimum permissible angle is {min_safe_limit}, clamping value to {min_safe_limit}!')
            return min_safe_limit
        max_safe_limit = max_val - margin
        if value > max_safe_limit:
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
        now  = time.time()
        self.serial.write(bytearray(packet))
        self.serial.flush()  # Ensure immediate transmission
        index = servo_id - 1
        if self.actuator_update_stamps[index].last_serial_update < 0.0:
            self.actuator_update_stamps[index].last_serial_update = now
        else:
            self.actuator_update_stamps[index].serial_update_interval = now - self.actuator_update_stamps[index].last_serial_update
            self.actuator_update_stamps[index].last_serial_update = now
            self.actuator_update_stamps[index].serial_update_hz = 1.0 / self.actuator_update_stamps[index].serial_update_interval
            self.motor_update_hz[index] = self.actuator_update_stamps[index].serial_update_hz
        time.sleep(0.005)

    def move_one_servo(self, servo_id, position_ticks):
        """
        Move LX-824 servo to position (0-1000).
        Default center ≈ 500.
        """
        ratio = self.actuation_time_ratio
        time_ms = int(self.update_interval_sec * 1000 * ratio)  # time in ms
        position_ticks = int(position_ticks)
        pos_l = position_ticks & 0xFF
        pos_h = (position_ticks >> 8) & 0xFF
        time_l = time_ms & 0xFF
        time_h = (time_ms >> 8) & 0xFF
        self.send_serial_command(servo_id, 1, [pos_l, pos_h, time_l, time_h])

    def publish_actuators_setpoint(self):
        self.actuator_msg_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.actuator_msg_setpoint.name = self.joints_states_names
        self.actuator_msg_setpoint.position = self.actuator_setpoint_degree
        self.setpoint_publisher_.publish(self.actuator_msg_setpoint)
    
    def publish_ticks_message(self):
        self.actuator_msg_ticks.header.stamp = self.get_clock().now().to_msg()
        self.actuator_msg_ticks.position = [float(tick) for tick in self.last_position_ticks]
        self.ticks_publisher_.publish(self.actuator_msg_ticks)
        self.motors_update_msg_hz.data = self.motor_update_hz
        self.serial_hz_publisher_.publish(self.motors_update_msg_hz)

    def declare_params(self):
        # Declare robot geometry parameters
        self.declare_parameter('limbs_num', 5)  # Default value 5
        self.declare_parameter('joints_per_limb', [3]*5)  # Default value 3
        # Hiwonder specific params
        self.declare_parameter('hiwonder.serial_port', DEFAULT_PORT)
        self.declare_parameter('hiwonder.baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('hiwonder.update_interval_sec', DEFAULT_UPDATE_INTERVAL)
        self.declare_parameter('hiwonder.actuator_angle_bias_at_joint_zero_degree', [0.0] * 15)  # Default bias
        self.declare_parameter('hiwonder.dir', [1.0] * 15)  # Default direction (1.0 for no inversion)
        self.declare_parameter('hiwonder.servo_actuation_range_degree', [270.0] * 15) # angular range degree
        self.declare_parameter('hiwonder.servo_min_ticks', [0] * 15) # ticks
        self.declare_parameter('hiwonder.servo_max_ticks', [1000] * 15) # ticks
        self.declare_parameter('hiwonder.actuation_time_ratio', 0.8)

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
        # Hiwonder specific params
        self.serial_port = self.get_parameter('hiwonder.serial_port').get_parameter_value().string_value
        self.get_logger().info(f'Hiwonder specified serial port: {self.serial_port}')
        self.baudrate = self.get_parameter('hiwonder.baudrate').get_parameter_value().integer_value
        self.get_logger().info(f'Hiwonder serial port baudrate is: {self.baudrate}')
        self.update_interval_sec = self.get_parameter('hiwonder.update_interval_sec').get_parameter_value().double_value
        self.get_logger().info(f'Hiwonder update interval [sec]: {self.update_interval_sec}')
        self.initial_joints_bias_degree = self.get_parameter('hiwonder.actuator_angle_bias_at_joint_zero_degree').get_parameter_value().double_array_value
        self.dir = self.get_parameter('hiwonder.dir').get_parameter_value().double_array_value
        self.servo_actuation_range_degree = self.get_parameter('hiwonder.servo_actuation_range_degree').get_parameter_value().double_array_value
        self.servo_min_ticks = self.get_parameter('hiwonder.servo_min_ticks').get_parameter_value().integer_array_value
        self.servo_max_ticks = self.get_parameter('hiwonder.servo_max_ticks').get_parameter_value().integer_array_value
        self.actuation_time_ratio = self.get_parameter('hiwonder.actuation_time_ratio').get_parameter_value().double_value
        self.get_logger().info(f'actuation_time_ratio specified is {self.actuation_time_ratio}')

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