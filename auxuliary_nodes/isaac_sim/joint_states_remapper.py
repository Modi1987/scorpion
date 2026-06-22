import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Remapper:
    def __init__(self, node: Node):
        self.node = node
        self.msg = self.init_msg()

        self.sub = self.node.create_subscription(
            JointState,
            "/joint_setpoints",
            self.sub_callback,
            10,
        )
        self.pub = self.node.create_publisher(
            JointState,
            "/isaac_joint_setpoints",
            10,
        )

    def init_msg(self) -> JointState:
        msg = JointState()
        msg.name = []

        for i in range(5):
            for j in range(3):
                msg.name.append(f"tn__limb{i}joint{j}_gH")

        return msg

    def sub_callback(self, msg: JointState) -> None:
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.name = self.msg.name
        out_msg.position = list(msg.position)
        out_msg.velocity = list(msg.velocity)
        out_msg.effort = list(msg.effort)

        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = Node("isaac_sim_remapper")
    Remapper(node)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()