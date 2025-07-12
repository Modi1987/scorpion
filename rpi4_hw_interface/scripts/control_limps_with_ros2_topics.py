#! /usr/bin/env python3
import rclpy
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk

class PentapodControlApp:
    def __init__(self, root, node):
        self.node = node
        self.topic_name = 'forward_position_controller/commands'
        self.publisher = self.node.create_publisher(Float64MultiArray, self.topic_name, 10)

        self.root = root
        self.root.title("PCA9568 motor control GUI")
        self.root.geometry("700x1000")

        self.slider_objects = []
        self.slider_value_labels = []
        self.slider_values = [135] * 16  # Initialize with 135 degrees
        self.number_of_channels_to_command = 16

        self.enable_publishing = False # disable ros2 publishing until the the GUI is ready

        number_of_motors_frame = ttk.LabelFrame(self.root, text=f"Number of channels to control on PCA9685, a number from 1 to 16")
        number_of_motors_frame.pack(padx=10, pady=10, fill="both", expand=True)
        self.number_of_motors_input = ttk.Entry(number_of_motors_frame)
        self.number_of_motors_input.pack(side="left", padx=5)
        self.number_of_motors_input.insert(0, str(15))

        for leg in range(5):
            leg_frame = ttk.LabelFrame(self.root, text=f"Leg {leg + 1}")
            leg_frame.pack(padx=10, pady=10, fill="both", expand=True)
            
            for motor in range(3):
                index = leg * 3 + motor
                frame = ttk.Frame(leg_frame)
                frame.pack(pady=5)

                label = ttk.Label(frame, text=f"Motor {index + 1}")
                label.pack(side="left", padx=5)

                minus_button = ttk.Button(frame, text="-", command=lambda idx=index: self.update_slider_values(self.slider_values[idx] - 0.5, idx))
                minus_button.pack(side="left", padx=5)

                slider = ttk.Scale(frame, from_=0, to=270, orient="horizontal", command=lambda value, idx=index: self.update_slider_values(value, idx))
                slider.pack(side="left", padx=5)

                plus_button = ttk.Button(frame, text="+", command=lambda idx=index: self.update_slider_values(self.slider_values[idx] + 0.5, idx))
                plus_button.pack(side="left", padx=5)

                self.slider_objects.append(slider)

                self.slider_value_labels.append(ttk.Label(frame, text=f"135"))
                self.slider_value_labels[-1].pack(side="left", padx=5)

        self.positions_label = ttk.Label(self.root, text="Slider Values: " + str(self.slider_values))
        self.positions_label.pack(pady=10)

        self.update_button = ttk.Button(self.root, text="Update Values to Wardware", command=self.flush_values_to_ros)
        self.update_button.pack(pady=10)

        for slider in self.slider_objects:
            slider.set(135)  # Set initial value to 135
        self.enable_publishing = True

    def update_slider_values(self, value, index):
        self.slider_values[index] = float(value)
        self.slider_value_labels[index].config(text=str(self.slider_values[index]))
        self.positions_label.config(text="Slider Values: " + str(self.slider_values))
        if self.enable_publishing:
            self.flush_values_to_ros()

    def flush_values_to_ros(self):
        self.number_of_channels_to_command = int(self.number_of_motors_input.get())
        print("Current Slider Values:", self.slider_values)
        # actuate the motors
        msg = Float64MultiArray()
        n = self.number_of_channels_to_command
        msg.data = [float(self.slider_values[i]) for i in range(n)]
        self.publisher.publish(msg)
        print(f"Published to {self.topic_name}: {msg.data}")

if __name__ == "__main__":
    rclpy.init()
    # Create a ROS2 node
    client_node = rclpy.create_node('manual_control_node')
    root = tk.Tk()
    app = PentapodControlApp(root, client_node)
    root.mainloop()
    rclpy.shutdown()
    client_node.destroy_node()
    print("Node shutdown and GUI closed.")
