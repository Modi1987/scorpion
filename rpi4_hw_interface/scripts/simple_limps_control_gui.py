import tkinter as tk
from tkinter import ttk

from adafruit_servokit import ServoKit

class PentapodControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Pentapod Control")

        self.sliders = []
        self.slider_values = [135] * 15  # Initialize with 135 degrees

        self.kit = ServoKit(channels=16) # 16 channel board is used
        # From data-sheets:
        # The pulse width for DSS-M15S is:
        # - Pulse range: 500-2500 us
        # - Median signal value: 1500us
        # The range is 270 degrees
        for i in range(15):
            self.kit.servo[i].set_pulse_width_range(500, 2500) 
            self.kit.servo[i].actuation_range = 270

        for leg in range(5):
            leg_frame = ttk.LabelFrame(self.root, text=f"Leg {leg + 1}")
            leg_frame.pack(padx=10, pady=10, fill="both", expand=True)
            
            for motor in range(3):
                index = leg * 3 + motor
                frame = ttk.Frame(leg_frame)
                frame.pack(pady=5)

                label = ttk.Label(frame, text=f"Motor {index + 1}")
                label.pack(side="left", padx=5)

                slider = ttk.Scale(frame, from_=0, to=270, orient="horizontal", command=lambda value, idx=index: self.update_values(value, idx))
                slider.pack(side="left", padx=5)
                self.sliders.append(slider)

        self.value_label = ttk.Label(self.root, text="Slider Values: " + str(self.slider_values))
        self.value_label.pack(pady=10)

        self.update_button = ttk.Button(self.root, text="Update Values to Wardware", command=self.display_values)
        self.update_button.pack(pady=10)

        for slider in self.sliders:
            slider.set(135)  # Set initial value to 135

    def update_values(self, value, index):
        self.slider_values[index] = int(float(value))
        self.value_label.config(text="Slider Values: " + str(self.slider_values))

    def display_values(self):
        print("Current Slider Values:", self.slider_values)
        # actuate the motors
        for i in range(15):
            self.kit.servo[i].angle  = self.slider_values[i]

if __name__ == "__main__":
    root = tk.Tk()
    app = PentapodControlApp(root)
    root.mainloop()
