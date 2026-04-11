import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk

class JointSliderGUI(Node):
    def __init__(self):
        super().__init__('joint_slider_gui')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.sliders = []
        self.setup_gui()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("UR5e Joint Controller")
        self.root.geometry("500x400")

        for i, name in enumerate(self.joint_names):
            tk.Label(self.root, text=name, font=("Arial", 10)).pack()
            slider = tk.Scale(
                self.root,
                from_=-3.14,
                to=3.14,
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=400,
                command=lambda val, idx=i: self.send_trajectory()
            )
            slider.pack()
            self.sliders.append(slider)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [s.get() for s in self.sliders]
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [point]
        self.publisher.publish(msg)

    def on_close(self):
        self.root.destroy()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = JointSliderGUI()

if __name__ == '__main__':
    main()
