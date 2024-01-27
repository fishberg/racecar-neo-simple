#!/usr/bin/env python3

# ROS 2
try:
    import rclpy
    from sensor_msgs.msg import Joy
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
except:
    print('ROS is not installed')
else:
    print('ROS initialized successfully')

#from camera_simple import Camera
    
from datetime import datetime
import threading

class Racecar():    
    JOY_TOPIC = "/joy"
    DRIVE_TOPIC = "/drive"
    
    def __init__(self):
        try:
            rclpy.init(args=None)
            self.node = rclpy.create_node('racecar_simple')

            # Spin in a separate thread to allow sleeping and waking in run()
            # Reference:
            # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
            thread = threading.Thread(target=rclpy.spin, args=(self.node, ), daemon=True)
            thread.start()
        except:
            print('racecar not created. Was it previously created and not shutdown?')
        else:
            print('racecar created successfully')
        
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        self.sub_joy = self.node.create_subscription(Joy, self.JOY_TOPIC, self.joy_callback, qos_profile)
        self.pub_drive = self.node.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, qos_profile)

        self.last_drive = AckermannDriveStamped()
        self.rate = 15.  # Rate is in Hz

        self.a_pressed = False
        self.b_pressed = False
        self.x_pressed = False
        self.y_pressed = False

        #self.cam = Camera()
    
    def joy_callback(self, msg):
        if msg.buttons[0] == 1:  # Check if A was pressed
            self.a_pressed = True
        elif msg.buttons[1] == 1:  # Check if B was pressed
            self.b_pressed = True
        elif msg.buttons[2] == 1:  # Check if X was pressed
            self.x_pressed = True
        elif msg.buttons[3] == 1:  # Check if Y was pressed
            self.y_pressed = True

    def was_pressed(self, button):
        upper_button = button.upper()
        if upper_button == "A" and self.a_pressed:
            self.a_pressed = False
            ret = True
        elif upper_button == "B" and self.b_pressed:
            self.b_pressed = False
            ret = True
        elif upper_button == "X" and self.x_pressed:
            self.x_pressed = False
            ret = True
        elif upper_button == "Y" and self.y_pressed:
            self.y_pressed = False
            ret = True
        else:
            ret = False
#         print("Button check: " + upper_button + ", " + str(ret))
        return ret
        
    # speed and angle on range [-1,1]
    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(angle)
        self.last_drive = msg
    
    def stop(self):
        self.drive(0., 0.)
    
    """
    run
    - func: function to run each cycle
    - runtime (optional): run for runtime seconds and then stop
    """
    def run(self, func, runtime=-1):
        rate = self.node.create_rate(self.rate)  # Rate is in Hz
        self.now = datetime.now()

        # set steps_remaining
        steps_remaining = -1
        if runtime > 0:
            steps_remaining = runtime * self.rate

        print('run start')
        if runtime >= 0:
            print(f'runtime: {runtime} seconds')

        continue_running = True
        while rclpy.ok() and continue_running:
            # calculate delta_time
            self.later = datetime.now()
            self.delta_time = (self.later - self.now).total_seconds()
            self.now = datetime.now()

            # run func()
            func()

            # publish last_drive
            self.pub_drive.publish(self.last_drive)

            if steps_remaining > 0:
                steps_remaining -= 1
            elif (runtime > 0) and (steps_remaining <= 0):
                continue_running = False
                self.stop()
                self.pub_drive.publish(self.last_drive)

            rate.sleep()
        print('run complete')
    
    def get_delta_time(self):
#         print("delta time: " + str(self.delta_time))
        return self.delta_time
    
    def shutdown(self):
        self.node.destroy_node()

import sys
if __name__ == '__main__':
    rc = Racecar()
    rc.drive(1.,1.)
    def func():
        pass
    rc.run(func,runtime=2)
    sys.exit(0)
