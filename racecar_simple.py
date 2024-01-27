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

from camera_simple import Camera
    
from datetime import datetime
import threading

class Racecar():    
    JOY_TOPIC = "/joy"
    DRIVE_TOPIC = "/drive"
    
    def __init__(self):
        try:
            rclpy.init(args=None)
            self.node = rclpy.create_node('racecar')
            # Spin in a separate thread to allow sleeping and waking in run()
            thread = threading.Thread(target=rclpy.spin, args=(self.node, ), daemon=True)
            thread.start()
        except:
            print('racecar not created. Was it previously created and not shutdown?')
        else:
            print('racecar created successfully')
        
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        
        self.sub_jou = self.node.create_subscription(Joy, self.JOY_TOPIC, self.joy_callback, qos_profile)
        self.pub_drive = self.node.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, qos_profile)
        self.last_drive = AckermannDriveStamped()
        self.rate = 15  # Rate is in Hz
        self.a_pressed = False
        self.b_pressed = False
        self.x_pressed = False
        self.y_pressed = False
        self.cam = Camera()
    
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
        
    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.last_drive = msg
    
    def stop(self):
        self.drive(0., 0.)
    
    def run(self, func, runtime=0):
        r = self.node.create_rate(self.rate)  # Rate is in Hz
        keeprunning = True
        runduration = -1
        self.now = datetime.now()
        if runtime > 0:
            runduration = runtime * self.rate
        while rclpy.ok() and keeprunning:
            self.later = datetime.now()
            self.delta_time = (self.later - self.now).total_seconds()
            self.now = datetime.now()
            func()
            self.pub_drive.publish(self.last_drive)
            if runduration > 0:
                runduration -= 1
            elif runduration <= 0 and runtime > 0:
                keeprunning = False
                self.stop()
                self.pub_drive.publish(self.last_drive)
            r.sleep()
    
    def get_delta_time(self):
#         print("delta time: " + str(self.delta_time))
        return self.delta_time
    
    def shutdown(self):
        self.node.destroy_node()
