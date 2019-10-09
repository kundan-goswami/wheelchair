#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from nav_msgs.msg import Odometry

class Mdu:

    def __init__(self):
        # Creates a node with name 'mdu_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('mdu_controller', anonymous=True)

        # Publisher which will publish to the topic 'md_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('odom',Odometry,self.update_pose)

        self.pose = Pose()
        self.theta = 0
        self.rate = rospy.Rate(500)
        self.odom = Odometry()

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.odom = data
        self.pose = data.pose.pose.position
 	self.theta = data.pose.pose.orientation.z
        #print(data.pose.pose)
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
	#print("Theta: " + str(self.theta))

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1):
        #print("Distance: " + str(self.euclidean_distance(goal_pose)))
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=1.5):
	#print("Angle: " + str(self.steering_angle(goal_pose)))
        return constant * (self.steering_angle(goal_pose) - self.theta)

    def vel_control(self):
        vel_msg = Twist()
        desired_lin_vel = 0.5
        desired_angu_vel = 0
        lin_vel_error   = 0
        angu_vel_error  = 0
        angu_vel_error_prev = 0
        lin_vel_error_intg = 0
        angu_vel_error_intg = 0
        lin_vel_Kp = 1.2
        lin_vel_Ki = 1.2
        angu_vel_Kp = 3.1
        angu_vel_Ki = 1.2
        angu_vel_Kd = 0.001
        while 1:
            lin_vel_error  = (desired_lin_vel - self.odom.twist.twist.linear.x)
            lin_vel_error_intg = lin_vel_error_intg + lin_vel_error
            angu_vel_error  = (desired_angu_vel - self.odom.twist.twist.angular.z)
            angu_vel_error_intg = angu_vel_error_intg +angu_vel_error
            diff_angu_vel_error = (angu_vel_error - angu_vel_error_prev)/(1.0/500)
            angu_vel_error_prev = angu_vel_error
            vel_msg.linear.x = desired_lin_vel#lin_vel_error*lin_vel_Kp + lin_vel_error_intg*lin_vel_Ki
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = angu_vel_error*angu_vel_Kp + angu_vel_error_intg* angu_vel_Ki + diff_angu_vel_error*angu_vel_Kd
            """
            if vel_msg.angular.z > 3:
                vel_msg.angular.z = 3
            elif vel_msg.angular.z < -3:
                vel_msg.angular.z = -3
            """
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        rospy.spin()

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = input("Set your x goal: ")
        goal_pose.y = input("Set your y goal: ")

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
	    #print("Linear Vel: " +  str(vel_msg.linear.x))
            #print("Angular Vel: " +  str(vel_msg.angular.z))
            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = Mdu()
        #x.move2goal()
        x.vel_control()
    except rospy.ROSInterruptException:
        exit()
