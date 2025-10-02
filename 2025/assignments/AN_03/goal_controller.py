#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Point
from move_base_msgs.msg import MoveBaseActionResult


##
## Exercise
## You need to fill the TODO parts to implement your logic
##

class GoalNavigation:
    def __init__(self):
        rospy.init_node('goal_controller', anonymous=True)
        rospy.loginfo("Goal controller has started")

        # Do some cleanup on shutdown
        rospy.on_shutdown(self.clean_shutdown)

        # Publisher to goal commands
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Subscriber to goal result information (receives once a message when the robot arrived to its destination goal)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)

        # Subscriber to current position information
        self.position_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.position_callback)

        # Initialize Pose message for goal commands    
        # This message will be used to send position (x,y,z) and orientation (quaternion) to the robot
        self.goal = PoseStamped()

        # pre-define the two goals
        self.goal1 = PoseStamped()
        self.goal1.header.frame_id = 'map'
        self.goal1.pose.position.x = 2.0
        self.goal1.pose.orientation.z = 1.0

        self.goal2 = PoseStamped()
        self.goal2.header.frame_id = 'map'
        self.goal2.pose.position.x = -2.0
        self.goal2.pose.orientation.z = 1.0


        self.goal_status = -1
        self.euclidean_distance = -1

        # flag to send once initial goal
        self.goaltimer = 20

        # Publisher to send velocity commands
        # used to stop the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # callback that handles the main robot logic every 0.1 second 
        self.timer = rospy.Timer(rospy.Duration(0.1), self.handle_robot_logic)

        # Main loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

    # Callback function to process goal result data
    def result_callback(self, msg):
        #rospy.loginfo("Goal result %s", msg.status.status)     
        self.goal_status = msg.status.status
        if self.goal_status == 3:
            rospy.loginfo("Goal reached successfully")
        elif self.goal_status == 4:
            rospy.loginfo("Goal was aborted by the action server")
        elif self.goal_status == 5:
            rospy.loginfo("Goal has been rejected by the action server")
        elif self.goal_status == 2:
            rospy.loginfo("Goal is being processed")
        elif self.goal_status == 1:
            rospy.loginfo("Goal received, but not yet processed")
        elif self.goal_status == 0:
            rospy.loginfo("Goal status is pending")


    # Callback function to process robot position data
    # This function will be called whenever a new amcl_pose message is received
    def position_callback(self, msg):

        rospy.loginfo(msg.pose.pose)
        
        # TODO 2 Calculate the euclidean distance to the goal 
        # and use this information to switch goal whenever you are within 50cm of the goal
        # Hint: you can use the self.goal variable to access the current goal position

        rospy.loginfo("distance to goal %f",self.euclidean_distance)          

    def gotogoal(self,goal):
        self.goal_status = -1  # reset goal status
        self.goal = goal
        self.goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(self.goal)
        rospy.loginfo("New goal is sent to the robot:")
        rospy.loginfo(self.goal)

    #This function callback handles the main robot logic every 0.1 second
    def handle_robot_logic(self, timer_event):
        
        # Issue a goal command after 2 seconds
        if self.goaltimer > 0:
            self.goaltimer -= 1
            if self.goaltimer == 0:
                self.gotogoal(self.goal1)
        
        # TODO 1 Check if the robot has reached its goal and is ready to issue a new goal
        # Hint: use a timer to wait a few seconds before sending the command    

    # Main loop. spin is blocking and only allows to processes callbacks
    def run(self):

        while not rospy.is_shutdown():
            rospy.spin()

    def clean_shutdown(self):
        rospy.loginfo("Goal controller is shutting down.")
        self.twist = Twist()

        # Send motion commands to motors
        self.cmd_vel_pub.publish(self.twist)

        rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    try:
        controller = GoalNavigation()
        controller.run()
    except rospy.ROSInterruptException:
        pass



