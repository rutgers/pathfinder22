from ssl import OP_ENABLE_MIDDLEBOX_COMPAT

from numpy import False_
import rospy
from nav_msgs.msg import Odometry
from math import atan2
import std_msgs.msg 

from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import tf
import utm
import roslaunch
from math import radians
x=0.0
y = 0.0
theta = 0.0

listener = None
def send_move_base_goal(posx, posy, quat, id):
    global listener
    pub_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size =1, latch =True)
    
    global last_goal_id
    ag = MoveBaseActionGoal()
    pose = PoseStamped()
    pose.header.frame_id = "utm"
    pose.pose.position.x = posx
    pose.pose.position.y = posy
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    map_pose = listener.transformPose("map", pose)
    ag.goal = MoveBaseGoal()   
    ag.header = std_msgs.msg.Header()
    ag.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    ag.header.frame_id = "map"
    ag.goal_id = GoalID()

    ag.goal_id.id = str(id)
    ag.goal_id.stamp = ag.header.stamp
    ag.goal.target_pose.pose.position.x = map_pose.pose.position.x
    ag.goal.target_pose.pose.position.y = map_pose.pose.position.y
    ag.goal.target_pose.header = ag.header
    ag.goal.target_pose.pose.orientation.x = map_pose.pose.orientation.x
    ag.goal.target_pose.pose.orientation.y = map_pose.pose.orientation.y
    ag.goal.target_pose.pose.orientation.z = map_pose.pose.orientation.z
    ag.goal.target_pose.pose.orientation.w = map_pose.pose.orientation.w

    
    pub_goal.publish(ag)


  
if __name__ == "__main__":
    rospy.init_node("robotcontroller")
    listener = tf.TransformListener()

    # message = rospy.wait_for_message("/mavros/global_position/global", NavSatFix)
    # compass = rospy.wait_for_message("/mavros/global_position/compass_hdg", std_msgs.msg.Float64)
    # print(compass.data)
    # # quat = quaternion_from_euler(0, 0, radians(float(compass.data)))
    # (x, y, zone, zone_num) = utm.from_latlon(message.latitude, message.longitude)
    # print(f"GPS_Recieved: {x}, {y}, {float(compass.data)}")
    # package = 'tf'
    # executable = 'static_transform_publisher'
    
    # args = f"{x} {y} {0} {radians(float(compass.data))} {0} {0} world map 0" 
    # node = roslaunch.core.Node(package, executable, name="GPS_Origin", args=args)

    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()

    # process = launch.launch(node)
    rospy.sleep(5)

    f = open("points.txt")
    lines = f.readlines()
    id = 0
    
    for l in lines:
        print(l)
        m = l.split(" ",7)
        coordsArray = [float(numeric_string) for numeric_string in m]
        send_move_base_goal(coordsArray[0], coordsArray[1], coordsArray[3:], id)

        print(coordsArray)
        checker = True
        
        while(checker):
            statusList = rospy.wait_for_message("move_base/status", GoalStatusArray)
            for g in statusList.status_list:
                if g.goal_id.id == str(id) and g.status == 3:
                    checker = False
                    break
            rospy.sleep(1)

        id = id+1 
    rospy.spin()

