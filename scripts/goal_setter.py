# !/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
#import tf
from functools import partial
import tf2_ros


# from move_base.move_base_msgs import MoveBaseActionResult
rospy.init_node('goal_setter21', anonymous=True)
def state_estimator_d(tagid):
    world_frame = rospy.get_param('/roomba21/roomba_control21/world_frame')
    print("fixed fram name:", world_frame)
    rate = rospy.Rate(1.0)
    try:

     tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
     tf_listener = tf2_ros.TransformListener(tf_buffer)
     trans = tf_buffer.lookup_transform(world_frame,
                                       tagid,  # source frame
                                       rospy.Time(0),  # get the tf at first available time
                                       rospy.Duration(1.0))
    #print("trans",trans)
     return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
     rate.sleep()


def goal_setter(x, y, w):
    # rospy.init_node('goal_setter', anonymous=True)
    pub = rospy.Publisher('/roomba21/goal', PoseStamped, queue_size=10)
    # sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=10)
    rate = rospy.Rate(30)  # 10hz

    # while not rospy.is_shutdown():
    #rate.sleep()
    msg = PoseStamped()

    #msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = w
    # rospy.loginfo("x", msg.pose.position.x, **kwargs)
    # ROS_INFO_STREAM(msg.pose.position.x,msg.pose.position.y)
    # print(msg.pose.position.x,msg.pose.position.y)
    rospy.sleep(1)
    pub.publish(msg)
    #rate.sleep()
    #rospy.spin()

if __name__ == '__main__':

    rospy.init_node('goal_setter', anonymous=True)
    goaltag = "tag23"

    goal_state = partial(state_estimator_d, goaltag)
    pos_goal = goal_state()

    goal_x = pos_goal.transform.translation.x
    goal_y = pos_goal.transform.translation.y
    goal_w = pos_goal.transform.rotation.w

    try:
        print("g_x", goal_x, "g_y", goal_y, "g_w", goal_w)
        goal_setter(goal_x, goal_y, goal_w)
    except rospy.ROSInterruptException:
        pass



