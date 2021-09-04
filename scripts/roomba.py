#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import pycreate2

def callback(data,bot):
    rospy.loginfo(data)
    v,w = data.linear.x, data.angular.z
    R = 0.072
    L = 0.235
    #while not rospy.is_shutdown():
    left,right = uniToDiff(v,w,L,R)
    print ("left:",left, "right:",right)
    bot.drive_direct(int(left), int(right))



def uniToDiff(v, w,length,radius):
    '''
    unicycle to differential-drive conversion
    :param v: linear velocity
    :param w: angular velocity
    :return: left and right wheels velocities
    '''
    vR = (2 * v + w * length) / (2 * radius)
    vL = (2 * v - w * length) / (2 * radius)
    return vR, vL

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    '''
    Robot serial connection
    '''
    port = '/dev/ttyUSB0'
    baud = {
    'default': 115200,
    'alt': 19200  # shouldn't need this unless you accidentally set it to this
       }

    bot = pycreate2.Create2(port=port, baud=baud['default'])
    bot.start()
    bot.safe()

    rospy.init_node('listener', anonymous=True)
    # http://wiki.ros.org/rospy/Overview/Parameter%20Server
    control_topic = rospy.get_param('~control_topic_pub')

    rospy.Subscriber(control_topic, Twist, callback,bot)
    rospy.spin()


if __name__ == '__main__':
    listener()



