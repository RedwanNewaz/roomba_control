#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pycreate2

def callback(data):
    rospy.loginfo(data)
    v,w = data.linear.x, data.angular.z
    R = 0.072
    L = 0.235
    left,right = uniToDiff(v,w,L,R)
    #left = scale(left)
    left =left*100
    right = right*100
    #right = scale(right)

    #bot.drive_direct(int(left), int(right))

def pushback (data)
    ros.loginfo(data)
    ## 

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
    # port = "COM3"
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    bot = pycreate2.Create2(port=port, baud=baud['default'])
    bot.start()
    bot.safe()
    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()



