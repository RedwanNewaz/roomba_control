//
// Created by redwan on 8/26/21.
//

#include "roomba_control/state_estimator.h"

state_estimator::state_estimator(const string &world_frame, const string &robot_frame, int update_fq, ros::NodeHandle& nh):
world_frame_(world_frame), robot_frame_(robot_frame), nh_(nh)
{
    // initialize state
    state_.resize(STATE_DIM);
    state_[0] = state_[1] = state_[2] = 0;
    // start update timer
    double update_time = 1.0 / (double) update_fq;
    update_loop_ = nh_.createTimer(ros::Duration(update_time), &state_estimator::update_timer_callback, this);
    tfBuffer_ = new tf2_ros::Buffer();
    tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);
}

state_estimator::~state_estimator() {
    delete tfListener_;
    delete tfBuffer_;

}

void state_estimator::update_timer_callback(const ros::TimerEvent &event) {
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer_->lookupTransform(world_frame_, robot_frame_,
                                                    ros::Time(0));

        tf2::Quaternion q;
        q.setX(transformStamped.transform.rotation.x);
        q.setY(transformStamped.transform.rotation.y);
        q.setZ(transformStamped.transform.rotation.z);
        q.setW(transformStamped.transform.rotation.w);
        // update state vector
        state_[0] = transformStamped.transform.translation.x;
        state_[1] = transformStamped.transform.translation.y;
        state_[2] = q.getAngle();

//        ROS_INFO_STREAM("x " << state_[0] << " y " << state_[1] << " theta " << state_[2]);
    }
    catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
    }
}



vector<double> state_estimator::get_state() {
    return state_;
}
