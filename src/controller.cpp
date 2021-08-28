//
// Created by redwan on 8/26/21.
//

#include "roomba_control/controller.h"

controller::controller(shared_ptr<state_estimator> localization, int control_fq, ros::NodeHandle& nh):
localization_(localization), nh_(nh)
{
    double update_time = 1.0 / (double) control_fq;
    control_loop_ = nh_.createTimer(ros::Duration(update_time), &controller::publish_control_cmd, this);
    double pos_kp, pos_kd, pos_ki;
    nh_.getParam("position_controller/kp", pos_kp);
    nh_.getParam("position_controller/kd", pos_kd);
    nh_.getParam("position_controller/ki", pos_ki);

    position_controller_ = new PID(update_time, -1, 1, pos_kp, pos_kd, pos_ki);

    double ori_kp, ori_kd, ori_ki;
    nh_.getParam("orientation_controller/kp", ori_kp);
    nh_.getParam("orientation_controller/kd", ori_kd);
    nh_.getParam("orientation_controller/ki", ori_ki);

    orientation_controller_ = new PID(update_time, -1, 1, ori_kp, ori_kd, ori_ki);

    nh_.getParam("goal_tolerance", threshold_);

    cntrl_state_ = IDLE;

    pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    rviz_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &controller::callback_rviz_goal, this);
    goal_state_.resize(2);
    goal_state_[0] = goal_state_[1] = 0;

}

void controller::publish_control_cmd(const ros::TimerEvent &event) {

    double cntrl_v, cntrl_w;
    cntrl_v = cntrl_w = 0;
    if(cntrl_state_ == RUNNING)
    {
        vector<double> state = localization_->get_state();
        double R_state, R_goal, theta_state, theta_goal;

        R_state = sqrt(state[0]*state[0] + state[1]*state[1]);
        R_goal = sqrt(goal_state_[0]*goal_state_[0] + goal_state_[1]*goal_state_[1]);

        theta_goal = atan2(goal_state_[1] - state[1], goal_state_[0] - state[0]);
        theta_state = state[2];

        cntrl_v = position_controller_->calculate(R_goal, R_state);
        cntrl_w = orientation_controller_->calculate(theta_goal, theta_state);



        if(abs(R_state - R_goal) < threshold_) // error +- 15 cm
        {
            ROS_INFO("[Controller] Target Reached !");
            cntrl_state_ = IDLE;
        }
        else{
            ROS_INFO_STREAM("[Controller] x " << state[0] << " y " << state[1] << " theta " << state[2]);
            ROS_INFO("[Controller] (v = %lf, w = %lf)", cntrl_v, cntrl_w);
        }
    }

    geometry_msgs::Twist msg;
    msg.linear.x = cntrl_v;
    msg.angular.z = cntrl_w;
//    if(cntrl_w + cntrl_v)
//    ROS_INFO("v = %lf, w = %lf", cntrl_v, cntrl_w);

    pub_.publish(msg);

}

controller::~controller() {
    delete position_controller_;
    delete orientation_controller_;
}

void controller::callback_rviz_goal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO_STREAM("[Controller] Recived RVIZ GOAL " << *msg);
    goal_state_[0] = msg->pose.position.x;
    goal_state_[1] = msg->pose.position.y;

    cntrl_state_ = RUNNING;
}
