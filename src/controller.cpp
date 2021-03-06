//
// Created by redwan on 8/26/21.
//

#include "roomba_control/controller.h"

controller::controller(shared_ptr<state_estimator> localization, int control_fq, ros::NodeHandle& nh):
localization_(localization), nh_(nh)
{
    double update_time = 1.0 / (double) control_fq;
    control_loop_ = nh_.createTimer(ros::Duration(update_time), &controller::publish_control_cmd, this);




    double pos_kp, pos_kd, pos_ki, pos_min, pos_max;
    nh_.getParam("position_controller/goal_tolerance", threshold_);
    nh_.getParam("position_controller/kp", pos_kp);
    nh_.getParam("position_controller/kd", pos_kd);
    nh_.getParam("position_controller/ki", pos_ki);
    nh_.getParam("position_controller/minVal", pos_min);
    nh_.getParam("position_controller/maxVal", pos_max);

    position_controller_ = new PID(update_time, pos_max, pos_min, pos_kp, pos_kd, pos_ki, threshold_);

    double ori_kp, ori_kd, ori_ki, ori_min, ori_max, thr;
    nh_.getParam("orientation_controller/goal_tolerance", thr);
    nh_.getParam("orientation_controller/kp", ori_kp);
    nh_.getParam("orientation_controller/kd", ori_kd);
    nh_.getParam("orientation_controller/ki", ori_ki);

    nh_.getParam("orientation_controller/minVal", ori_min);
    nh_.getParam("orientation_controller/maxVal", ori_max);

    orientation_controller_ = new PID(update_time, ori_max, ori_min, ori_kp, ori_kd, ori_ki, thr);




    cntrl_state_ = IDLE;
    string goal_topic, control_topic;
    nh_.getParam("control_topic_pub", control_topic);
    nh_.getParam("goal_topic_sub", goal_topic);
    pub_ = nh_.advertise<geometry_msgs::Twist>(control_topic, 10);
    sub_goal_ = nh_.subscribe(goal_topic, 1, &controller::callback_rviz_goal, this);
    goal_state_.resize(2);
    goal_state_[0] = goal_state_[1] = 0;

    // visualizer config
    string world_frame;
    int robot_index;
    nh_.getParam("robot_index", robot_index);
    nh_.getParam("world_frame", world_frame);
    viz_ = new display(robot_index, "roomba", world_frame);

}

void controller::publish_control_cmd(const ros::TimerEvent &event) {

    double cntrl_v, cntrl_w;
    cntrl_v = cntrl_w = 0;
    if(cntrl_state_ == RUNNING)
    {
        vector<double> state = localization_->get_state();
        double R_state, R_goal, theta_state, theta_goal;
        // given the co-ordinates of a point P (robot) and line segment (goal location and origin ),
        // and we have to determine the direction of point P from the line segment.
        // That is whether the Point lies to the Right of Line Segment or to the Left of Line Segment.
        // This Problem can be solved using cross-product of vector algebra
        // Assume that our origin at (0, 0)
        // https://www.geeksforgeeks.org/direction-point-line-segment/
        R_state = state[0] * goal_state_[1];
        R_goal = state[1] * goal_state_[0];
        
        if (R_state < R_goal)
        	swap(R_state,R_goal);
        
        theta_goal = -fmod((atan2(goal_state_[1] - state[1], goal_state_[0] - state[0]) - M_PI/2),(2 * M_PI))  ;
        theta_state = state[2];

        cntrl_v = position_controller_->calculate(R_goal, R_state);
        cntrl_w = orientation_controller_->calculate(theta_goal, theta_state);


        // update display
        viz_->update(goal_state_, state);
        if(abs(R_state - R_goal) < threshold_) // error +- 15 cm
        {
            ROS_INFO("[Controller] Target Reached !");
            cntrl_state_ = IDLE;
        }
        else{
            ROS_INFO_STREAM("[Controller] x " << R_state << " y " << R_goal << " theta " << theta_state << " theta_goal " << theta_goal);
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
