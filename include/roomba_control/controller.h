//
// Created by redwan on 8/26/21.
//

#ifndef ROOMBA_CONTROL_CONTROLLER_H
#define ROOMBA_CONTROL_CONTROLLER_H
#include "state_estimator.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "pid.h"
#include <numeric>
#include <math.h>

enum CONTROLLER_STATE{
    IDLE,
    RUNNING,
    FAILED
};

class controller {
public:
    controller(shared_ptr<state_estimator> localization, int control_fq);
    virtual ~controller();

private:
    shared_ptr<state_estimator> localization_;
    ros::Timer control_loop_;
    ros::NodeHandle nh_;
    ros::Subscriber rviz_goal_;
    ros::Publisher pub_;
    PID *position_controller_, *orientation_controller_;
    CONTROLLER_STATE cntrl_state_;
    vector<double> goal_state_;
    double threshold_;

protected:
    void publish_control_cmd(const ros::TimerEvent& event);
    void callback_rviz_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

};


#endif //ROOMBA_CONTROL_CONTROLLER_H
