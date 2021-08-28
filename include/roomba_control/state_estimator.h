//
// Created by redwan on 8/26/21.
//

#ifndef ROOMBA_CONTROL_STATE_ESTIMATOR_H
#define ROOMBA_CONTROL_STATE_ESTIMATOR_H
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <vector>
#define STATE_DIM (3)

using namespace std;

class state_estimator {
public:
    state_estimator(const string& world_frame, const string& robot_frame, int update_fq);
    virtual ~state_estimator();
    vector<double> get_state();
private:
    string world_frame_, robot_frame_;
    ros::NodeHandle nh_;
    ros::Timer update_loop_;
    tf2_ros::TransformListener *tfListener_;
    tf2_ros::Buffer *tfBuffer_;
    vector<double> state_;


protected:
    void update_timer_callback(const ros::TimerEvent& event);
};


#endif //ROOMBA_CONTROL_STATE_ESTIMATOR_H
