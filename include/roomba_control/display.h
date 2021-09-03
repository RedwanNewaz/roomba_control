//
// Created by redwan on 8/28/21.
//

#ifndef ROOMBA_CONTROL_DISPLAY_H
#define ROOMBA_CONTROL_DISPLAY_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

class display {
public:
    display(int index, const string& name, const string& world_frame);
    void update(const vector<double>& goal, const vector<double>& robot);

private:
    vector<double> goal_, robot_;
    int robot_index_;
    string robot_name_;
    visualization_msgs::Marker line_;
    ros::Publisher marker_pub_;
    ros::NodeHandle nh_;


protected:
    void publish_traj(const vector<double> &goal, const vector<double> &robot);
    void publish_robot_state();

};


#endif //ROOMBA_CONTROL_DISPLAY_H
