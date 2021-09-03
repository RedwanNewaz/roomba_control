//
// Created by redwan on 8/28/21.
//

#include "roomba_control/display.h"

display::display(int index, const string &name, const string& world_frame) {

    line_.header.frame_id = world_frame;
    line_.header.stamp = ros::Time::now();
    line_.ns  = name + "_" + to_string(index);
    line_.action = visualization_msgs::Marker::ADD;
    line_.pose.orientation.w  = 1.0;
    line_.id = index;
    line_.type = visualization_msgs::Marker::LINE_STRIP;

    // line_ markers use x and y scale for width/height respectively
    line_.scale.x = 0.05;
    line_.scale.y = 0.05;

    // line_ are green
    line_.color.g = 0.50f;
    line_.color.a = 0.50;

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_roomba_action", 10);

}

void display::update(const vector<double> &goal, const vector<double> &robot) {
    publish_traj(goal, robot);
}

void display::publish_traj(const vector<double> &goal, const vector<double> &robot) {

    // replaced point trajectory with line strip

    geometry_msgs::Point p, q;
    p.x = robot[0];
    p.y = robot[1];

    q.x = goal[0];
    q.y = goal[1];

    p.z = q.z = 0.1;

    line_.points.clear();
    line_.points.push_back(p);
    line_.points.push_back(q);
    marker_pub_.publish(line_);


}

void display::publish_robot_state() {

}
