//
// Created by redwan on 8/28/21.
//

#include "roomba_control/display.h"

display::display(int index, const string &name, const string& world_frame) {

    points_.header.frame_id = world_frame;
    points_.header.stamp = ros::Time::now();
    points_.ns  = name + "_" + to_string(index);
    points_.action = visualization_msgs::Marker::ADD;
    points_.pose.orientation.w  = 1.0;
    points_.id = index;
    points_.type = visualization_msgs::Marker::POINTS;

    // points_ markers use x and y scale for width/height respectively
    points_.scale.x = 0.05;
    points_.scale.y = 0.05;

    // points_ are green
    points_.color.g = 0.50f;
    points_.color.a = 0.50;

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_roomba_action", 10);

}

void display::update(const vector<double> &goal, const vector<double> &robot) {
    publish_traj(goal, robot);
}

void display::publish_traj(const vector<double> &goal, const vector<double> &robot) {


    double x0, y0, x1, y1;
    x0 = robot[0];
    y0 = robot[1];

    x1 = goal[0];
    y1 = goal[1];
    // interpolate points in line
    double x, y;
    x = x0;
    points_.points.clear();

    double dx = x1 - x0;
    double dy = y1 - y0;

    double delta = 0.015;
    double threshold = 0.025;

    if (dx != 0.0 && dy != 0.0)
    {
        delta = (x1 > x0)? delta: - delta;
        while( abs(x - x1) >= threshold || abs(y -y1) >= threshold) {
            // avoid inf
            y = y0 + (x - x0) * dy / dx;

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.1;

            // update
            points_.points.push_back(p);
            x +=  delta;
        }
    }
    else if (dx == 0.0)
    {
        // increment y axis only
        delta = (y1 > y0)? delta: - delta;
        y = y0;
        while(abs(y -y1) >= threshold)
        {
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.1;

            // update
            points_.points.push_back(p);
            y += delta;
        }
    }
    else
    {
        // increment x-axis only
        delta = (x1 > x0)? delta: - delta;
        y = y0;
        while(abs(x -x1) >= threshold)
        {
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.1;

            // update
            points_.points.push_back(p);
            x += delta;
        }

    }

    ROS_INFO_STREAM("[display] points size " << points_.points.size());

    if (points_.points.size() > 0)
    {
        marker_pub_.publish(points_);

    }



}

void display::publish_robot_state() {

}
