//
// Created by redwan on 8/26/21.
//

#include "roomba_control/state_estimator.h"
#include "roomba_control/controller.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "roomba_controller");
    ros::NodeHandle nh("~");
    string world_frame, robot_frame;
    int state_update_fq;
    nh.getParam("world_frame", world_frame);
    nh.getParam("robot_frame", robot_frame);
    nh.getParam("state_update_fq", state_update_fq);

    ROS_INFO_STREAM("[Roomba] (world frame, robot frame): = " << world_frame << ", "<< robot_frame);

    std::shared_ptr<state_estimator>state = std::make_shared<state_estimator>(world_frame, robot_frame, state_update_fq, nh);
    controller Controller(state, state_update_fq, nh);
    ros::spin();



    return 0;
}