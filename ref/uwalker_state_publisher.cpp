#include "uwalker_state_publisher.h"

void initializeJoints(void)
{
    TC_joint_0 = 0;
    TC_joint_1 = 0;
    TC_joint_2 = 0;
    TC_joint_3 = 0;
    TC_joint_4 = 0;
    TC_joint_5 = 0;

    CF_joint_0 = 0;
    CF_joint_1 = 0;
    CF_joint_2 = 0;
    CF_joint_3 = 0;
    CF_joint_4 = 0;
    CF_joint_5 = 0;

    FT_joint_0 = 0;
    FT_joint_1 = 0;
    FT_joint_2 = 0;
    FT_joint_3 = 0;
    FT_joint_4 = 0;
    FT_joint_5 = 0;
}

void legs_jointsPositionsCallback(const uwalker::LegsJointsPositions::ConstPtr &msg)
{
    TC_joint_0 = msg->leg[0].joint[0];
    CF_joint_0 = msg->leg[0].joint[1];
    FT_joint_0 = msg->leg[0].joint[2];

    TC_joint_1 = msg->leg[1].joint[0];
    CF_joint_1 = msg->leg[1].joint[1];
    FT_joint_1 = msg->leg[1].joint[2];

    TC_joint_2 = msg->leg[2].joint[0];
    CF_joint_2 = msg->leg[2].joint[1];
    FT_joint_2 = msg->leg[2].joint[2];

    TC_joint_3 = msg->leg[3].joint[0];
    CF_joint_3 = msg->leg[3].joint[1];
    FT_joint_3 = msg->leg[3].joint[2];

    TC_joint_4 = msg->leg[4].joint[0];
    CF_joint_4 = msg->leg[4].joint[1];
    FT_joint_4 = msg->leg[4].joint[2];

    TC_joint_5 = msg->leg[5].joint[0];
    CF_joint_5 = msg->leg[5].joint[1];
    FT_joint_5 = msg->leg[5].joint[2];
}

void leg0_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg)
{
    TC_joint_0 = msg->joint[0];
    CF_joint_0 = msg->joint[1];
    FT_joint_0 = msg->joint[2];
}

void leg1_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg)
{
    TC_joint_1 = msg->joint[0];
    CF_joint_1 = msg->joint[1];
    FT_joint_1 = msg->joint[2];
}

void leg2_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg)
{
    TC_joint_2 = msg->joint[0];
    CF_joint_2 = msg->joint[1];
    FT_joint_2 = msg->joint[2];
}

void leg3_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg)
{
    TC_joint_3 = msg->joint[0];
    CF_joint_3 = msg->joint[1];
    FT_joint_3 = msg->joint[2];
}

void leg4_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg)
{
    TC_joint_4 = msg->joint[0];
    CF_joint_4 = msg->joint[1];
    FT_joint_4 = msg->joint[2];
}

void leg5_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg)
{
    TC_joint_5 = msg->joint[0];
    CF_joint_5 = msg->joint[1];
    FT_joint_5 = msg->joint[2];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwalker_joint_state_publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(150);

    jointStatePublisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    legs_jointsPositionsSubscriber = n.subscribe<uwalker::LegsJointsPositions>("legs_joints_positions", 1, legs_jointsPositionsCallback);
    leg0_jointsPositionsSubscriber = n.subscribe<uwalker::LegJointsPositions>("leg0_joints_positions", 1, leg0_jointsPositionsCallback);
    leg1_jointsPositionsSubscriber = n.subscribe<uwalker::LegJointsPositions>("leg1_joints_positions", 1, leg1_jointsPositionsCallback);
    leg2_jointsPositionsSubscriber = n.subscribe<uwalker::LegJointsPositions>("leg2_joints_positions", 1, leg2_jointsPositionsCallback);
    leg3_jointsPositionsSubscriber = n.subscribe<uwalker::LegJointsPositions>("leg3_joints_positions", 1, leg3_jointsPositionsCallback);
    leg4_jointsPositionsSubscriber = n.subscribe<uwalker::LegJointsPositions>("leg4_joints_positions", 1, leg4_jointsPositionsCallback);
    leg5_jointsPositionsSubscriber = n.subscribe<uwalker::LegJointsPositions>("leg5_joints_positions", 1, leg5_jointsPositionsCallback);

    initializeJoints();

    sensor_msgs::JointState joint_state;

    while (ros::ok())
    {
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(18);
        joint_state.position.resize(18);

        joint_state.name[0] = "TC_joint_0";
        joint_state.position[0] = TC_joint_0;
        joint_state.name[1] = "TC_joint_1";
        joint_state.position[1] = TC_joint_1;
        joint_state.name[2] = "TC_joint_2";
        joint_state.position[2] = TC_joint_2;
        joint_state.name[3] = "TC_joint_3";
        joint_state.position[3] = TC_joint_3;
        joint_state.name[4] = "TC_joint_4";
        joint_state.position[4] = TC_joint_4;
        joint_state.name[5] = "TC_joint_5";
        joint_state.position[5] = TC_joint_5;

        joint_state.name[6] = "CF_joint_0";
        joint_state.position[6] = CF_joint_0;
        joint_state.name[7] = "CF_joint_1";
        joint_state.position[7] = CF_joint_1;
        joint_state.name[8] = "CF_joint_2";
        joint_state.position[8] = CF_joint_2;
        joint_state.name[9] = "CF_joint_3";
        joint_state.position[9] = CF_joint_3;
        joint_state.name[10] = "CF_joint_4";
        joint_state.position[10] = CF_joint_4;
        joint_state.name[11] = "CF_joint_5";
        joint_state.position[11] = CF_joint_5;

        joint_state.name[12] = "FT_joint_0";
        joint_state.position[12] = FT_joint_0;
        joint_state.name[13] = "FT_joint_1";
        joint_state.position[13] = FT_joint_1;
        joint_state.name[14] = "FT_joint_2";
        joint_state.position[14] = FT_joint_2;
        joint_state.name[15] = "FT_joint_3";
        joint_state.position[15] = FT_joint_3;
        joint_state.name[16] = "FT_joint_4";
        joint_state.position[16] = FT_joint_4;
        joint_state.name[17] = "FT_joint_5";
        joint_state.position[17] = FT_joint_5;

        jointStatePublisher.publish(joint_state);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
