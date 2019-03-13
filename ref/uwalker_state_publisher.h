#ifndef UWALKER_STATE_PUBLISHER_H_
#define UWALKER_STATE_PUBLISHER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <uwalker/LegJointsPositions.h>
#include <uwalker/LegsJointsPositions.h>

double TC_joint_0, TC_joint_1, TC_joint_2, TC_joint_3, TC_joint_4, TC_joint_5;
double CF_joint_0, CF_joint_1, CF_joint_2, CF_joint_3, CF_joint_4, CF_joint_5;
double FT_joint_0, FT_joint_1, FT_joint_2, FT_joint_3, FT_joint_4, FT_joint_5;

ros::Publisher jointStatePublisher;
ros::Subscriber legs_jointsPositionsSubscriber;
ros::Subscriber leg0_jointsPositionsSubscriber;
ros::Subscriber leg1_jointsPositionsSubscriber;
ros::Subscriber leg2_jointsPositionsSubscriber;
ros::Subscriber leg3_jointsPositionsSubscriber;
ros::Subscriber leg4_jointsPositionsSubscriber;
ros::Subscriber leg5_jointsPositionsSubscriber;

void initializeJoints(void);
void legs_jointsPositionsCallback(const uwalker::LegsJointsPositions::ConstPtr &msg);
void leg0_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg);
void leg1_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg);
void leg2_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg);
void leg3_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg);
void leg4_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg);
void leg5_jointsPositionsCallback(const uwalker::LegJointsPositions::ConstPtr &msg);

#endif
