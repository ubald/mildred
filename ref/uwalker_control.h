#ifndef UWALKER_CONTROL_H_
#define UWALKER_CONTROL_H_

#define VIZ_DEBUG

#include <ros/ros.h>

#include "uwalker/RemoteControl.h"
#include "frame/Body.h"

#include <uwalker/LegsJointsPositions.h>
#include <uwalker/LegJointsPositions.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

ros::Subscriber controlSubscriber;
ros::Publisher legsJointsPositionsPublisher;
ros::Publisher targetMarkersPublisher;

UWalker::Body * body;

int main(int argc, char **argv);
void controlMessageCallback(uwalker::RemoteControl controlMessage);

#endif
