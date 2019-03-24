#pragma once

#define VIZ_DEBUG

#include <memory>

#include <ros/ros.h>

#include "frame/Body.h"

#include "mildred_core/RemoteControlMessage.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

ros::Subscriber controlSubscriber;
//ros::Publisher legsJointsPositionsPublisher;
ros::Publisher targetMarkersPublisher;

std::unique_ptr<Mildred::Body> body;

int main(int argc, char **argv);
void controlMessageCallback(mildred_core::RemoteControlMessage controlMessage);