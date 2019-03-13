#ifndef JOINT_H_
#define JOINT_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;

namespace UWalker
{
	class Joint
	{
		public:
			Joint();
			virtual ~Joint();

			std::string name;
			double targetPosition;
			double currentPosition;

		protected:
		    ros::NodeHandle n;

		    ros::Subscriber jointStatesSubscriber;;
			void jointsStatesCallback(const JointStateConstPtr&  jointStatesMessage);
	};
}
#endif
