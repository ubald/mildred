#ifndef BODY_H_
#define BODY_H_

#include <ros/ros.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include "../gait/gaits.h"

#include "Leg.h"

namespace UWalker
{
	class Body
	{
		public:
			Body();
			virtual ~Body();

			static const unsigned int LEG_COUNT = 6;
			UWalker::Leg * leg[ LEG_COUNT ];

			bool setup( urdf::Model model, std::string root, std::string leg_tip_prefix );
			void setGait( UWalker::EGaitShape shape, UWalker::EGaitSequence sequence );

			void tick();

			KDL::Rotation rotation;
			KDL::Frame frame;
			KDL::Vector2 velocity;

		protected:
			ros::NodeHandle n;
			KDL::Tree tree;

			UWalker::Gait * gait;

			double speed;
			double direction;

	};
}
#endif
