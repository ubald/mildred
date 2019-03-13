#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>
#include <pr2_mechanism_model/joint.h>
#include <controller_interface/controller.h>
#include <uwalker/LegsJointsPositions.h>

namespace UWalker
{
    class UWalkerController: public controller_interface::Controller
    {
        private:

        protected:

            /**
             * Robot Pointer
             */
            pr2_mechanism_model::RobotState * robot;

            /**
             * ROS Node Handle
             */
            ros::NodeHandle n;

            /**
             * Legs Joints Positions Topic Subscriber
             * @see UWalker::UWalkerControl
             */
            ros::Subscriber legs_jointsPositionsSubscriber;

            /**
             * Legs Joints Positions Subscriber Callback
             * @param msg
             */
            void legs_jointsPositionsCallback(const uwalker::LegsJointsPositions::ConstPtr &msg);

            /**
             * Joint State Vector
             */
            std::vector<pr2_mechanism_model::JointState*> joints;

            /**
             * Joints target positions
             *
             * Values come from the Legs Joints Positions Subscriber Callback.
             *
             * TODO: Define joint count somewhere
             */
            std::vector<double> targetPositions;

            /**
             * PID Controllers for each joitns
             *
             * TODO: Define joint count somewhere
             */
            std::vector<control_toolbox::Pid*> pidController;

            /**
             * Last controller run time
             */
            ros::Time lastTime;

        public:
            virtual bool init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n );
            virtual void starting();
            virtual void update();
            virtual void stopping();
    };
}
