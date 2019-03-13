#include "uwalker/controllers/uwalker_controller.h"

namespace UWalker
{
    bool UWalkerController::init( pr2_mechanism_model::RobotState *_robot, ros::NodeHandle &_n )
    {
        assert( _robot );
        robot = _robot;
        n = _n;

        // Gets all of the joints

        XmlRpc::XmlRpcValue jointNames;

        if ( !n.getParam( "joints", jointNames ) )
        {
            ROS_ERROR( "No joints given. (namespace: %s)", n.getNamespace().c_str() );
            return false;
        }

        if ( jointNames.getType() != XmlRpc::XmlRpcValue::TypeArray )
        {
            ROS_ERROR( "Malformed joint specification. (namespace: %s)", n.getNamespace().c_str() );
            return false;
        }

        for ( int i = 0; i < jointNames.size(); i++ )
        {
            XmlRpc::XmlRpcValue &jointName = jointNames[i];
            if ( jointName.getType() != XmlRpc::XmlRpcValue::TypeString )
            {
                ROS_ERROR( "Array of joint names should contain all strings. (namespace: %s)", n.getNamespace().c_str() );
                return false;
            }

            pr2_mechanism_model::JointState *jointState = robot->getJointState( (std::string) jointName );
            if ( !jointState )
            {
                ROS_ERROR( "Joint not found: %s. (namespace: %s)", ((std::string)jointName).c_str(), n.getNamespace().c_str() );
                return false;
            }
            joints.push_back( jointState );

            //Create PID Controller
            control_toolbox::Pid *pid = new control_toolbox::Pid();
            if ( !pid->init( ros::NodeHandle( n, "pid_parameters" ) ) )
            {
                ROS_ERROR( "Could not initialize PID controller for joint '%s'", ((std::string)jointName).c_str() );
                return false;
            }
            pidController.push_back( pid );
        }

        targetPositions.resize( jointNames.size() );

        // Subscribe to joints positions topic
        legs_jointsPositionsSubscriber = n.subscribe<uwalker::LegsJointsPositions>( "/legs_joints_positions", 1, &UWalkerController::legs_jointsPositionsCallback, this );

        return true;
    }

    void UWalkerController::legs_jointsPositionsCallback( const uwalker::LegsJointsPositions::ConstPtr &msg )
    {
        // NOTE If the way we define leg/joint numbers changes this needs to be updated accordingly
        unsigned int count = 0;
        for ( unsigned int i = 0; i < msg->leg.size(); i++ )
        {
            for ( unsigned int j = 0; j < msg->leg[i].joint.size(); j++ )
            {
                targetPositions[count] = msg->leg[i].joint[j];
                count++;
            }
        }
    }

    void UWalkerController::starting()
    {
        lastTime = robot->getTime();
        for ( unsigned int i = 0; i < pidController.size(); i++ )
        {
            pidController[i]->reset();
        }
    }

    void UWalkerController::update()
    {
        for ( unsigned int i = 0; i < joints.size(); i++ )
        {
            ros::Duration dt = robot->getTime() - lastTime;
            joints[i]->commanded_effort_ = pidController[i]->updatePid( joints[i]->position_ - targetPositions[i], dt );
        }
    }

    void UWalkerController::stopping()
    {

    }
}

PLUGINLIB_DECLARE_CLASS( uwalker, UWalkerControllerPlugin, UWalker::UWalkerController, controller_interface::Controller )
