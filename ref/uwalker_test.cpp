#include <ros/ros.h>

int main( int argc, char **argv )
{
    ros::init( argc, argv, "uwalker_test" );
    ros::NodeHandle n;
    ros::Rate loop_rate( 150 );

    unsigned int count = 1000000000;
    unsigned int i;
    double start;
    double end;

    double stepPosition;

    start = ros::Time::now().sec + ( ros::Time::now().nsec * 1e-9 );
    for ( i = 0; i < count; i++ )
    {
        double cycleStep = 1.00 / 0.25;
        unsigned int step = floor( cycleStep );
        stepPosition = cycleStep - step;//fmod( cycleTime, stepDuration ) * cycle.steps;
    }
    end = ros::Time::now().sec + ( ros::Time::now().nsec * 1e-9 );
    ROS_ERROR_STREAM( "time:" << end-start );

    start = ros::Time::now().sec + ( ros::Time::now().nsec * 1e-9 );
    double a;
    unsigned int b;
    for ( i = 0; i < count; i++ )
    {
        a = 1.00 / 0.25;
        b = floor( a );
        stepPosition = a - b;//fmod( cycleTime, stepDuration ) * cycle.steps;
    }
    end = ros::Time::now().sec + ( ros::Time::now().nsec * 1e-9 );
    ROS_ERROR_STREAM( "time:" << end-start );



    start = ros::Time::now().sec + ( ros::Time::now().nsec * 1e-9 );
    for ( i = 0; i < count; i++ )
    {
        unsigned int step = floor( 1.00 / 0.25 );
        stepPosition = fmod( 1.00, 0.25 ) * 6;
    }
    end = ros::Time::now().sec + ( ros::Time::now().nsec * 1e-9 );
    ROS_ERROR_STREAM( "time:" << end-start );

    return 0;
    while ( ros::ok() )
    {

    }

    return 0;
}
