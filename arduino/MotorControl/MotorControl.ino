#include <Servo.h>
#include <ros.h>
#include <uwalker/LegsJoints.h>
#include <uwalker/LegJoints.h>

ros::NodeHandle nh;

Servo leg[1][3];
float ONE_EIGHTY_ON_PI = 180 / M_PI;

void onLegsPositions( const uwalker::LegsJoints& legs_msg )
{
	for ( int i = 0 ; i < 1; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			leg[i][j].write( legs_msg.leg[i].joint[j] * ONE_EIGHTY_ON_PI );
		}
	}
}
ros::Subscriber<uwalker::LegsJoints> legsSubscriber( "legs_positions", &onLegsPositions );

void setup() 
{
	leg[0][0].attach(9);
	leg[0][1].attach(10);
	leg[0][2].attach(11);
	
	nh.initNode();
	nh.subscribe( legsSubscriber );
}

void loop()
{
	nh.spinOnce();
}