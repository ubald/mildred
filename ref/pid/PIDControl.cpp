#include "PIDControl.h"

PIDControl::PIDControl()
{

}

PIDControl::~PIDControl()
{

}

void PIDControl::setup( double p, double i, double d, double frequency, double min = -1.0f, double max = 1.0f, int direction = DIRECT )
{
    setOutputLimits( min, max );
    setDirection( direction );
    setSamplingFrequency( frequency );
    setTuning( p, i, d );

    lastMeasuredValue = 0;
    integral = 0;
    derivative = 0;
    output = 0;
    automatic = true;
}

void PIDControl::setSamplingFrequency( double frequency )
{
    double intervalInSeconds = 1 / frequency;
    double ratio = intervalInSeconds / samplingInterval;
    samplingInterval = intervalInSeconds;
    Ki *= ratio;
    Kd /= ratio;
}

void PIDControl::setOutputLimits( double min, double max )
{
    if ( min >= max )
        return;

    outMin = min;
    outMax = max;

    if ( automatic )
    {
        if ( output > outMax )
            output = outMax;
        else if ( output < outMin )
            output = outMin;

        if ( integral > outMax )
            integral = outMax;
        else if ( integral < outMin )
            integral = outMin;
    }
}

void PIDControl::setTuning( double p, double i, double d )
{
    Kp = ( p < 0.0 ) ? 0.0 : p;
    Ki = ( i < 0.0 ) ? 0.0 : i * samplingInterval;
    Kd = ( d < 0.0 ) ? 0.0 : d / samplingInterval;

    if ( controllerDirection == REVERSE )
    {
        Kp = -Kp;
        Ki = -Ki;
        Kd = -Kd;
    }
}

void PIDControl::setMode( int mode )
{
    bool willBeAutomatic = ( mode == AUTOMATIC );

    if ( willBeAutomatic && !automatic )
    {
        initialize();
    }

    automatic = willBeAutomatic;
}

void PIDControl::setDirection( int direction )
{
    controllerDirection = direction;
}

void PIDControl::initialize()
{
    if ( integral > outMax )
        integral = outMax;
    else if ( integral < outMin )
        integral = outMin;
}

double PIDControl::compute( double setPoint, double measuredValue )
{
    if ( !automatic )
    {
        lastMeasuredValue = measuredValue;
        integral = output;
        return 0;
    }

    double error = setPoint - measuredValue;

    proportional = error;

    integral += ( Ki * error );
    if ( integral > outMax )
        integral = outMax;
    else if ( integral < outMin )
        integral = outMin;

    derivative = ( measuredValue - lastMeasuredValue );

    output = Kp * proportional + integral - Kd * derivative;
    if ( output > outMax )
        output = outMax;
    else if ( output < outMin )
        output = outMin;

    lastMeasuredValue = measuredValue;

    return output;
}
