#ifndef PIDCONTROL_H_
#define PIDCONTROL_H_

#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1

class PIDControl
{
    public:
        PIDControl();
        virtual ~PIDControl();

        void setup( double p, double i, double d, double interval, double min, double max, int direction );

        void setSamplingFrequency( double interval );
        void setOutputLimits( double min, double max );
        void setTuning( double p, double i, double d );
        void setMode( int mode );
        void setDirection( int direction );

        void initialize();

        double compute( double setPoint, double measuredValue );
    protected:
        bool automatic;
        int controllerDirection;
        double Kp;
        double Ki;
        double Kd;
        double samplingInterval;
        double outMin;
        double outMax;
        double proportional;
        double integral;
        double derivative;
        double lastMeasuredValue;
        double output;
    private:
};

#endif
