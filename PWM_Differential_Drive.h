#ifndef ROBOTCAR_H 
#define ROBOTCAR_H 
#include "mbed.h" 

class RobotCar { 
    public: RobotCar(PinName rwPin, PinName lwPin, PinName in1Pin, PinName in2Pin, PinName in3Pin, PinName in4Pin); 

    void forward(float Dutty_Cucle); 
    void back(float Dutty_Cucle); 
    void back_curve(float Dutty_Cucle); 
    void on_spot_right(float Dutty_Cucle); 
    void on_spot_left(float Dutty_Cucle); 
    void curve_right(float Dutty_Cucle); 
    void curve_left(float Dutty_Cucle); 

    
    private: 
    
        PwmOut rightWheel; 
        PwmOut leftWheel; 
        DigitalOut in1; 
        DigitalOut in2; 
        DigitalOut in3; 
        DigitalOut in4; 
        float timePeriod; };
        
         #endif
