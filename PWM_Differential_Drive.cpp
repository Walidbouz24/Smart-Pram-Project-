#include "PWM_Differential_Drive.h" 
#include "mbed.h" 



RobotCar::RobotCar(PinName rwPin, PinName lwPin, PinName in1Pin, PinName in2Pin, PinName in3Pin, PinName in4Pin) : rightWheel(rwPin), leftWheel(lwPin), in1(in1Pin), in2(in2Pin), in3(in3Pin), in4(in4Pin), timePeriod(0.00009) {} 
void RobotCar::forward(float Dutty_Cucle) { 
      in1.write(0) ;
     in2.write(1);
     in3.write(0);
     in4.write(1);
      rightWheel.period(timePeriod); 
      rightWheel.write(Dutty_Cucle); 
      leftWheel.period(timePeriod); 
      leftWheel.write(Dutty_Cucle);
} 

void RobotCar::back(float Dutty_Cucle) { 
     in1.write(1) ;
     in2.write(0);
     in3.write(1);
     in4.write(0);
      rightWheel.period(timePeriod); 
      rightWheel.write(Dutty_Cucle); 
      leftWheel.period(timePeriod); 
      leftWheel.write(Dutty_Cucle);
} 
void RobotCar::back_curve(float Dutty_Cucle) { 
     in1.write(1) ;
     in2.write(0);
     in3.write(1);
     in4.write(0);
      rightWheel.period(timePeriod); 
      rightWheel.write(Dutty_Cucle); 
      leftWheel.period(timePeriod); 
      leftWheel.write(0.00);
} 

void RobotCar::on_spot_right(float Dutty_Cucle) { 
     in1.write(1) ;
     in2.write(0);
     in3.write(0);
     in4.write(1);
      rightWheel.period(timePeriod); 
      rightWheel.write(Dutty_Cucle); 
      leftWheel.period(timePeriod); 
      leftWheel.write(Dutty_Cucle);
} 
void RobotCar::on_spot_left(float Dutty_Cucle) { 
     in1.write(0) ;
     in2.write(1);
     in3.write(1);
     in4.write(0);
      rightWheel.period(timePeriod); 
      rightWheel.write(Dutty_Cucle); 
      leftWheel.period(timePeriod); 
      leftWheel.write(Dutty_Cucle);
} 
void RobotCar::curve_right(float Dutty_Cucle) { 
     in1.write(0) ;
     in2.write(1);
     in3.write(0);
     in4.write(1);
      rightWheel.period(timePeriod); 
      rightWheel.write(Dutty_Cucle - 0.20); 
      leftWheel.period(timePeriod); 
      leftWheel.write(Dutty_Cucle);
} 
void RobotCar::curve_left(float Dutty_Cucle) { 
     in1.write(0) ;
     in2.write(1);
     in3.write(0);
     in4.write(1);
      rightWheel.period(timePeriod); 
      rightWheel.write(Dutty_Cucle); 
      leftWheel.period(timePeriod); 
      leftWheel.write(Dutty_Cucle - 0.20);
} 
