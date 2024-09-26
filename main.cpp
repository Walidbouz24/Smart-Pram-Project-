#include <mbed.h>
#include "ultrasonic.h"
#include "Speaker_song.h"
#include "N5110.h"
#include "DHT.h"
#include "MQ135.h"
#include "OLED_Display.h" 



// Baby stroller sprite, 24x24
int baby_stroller_sprite[24][24] = {
    {0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
    {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,1,0,1,0,0,0,0,1,0,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,1,1,1,1,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0},
    {0,0,0,0,0,1,1,1,1,0,1,1,1,1,0,1,1,1,1,0,0,0,0,0},
    {0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};

//       ( SCE,  RST,  D/C, MOSI, SCLK,  LED)
//N5110 lcd(PA_4, PA_15, PB_7, PC_12, PC_10, PA_8);
//N5110 lcd(PA_4, PA_15, PB_7, PA_7, PA_5, PA_8);
//OLED_Display oled(PC_9, PA_8); // Use correct SDA and SCL pins for STM32 Nucleo F446RE 

PwmOut Rwheel(PB_4); // D5
PwmOut Lwheel(PB_10);  // D6

/*
DigitalOut ledGreen(LED1);
DigitalIn blueButton(BUTTON1); // USER_BUTTON
*/
DigitalOut IN_1(PB_3); //D3
DigitalOut IN_2(PB_5); //D4
DigitalOut IN_3(PA_8); //D7 //////////////////////
DigitalOut IN_4(PC_7); //D9
// movement functions:
void forward(float dc_rw);
void back(float dc_rw);
void curve_r(float dc_rw);
void curve_l(float dc_rw);
void on_spot_l(float dc_rw);
void on_spot_r(float dc_rw);
void stop(float dc_rw);
void buzz();
void buzzG();
void back_curve(float dc_rw);




 int dis_R,dis_M,dis_L;
 int dis_Fr,dis_Fl,dis_FM;

volatile bool updateMovement_ol = false;
volatile bool updateMovement_om = false;
volatile bool updateMovement_oml = false;
volatile bool updateMovement_or = false;
volatile bool updateMovement_olr = false;
volatile bool updateMovement_of = false;
volatile bool updateMovement_omr = false;
volatile bool updateMovement_none = false;
//=====================================//
volatile bool com_N = false;
volatile bool com_B = false;
volatile bool com_F = false;
volatile bool com_CR = false;
volatile bool com_CL = false;
volatile bool com_none = false;

volatile bool mode = false ;
volatile bool mode1 = false ;
//////////////////////////////
  // Define the notes and their frequencies (in Hz)
    const float C4 = 261.63;
    const float DD4 = 293.66;
    const float E4 = 329.63;
    const float F4 = 349.23;
    const float G4 = 392.00;
    const float AA4 = 440.00;
    const float B4 = 493.88;
    const float C5 = 523.25;
/////////////////////////////

//Serial bluetooth(PA_9, PA_10)
BufferedSerial pc(USBTX, USBRX, 9600);
BufferedSerial bluetooth(PA_9, PA_10, 9600); // D8  D2
// ultrasonicsens
   Speaker mySpeaker(PC_8);//pc_8

  void dist1(int dis_L)
    {
             //  printf("Distance M to %dmm\r\n", dis_L);
 

        //put code here to happen when the distance is changed
    }
    void dist2(int dis_M)
    {
         // printf("Distance M to %dmm\r\n", dis_M);

        //put code here to happen when the distance is changed
    }
    void dist3(int dis_R)
    {
         // printf("Distance M to %dmm\r\n", dis_R);

        //put code here to happen when the distance is changed
    }
    void dist4(int dis_Fr)
    {
         // printf("Distance M to %dmm\r\n", dis_R);

        //put code here to happen when the distance is changed
    }
    void dist5(int dis_Fl)
    {
         // printf("Distance M to %dmm\r\n", dis_R);

        //put code here to happen when the distance is changed
    }
       void dist6(int dis_FM)
    {
         // printf("Distance M to %dmm\r\n", dis_R);

        //put code here to happen when the distance is changed
    }
 ultrasonic mu_L(PB_12, D10, 100ms, 1s, &dist1);    //Set the trigger pin to D8 and the echo pin to D9 ( D11 => PB_12)
 ultrasonic mu_M(D12, PA_11, 100ms, 1s, &dist2);    //Set the trigger pin to D8 and the echo pin to D9 ( D13 => PA_11)
 ultrasonic mu_R(D14, D15, 100ms, 1s, &dist3);    //Set the trigger pin to D8 and the echo pin to D9
 // hand_free us
  ultrasonic free_L(PB_2, PB_1, 100ms, 1s, &dist4);    //Set the trigger pin to D8 and the echo pin to D9
 ultrasonic free_R(PB_15, PB_14, 100ms, 1s, &dist5);    //Set the trigger pin to D8 and the echo pin to D9
 ultrasonic free_M(PA_15, PB_7, 100ms, 1s, &dist6);    //Set the trigger pin to D8 and the echo pin to D9
// sensing inot 
MQ135 mq135_pin(PA_0);
DHT sensor(PC_6,DHT22);
//BufferedSerial Serial(USBTX, USBRX, 115200); //TX, RX BAUD
 int h, c,f;
 float ppm,dp ;
void task_DHT();
      
        float correctedPPM  ; 


float t_period = 0.00005;
 char recv ;
 float dc = 0.50f;
  float dc_ON = 0.20f;

 int t = 0;
 int thr=800;
bool buzz_called1 = false;

Mutex lcd_mutex;
Mutex data_mutex;

EventFlags sensorEvent;


 // thread sensing 
 void thread_sens() {
   
    while(1){
       
          
                         task_DHT();
                    //              ThisThread::sleep_for(4000ms);
                    lcd_mutex.lock();

 correctedPPM = mq135_pin.getCorrectedPPM(c, h)/100000 ; 
lcd_mutex.unlock();
 //printf("correctedPPM: %f\r\n", correctedPPM);
 
if(correctedPPM > 500) {
    buzzG();  
        buzzG();  

    buzzG();  

    
             // Call buzz() once
  
}else {}
    /*
        float rzero = mq135_pin.getRZero();
        float correctedRZero = mq135_pin.getCorrectedRZero(c, h);
        float resistance = mq135_pin.getResistance();
        float ppm = mq135_pin.getPPM()/10000 ;
      
        float correctedPPM = mq135_pin.getCorrectedPPM(c, h)/10000 ; 

        //printf("Analog Value (Scaled): %d\r\n", analog_value_scaled);
        printf("rzero: %.2f\r\n", rzero);
        printf("correctedRZero: %.2f\r\n", correctedRZero);
        printf("resistance: %f\r\n", resistance);
        printf("ppm: %f\r\n", ppm);
        printf("correctedPPM: %f\r\n", correctedPPM);
            //    serial.printf("correctedres: %.2f\r\n", res);

        thread_sleep_for(1000);*/
    }
    
}
 // thread of the music 
 void playLullaby() {
    // Define the note duration (in seconds)
  
    // Play "Twinkle Twinkle Little Star"
   while(mode1){
     on_spot_l(0.20); // Move to the left
    wait_us(1000000); // Wait for 1 second

    on_spot_r(0.20); // Move to the right
    wait_us(1000000); // Wait for 1 second
      

   }
   
   stop(dc);
   
}

void thread_RW() {
    
    while(1) {
                    //  bluetooth.read(&recv, 1);

 switch (recv){
    
case 'F' :

   //forward(*dc);
    if (updateMovement_ol ) {
   curve_r(dc); 
  // thread_sleep_for(100);
   updateMovement_ol = false;
  }
      if (updateMovement_om ) {
   curve_r(dc); 

     updateMovement_om = false;
  
 
  }
       if (  updateMovement_oml ) {
   on_spot_r(dc_ON); 
     updateMovement_oml = false;

  }
      if (  updateMovement_or ) {
    curve_l(dc_ON); 

      updateMovement_or = false;

  }
     if (  updateMovement_olr ) {  
     forward(dc);
       updateMovement_olr = false;

  }
     if (  updateMovement_of) {
        //stop(0.90);
back_curve(0.25);
  updateMovement_of = false;

  }
     if (  updateMovement_omr ) {
    curve_l(dc); 
      updateMovement_omr = false;

  }
// else {
     //  forward(dc);

// }
     if (  updateMovement_none ) {
     forward(dc);
      updateMovement_none = false;

  }
   

  


break;
case 'l' :
curve_l(dc);

break;
case 'r' :
curve_r(dc);
break;
case 'B' :
back(0.25);
 

break; 
case 'N' :
stop(dc);
break; 
case 'L' :
on_spot_l(dc_ON);
 
break;
case 'R' :
on_spot_r(dc_ON);
break;
       }
    }
}

void thread_LW() {
   
  while(1){

 bluetooth.read(&recv, 1);
  

 if (recv == 'N') {
  
  mode = false; 
    mode1 = false; 

 }
 if (recv == 'm') {
  mode = true; 
  mode1=false;
 }
  if (recv == 'z') {
  mode1 = true; 
  mode = false;
 }
  }
     
    
}

void thread_bt() {
    

  
         mu_L.startUpdates();//start mesuring the distance
     mu_M.startUpdates();//start mesuring the distance
       mu_R.startUpdates();//start mesuring the distance
       while(1){
   

         dis_L =  mu_L.getCurrentDistance();       
 
                  
                  dis_M =  mu_M.getCurrentDistance(); 


                 
                  dis_R =  mu_R.getCurrentDistance();  
 
       
       }
}
 // Constants
#define NUM_READINGS 10
#define DEBOUNCE_THRESHOLD 200
#define STABLE_COUNT_THRESHOLD 3
#define MAX_NOISE_THRESHOLD 2000 // Set a high threshold to identify extreme noise

// Globals
int readings_Fl[NUM_READINGS]; // Array to store the readings for the left sensor
int readings_Fr[NUM_READINGS]; // Array to store the readings for the right sensor
int readIndex_Fl = 0; // Index of the current reading for the left sensor
int readIndex_Fr = 0; // Index of the current reading for the right sensor
int total_Fl = 0; // Running total of the readings for the left sensor
int total_Fr = 0; // Running total of the readings for the right sensor
int average_Fl = 0; // Smoothed reading for the left sensor
int average_Fr = 0; // Smoothed reading for the right sensor

// Helper function to sort an array
void sortArray(int arr[], int size) {
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (arr[i] > arr[j]) {
                int temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
            }
        }
    }
}

// Median filter function
int medianFilter(int arr[], int size) {
    sortArray(arr, size);
    if (size % 2 == 0) {
        return (arr[size / 2 - 1] + arr[size / 2]) / 2;
    } else {
        return arr[size / 2];
    }
}

// Function to update the readings and calculate the median
int updateReadings(int* readings, int& readIndex, int newValue) {
    readings[readIndex] = newValue;
    readIndex = (readIndex + 1) % NUM_READINGS;
    return medianFilter(readings, NUM_READINGS);
}

// Error handling for outliers
bool isOutlier(int value, int median) {
    return abs(value - median) > DEBOUNCE_THRESHOLD;
}

// Variables for stable readings
int stableCount_Fl = 0;
int stableCount_Fr = 0;
int lastStable_Fl = 750; // Initial assumed stable value
int lastStable_Fr = 750; // Initial assumed stable value
int filtered_Flr;
int filtered_Frr;
int filtered_Fl;
int filtered_Fr;
// Main hand_free_s function
void hand_free_s() {
    free_R.startUpdates(); // Start measuring the distance
    free_L.startUpdates(); // Start measuring the distance
      //  free_M.startUpdates(); // Start measuring the distance

    
    // Initialize readings
    for (int i = 0; i < NUM_READINGS; i++) {
        readings_Fl[i] = 0;
        readings_Fr[i] = 0;
    }
    
    while (1) {  
         dis_Fr = free_R.getCurrentDistance(); 
         dis_Fl = free_L.getCurrentDistance(); 
          //        dis_FM = free_M.getCurrentDistance(); 

        
           // Skip extreme noise values
        
         filtered_Fl= updateReadings(readings_Fl, readIndex_Fl, dis_Fl);
         filtered_Fr = updateReadings(readings_Fr, readIndex_Fr, dis_Fr);

//if (filtered_Flr > MAX_NOISE_THRESHOLD || filtered_Frr > MAX_NOISE_THRESHOLD) {


     //    continue;
      // }
      // filtered_Fr = filtered_Frr ;
       // filtered_Fl = filtered_Flr ;


  
        // Debouncing logic for left sensor
        /*if (!isOutlier(dis_Fl, filtered_Fl)) {
            stableCount_Fl++;
        } else {
            stableCount_Fl = 0;
        }

        if (stableCount_Fl >= STABLE_COUNT_THRESHOLD) {
            lastStable_Fl = filtered_Fl;
        }

        // Debouncing logic for right sensor
        if (!isOutlier(dis_Fr, filtered_Fr)) {
            stableCount_Fr++;
        } else {
            stableCount_Fr = 0;
        }

        if (stableCount_Fr >= STABLE_COUNT_THRESHOLD) {
            lastStable_Fr = filtered_Fr;
        }*/

        // Print stable values
     //   printf("Distance changed to %d //// %dsmm\n", lastStable_Fl, lastStable_Fr);

        ThisThread::sleep_for(10ms); // Adjust delay as necessary
    }
}

    
   
   
// Define the notes frequencies (in Hertz)
#define NOTE_C4  261.63
#define NOTE_D4  293.66
#define NOTE_E4  329.63
#define NOTE_F4  349.23
#define NOTE_G4  392.00
#define NOTE_A4  440.00
#define NOTE_B4  493.88
#define NOTE_C5  523.25

// Define the durations (in seconds)
#define QUARTER 0.5
#define HALF    1.0

int main() {
   // oled.begin(); 
  //             oled.clearDisplay(); 
 //   oled.setCursor(0, 0); 
                       //   oled.print_string("< Moderate >",0,0);

       // lcd.normalMode();
           // lcd.inverseMode();

      //  lcd.setBrightness(0.5f);
  // Calculate the position to center the sprite
    // int x = 0; // Start position of the stroller off-screen to the left
   // int y = (48 - 24) / 2; // Vertically center the stroller

  //  while(x < 86) {
       // lcd.clear(); // Clear the display
               //  oled.clearDisplay(); 

      //  lcd.drawSprite(x, y, 24, 24, (int *)baby_stroller_sprite); // Draw the stroller
      //         oled.print_string_logo("!",x,0);

        //lcd.refresh(); // Refresh the display
     //   thread_sleep_for(100); // Wait for 100ms
     ///   x += 2; // Move the stroller to the right
   // }
     //  lcd.clear(); // clear buffer at start of every loop
      //                oled.clearDisplay(); 

        //    lcd.drawRect(0, 0, 84, 48, FILL_TRANSPARENT); // x, y, width, height, color (1 for white)
      //     oled.print_string_logo("#",20,2);

        // can directly print strings at specified co-ordinates (must be less than 84 pixels to fit on display)
        
      //  lcd.printString("    SMART  ",1,1);
       // lcd.printString("BABY STROLLER",1,3);
      
       //  lcd.refresh(); // Refresh the display
      //  thread_sleep_for(4000); // Wait for 100m 

    
   

    //  Thread myThread5(osPriorityNormal, 4096); // Increase the stack size to 2048 bytes
 //myThread5.start(thread_sens);
    
 //   myThread5.join(); // waits for myThread to finish execution
 //   myThread5.terminate(); // stops the thread
 //task_DHT();
 // Thread myThread6(osPriorityNormal, 4096); // Increase the stack size to 2048 bytes

 //myThread6.start(thread_disp);

           Thread myThread5(osPriorityNormal, 4096); // Increase the stack size to 2048 bytes
 myThread5.start(thread_sens);
  
   //         sensorEvent.set(0x1); // Set event flag to notify other threads
 Thread myThread_hfs; 
  myThread_hfs.start(hand_free_s); 


 while(1) {
   


   //   printf("Distance changed to %d  //// %dsmm\r\n", dis_L,dis_M);  
 // mode1 = false; 
bool buzz_called = false; // Step 1: Declare the flag
 
   //dc = 0.98f;
/////////////////////////////////////////////////////////////


Thread myThread;
    
    
   Thread myThread1;
  

  Thread myThread2;

     myThread1.start(thread_bt);
    myThread2.start(thread_LW);
   
     myThread.start(thread_RW); 
                  //   myThread6.flags_set(0x1);

      //  printf("Distance changed to %dmm\r\n", dis_Fr);  
             
 // printf("the received char is: %c\r\n", recv);

 

 
            
///////////////////////////////////////////////////////////////////////////////////////////////////
       
        if (dis_R > thr && dis_M > thr && dis_L < thr ) {
        updateMovement_ol = true;
              


        
  }
     else if (dis_R > thr && dis_M < thr && dis_L > thr ) {
    updateMovement_om = true;
   
  
 
  }
     else if (dis_R > thr && dis_M < thr && dis_L < thr ) {
     updateMovement_oml = true;
   

  }
    else if (dis_R < thr && dis_M > thr && dis_L > thr ) {
     updateMovement_or = true;
   

  }
    else if (dis_R < thr && dis_M > thr && dis_L < thr ) {  
      updateMovement_olr = true;
     

  }
   else if (dis_R < thr && dis_M < thr && dis_L < thr ) {
        //stop(0.90);
          updateMovement_of = true;


  }
    else if (dis_R < thr && dis_M < thr && dis_L > thr ) {
      updateMovement_omr = true;
    

  }
   

   else  {
//on_spot_r(0.90);
  updateMovement_none = true;
    // forward(*dc);

  
   }
    // 3 mode ***************************************************************************************
   while(mode1){
      
  //Thread myThreadmsc;

    // myThreadmsc.start(playLullaby);
      // Notes and durations for "Twinkle, Twinkle, Little Star"
   /* float melody[] = {
        NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
        NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4,
        NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
        NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
        NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
        NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4
    };

    float durations[] = {
        QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
        QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
        QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
        QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
        QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
        QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF
    };
  
    // Play the melody
    for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); ++i) {
        mySpeaker.PlayNote(melody[i], durations[i], 0.5); // Play note at 50% volume
    }*/
   forward(0.25);
            ThisThread::sleep_for(200ms);  // Short pause between notes
            back(0.25);
            ThisThread::sleep_for(500ms);  // Short pause between notes
      on_spot_l(0.25); // Move to the left
            ThisThread::sleep_for(1000ms);  // Short pause between notes
   forward(0.25);
            ThisThread::sleep_for(200ms);  // Short pause between notes
            back(0.25);
            ThisThread::sleep_for(500ms);  // Short pause between notes
    on_spot_r(0.25); // Move to the right
            ThisThread::sleep_for(1000ms);  // Short pause between notes
 


   }
  
   // Hand-Free algo ************************************************************************************

 while(mode){
  

 if (!buzz_called) { // Step 3: Check the flag
    buzz();           // Call buzz() once
    buzz_called = true; // Set the flag to false after calling buzz()
  }
  
 // Thread myThread_hfs; 
 // myThread_hfs.start(hand_free_s); 
 
  //Thread myThread_hfm;       
   //  myThread_hfm.start(hand_free_m);   
       // printf("Distance M to %d,////////////%dmm\r\n", dis_Fl,dis_Fr); 
       //           ThisThread::sleep_for(100ms);

////////////////////////////////////////////////////////////
                  //    printf("Distance changed to %d  //// %dsmm\r\n", filtered_Fl,filtered_Fr);  
                                  //...          printf("Distance changed to %d \r\n", dis_FM);   
 
                      
                        // printf("Distance changed to %d //// %dsmm\n", lastStable_Fl, lastStable_Fr);       

    if (filtered_Fl < 400 && filtered_Fr < 400 ) {
        // Move forward if both distances are less than 450
    if (dis_R < thr && dis_M < thr && dis_L < thr ) { // this is for the driving asstance while hand-free control
        //stop(0.90);
          stop(dc);
           if (filtered_Fl < 1100 && filtered_Fl > 800 && filtered_Fr < 1100 && filtered_Fr > 800 ) {
        // Move backward if left distance is between 600 and 1000 and right distance is between 600 and 1000
        back(0.30);


  }
    }
  else {
          forward(0.45);

  }
     //   wait_ns(50000);
    } 
     else if (filtered_Fl < 400  ) {
        // Curve right if right distance is less than 450
         curve_l(0.45);
    } else if (filtered_Fr < 400) {
        // Curve left if left distance is less than 450
               curve_r(0.45);
    }else if (filtered_Fl < 1000 && filtered_Fl > 800 && filtered_Fr < 1000 && filtered_Fr > 800 ) {
        // Move backward if left distance is between 600 and 1000 and right distance is between 600 and 1000
        back(0.30);
    }else if (filtered_Fr < 1000 && filtered_Fr > 800) {
        // Move backward if left distance is greater than 1000 and right distance is greater than 600
        back(0.30);
     //   wait_ns(50000);
    }
    else if (filtered_Fl < 1000 && filtered_Fl > 800) {
        // Move backward if left distance is greater than 1000 and right distance is greater than 600
        back(0.30);
     //   wait_ns(50000);
    }
    /*else if (average_Fl > 1000 && average_Fr > 1000) {
        // Stop if both distances are greater than 1000
        stop(dc);
      //  wait_ns(50000);
    } else if (average_Fl > 1000 && average_Fr > 600) {
        // Move backward if left distance is greater than 1000 and right distance is greater than 600
        back(dc);
     //   wait_ns(50000);
    } else if (average_Fl < 1000 && average_Fl > 600 && average_Fr > 600 && average_Fr < 1000) {
        // Move backward if left distance is between 600 and 1000 and right distance is between 600 and 1000
        back(dc);
      //  wait_ns(50000);
    } else if (average_Fl < 1000 && average_Fl > 600) {
        // Move backward if left distance is between 600 and 1000
        back(dc);
     //   wait_ns(50000);
    } else if (450 < average_Fl && average_Fl < 600 && 450 < average_Fr && average_Fr < 600) {
        // Stop if both distances are between 450 and 600
        stop(dc);
    } else if (450 < average_Fr && average_Fr < 600) {
        // Stop if right distance is between 450 and 600
        stop(dc);
    } else if (450 < average_Fl && average_Fl < 600) {
        // Stop if left distance is between 450 and 600
        stop(dc);
    } else if (average_Fr < 450) {
        // Curve right if right distance is less than 450
        curve_r(dc);
    } else if (average_Fl < 450) {
        // Curve left if left distance is less than 450
        curve_l(dc);
    } */else {
        // Default to stop if none of the conditions are met
        stop(dc);
    }
 }   

    }

   
}

// functions dec **********************************
void forward(float dc){
    // right wheel
   IN_1.write(0) ;
     IN_2.write(1);
       IN_3.write(0);
    IN_4.write(1);



    Rwheel.period(t_period);     
    Rwheel.write(dc+0.10);  
    Lwheel.period(t_period);      
    Lwheel.write(dc+0.10);


}

// back 
void back(float dc){
    // right wheel
   IN_1.write(1) ;
     IN_2.write(0);
       IN_3.write(1);
    IN_4.write(0);

    Rwheel.period(t_period);     
    Rwheel.write(dc-0.05);
    Lwheel.period(t_period);      
    Lwheel.write(dc-0.05); 
}
void back_curve(float dc){
    // right wheel
   IN_1.write(1) ;
     IN_2.write(0);
       IN_3.write(1);
    IN_4.write(0);

    Rwheel.period(t_period);     
    Rwheel.write(dc);
    Lwheel.period(t_period);      
    Lwheel.write(00.0); 
}

void on_spot_r(float dc){ 
    // right wheel
  IN_1.write(1) ;
     IN_2.write(0);
       IN_3.write(0);
    IN_4.write(1);


    Rwheel.period(t_period);     
    Rwheel.write(dc);  
    Lwheel.period(t_period);      
    Lwheel.write(dc);
}
void on_spot_l(float dc){
    // right wheel
IN_1.write(0) ;
     IN_2.write(1);
       IN_3.write(1);
    IN_4.write(0);


    Rwheel.period(t_period);     
    Rwheel.write(dc);  
    Lwheel.period(t_period);      
    Lwheel.write(dc);
}
void curve_r(float dc){
    // right wheel
   IN_1.write(0) ;
     IN_2.write(1);
       IN_3.write(0);
    IN_4.write(1);


    Rwheel.period(t_period);     
    Rwheel.write(dc-0.20);  
    Lwheel.period(t_period);      
    Lwheel.write(dc);
}
void curve_l(float dc){
    // right wheel
   IN_1.write(0) ;
     IN_2.write(1);
       IN_3.write(0);
    IN_4.write(1);


    Rwheel.period(t_period);     
    Rwheel.write(dc);  
    Lwheel.period(t_period);      
    Lwheel.write(dc-0.20);
}
void stop(float dc){
    // right wheel
   IN_1.write(0) ;
     IN_2.write(0);
       IN_3.write(0);
    IN_4.write(0);


    Rwheel.period(t_period);     
    Rwheel.write(00);  
    Lwheel.period(t_period);      
    Lwheel.write(00);
    
}
void buzz(){
    // right wheel
   mySpeaker.PlayNote(969.0,0.5,0.9);
      ThisThread::sleep_for(500ms);
      mySpeaker.PlayNote(800.0,0.5,0.9);
         ThisThread::sleep_for(500ms);
    
}
void buzzG(){
    // right wheel
   mySpeaker.PlayNote(969.0,0.5,0.9);
      ThisThread::sleep_for(200ms);
      mySpeaker.PlayNote(969.0,0.5,0.9);
         ThisThread::sleep_for(200ms);
         mySpeaker.PlayNote(969.0,0.5,0.9);
         ThisThread::sleep_for(200ms);
    
}

void task_DHT(){
    int error = 0;
    int h, c,f;
    float dp = 0.0f;
 
        ThisThread::sleep_for(2000ms);
        error = sensor.readData();
        if (0 == error) {
             data_mutex.lock();
            c = sensor.ReadTemperature(CELCIUS);
            h = sensor.ReadHumidity();
            data_mutex.unlock();
           // c   = sensor.ReadTemperature(CELCIUS);
          //  f   = sensor.ReadTemperature(FARENHEIT);
          //  h   = sensor.ReadHumidity();
          //  dp  = sensor.CalcdewPoint(c, h);
    
         //   printf("Temperature in Celcius: %d, Farenheit %d\r\n", c, f);
          //  printf("Humidity is %d, Dewpoint: %4.2f\r\n\n", h, dp);
            
        } else {
         //   printf("Error: %d\r\n", error);
        }
        //    sensorEvent.set(0x1); // Set event flag to notify other threads

}
