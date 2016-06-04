#include<Wire.h>
int16_t acc_x,acc_y,acc_z,Tmp,gyro_x,gyro_y,gyro_z;
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "mpu6050.h"


int thr, roll, pitch, yaw, switches;
unsigned long lastRecvTime = 0;


float pid_pitch_level = 0.2;               // Auto level pitch pid
float pid_roll_level = pid_pitch_level;    // Auto level roll pid = Auto level pitch pid

float pid_p_gain_roll = 0.95;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller (0.03)
float pid_d_gain_roll = 11;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3.1;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error, angle_pitch_setpoint, angle_roll_setpoint;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_pitch_input, angle_roll_input;
int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int start;
int esc_1, esc_2, esc_3, esc_4;
unsigned long loop_timer;

const uint64_t pipeIn =  0xE8E8F0F0E1LL;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte switches;
};

MyData data;

/*  void print_rx_data() {
  Serial.print(thr);
  Serial.print("        ");
  Serial.print(yaw);
  Serial.print("        ");
  Serial.print(pitch);
  Serial.print("        ");
  Serial.print(roll);
  Serial.print("        ");
  Serial.print(switches);
  Serial.print("        ");
  Serial.print(data.throttle);                   DEBUGGING THING : UNCOMMENT THIS FUNTION AND UNCOMMENT LINE 110 AND LINE 163.
  Serial.print("        ");
  Serial.print(data.yaw);
  Serial.print("        ");
  Serial.print(data.pitch);
  Serial.print("        ");
  Serial.print(data.roll);
  Serial.println("        ");
  
  
} */

void set_data() {
  thr = map(data.throttle, 6, 255, 1000, 2000);
  yaw = map(data.yaw,      0, 255, 1000, 2000);
  pitch = map(data.pitch,   2, 255, -70, 70);                     // Map the pilot's desired angle
  roll  = map(data.roll,    0, 255, -70 , 70);                    // Map the pilot's desired angle
  switches = data.switches & 0x1 ? 2000 : 1000;
}

void resetData() {
  data.throttle = 0; 
  data.yaw = 127;       
  data.pitch = 127;
  data.roll = 127;
  data.switches = 0; 
  set_data(); 
}
////RECEIVE NRF24 DATA ROUTINE/////
void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis();
  }
}


void setup() {
 pinMode(2, OUTPUT);
 //Serial.begin(9600);  //for debugging
 Wire.begin();
 DDRD |= B11110000;
 set_registers();
 delay(250);
 ////////////////////NRF24L01 STARTUP/////////////////////////  
   resetData();

   radio.begin();
   radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
   radio.setAutoAck(false);
   radio.setPALevel(RF24_PA_MAX); 

   radio.openReadingPipe(1,pipeIn);
   radio.startListening();
////////////////////////END OF THE INITIALIZATION////////////////////////
/////////////////GYRO CALIBRATION//////////////////////////   
   
   //Let's take multiple samples so we can determine the average gyro offset
  Serial.print("Starting calibration...");           //Print message
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){    //Take 2000 readings for calibration
   if(cal_int % 15 == 0)digitalWrite(2, !digitalRead(2));   //Change the led status to indicate calibration.
    read_imu_data();                                 //Read the gyro output
    gyro_roll_cal += gyro_y;                      //Add roll value to gyro_roll_cal
    gyro_pitch_cal += gyro_x;                    //Add pitch value to gyro_pitch_cal
    gyro_yaw_cal += gyro_z;                        //Add yaw value to gyro_yaw_cal
    if(cal_int%100 == 0)Serial.print(".");           //Print a dot every 100 readings
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                //Wait 4 milliseconds before the next loop
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset
  Serial.println(" done!");                          //2000 measures are done!
  gyro_roll_cal /= 2000;                             //Divide the roll total by 2000
  gyro_pitch_cal /= 2000;                            //Divide the pitch total by 2000
  gyro_yaw_cal /= 2000;                              //Divide the yaw total by 2000
 
}           





void loop() {
  recvData();
 unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
  }
  set_data();

 // print_rx_data();     //for debugging
    
  read_imu_data();
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_y / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_x / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_z / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  get_acc_angle();

  angle_pitch_input = constrain(acc_x_angle, -70, 70);             // Limit the useable angle input from -70* to 70*
  angle_roll_input = constrain(acc_y_angle, -70, 70);              // Limit the useable angle input from -70* to 70*
 
  
  
  //For starting the motors: throttle low and yaw left (step 1).
  if( thr < 1162 && yaw < 1150)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && thr < 1162 && yaw > 1450){
    start = 2;
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //For stopping the motors : throttle low and yaw right :
  if(start == 2 && thr < 1162 && yaw > 1850)start = 0;
  
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if( data.roll > 140 )pid_roll_setpoint = (roll - angle_roll_input )*pid_roll_level;
  else if(data.roll < 110)pid_roll_setpoint = (roll - angle_roll_input)*pid_roll_level;

   //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(data.pitch > 140)pid_pitch_setpoint = (pitch - angle_pitch_input)*pid_pitch_level;
  else if(data.pitch < 110)pid_pitch_setpoint = (pitch - angle_pitch_input)*pid_pitch_level;

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(thr > 1162){ //Do not yaw when turning off the motors.
    if(yaw > 1508)pid_yaw_setpoint = (yaw - 1508)/3.0;
    else if(yaw < 1492)pid_yaw_setpoint = (yaw - 1492)/3.0;
  }

//PID inputs are known. So we can calculate the pid output.
  calculate_pid();
  
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  
  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1050 && battery_voltage > 600)digitalWrite(2, HIGH);
  
  throttle = thr;                                                           //We need the throttle signal as a base signal.
  
  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 
    
    if (esc_1 < 1064) esc_1 = 1064;                                         //Keep the motors running.
    if (esc_2 < 1064) esc_2 = 1064;                                         //Keep the motors running.
    if (esc_3 < 1064) esc_3 = 1064;                                         //Keep the motors running.
    if (esc_4 < 1064) esc_4 = 1064;                                         //Keep the motors running.
    
    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }
  
  else{
    esc_1 = 900;                                                           //If start is not 2 keep a 900us pulse for esc-1.
    esc_2 = 900;                                                           //If start is not 2 keep a 900us pulse for esc-2.
    esc_3 = 900;                                                           //If start is not 2 keep a 900us pulse for esc-3.
    esc_4 = 900;                                                           //If start is not 2 keep a 900us pulse for esc-4.
  }
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }

}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}


