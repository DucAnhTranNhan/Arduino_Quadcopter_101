#define M_PI  3.14159265358979323846
#define acce_sensitivity 16384
#define acc_x_gain 16418
#define acc_y_gain 16403
#define acc_z_gain 16600



float gyro_x_rate,gyro_y_rate,gyro_z_rate;
float acc_x_temp,acc_y_temp,acc_z_temp;
float acc_x_angle,acc_y_angle;
int cal_int = 0;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;



void get_acc_value()
{
  acc_x_temp= (float)acc_x/acc_x_gain;
  acc_y_temp= (float)acc_y/acc_y_gain;
  acc_z_temp= (float)acc_z/acc_z_gain;// [acc_z]=g=9.8 m/s.s
}


void get_acc_angle(void)
{
  get_acc_value();
  acc_x_angle=(180/M_PI)*atan(acc_y_temp/sqrt(pow(acc_x_temp,2)+pow(acc_z_temp,2)));
  acc_y_angle=(180/M_PI)*atan(-acc_x_temp/sqrt(pow(acc_y_temp,2)+pow(acc_z_temp,2)));
  
  if(acc_z<0)//kit bi dao nguoc: dung de tinh toan goc 0~(+-180) degrees
  {
    if(acc_x_angle<0)
    {
      acc_x_angle=-180-acc_x_angle;// 0>= acc_x_angle> -180
    }
    else
    {
      acc_x_angle=180-acc_x_angle;//0 =< acc_x_angle =< 180
    }
    
    if(acc_y_angle<0)
    {
      acc_y_angle=-180-acc_y_angle;// 0>= acc_x_angle> -180
    }
    else
    {
      acc_y_angle=180-acc_y_angle;//0 =< acc_x_angle =< 180
    }
  }
}

void set_registers() {
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void read_imu_data(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true);  // request a total of 14 registers
  acc_x=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  acc_y=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc_z=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyro_x=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  if(cal_int == 2000)gyro_x -= gyro_pitch_cal;
  gyro_y=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  if(cal_int == 2000)gyro_y -= gyro_roll_cal;
  gyro_z=Wire.read()<<8|Wire.read();// ???
  gyro_z *= -1;
  if(cal_int == 2000)gyro_z -= gyro_yaw_cal;
}
