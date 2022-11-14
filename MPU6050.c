/*
 * MPU6050.c
 *
 *  Created on: Nov 12, 2022
 *      Author: efeoflus
 */

/* MPU6050_light library for Arduino
 *
 * Authors: Romain JL. F�tick (github.com/rfetick)
 *              simplifications and corrections
 *          Tockn (github.com/tockn)
 *              initial author (v1.5.2)
 */

#include "MPU6050.h"
#include "15348.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include "timer.h"
#include <math.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"




#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b

#define MPU6050_GYRO_OUT_REGISTER     0x43
#define MPU6050_ACCEL_OUT_REGISTER    0x3B

#define RAD_2_DEG             57.29578 // [deg/rad]
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0

#define DEFAULT_GYRO_COEFF    0.98


uint8_t address = MPU6050_ADDR; // 0x68 or 0x69
float gyro_lsb_to_degsec, acc_lsb_to_g;
float gyroXoffset, gyroYoffset, gyroZoffset;
float accXoffset, accYoffset, accZoffset;
float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
unsigned long preInterval;
float filterGyroCoef; // complementary filter coefficient to balance gyro vs accelero data to get angle

char upsideDownMounting = 0;

void InitI2C0(void);
char I2C0_Wr(int slaveAddr, char memAddr, char data);

//initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}


/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

/* INIT and BASIC FUNCTIONS */

void MPU6050_init(){
  //I2C3_Init();
  InitI2C0();
  SysTick_Wait10ms(100);
  MPU6050_setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  MPU6050_setGyroOffsets(0,0,0);
  MPU6050_setAccOffsets(0,0,0);
}




/* Wait until I2C master is not busy and return error code */
/* If there is no error, return 0 */

static int I2C0_wait_till_done(void)
{
    while(I2C0_MCS_R & 1);   /* wait until I2C master is not busy */
    return I2C0_MCS_R & 0xE; /* return I2C error code */
}

/* Write one byte only */
/* byte write: S-(saddr+w)-ACK-maddr-ACK-data-ACK-P */
char I2C0_Wr(int slaveAddr, char memAddr, char data)
{

    char error;

    /* send slave address and starting address */
    I2C0_MSA_R = slaveAddr << 1;
    I2C0_MDR_R = memAddr;
    I2C0_MCS_R = 3;                      /* S-(saddr+w)-ACK-maddr-ACK */

    error = I2C0_wait_till_done();       /* wait until write is complete */
    if (error) return error;

    /* send data */
    I2C0_MDR_R = data;
    I2C0_MCS_R = 5;                      /* -data-ACK-P */
    error = I2C0_wait_till_done();       /* wait until write is complete */
    while(I2C0_MCS_R & 0x40);            /* wait until bus is not busy */
    error = I2C0_MCS_R & 0xE;
    if (error) return error;

    return 0;       /* no error */
}

char I2C0_Rd(int slaveAddr, char memAddr, int byteCount, char* data)
{
     char error;

    if (byteCount <= 0)
        return -1;         /* no read was performed */

    /* send slave address and starting address */
    I2C0_MSA_R = slaveAddr << 1;
    I2C0_MDR_R = memAddr;
    I2C0_MCS_R = 3;       /* S-(saddr+w)-ACK-maddr-ACK */
    error = I2C0_wait_till_done();
    if (error)
        return error;

    /* to change bus from write to read, send restart with slave addr */
    I2C0_MSA_R = (slaveAddr << 1) + 1;   /* restart: -R-(saddr+r)-ACK */

    if (byteCount == 1)             /* if last byte, don't ack */
        I2C0_MCS_R = 7;              /* -data-NACK-P */
    else                            /* else ack */
        I2C0_MCS_R = 0xB;            /* -data-ACK- */
    error = I2C0_wait_till_done();
    if (error) return error;

    *data++ = I2C0_MDR_R;            /* store the data received */

    if (--byteCount == 0)           /* if single byte read, done */
    {
        while(I2C0_MCS_R & 0x40);    /* wait until bus is not busy */
        return 0;       /* no error */
    }

    /* read the rest of the bytes */
    while (byteCount > 1)
    {
        I2C0_MCS_R = 9;              /* -data-ACK- */
        error = I2C0_wait_till_done();
        if (error) return error;
        byteCount--;
        *data++ = I2C0_MDR_R;        /* store data received */
    }

    I2C0_MCS_R = 5;                  /* -data-NACK-P */
    error = I2C0_wait_till_done();
    *data = I2C0_MDR_R;              /* store data received */
    while(I2C0_MCS_R & 0x40);        /* wait until bus is not busy */

    return 0;       /* no error */
}


void Delay(unsigned long counter)
{
    unsigned long i = 0;

    for(i=0; i< counter*10000; i++);
}


char MPU6050_begin(int gyro_config_num, int acc_config_num){
  // changed calling register sequence [https://github.com/rfetick/MPU6050_light/issues/1] -> thanks to augustosc
  writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01); // check only the first connection with status
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER, 0x00);
  MPU6050_setGyroConfig(gyro_config_num);
  MPU6050_setAccConfig(acc_config_num);

  MPU6050_update();
  SysTick_Wait10ms(10);
  angleX = MPU6050_getAccAngleX();
  angleY = MPU6050_getAccAngleY();

  return 0;
}


char writeData(char memAddr, char data)
{
    return I2C0_Wr(0x68,memAddr, data);
    //I2CSend(0x68, 2, memAddr, data);
    return 0;
}

/* SETTER */

char MPU6050_setGyroConfig(int config_num){
  char status;
  switch(config_num){
    case 0: // range = +- 250 deg/s
      gyro_lsb_to_degsec = 131.0;
      status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
      break;
    case 1: // range = +- 500 deg/s
      gyro_lsb_to_degsec = 65.5;
      status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x08);
      break;
    case 2: // range = +- 1000 deg/s
      gyro_lsb_to_degsec = 32.8;
      status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x10);
      break;
    case 3: // range = +- 2000 deg/s
      gyro_lsb_to_degsec = 16.4;
      status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x18);
      break;
    default: // error
      status = 1;
      break;
  }
  return status;
}

char MPU6050_setAccConfig(int config_num){
  char status;
  switch(config_num){
    case 0: // range = +- 2 g
      acc_lsb_to_g = 16384.0;
      status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
      break;
    case 1: // range = +- 4 g
      acc_lsb_to_g = 8192.0;
      status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x08);
      break;
    case 2: // range = +- 8 g
      acc_lsb_to_g = 4096.0;
      status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x10);
      break;
    case 3: // range = +- 16 g
      acc_lsb_to_g = 2048.0;
      status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x18);
      break;
    default: // error
      status = 1;
      break;
  }
  return status;
}

void MPU6050_setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050_setAccOffsets(float x, float y, float z){
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void MPU6050_calcGyroOffsets(){ MPU6050_calcOffsets(1,0); }; // retro-compatibility with v1.0.0
void MPU6050_calcAccOffsets(){ MPU6050_calcOffsets(0,1); }; // retro-compatibility with v1.0.0
void MPU6050_setAddress(uint8_t addr){ address = addr; };
uint8_t MPU6050_getAddress(){ return address; };

// MPU CONFIG GETTER
float MPU6050_getGyroXoffset(){ return gyroXoffset; };
float MPU6050_getGyroYoffset(){ return gyroYoffset; };
float MPU6050_getGyroZoffset(){ return gyroZoffset; };

float MPU6050_getAccXoffset(){ return accXoffset; };
float MPU6050_getAccYoffset(){ return accYoffset; };
float MPU6050_getAccZoffset(){ return accZoffset; };

float MPU6050_getFilterGyroCoef(){ return filterGyroCoef; };
float MPU6050_getFilterAccCoef(){ return 1.0-filterGyroCoef; };

// DATA GETTER
float MPU6050_getTemp(){ return temp; };

float MPU6050_getAccX(){ return accX; };
float MPU6050_getAccY(){ return accY; };
float MPU6050_getAccZ(){ return accZ; };

float MPU6050_getGyroX(){ return gyroX; };
float MPU6050_getGyroY(){ return gyroY; };
float MPU6050_getGyroZ(){ return gyroZ; };

float MPU6050_getAccAngleX(){ return angleAccX; };
float MPU6050_getAccAngleY(){ return angleAccY; };

float MPU6050_getAngleX(){ return angleX; };
float MPU6050_getAngleY(){ return angleY; };
float MPU6050_getAngleZ(){ return angleZ; };

void MPU6050_setFilterGyroCoef(float gyro_coeff){
  if ((gyro_coeff<0) || (gyro_coeff>1)){ gyro_coeff = DEFAULT_GYRO_COEFF; } // prevent bad gyro coeff, should throw an error...
  filterGyroCoef = gyro_coeff;
}

void MPU6050_setFilterAccCoef(float acc_coeff){
  MPU6050_setFilterGyroCoef(1.0-acc_coeff);
}

/* CALC OFFSET */

void MPU6050_calcOffsets(char is_calc_gyro, char is_calc_acc){
    int i;
  if(is_calc_gyro){ MPU6050_setGyroOffsets(0,0,0); }
  if(is_calc_acc){ MPU6050_setAccOffsets(0,0,0); }
  float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro

  for(i = 0; i < CALIB_OFFSET_NB_MES; i++){
    MPU6050_fetchData();
    ag[0] += accX;
    ag[1] += accY;
    ag[2] += (accZ-1.0);
    ag[3] += gyroX;
    ag[4] += gyroY;
    ag[5] += gyroZ;
    Delay(1); // wait a little bit between 2 measurements
  }

  if(is_calc_acc){
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }

  if(is_calc_gyro){
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}

/* UPDATE */

void MPU6050_fetchData(){
  int rawData[7]; // [ax,ay,az,temp,gx,gy,gz]
  unsigned char sensordata[14];
  //I2C_read(0x68, MPU6050_ACCEL_OUT_REGISTER, sensordata, 14);
  I2C0_Rd(0x68,MPU6050_ACCEL_OUT_REGISTER, 14, sensordata);
       rawData[0] = (int16_t) ( ((uint8_t)sensordata[0] << 8 ) |(uint8_t)sensordata[1] );
       rawData[1] = (int16_t) ( ((uint8_t)sensordata[2] << 8 ) |(uint8_t)sensordata[3] );
       rawData[2] = (int16_t) ( ((uint8_t)sensordata[4] << 8 ) |(uint8_t)sensordata[5] );
       rawData[3] = (int16_t) ( ((uint8_t)sensordata[6] << 8 ) | (uint8_t)sensordata[7] );
       rawData[4] = (int16_t) ( ((uint8_t)sensordata[8] << 8 ) |(uint8_t)sensordata[9] );
       rawData[5] = (int16_t) ( ((uint8_t)sensordata[10] << 8 ) | (uint8_t)sensordata[11] );
       rawData[6] = (int16_t) ( ((uint8_t)sensordata[12] << 8 ) | (uint8_t)sensordata[13] );


  accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
  accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
  accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  //temp = rawData[3];
  gyroX = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
  gyroY = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
  gyroZ = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
}

void MPU6050_update(){
  // retrieve raw data
  MPU6050_fetchData();

  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
  angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180,+180] deg
  angleAccY = - atan2(accX,     sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG; // [- 90,+ 90] deg


  float dt = 100 * 1e-3;  // assuming that update will be called every 100 milliseconds


  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyMPU6050/issues/6
  angleX = wrap(filterGyroCoef*(angleAccX + wrap(angleX +     gyroX*dt - angleAccX,180)) + (1.0-filterGyroCoef)*angleAccX,180);
  angleY = wrap(filterGyroCoef*(angleAccY + wrap(angleY + sgZ*gyroY*dt - angleAccY, 90)) + (1.0-filterGyroCoef)*angleAccY, 90);
  angleZ += gyroZ*dt; // not wrapped

}


