#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifndef MT_2PI
#  define MT_2PI 6.283195307179587
#endif

static const double EMG30_count_per_motorTurn = 360.;
static const double EMG30_motorTurn_per_wheelTurn = 30.;
static const double EMG30_rad_per_count = MT_2PI/(EMG30_count_per_motorTurn*EMG30_motorTurn_per_wheelTurn);
static const double Wheel_radius = .05;
static const double MD25_encAcc_per_accLevel = 1./0.025;

enum Mode {SEPERATE_0_TO_255 = 0,
           SEPERATE_128_TO_127 = 1,
           SHARED_AND_TURNMODE_0_TO_255 = 2,
           SHARED_AND_TURNMODE_128_TO_127 = 3};

enum Motor {MOTOR_1 = 0, MOTOR_2 = 1};  
 
static const char* DEVICE = "/dev/ttyACM0";

static const unsigned char I2C_INTERFACE = 0x55;

static const unsigned char MD_25_WRITE = 0xB0;
static const unsigned char MD_25_READ = 0xB1;

static const unsigned char MD_25_MODE = 0x0F;
static const unsigned char MD_25_MOTOR_1 = 0x00;
static const unsigned char MD_25_MOTER_2 = 0x01;
static const unsigned char MD_25_VOLTAGE = 0x0A;
static const unsigned char MD_25_COMMAND = 0x10;
static const unsigned char MD_25_ACCELERATION = 0x0E;
static const unsigned char MD_25_RESET_ENCODER = 0x20;
static const unsigned char MD_25_ENCODER_1_MSB = 0x02;
static const unsigned char MD_25_ENCODER_2_MSB = 0x06;

static const unsigned char MPU_9150_WRITE = 0xD0;
static const unsigned char MPU_9150_READ = 0xD1;

static const unsigned char MPU_CONFIG = 0x1A;
static const unsigned char MPU_GYRO_CONFIG = 0x1B;
static const unsigned char MPU_ACCEL_CONFIG = 0x1C;
static const unsigned char MPU_PWR_MGMT_1 = 0x6B;
static const unsigned char MPU_PWR_MGMT_2 = 0x6C;

static const unsigned char MPU_ACCEL_XOUT_H = 0x3B;
static const unsigned char MPU_ACCEL_XOUT_L = 0x3C;
static const unsigned char MPU_ACCEL_YOUT_H = 0x3D;
static const unsigned char MPU_ACCEL_YOUT_L = 0x3E;
static const unsigned char MPU_ACCEL_ZOUT_H = 0x3F;
static const unsigned char MPU_ACCEL_ZOUT_L = 0x40;

static const unsigned char MPU_GYRO_XOUT_H = 0x43; 
static const unsigned char MPU_GYRO_XOUT_L = 0x44;
static const unsigned char MPU_GYRO_YOUT_H = 0x45;
static const unsigned char MPU_GYRO_YOUT_L = 0x46;
static const unsigned char MPU_GYRO_ZOUT_H = 0x47;
static const unsigned char MPU_GYRO_ZOUT_L = 0x48;

static const double EXCHANGEVALUE = 0.0076294;

#endif // CONSTANTS_H
