#define DXL_MODELNUMBER_L 0x00
#define DXL_MODELNUMBER_H 0x01
#define DXL_FIRMWARE 0x02
#define DXL_ID 0x03
#define DXL_BAUDRATE 0x04
#define DXL_RETURN_DELAY 0x05
#define DXL_CW_ANGLE_LIMIT_L 0x06
#define DXL_CW_ANGLE_LIMIT_H 0x07
#define DXL_CCW_ANGLE_LIMIT_L 0x08
#define DXL_CCW_ANGLE_LIMIT_H 0x09
#define DXL_MAX_TEMP 0x0B
#define DXL_LOW_VOLTAGE 0x0C
#define DXL_HIGH_VOLTAGE 0x0D
#define DXL_MAX_TORQUE_L 0x0E
#define DXL_MAX_TORQUE_H 0x0F
#define DXL_ALARM_LED 0x11
#define DXL_ALARM_SHUTDOWN 0x12
#define DXL_TORQUE_ENABLE 0x18
#define DXL_LED 0x19
#define DXL_D_GAIN 0x1A
#define DXL_I_GAIN 0x1B
#define DXL_P_GAIN 0x1C
#define DXL_GOAL_POSITION_L 0x1E
#define DXL_GOAL_POSITION_H 0x1F
#define DXL_GOAL_SPEED_L 0x20
#define DXL_GOAL_SPEED_H 0x21
#define DXL_TORQUE_LIMIT_L 0x22
#define DXL_TORQUE_LIMIT_H 0x23
#define DXL_POSITION_L 0x24
#define DXL_POSITION_H 0x25
#define DXL_SPEED_L 0x26
#define DXL_SPEED_H 0x27
#define DXL_LOAD_L 0x28
#define DXL_LOAD_H 0x29
#define DXL_VOLTAGE 0x2A
#define DXL_TEMP 0x2B
#define DXL_REGISTERED 0x2C
#define DXL_MOVING 0x2E
#define DXL_EEPROM_LOCK 0x2F
#define DXL_PUNCH_L 0x30
#define DXL_PUNCH_H 0x31

class Dynamixel {
  private:
    class sDynamixel *s;
  public:
		Dynamixel();

    void openBus();
    void closeBus();

    void setBusBaudrate(int baudnum);
    void setDeviceNumber(int dev);

    void setMotorBaudrate(int id, int baudnum);
    void setID(int old_id, int new_id);

    void setGoalPosition(int id, int position);
    void setGoalAngle(int id, double angle);
    int getPosition(int id);
    double getAngle(int id);

    void setSpeed(int id, int speed);
    int getSpeed(int id);

    void setLED(int id, bool on);
    bool getLED(int id);

		double pos2ang(int pos) { return  pos * (360./4096.); }
		int ang2pos(double ang) { return (int) (ang * (4096./360.)); }
};
