/*******************************************************************************
Based on library at: https://os.mbed.com/users/dev_num_ecam/code/XL320_Driver/

Based on Dynamixel XL-330 data sheet: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/

Using Dynamixel Protocol 2.0: https://emanual.robotis.com/docs/en/dxl/protocol2/
*******************************************************************************/

#ifndef dynamixel_XL330_H
#define dynamixel_XL330_H

#include "mbed.h"

////////// SHIFT BYTES ///////////
#define SHIFT_TO_LSB(w)         (uint8_t((uint16_t(w)) & 0x00ff))
#define SHIFT_TO_MSB(w)         (uint8_t((uint16_t(w) >> 8) & 0x00ff))

////////// EEPROM Address ////////      //Access    Initial Value and Units
#define MODEL_NUMBER            0x00    //R         1200    -
#define MODEL_INFO              0x02    //R         -       -
#define VERSION                 0x06    //R         -       -
#define XL_ID_ADDRESS           0x07    //RW        1       0 ~ 252(0xFC) Broadcast = 0xFE(?)
#define BAUD_RATE               0x08    //RW        1       0 : 9600, 1 : 57600, 2 : 115200, 3 : 1Mbps
#define RETURN_DELAY_TIME       0x09    //RW        250     0 ~ 254 (in 2 usec)
#define DRIVE_MODE              0x0A    //RW        0       0 ~ 5 (0 is normal operation)
#define OPERATING_MODE          0x0B    //RW        3       0 ~ 16
#define SHADOW_ID               0x0C    //RW        255     0 ~ 252
#define PROTOCOL                0x0D    //RW        2       2 ~ 22
#define HOMING OFFSET           0x14    //RW        0       -1044479 ~ 1044479 (in pulses)
#define MOVING_THRESHOLD        0x18    //RW        10      0 ~ 1023 (in 0.229 rev/min) 
#define TEMPERATURE_LIMIT       0x1F    //RW        70      0 ~ 100 (in deg C)
#define MAX_VOLTAGE_LIMIT       0x20    //RW        70      31 ~ 70 (in 0.1 V)
#define MIN_VOLTAGE_LIMIT       0x22    //RW        35      31 ~ 70 (in 0.1 V)
#define PWM_LIMIT               0x24    //RW        885     0 ~ 885 (in 0.113%)
#define CURRENT_LIMIT           0x26    //RW        1750    0 ~ 1750 (in mA)
#define VELOCITY_LIMIT          0x2C    //RW        445     0 ~ 2047 (in 0.229 rev/min)
#define MAX_POSITION_LIMIT      0x30    //RW        4095    0 ~ 4095 (in pulses)
#define MIN_POSITION_LIMIT      0x34    //RW        0       0 ~ 4095 (in pulses)
#define STARTUP_CONFIG          0x3C    //RW        0       3
#define PWM_SLOPE               0x3E    //RW        140     1 ~ 255 (in 1.997 mV/sec)
#define SHUTDOWN                0x3F    //RW        53      -

////////// RAM Address ///////////
#define TORQUE_ENABLE           0X40    //RW        0       0 ~ 1 (0 is off, 1 is on)
#define LED                     0x41    //RW        0       0 ~ 1 (0 is off, 1 is on)
#define STATUS_RET_LVL          0x44    //RW        2       0 ~ 2   
#define REGISTERED_INSTR        0x45    //R         0       0 ~ 1
#define HW_ERR_STATUS           0x46    //R         0       -
#define VEL_I_GAIN              0x4C    //RW        1600    0 ~ 16383
#define VEL_P_GAIN              0x4E    //RW        180     0 ~ 16383
#define POS_D_GAIN              0x50    //RW        0       0 ~ 16383
#define POS_I_GAIN              0x52    //RW        0       0 ~ 16383
#define POS_P_GAIN              0x54    //RW        400     0 ~ 16383
#define FF_SECOND_GAIN          0x58    //RW        0       0 ~ 16383
#define FF_FIRST_GAIN           0x5A    //RW        0       0 ~ 16383
#define BUS_WATCHDOG            0x62    //RW        0       1 ~ 127 (in 20 msec)
#define GOAL_PWM                0x64    //RW        -       -PWM_LIMIT ~ PWM_LIMIT
#define GOAL_CURRENT            0x66    //RW        -       -CURRENT_LIMIT ~ CURRENT_LIMIT (in mA)
#define GOAL_VELOCITY           0x68    //RW        -       -VELOCITY_LIMIT ~ VELOCITY_LIMIT (in 0.229 rev/min)
#define PROF_ACCEL              0x6C    //RW        0       0 ~ 32767 (in 214.577 rev/min2 or in msec)
#define PROF_VELOCITY           0x70    //RW        0       0 ~ 32767 (in 0.229 rev/min)
#define GOAL_POSITION           0x74    //RW        -       MIN_POSITION_LIMIT ~ MAX_POSITION_LIMIT (in pulses)
#define REALTIME_TICK           0x78    //R         -       0 ~ 32767 (in msec)
#define MOVING                  0x7A    //R         0       0 ~ 1
#define MOVING_STATUS           0x7B    //R         0       -
#define PRESENT_PWM             0x7C    //R         -       -
#define PRESENT_CURRENT         0x7E    //R         -       (in mA)
#define PRESENT_VELOCITY        0x80    //R         -       (in 0.229 rev/min)
#define PRESENT_POSITION        0x84    //R         -       (in pulses)
#define VELOCITY_TRAJ           0x88    //R         -       (in 0.229 rev/min)
#define POSITION_TRAJ           0x8C    //R         -       (in pulses)
#define PRESENT_VOLTAGE         0x90    //R         -       (in 0.1 V)
#define PRESENT_TEMPERATURE     0x92    //R         -       (in deg C) 
#define BACKUP_READY            0x93    //R         -       0 ~ 1

////////// INSTRUCTION ///////////
#define PING                    0x01
#define READ_DATA               0x02
#define WRITE_DATA              0x03
#define REG_WRITE               0x04
#define ACTION                  0x05
#define FACTORY_RESET           0x06
#define REBOOT                  0x08
#define CLEAR                   0x10
#define CTRL_TBL_BACKUP         0x20
#define STATUS                  0x55
#define SYNC_READ               0x82
#define SYNC_WRITE              0x83
#define FAST_SYNC_READ          0x8A
#define BULK_READ               0x92
#define BULK_WRITE              0x93
#define FAST_BULK_READ          0x9A

////////// ERROR ///////////
#define RESULT_FAIL             0x01
#define INS_ERR                 0x02
#define CRC_ERR                 0x03`
#define DATA_RANGE_ERR          0x04
#define DATA_LENGTH_ERR         0x05
#define DATA_LIMIT_ERR          0x06
#define ACCESS_ERR              0x07

////////// RESET ///////////
#define RESET_ALL               0xFF
#define RESET_EXP_ID            0x01
#define RESET_EXP_ID_BR         0x02

////////// CONTROL MODES ///////////
#define CURRENT_CONTROL         0x00
#define VELOCITY_CONTROL        0x01
#define POSITION_CONTROL        0x03
#define EXTEND_POS_CONTROL      0x04
#define CURRENT_POS_CONTROL     0x05
#define PWM_CONTROL             0x10

class XL330_bus
{
public:
    XL330_bus(uint32_t baud, PinName tx, PinName rx, PinName rtswitch);
    ~XL330_bus();

    ////// EEPROM ACCESS METHODS //////

    /***** XL330 Network Parameter *****/
    void SetID(uint8_t id, uint8_t newId);
    uint8_t GetID(uint8_t id);
    void SetBaudRate(uint8_t id, uint8_t baudrate);
    uint8_t GetBaudRate(uint8_t id);
    void SetRetDelTime(uint8_t id, uint8_t time);
    uint8_t GetRetDelTime(uint8_t id);

    /***** XL330 Motor Setting *****/
    void SetControlMode(uint8_t id, uint8_t mode);
    uint8_t GetControlMode(uint8_t id);
    void SetTempLim(uint8_t id, uint8_t temp);
    uint8_t GetTempLim(uint8_t id);
    void SetCurrentLimit(uint8_t id, uint16_t current);
    uint16_t GetCurrentLimit(uint8_t id);
    void SetRetLev(uint8_t id, uint8_t level);
    uint8_t GetRetLev(uint8_t id);
    uint8_t GetShutdown(uint8_t id);
    
    ////// RAM ACCESS METHODS //////

    /***** XL330 On/Off *****/
    void SetTorqueEn(uint8_t id, uint8_t enable);
    uint8_t GetTorqueEn(uint8_t id);
    void TurnOnLED(uint8_t id, uint8_t led);
    uint8_t GetStatusLED(uint8_t id);

    /***** XL330 Motor Control *****/
    void SetPosDGain(uint8_t id, uint16_t d_cons);
    uint16_t GetPosDGain(uint8_t id);
    void SetPosIGain(uint8_t id, uint16_t i_cons);
    uint16_t GetPosIGain(uint8_t id);
    void SetPosPGain(uint8_t id, uint16_t p_cons);
    uint16_t GetPosPGain(uint8_t id);
    void SetGoalPosition(uint8_t id, uint32_t position);
    uint32_t GetGoalPosition(uint8_t id);
    void SetGoalVelocity(uint8_t id, uint32_t velocity);
    uint32_t GetGoalVelocity(uint8_t id);
    void SetGoalCurrent(uint8_t id, uint16_t current);
    uint16_t GetGoalCurrent(uint8_t id);

    void SetMultGoalPositions(uint8_t ids[], uint8_t idLength, uint32_t positions[]);
    void GetMultGoalPositions(int32_t* ret_pos, uint8_t ids[], uint8_t idLength);
    void SetMultGoalCurrents(uint8_t ids[], uint8_t idLength, uint16_t currents[]);
    void GetMultGoalCurrents(int16_t* ret_cur, uint8_t ids[], uint8_t idLength);
    
    /***** XL330 Feedback *****/
    uint32_t GetPosition(uint8_t id);
    uint32_t GetVelocity(uint8_t id);
    uint16_t GetCurrent(uint8_t id);
    uint16_t GetVoltage(uint8_t id);
    uint8_t GetTemperature(uint8_t id);

    void GetMultPositions(int32_t* ret_pos, uint8_t ids[], uint8_t idLength);
    void GetMultVelocities(int32_t* ret_vel, uint8_t ids[], uint8_t idLength);
    void GetMultCurrents(int16_t* ret_cur, uint8_t ids[], uint8_t idLength);

    /***** XL330 Status *****/
    uint8_t GetRegInst(uint8_t id);
    uint8_t GetMoving(uint8_t id);
    uint8_t GetHWErr(uint8_t id);

    /***** XL330 Instruction Method *****/
    void FactoryReset(uint8_t id, uint8_t option);
    void Ping(uint8_t id); // TODO: fix this, it currently hangs and doesn't work!!
    void Reboot(uint8_t id);

private:
    /***** Generic functions *****/
    void SetSomething(uint8_t id, uint16_t address, uint8_t param[], uint8_t paramLength);
    uint8_t GetSomething(uint8_t id, uint16_t address);
    uint16_t GetSomething16(uint8_t id, uint16_t address);
    uint32_t GetSomething32(uint8_t id, uint16_t address);
    void SetManyThings(uint8_t length, uint16_t address, uint8_t param[], uint8_t paramLength);
    void GetManyThings(uint8_t* ret_vals, uint16_t address, uint8_t ids[], uint8_t idLength);
    void GetManyThings16(uint16_t* ret_vals, uint16_t address, uint8_t ids[], uint8_t idLength);
    void GetManyThings32(uint32_t* ret_vals, uint16_t address, uint8_t ids[], uint8_t idLength);

    /***** XL330 Instruction Method *****/
    void iRead(uint8_t id, uint8_t length, uint16_t address);
    void iWrite(uint8_t id, uint16_t address, uint8_t param[], uint8_t paramLength);
    void sRead(uint8_t length, uint16_t address, uint8_t ids[], uint8_t idLength); // uses fast sync read
    void sWrite(uint8_t length, uint16_t address, uint8_t param[], uint8_t paramLength);
//    void bRead();
//    void bReadFast();
//    void bWrite();    

    // todo: finish sync functions, don't need bulk for now  
    // todo: add indirect addresses to be able to get all data in one read?
    
    /***** Dynamixel Protocol 2.0 Methods *****/
    void sendIPacket();
    void getRPacket();

    /***** Calculating CRC Method *****/
    uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

    /***** Variables *****/
    uint32_t baudrate;      //Baudrate
    float return_delay;     // return delay time in us
    uint8_t iPacket[100];    //Instruction Packet
    uint8_t iPacketLength;  //Instruction Packet Length
    uint8_t rPacket[100];    //Return Packet
    uint8_t rPacketLength;  //Return Packet Length
    
    RawSerial sbus;          //Serial Line for motor bus
    DigitalInOut rtswitch;
    
};

#endif