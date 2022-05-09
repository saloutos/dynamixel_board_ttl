#include "dynamixel_XL330.h"

XL330_bus::XL330_bus(uint32_t baud, PinName tx, PinName rx, PinName rts): sbus(tx, rx), rtswitch(rts)
{
    baudrate = baud;
    return_delay = 0.0005f; // defaults to 500us
    wait_us(300000);
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    rtswitch.mode(OpenDrainNoPull);
    rtswitch.output();
    sbus.baud(baudrate);
    rtswitch = 0;
    wait_us(300000); // todo: how long does this need to be?
    
    // todo: pass PC serial to init functions? might be okay
    
    iPacket[0] = 0xFF; //Header
    iPacket[1] = 0xFF; //Header
    iPacket[2] = 0xFD; //Header
    iPacket[3] = 0x00; //Reserved  
    
}

/***** XL330 Instruction Method *****/
void XL330_bus::iRead(uint8_t id, uint8_t length, uint16_t address)
{
    //Creates instruction packet to read data
    iPacket[4] = id;
    iPacket[5] = 0x07;
    iPacket[6] = 0x00;
    iPacket[7] = READ_DATA; //Instruction
    iPacket[8] = SHIFT_TO_LSB(address);
    iPacket[9] = SHIFT_TO_MSB(address);
    iPacket[10] = SHIFT_TO_LSB(length);
    iPacket[11] = SHIFT_TO_MSB(length);
    
    uint16_t CRC_temp;
    CRC_temp = update_crc(0, iPacket, 12);
    iPacket[12] = SHIFT_TO_LSB(CRC_temp);
    iPacket[13] = SHIFT_TO_MSB(CRC_temp);

    iPacketLength = 14;
}

void XL330_bus::iWrite(uint8_t id, uint16_t address, uint8_t param[], uint8_t paramLength)
{
    //Creates instruction packet to write action
    int i;

    iPacket[4] = id;            //PacketID
    iPacket[5] = (5 + paramLength);  //Packet Lenght LEN_L
    iPacket[6] = 0x00;          //Packet Lenght LEN_H
    iPacket[7] = WRITE_DATA;    //Instruction
    iPacket[8] = SHIFT_TO_LSB(address);
    iPacket[9] = SHIFT_TO_MSB(address);
    for(i = 10; i < 10 + paramLength ; i++) iPacket[i] = param[i - 10];

    uint16_t CRC_temp;
    CRC_temp = update_crc(0, iPacket, 10 + paramLength);
    iPacket[i] = SHIFT_TO_LSB(CRC_temp);
    iPacket[i + 1] = SHIFT_TO_MSB(CRC_temp);

    iPacketLength = i + 2;
}

void XL330_bus::sRead(uint8_t length, uint16_t address, uint8_t ids[], uint8_t idLength)
{
    //Creates instruction packet to read data
    int i;
    
    iPacket[4] = 0xFE; // broadcast packet
    iPacket[5] = (7 + idLength);
    iPacket[6] = 0x00;
    iPacket[7] = FAST_SYNC_READ; //Instruction
    iPacket[8] = SHIFT_TO_LSB(address);
    iPacket[9] = SHIFT_TO_MSB(address);
    iPacket[10] = SHIFT_TO_LSB(length);
    iPacket[11] = SHIFT_TO_MSB(length);
    for(i = 12; i < 12 + idLength ; i++) iPacket[i] = ids[i - 12];
    
    
    uint16_t CRC_temp;
    CRC_temp = update_crc(0, iPacket, 12 + idLength);
    iPacket[i] = SHIFT_TO_LSB(CRC_temp);
    iPacket[i + 1] = SHIFT_TO_MSB(CRC_temp);

    iPacketLength = i + 2;
}

void XL330_bus::sWrite(uint8_t data_length, uint16_t address, uint8_t param[], uint8_t paramLength)
{
    //Creates instruction packet to write action
    int i;

    iPacket[4] = 0xFE;            //PacketID
    iPacket[5] = (7 + paramLength);  //Packet Lenght LEN_L
    iPacket[6] = 0x00;          //Packet Lenght LEN_H
    iPacket[7] = SYNC_WRITE;    //Instruction
    iPacket[8] = SHIFT_TO_LSB(address);
    iPacket[9] = SHIFT_TO_MSB(address);
    iPacket[10] = SHIFT_TO_LSB(data_length);
    iPacket[11] = SHIFT_TO_MSB(data_length);
    for(i = 12; i < 12 + paramLength ; i++) iPacket[i] = param[i - 12];

    uint16_t CRC_temp;
    CRC_temp = update_crc(0, iPacket, 12 + paramLength);
    iPacket[i] = SHIFT_TO_LSB(CRC_temp);
    iPacket[i + 1] = SHIFT_TO_MSB(CRC_temp);

    iPacketLength = i + 2;
}


/***** Dynamixel Protocol 2.0 Methods *****/
void XL330_bus::sendIPacket()
{
    float timeOut = 12.0 * (float) ((float) iPacketLength) / ((float) baudrate); //12 = 10 (start + 8 bits + stop) + 2 (gives a bit more time)

    //Send packet
    rtswitch = 1; // set switch output to "transmit"
    
    wait_us(15); //from Adi
    
    for(int i = 0; i < iPacketLength; i++) sbus.putc(iPacket[i]);

    //Waits for sending before doing something else
    wait_us(25); // 25us from Adi
    
//    wait(timeOut); // should be on the order of 120us? // float is too small?
    rtswitch = 0; // set switch output to "receive"
}

void XL330_bus::getRPacket()
{
    int i = 0;
    float timeOut = return_delay + (12.0 * (float) ((float) rPacketLength) / ((float) baudrate)); //12 = 10 (start + 8 bits + stop) + 2 (gives a bit more time)
    wait_us(15); // same as delay in sendIPacket();
    Timer tr;
    tr.start();
    while((i < rPacketLength)){ // && (tr.read() <= timeOut*2.0)) {
//        rPacket[i] = sbus.getc();
//        i++;
        if (sbus.readable()) {
            rPacket[i] = sbus.getc();
            //pc.putc(rPacket[i]); //Prints packets
            i++;
        }
    }
    tr.stop();

    if (tr.read() >= timeOut) rPacket[8] = 0x80; //Creates an error code for the missed packet
}

/***** Generic functions *****/
void XL330_bus::SetSomething(uint8_t id, uint16_t address, uint8_t param[], uint8_t paramLength)
{
    iWrite(id, address, param, paramLength);
    rPacketLength = 11;

    sendIPacket();
    if (id != 0xFE) getRPacket(); //Broadcast does not returns packets
    else rPacket[8] = 0x00; //No error because no rPacket in broadcast

    switch(rPacket[8]) {
        case 0x00 : //No error
            break;

        case 0x80 :
            //pc.printf("Missed status packet\r\n"); // Print error 
            break;

        default :   //Error occurred
            //pc.printf("Error %d in status packet\r\n", rPacket[8]); // Print error
            break;
    }
}

uint8_t XL330_bus::GetSomething(uint8_t id, uint16_t address)
{
    iRead(id, 1, address);
    rPacketLength = 12;

    sendIPacket();
    getRPacket();

    switch(rPacket[8]) {
        case 0x00 : //No error
            return rPacket[9];

        case 0x80 :
            //pc.printf("Missed status packet\r\n"); // Print error
            return rPacket[8];

        default :   //Error occurred
            //pc.printf("Error %d in status packet\r\n", rPacket[8]); // Print error
            return rPacket[8]; //Return error status
    }
}

uint16_t XL330_bus::GetSomething16(uint8_t id, uint16_t address)
{
    iRead(id, 2, address);
    rPacketLength = 13;

    sendIPacket();
    getRPacket();

    switch(rPacket[8]) {
        case 0x00 : //No error
            return (uint16_t)rPacket[9] | (((uint16_t)rPacket[10]<<8)&0xFF00);

        case 0x80 :
            //pc.printf("Missed status packet\r\n"); // Print Error
            return (uint16_t)rPacket[8];

        default :   //Error occurred
            //pc.printf("Error %d in status packet\r\n", rPacket[8]); // Print error
            return (uint16_t)rPacket[8]; //Return error status
    }
}

uint32_t XL330_bus::GetSomething32(uint8_t id, uint16_t address)
{
    iRead(id, 4, address);
    rPacketLength = 15;

    sendIPacket();
    getRPacket();

    switch(rPacket[8]) {
        case 0x00 : //No error
            return (uint32_t)rPacket[9] | (((uint32_t)rPacket[10]<<8)&0x0000FF00) | (((uint32_t)rPacket[11]<<16)&0x00FF0000) | (((uint32_t)rPacket[12]<<24)&0xFF000000);

        case 0x80 :
            //pc.printf("Missed status packet\r\n"); // Print error
            return (uint32_t)rPacket[8];

        default :   //Error occurred
            //pc.printf("Error %d in status packet\r\n", rPacket[8]); // Print error
            return (uint32_t)rPacket[8]; //Return error status
    }
}

void XL330_bus::SetManyThings(uint8_t length, uint16_t address, uint8_t param[], uint8_t paramLength)
{   
    
    sWrite(length, address, param, paramLength);           
    sendIPacket();
    // sync write won't return an rPacket
    
}

void XL330_bus::GetManyThings(uint8_t* ret_vals, uint16_t address, uint8_t ids[], uint8_t idLength)
{
    // make sure ret_vals has size equal to idLength   
    sRead(1, address, ids, idLength);
    rPacketLength = 8 + idLength*5; // 4 necessary bytes, 1 due to data length

    sendIPacket();
    getRPacket();
    
    // for loop to extract ret_vals from rPacket
    int i = 10;
    for(int j=0; j<idLength; j++){
            ret_vals[j] = rPacket[i];
            i+=5; // 4 + data length
        }
   
}

void XL330_bus::GetManyThings16(uint16_t* ret_vals, uint16_t address, uint8_t ids[], uint8_t idLength)
{
    // make sure ret_vals has size equal to idLength   
    sRead(2, address, ids, idLength);
    rPacketLength = 8 + idLength*6; // 4 necessary bytes, 2 due to data length

    sendIPacket();
    getRPacket();
    
    // for loop to extract ret_vals from rPacket
    int i = 10;
    for(int j=0; j<idLength; j++){
            ret_vals[j] = (uint16_t)rPacket[i] | (((uint16_t)rPacket[i+1]<<8)&0xFF00);
            i+=6; // 4 + data length
        }
   
}

void XL330_bus::GetManyThings32(uint32_t* ret_vals, uint16_t address, uint8_t ids[], uint8_t idLength)
{
    // make sure ret_vals has size equal to idLength   
    sRead(4, address, ids, idLength);
    rPacketLength = 8 + idLength*8; // 4 necessary bytes, 4 due to data length

    sendIPacket();
    getRPacket();
    
    // for loop to extract ret_vals from rPacket
    int i = 10;
    for(int j=0; j<idLength; j++){
            ret_vals[j] = (uint32_t)rPacket[i] | (((uint32_t)rPacket[i+1]<<8)&0x0000FF00) | (((uint32_t)rPacket[i+2]<<16)&0x00FF0000) | (((uint32_t)rPacket[i+3]<<24)&0xFF000000);
            i+=8; // 4 + data length
        }
   
}

////// EEPROM ACCESS METHODS //////

/***** XL330 Network Parameter *****/
void XL330_bus::SetID(uint8_t id, uint8_t newId)
{
    uint8_t parameter[1];
    parameter[0] = newId;

    SetSomething(id, XL_ID_ADDRESS, parameter, 1);
    wait_us(200000);   //Waits for the value to be written in EEPROM
}

uint8_t XL330_bus::GetID(uint8_t id)
{
    return GetSomething(id, XL_ID_ADDRESS);
}

void XL330_bus::SetBaudRate(uint8_t id, uint8_t baudrt)
{
    uint8_t parameter[1];
    parameter[0] = baudrt;

    SetSomething(id, BAUD_RATE, parameter, 1);

    switch (baudrt) {
        case 0:
            sbus.baud(9600);
            break;
        case 1:
            sbus.baud(57600);
            break;
        case 2:
            sbus.baud(115200);
            break;
        case 3:
            sbus.baud(1000000);
            break;
    }

    wait_us(200000);   //Waits for the value to be written in EEPROM
}

uint8_t XL330_bus::GetBaudRate(uint8_t id)
{
    return GetSomething(id, BAUD_RATE);
}

void XL330_bus::SetRetDelTime(uint8_t id,uint8_t time)
{
    uint8_t parameter[1];
    parameter[0] = time;

    SetSomething(id, RETURN_DELAY_TIME, parameter, 1);
    wait_us(200000);   //Waits for the value to be written in EEPROM
    return_delay = (((float)time)*2.0f)/1000000.0f;
    
}

uint8_t XL330_bus::GetRetDelTime(uint8_t id)
{
    return GetSomething(id, RETURN_DELAY_TIME);
}

/***** XL330 Motor Setting *****/
void XL330_bus::SetControlMode(uint8_t id, uint8_t mode)
{
    uint8_t parameter[1];
    parameter[0] = mode;
    
    SetSomething(id, OPERATING_MODE, parameter, 1);
    wait_us(200000);   //Waits for the value to be written in EEPROM
}

uint8_t XL330_bus::GetControlMode(uint8_t id)
{
    return GetSomething(id, OPERATING_MODE);
}

void XL330_bus::SetTempLim(uint8_t id, uint8_t temp)
{
    uint8_t parameter[1];
    parameter[0] = temp;

    SetSomething(id, TEMPERATURE_LIMIT, parameter, 1);
    wait_us(200000);   //Waits for the value to be written in EEPROM
}

uint8_t XL330_bus::GetTempLim(uint8_t id)
{
    return GetSomething(id, TEMPERATURE_LIMIT);
}

void XL330_bus::SetCurrentLimit(uint8_t id, uint16_t current)
{
    uint8_t parameter[2];
    parameter[0] = SHIFT_TO_LSB(current);
    parameter[1] = SHIFT_TO_MSB(current);

    SetSomething(id, CURRENT_LIMIT, parameter, 2);
    wait_us(200000);   //Waits for the value to be written in EEPROM
}

uint16_t XL330_bus::GetCurrentLimit(uint8_t id)
{
    return GetSomething16(id, CURRENT_LIMIT);
}

void XL330_bus::SetRetLev(uint8_t id, uint8_t level)
{
    uint8_t parameter[1];
    parameter[0] = level;

    SetSomething(id, STATUS_RET_LVL, parameter, 1);
    wait_us(200000);   //Waits for the value to be written in EEPROM
}

uint8_t XL330_bus::GetRetLev(uint8_t id)
{
    return GetSomething(id, STATUS_RET_LVL);
}

uint8_t XL330_bus::GetShutdown(uint8_t id)
{
    return GetSomething(id, SHUTDOWN);
}

////// RAM ACCESS METHODS //////

/***** XL330 On/Off *****/
void XL330_bus::SetTorqueEn(uint8_t id, uint8_t enable)
{
    uint8_t parameter[1];
    parameter[0] = enable;

    SetSomething(id, TORQUE_ENABLE, parameter, 1);
}

uint8_t XL330_bus::GetTorqueEn(uint8_t id)
{
    return GetSomething(id, TORQUE_ENABLE);
}

void XL330_bus::TurnOnLED(uint8_t id, uint8_t led)
{
    uint8_t parameter[1];
    parameter[0] = led;

    SetSomething(id, LED, parameter, 1);
}

uint8_t XL330_bus::GetStatusLED(uint8_t id)
{
    return GetSomething(id, LED);
}

/***** XL330 Motor Control *****/
void XL330_bus::SetPosDGain(uint8_t id, uint16_t d_cons)
{
    uint8_t parameter[2];
    parameter[0] = SHIFT_TO_LSB(d_cons);
    parameter[1] = SHIFT_TO_MSB(d_cons);

    SetSomething(id, POS_D_GAIN, parameter, 2);
}

uint16_t XL330_bus::GetPosDGain(uint8_t id)
{
    return GetSomething16(id, POS_D_GAIN);
}

void XL330_bus::SetPosIGain(uint8_t id, uint16_t i_cons)
{
    uint8_t parameter[2];
    parameter[0] = SHIFT_TO_LSB(i_cons);
    parameter[1] = SHIFT_TO_MSB(i_cons);

    SetSomething(id, POS_I_GAIN, parameter, 2);
}

uint16_t XL330_bus::GetPosIGain(uint8_t id)
{
    return GetSomething16(id, POS_I_GAIN);
}

void XL330_bus::SetPosPGain(uint8_t id, uint16_t p_cons)
{
    uint8_t parameter[2];
    parameter[0] = SHIFT_TO_LSB(p_cons);
    parameter[1] = SHIFT_TO_MSB(p_cons);

    SetSomething(id, POS_P_GAIN, parameter, 2);
}

uint16_t XL330_bus::GetPosPGain(uint8_t id)
{
    return GetSomething16(id, POS_P_GAIN);
}

void XL330_bus::SetGoalPosition(uint8_t id, uint32_t position)
{
    uint8_t parameter[4];
    parameter[0] = (uint8_t) (position&0x00000FF);
    parameter[1] = (uint8_t) ((position&0x0000FF00)>>8);
    parameter[2] = (uint8_t) ((position&0x00FF0000)>>16);
    parameter[3] = (uint8_t) (position>>24);

    SetSomething(id, GOAL_POSITION, parameter, 4);
}

uint32_t XL330_bus::GetGoalPosition(uint8_t id)
{
    return GetSomething32(id, GOAL_POSITION);
}

void XL330_bus::SetGoalVelocity(uint8_t id, uint32_t velocity)
{
    uint8_t parameter[4];
    parameter[0] = (uint8_t) (velocity&0x00000FF);
    parameter[1] = (uint8_t) ((velocity&0x0000FF00)>>8);
    parameter[2] = (uint8_t) ((velocity&0x00FF0000)>>16);
    parameter[3] = (uint8_t) (velocity>>24);

    SetSomething(id, GOAL_VELOCITY, parameter, 4);
}

uint32_t XL330_bus::GetGoalVelocity(uint8_t id)
{
    return GetSomething32(id, GOAL_VELOCITY);
}

void XL330_bus::SetGoalCurrent(uint8_t id, uint16_t current)
{
    uint8_t parameter[2];
    parameter[0] = SHIFT_TO_LSB(current);
    parameter[1] = SHIFT_TO_MSB(current);

    SetSomething(id, GOAL_CURRENT, parameter, 2);
}

uint16_t XL330_bus::GetGoalCurrent(uint8_t id)
{
    return GetSomething16(id, GOAL_CURRENT);
}

void XL330_bus::SetMultGoalPositions(uint8_t ids[], uint8_t idLength, uint32_t positions[])
{
    uint8_t num_params = idLength*5; // 1 for ID + 4 for data length
    uint8_t parameter[num_params];
    for (int i=0; i<idLength; i++){
            int j = i*5;
            uint32_t pos = positions[i]; 
            parameter[j] = ids[i];
            parameter[j+1] = (uint8_t) (pos&0x00000FF);
            parameter[j+2] = (uint8_t) ((pos&0x0000FF00)>>8);
            parameter[j+3] = (uint8_t) ((pos&0x00FF0000)>>16);
            parameter[j+4] = (uint8_t) (pos>>24);
        }
    
    SetManyThings(4, GOAL_POSITION, parameter, num_params);
}

void XL330_bus::GetMultGoalPositions(int32_t* ret_pos, uint8_t ids[], uint8_t idLength)
{
    uint32_t ret_vals[idLength];
    GetManyThings32(ret_vals, GOAL_POSITION, ids, idLength);
    for (int i=0; i<idLength; i++){
        ret_pos[i] = (int32_t)ret_vals[i];
    }
}

void XL330_bus::SetMultGoalCurrents(uint8_t ids[], uint8_t idLength, uint16_t currents[])
{
    uint8_t num_params = idLength*3; // 1 for ID + 2 for data length
    uint8_t parameter[num_params];
    for (int i=0; i<idLength; i++){
            int j = i*3;
            uint32_t cur = currents[i]; 
            parameter[j] = ids[i];
            parameter[j+1] = SHIFT_TO_LSB(cur);
            parameter[j+2] = SHIFT_TO_MSB(cur);
        }
    
    SetManyThings(2, GOAL_CURRENT, parameter, num_params);
}

void XL330_bus::GetMultGoalCurrents(int16_t* ret_cur, uint8_t ids[], uint8_t idLength)
{
    uint16_t ret_vals[idLength];
    GetManyThings16(ret_vals, GOAL_CURRENT, ids, idLength);
    for (int i=0; i<idLength; i++){
        ret_cur[i] = (int16_t)ret_vals[i];
    }
}

/***** XL330 Feedback *****/
uint32_t XL330_bus::GetPosition(uint8_t id)
{
    return GetSomething32(id, PRESENT_POSITION);
}

uint32_t XL330_bus::GetVelocity(uint8_t id)
{
    return GetSomething32(id, PRESENT_VELOCITY);
}

uint16_t XL330_bus::GetCurrent(uint8_t id)
{
    return GetSomething16(id, PRESENT_CURRENT);
}

uint16_t XL330_bus::GetVoltage(uint8_t id)
{
    return GetSomething16(id, PRESENT_VOLTAGE);
}

uint8_t XL330_bus::GetTemperature(uint8_t id)
{
    return GetSomething(id, PRESENT_TEMPERATURE);
}

void XL330_bus::GetMultPositions(int32_t* ret_pos, uint8_t ids[], uint8_t idLength)
{
    uint32_t ret_vals[idLength];
    GetManyThings32(ret_vals, PRESENT_POSITION, ids, idLength);
    for (int i=0; i<idLength; i++){
        ret_pos[i] = (int32_t)ret_vals[i];
    }
}

void XL330_bus::GetMultVelocities(int32_t* ret_vel, uint8_t ids[], uint8_t idLength)
{
    uint32_t ret_vals[idLength];
    GetManyThings32(ret_vals, PRESENT_VELOCITY, ids, idLength);
    for (int i=0; i<idLength; i++){
        ret_vel[i] = (int32_t)ret_vals[i];
    }
}

void XL330_bus::GetMultCurrents(int16_t* ret_cur, uint8_t ids[], uint8_t idLength)
{
    uint16_t ret_vals[idLength];
    GetManyThings16(ret_vals, PRESENT_CURRENT, ids, idLength);
    for (int i=0; i<idLength; i++){
        ret_cur[i] = (int16_t)ret_vals[i];
    }
}

/***** XL330 Status *****/
uint8_t XL330_bus::GetRegInst(uint8_t id)
{
    return GetSomething(id, REGISTERED_INSTR);
}

uint8_t XL330_bus::GetMoving(uint8_t id)
{
    return GetSomething(id, MOVING);
}

uint8_t XL330_bus::GetHWErr(uint8_t id)
{
    return GetSomething(id, HW_ERR_STATUS);
}

/***** XL330 Instruction Method *****/
void XL330_bus::FactoryReset(uint8_t id, uint8_t option)
{
    iPacket[4] = id;
    iPacket[5] = 0x04;
    iPacket[6] = 0x00;
    iPacket[7] = FACTORY_RESET; //Instruction
    iPacket[8] = option;

    uint16_t CRC_temp;
    CRC_temp = update_crc(0, iPacket, 9);
    iPacket[9] = SHIFT_TO_LSB(CRC_temp);
    iPacket[10] = SHIFT_TO_MSB(CRC_temp);

    iPacketLength = 11;
    rPacketLength = 11;

    sendIPacket();
    getRPacket();
}

void XL330_bus::Ping(uint8_t id)
{
    iPacket[4] = id;
    iPacket[5] = 0x03;
    iPacket[6] = 0x00;
    iPacket[7] = PING; //Instruction

    uint16_t CRC_temp;
    CRC_temp = update_crc(0, iPacket, 9);
    iPacket[8] = SHIFT_TO_LSB(CRC_temp);
    iPacket[9] = SHIFT_TO_MSB(CRC_temp);

    iPacketLength = 10;
    rPacketLength = 14;

    sendIPacket();
    getRPacket();
    
//    switch(rPacket[8]) {
//        case 0x00 : //No error
//            return true;
//        case 0x80 :
//            return false;
//        default :   //Error occurred
//            return false; //Return error status
//        }
    
}

void XL330_bus::Reboot(uint8_t id)
{
    iPacket[4] = id;
    iPacket[5] = 0x03;
    iPacket[6] = 0x00;
    iPacket[7] = REBOOT; //Instruction

    uint16_t CRC_temp;
    CRC_temp = update_crc(0, iPacket, 9);
    iPacket[8] = SHIFT_TO_LSB(CRC_temp);
    iPacket[9] = SHIFT_TO_MSB(CRC_temp);

    iPacketLength = 10;
    rPacketLength = 11;

    sendIPacket();
    getRPacket();
}

/***** Calculating CRC Method *****/
uint16_t XL330_bus::update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i, j;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++) {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

XL330_bus::~XL330_bus() {};