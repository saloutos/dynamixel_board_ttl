/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include "mbed.h"
#include "dynamixel_XL330.h"
#include "math.h"
#include "math_ops.h"

#define REST_MODE           0
#define CAN_MODE            1
#define DATA_MODE           2

#define CAN_SENSORS            0

#define CAN_TX_DXL1            1 // commands from controller to DXLs
#define CAN_TX_DXL2            2
#define CAN_TX_DXL3            3
#define CAN_TX_DXL4            4
#define CAN_TX_DXL5            5
#define CAN_TX_DXL6            6
#define CAN_TX_DXL7            7
#define CAN_TX_DXL8            8

#define CAN_FORCE_1            9
#define CAN_FORCE_2            10
#define CAN_TOF_1              11
#define CAN_TOF_2              12

#define CAN_RX_DXL1            13 // responses from DXLs to controller
#define CAN_RX_DXL2            14
#define CAN_RX_DXL3            15
#define CAN_RX_DXL4            16
#define CAN_RX_DXL5            17
#define CAN_RX_DXL6            18
#define CAN_RX_DXL7            19
#define CAN_RX_DXL8            20

/// Value Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 10.0f
#define T_MIN -72.0f
#define T_MAX 72.0f

#define KP_SCALE 50.0f
#define T_SCALE 50.0f


// Initialize serial port
RawSerial pc(PA_9, PA_10, 921600);
 
// Pins
PinName bus_1_rx = PC_11;
PinName bus_1_tx = PC_10;
PinName bus_1_rts = PA_4;
PinName bus_2_rx = PA_1;
PinName bus_2_tx = PA_0;  
PinName bus_2_rts = PA_5;
PinName bus_3_rx = PD_2;
PinName bus_3_tx = PC_12;
PinName bus_3_rts = PA_6;
PinName bus_4_rx = PC_7;
PinName bus_4_tx = PC_6;
PinName bus_4_rts = PA_7;

PinName SYS_CAN_TX = PA_12;
PinName SYS_CAN_RX = PA_11;
PinName SENSOR_CAN_TX = PB_6;
PinName SENSOR_CAN_RX = PB_5;
 
// CAN setup
CAN cansys(SYS_CAN_RX, SYS_CAN_TX, 1000000);
CANMessage rxMsg;
CANMessage txDxl1, txDxl2, txDxl3, txDxl4, txDxl5, txDxl6;

// Serial and state machine setup
volatile bool serial_flag = false;
volatile char serial_input;
volatile int state = REST_MODE;
volatile int print_data_flag = 0;
volatile int motor_data_flag = 0;
Ticker motor_data;
Ticker print_data;
Timer t;
Timer t2;
int dxl_time, can_time;
void get_motor_data(){
    motor_data_flag = 1;
}
void print_new_data(){
    print_data_flag = 1;
} 


// Variables for  dynamixel bus
uint8_t dxl_IDs_1[] = {0x01, 0x02, 0x03};
const int num_IDs_1 = 3;
uint8_t dxl_IDs_2[] = {0x04, 0x05, 0x06};
const int num_IDs_2 = 3;  

// Set up dynamixel
XL330_bus dxl_bus1(1000000, bus_1_tx, bus_1_rx, bus_1_rts); // baud, tx, rx, rts
XL330_bus dxl_bus2(1000000, bus_2_tx, bus_2_rx, bus_2_rts); // baud, tx, rx, rts

//Variables to store values
float currentPos1[num_IDs_1];
float currentPos2[num_IDs_2];
float currentVel1[num_IDs_1];
float currentVel2[num_IDs_2];
float currentCur1[num_IDs_1];
float currentCur2[num_IDs_2];

// Controller variables
int32_t dxl_position1[3];
int32_t dxl_velocity1[3];
int16_t dxl_current1[3];
float desired_current1[3]; // in A!
uint16_t current_command1[3]; // in mA!
uint16_t current_temp1[] = {0, 0, 0};
float dxl_offsets1[] = {0.785f, 3.927f, 3.927f};
int32_t dxl_position2[3];
int32_t dxl_velocity2[3];
int16_t dxl_current2[3];
float desired_current2[3]; // in A!
uint16_t current_command2[3]; // in mA!
uint16_t current_temp2[] = {0, 0, 0};
float dxl_offsets2[] = {3.927f, 3.927f, 3.927f};

// CAN command variables
float dxl_pos_des[6];
float dxl_vel_des[6];
float dxl_tff_des[6];
float dxl_kp[6];
float dxl_kd[6];


float KdJ = 0.8f; // bonus joint damping
float Kt = 0.60f/1.74f; // effective torque constant, Nm/A
float Kt_inv = 1.0f/Kt;
float pulse_to_rad = (2.0f*3.14159f)/4096.0f; // = 0.001534
float rpm_to_rads = (0.229f*2.0f*3.14159f)/60.0f; // = 0.0239
float current_limit = 1.75f; // in A, max is 1.75
float torque_limit = 0.40f; // in Nm, stall is 0.6Nm at 6V

void updateBus1(){

    // Get dynamixel states...sync version
    dxl_bus1.GetMultPositions(dxl_position1, dxl_IDs_1, num_IDs_1);
    dxl_bus1.GetMultVelocities(dxl_velocity1, dxl_IDs_1, num_IDs_1);
    dxl_bus1.GetMultCurrents(dxl_current1, dxl_IDs_1, num_IDs_1);
    
    // convert states
    for(int i=0; i<num_IDs_1; i++){
        currentPos1[i] = (pulse_to_rad*(float)dxl_position1[i])-dxl_offsets1[i];
        currentVel1[i] = rpm_to_rads*(float)dxl_velocity1[i];
        currentCur1[i] = 0.001f*(float)dxl_current1[i];
    }
    
    // commands are of the form tau_des[i] = Kp[i]*(pos_des[i]-pos[i]) + Kd[i]*(vel_des[i]-vel[i]) + tau_ff[i]; desired_current[i] = tau_des[i]/Kt;
    desired_current1[0] = Kt_inv*( dxl_kp[0]*(dxl_pos_des[0]-currentPos1[0]) + dxl_kd[0]*(dxl_vel_des[0]-currentVel1[0]) + dxl_tff_des[0] );
    desired_current1[0] = fmaxf(fminf(desired_current1[0],current_limit),-current_limit);
    current_command1[0] = (int16_t)(desired_current1[0]*1000.0f); 

    desired_current1[1] = Kt_inv*( dxl_kp[1]*(dxl_pos_des[1]-currentPos1[1]) + dxl_kd[1]*(dxl_vel_des[1]-currentVel1[1]) + dxl_tff_des[1] );
    desired_current1[1] = fmaxf(fminf(desired_current1[1],current_limit),-current_limit);
    current_command1[1] = (int16_t)(desired_current1[1]*1000.0f); 

    desired_current1[2] = Kt_inv*( dxl_kp[2]*(dxl_pos_des[2]-currentPos1[2]) + dxl_kd[2]*(dxl_vel_des[2]-currentVel1[2]) + dxl_tff_des[2] );
    desired_current1[2] = fmaxf(fminf(desired_current1[2],current_limit),-current_limit);
    current_command1[2] = (int16_t)(desired_current1[2]*1000.0f); 

    // // could limit control signal (non tau_ff) too to prevent some oscillations due to high velocities
    // desired_current1[0] = 0.0;
    // desired_current1[1] =  0.0;
    // desired_current1[2] = 0.0;
    // // check against current limit and convert back to uint16_t to send to motors
    // desired_current1[0] = fmaxf2(fminf2(desired_current1[0],current_limit),-current_limit);
    // desired_current1[1] = fmaxf2(fminf2(desired_current1[1],current_limit),-current_limit);
    // desired_current1[2] = fmaxf2(fminf2(desired_current1[2],current_limit),-current_limit);
    // current_command1[0] = (int16_t)(desired_current1[0]*1000.0f); //(0.0f); // need to convert desired_current to mA before type-casting
    // current_command1[1] = (int16_t)(desired_current1[1]*1000.0f); //(0.0f);
    // current_command1[2] = (int16_t)(desired_current1[2]*1000.0f); //(0.0f);
    
    // send commands
    dxl_bus1.SetMultGoalCurrents(dxl_IDs_1, num_IDs_1, current_command1); // average of ~2300us to read position, velocity, current, and set current, for 3 motors; 300us to print 3 floats            
    // dxl_bus1.SetMultGoalCurrents(dxl_IDs_1, num_IDs_1, current_temp1); // average of ~2300us to read position, velocity, current, and set current, for 3 motors; 300us to print 3 floats            

}

void updateBus2(){

    // Get dynamixel states...sync version
    dxl_bus2.GetMultPositions(dxl_position2, dxl_IDs_2, num_IDs_2);
    dxl_bus2.GetMultVelocities(dxl_velocity2, dxl_IDs_2, num_IDs_2);
    dxl_bus2.GetMultCurrents(dxl_current2, dxl_IDs_2, num_IDs_2);
    
    // convert states
    for(int i=0; i<num_IDs_2; i++){
        currentPos2[i] = (pulse_to_rad*(float)dxl_position2[i])-dxl_offsets2[i];
        currentVel2[i] = rpm_to_rads*(float)dxl_velocity2[i];
        currentCur2[i] = 0.001f*(float)dxl_current2[i];
    }
    
    // commands are of the form tau_des[i] = Kp[i]*(pos_des[i]-pos[i]) + Kd[i]*(vel_des[i]-vel[i]) + tau_ff[i]; desired_current[i] = tau_des[i]/Kt;
    desired_current2[0] = Kt_inv*( dxl_kp[3]*(dxl_pos_des[3]-currentPos2[0]) + dxl_kd[3]*(dxl_vel_des[3]-currentVel2[0]) + dxl_tff_des[3] );
    desired_current2[0] = fmaxf(fminf(desired_current2[0],current_limit),-current_limit);
    current_command2[0] = (int16_t)(desired_current2[0]*1000.0f); 

    desired_current2[1] = Kt_inv*( dxl_kp[4]*(dxl_pos_des[4]-currentPos2[1]) + dxl_kd[4]*(dxl_vel_des[4]-currentVel2[1]) + dxl_tff_des[4] );
    desired_current2[1] = fmaxf(fminf(desired_current2[1],current_limit),-current_limit);
    current_command2[1] = (int16_t)(desired_current2[1]*1000.0f); 

    desired_current2[2] = Kt_inv*( dxl_kp[5]*(dxl_pos_des[5]-currentPos2[2]) + dxl_kd[5]*(dxl_vel_des[5]-currentVel2[2]) + dxl_tff_des[5] );
    desired_current2[2] = fmaxf(fminf(desired_current2[2],current_limit),-current_limit);
    current_command2[2] = (int16_t)(desired_current2[2]*1000.0f); 


    // // could limit control signal (non tau_ff) too to prevent some oscillations due to high velocities
    // desired_current2[0] = 0.0;
    // desired_current2[1] =  0.0;
    // desired_current2[2] = 0.0;
    // // check against current limit and convert back to uint16_t to send to motors
    // desired_current2[0] = fmaxf2(fminf2(desired_current2[0],current_limit),-current_limit);
    // desired_current2[1] = fmaxf2(fminf2(desired_current2[1],current_limit),-current_limit);
    // desired_current2[2] = fmaxf2(fminf2(desired_current2[2],current_limit),-current_limit);
    // current_command2[0] = (int16_t)(desired_current2[0]*1000.0f); //(0.0f); // need to convert desired_current to mA before type-casting
    // current_command2[1] = (int16_t)(desired_current2[1]*1000.0f); //(0.0f);
    // current_command2[2] = (int16_t)(desired_current2[2]*1000.0f); //(0.0f);
    
    // send commands
    dxl_bus2.SetMultGoalCurrents(dxl_IDs_2, num_IDs_2, current_command2); // average of ~2300us to read position, velocity, current, and set current, for 3 motors; 300us to print 3 floats            
    // dxl_bus2.SetMultGoalCurrents(dxl_IDs_2, num_IDs_2, current_temp2); // average of ~2300us to read position, velocity, current, and set current, for 3 motors; 300us to print 3 floats            
   
}

void enter_menu_state(void){
    
    // TODO: make more useful menu
    pc.printf("\n\r\n\r\n\r");
    pc.printf(" Commands:\n\r");
    wait_us(10);
    pc.printf(" e - Enter CAN Mode\n\r");
    wait_us(10);
    pc.printf(" d - Print Motor Data\n\r"); // TODO: add Ticker and interrupt for this state
    wait_us(10);
    pc.printf(" esc - Exit to Menu\n\r\n\r");
    wait_us(10);
    }
    
    
/// Manage state machine with commands from serial terminal or configurator gui ///
/// Called when data received over serial ///
void serial_interrupt(void){
    serial_flag = true;
    serial_input = pc.getc();
}


/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANMessage *msg, int dxl_id, float p, float v, float t){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t*T_SCALE, -T_MAX, T_MAX, 12);
    msg->data[0] = dxl_id;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    }
    
/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANMessage msg){
    int p_int = (msg.data[0]<<8)|msg.data[1];
    int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
    int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
    int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
    int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];

    dxl_pos_des[msg.id-CAN_TX_DXL1] = uint_to_float(p_int, P_MIN, P_MAX, 16);
    dxl_vel_des[msg.id-CAN_TX_DXL1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
    dxl_kp[msg.id-CAN_TX_DXL1] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12)/KP_SCALE;
    dxl_kd[msg.id-CAN_TX_DXL1] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
    dxl_tff_des[msg.id-CAN_TX_DXL1] = uint_to_float(t_int, T_MIN, T_MAX, 12)/T_SCALE;

    }





// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {

    wait_us(10000);
    
    // Set up CAN
    // cansys.filter(CAN_ID , 0xFFF, CANStandard, 0); 
    txDxl1.id = CAN_RX_DXL1;
    txDxl1.len = 8;
    txDxl2.id = CAN_RX_DXL2;
    txDxl2.len = 8;
    txDxl3.id = CAN_RX_DXL3;
    txDxl3.len = 8;
    txDxl4.id = CAN_RX_DXL4;
    txDxl4.len = 8;
    txDxl5.id = CAN_RX_DXL5;
    txDxl5.len = 8;
    txDxl6.id = CAN_RX_DXL6;
    txDxl6.len = 8;
    rxMsg.len = 8;

    pc.attach(&serial_interrupt);        // attach serial interrupt


    // dynamixel
    pc.printf("Setting up Dynamixel bus 1.\n\r");
    // Enable dynamixels and set control mode...individual version
    for (int i=0; i<num_IDs_1; i++){
        dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x00);    
        dxl_bus1.SetRetDelTime(dxl_IDs_1[i],0x32); // 4us delay time?
//        dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
//        dxl_bus.SetControlMode(dxl_IDs[i], EXTEND_POS_CONTROL);
        dxl_bus1.SetControlMode(dxl_IDs_1[i], CURRENT_CONTROL);
        wait(0.1);    
        dxl_bus1.TurnOnLED(dxl_IDs_1[i], 0x01);
        // dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x01); // comment out to disable motors 
        wait(0.1);
    }

    pc.printf("Setting up Dynamixel bus 2.\n\r");
    for (int i=0; i<num_IDs_2; i++){
        dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x00);    
        dxl_bus2.SetRetDelTime(dxl_IDs_2[i],0x32); // 4us delay time?
//        dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
//        dxl_bus.SetControlMode(dxl_IDs[i], EXTEND_POS_CONTROL);
        dxl_bus2.SetControlMode(dxl_IDs_2[i], CURRENT_CONTROL);
        wait(0.1);    
        dxl_bus2.TurnOnLED(dxl_IDs_2[i], 0x01);
        // dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x01); // comment out to disable motors 
        wait(0.1);
    }

    pc.printf("Dynamixels are enabled.\n\r");
    wait_us(10000);
    motor_data.attach_us(&get_motor_data,10000); // 100Hz
    print_data.attach_us(&print_new_data,50000); // 20Hz
    // TODO: how fast can we push this?
    t.reset();
    t.start();
    t2.reset();
    t2.start();

    while(true){

        if (serial_flag){
            if(serial_input == 27){
                state = REST_MODE;

                // disable dynamixels
                for (int i=0; i<num_IDs_1; i++){
                    dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x00);
                    wait_us(100);
                }
                for (int i=0; i<num_IDs_2; i++){
                    dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x00);
                    wait_us(100);
                }
                
                // set current commands to zero
                current_command1[0] = 0;
                current_command1[1] = 0;
                current_command1[2] = 0;
                current_command2[0] = 0;
                current_command2[1] = 0;
                current_command2[2] = 0;

                // zero out stored command values
                for (int i=0; i<6; i++){
                    dxl_pos_des[i] = 0;
                    dxl_vel_des[i] = 0;
                    dxl_tff_des[i] = 0;
                    dxl_kp[i] = 0;
                    dxl_kd[i] = 0;
                }

                enter_menu_state();
            }
            if(state == REST_MODE){
                switch (serial_input){
                    case 'e':
                        state = CAN_MODE; // enabled for CAN
                        pc.printf("Entering CAN mode.\n\r");

                        // enable dynamixels
                        for (int i=0; i<num_IDs_1; i++){
                            dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x01); 
                            wait_us(100);
                        }
                        for (int i=0; i<num_IDs_2; i++){
                            dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x01);
                            wait_us(100);
                        }

                        break;

                    case 'd':
                        state = DATA_MODE;
                        pc.printf("Printing motor data.\n\r");
                        break;
                    }
            }  
            serial_flag = false;
        }

        if(cansys.read(rxMsg)){ // TODO: move this to a check_CAN function
            
            if((rxMsg.id>=CAN_TX_DXL1)&&(rxMsg.id<=CAN_TX_DXL6)){
                
                t2.reset();

                // Enable messages
                if((rxMsg.id==CAN_TX_DXL1) & (rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC)){
                    state = CAN_MODE;
                    pc.printf("Entering CAN mode.\n\r");
                    // enable dynamixels
                    for (int i=0; i<num_IDs_1; i++){
                        dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x01); 
                        wait_us(100);
                    }
                    for (int i=0; i<num_IDs_2; i++){
                        dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x01);
                        wait_us(100);
                    }
                }
                // Disable messages
                else if ((rxMsg.id==CAN_TX_DXL1) & (rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD)){
                    state = REST_MODE;
                    pc.printf("Entering rest mode.\n\r");
                    // disable dynamixels
                    for (int i=0; i<num_IDs_1; i++){
                        dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x00);
                        wait_us(100);
                    }
                    for (int i=0; i<num_IDs_2; i++){
                        dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x00);
                        wait_us(100);
                    }

                    // set current commands to zero
                    current_command1[0] = 0;
                    current_command1[1] = 0;
                    current_command1[2] = 0;
                    current_command2[0] = 0;
                    current_command2[1] = 0;
                    current_command2[2] = 0;

                    // zero out stored command values
                    for (int i=0; i<6; i++){
                        dxl_pos_des[i] = 0;
                        dxl_vel_des[i] = 0;
                        dxl_tff_des[i] = 0;
                        dxl_kp[i] = 0;
                        dxl_kd[i] = 0;
                    }

                } else { // motor control message
                    
                    if (state==CAN_MODE){
                        unpack_cmd(rxMsg); // unpack command

                        //pc.printf("Received: %d\n\r", rxMsg.id);

                        /*
                        if(rxMsg.id==CAN_TX_DXL1) { 
                            pack_reply(&txDxl1, dxl_IDs_1[0], currentPos1[0], currentVel1[0], Kt*currentCur1[0]); 
                            cansys.write(txDxl1);
                            wait_us(100);
                            }
                        if(rxMsg.id==CAN_TX_DXL2) { 
                            pack_reply(&txDxl2, dxl_IDs_1[1], currentPos1[1], currentVel1[1], Kt*currentCur1[1]); 
                            cansys.write(txDxl2);
                            wait_us(100);
                            
                            }
                        if(rxMsg.id==CAN_TX_DXL3) { 
                            pack_reply(&txDxl3, dxl_IDs_1[2], currentPos1[2], currentVel1[2], Kt*currentCur1[2]); 
                            cansys.write(txDxl3);
                            wait_us(100);
                            }
                        if(rxMsg.id==CAN_TX_DXL4) { 
                            pack_reply(&txDxl4, dxl_IDs_2[0], currentPos2[0], currentVel2[0], Kt*currentCur2[0]); 
                            cansys.write(txDxl4);
                            wait_us(100);
                            }
                        if(rxMsg.id==CAN_TX_DXL5) { 
                            pack_reply(&txDxl5, dxl_IDs_2[1], currentPos2[1], currentVel2[1], Kt*currentCur2[1]); 
                            cansys.write(txDxl5);
                            wait_us(100);
                            }
                        if(rxMsg.id==CAN_TX_DXL6) { 
                            pack_reply(&txDxl6, dxl_IDs_2[2], currentPos2[2], currentVel2[2], Kt*currentCur2[2]); 
                            cansys.write(txDxl6);
                            wait_us(100);
                            }  
                        */
                        
                    }

                }

                can_time = t2.read_us();

            }

        }

        if (motor_data_flag==1) {
            motor_data_flag = 0;
            t2.reset();




            if(cansys.read(rxMsg)){ // TODO: move this to a check_CAN function
            
                if((rxMsg.id>=CAN_TX_DXL1)&&(rxMsg.id<=CAN_TX_DXL6)){
                    
                    t2.reset();

                    // Enable messages
                    if((rxMsg.id==CAN_TX_DXL1) & (rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC)){
                        state = CAN_MODE;
                        pc.printf("Entering CAN mode.\n\r");
                        // enable dynamixels
                        for (int i=0; i<num_IDs_1; i++){
                            dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x01); 
                            wait_us(100);
                        }
                        for (int i=0; i<num_IDs_2; i++){
                            dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x01);
                            wait_us(100);
                        }
                    }
                    // Disable messages
                    else if ((rxMsg.id==CAN_TX_DXL1) & (rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD)){
                        state = REST_MODE;
                        pc.printf("Entering rest mode.\n\r");
                        // disable dynamixels
                        for (int i=0; i<num_IDs_1; i++){
                            dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x00);
                            wait_us(100);
                        }
                        for (int i=0; i<num_IDs_2; i++){
                            dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x00);
                            wait_us(100);
                        }

                        // set current commands to zero
                        current_command1[0] = 0;
                        current_command1[1] = 0;
                        current_command1[2] = 0;
                        current_command2[0] = 0;
                        current_command2[1] = 0;
                        current_command2[2] = 0;

                        // zero out stored command values
                        for (int i=0; i<6; i++){
                            dxl_pos_des[i] = 0;
                            dxl_vel_des[i] = 0;
                            dxl_tff_des[i] = 0;
                            dxl_kp[i] = 0;
                            dxl_kd[i] = 0;
                        }

                    } else { // motor control message
                        
                        if (state==CAN_MODE){
                            unpack_cmd(rxMsg); // unpack command

                            //pc.printf("Received: %d\n\r", rxMsg.id);

                            /*
                            if(rxMsg.id==CAN_TX_DXL1) { 
                                pack_reply(&txDxl1, dxl_IDs_1[0], currentPos1[0], currentVel1[0], Kt*currentCur1[0]); 
                                cansys.write(txDxl1);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL2) { 
                                pack_reply(&txDxl2, dxl_IDs_1[1], currentPos1[1], currentVel1[1], Kt*currentCur1[1]); 
                                cansys.write(txDxl2);
                                wait_us(100);
                                
                                }
                            if(rxMsg.id==CAN_TX_DXL3) { 
                                pack_reply(&txDxl3, dxl_IDs_1[2], currentPos1[2], currentVel1[2], Kt*currentCur1[2]); 
                                cansys.write(txDxl3);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL4) { 
                                pack_reply(&txDxl4, dxl_IDs_2[0], currentPos2[0], currentVel2[0], Kt*currentCur2[0]); 
                                cansys.write(txDxl4);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL5) { 
                                pack_reply(&txDxl5, dxl_IDs_2[1], currentPos2[1], currentVel2[1], Kt*currentCur2[1]); 
                                cansys.write(txDxl5);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL6) { 
                                pack_reply(&txDxl6, dxl_IDs_2[2], currentPos2[2], currentVel2[2], Kt*currentCur2[2]); 
                                cansys.write(txDxl6);
                                wait_us(100);
                                }  
                            */
                            
                        }

                    }

                    can_time = t2.read_us();

                }

            }





            updateBus1();

            if(cansys.read(rxMsg)){ // TODO: move this to a check_CAN function
            
                if((rxMsg.id>=CAN_TX_DXL1)&&(rxMsg.id<=CAN_TX_DXL6)){
                    
                    t2.reset();

                    // Enable messages
                    if((rxMsg.id==CAN_TX_DXL1) & (rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC)){
                        state = CAN_MODE;
                        pc.printf("Entering CAN mode.\n\r");
                        // enable dynamixels
                        for (int i=0; i<num_IDs_1; i++){
                            dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x01); 
                            wait_us(100);
                        }
                        for (int i=0; i<num_IDs_2; i++){
                            dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x01);
                            wait_us(100);
                        }
                    }
                    // Disable messages
                    else if ((rxMsg.id==CAN_TX_DXL1) & (rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD)){
                        state = REST_MODE;
                        pc.printf("Entering rest mode.\n\r");
                        // disable dynamixels
                        for (int i=0; i<num_IDs_1; i++){
                            dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x00);
                            wait_us(100);
                        }
                        for (int i=0; i<num_IDs_2; i++){
                            dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x00);
                            wait_us(100);
                        }

                        // set current commands to zero
                        current_command1[0] = 0;
                        current_command1[1] = 0;
                        current_command1[2] = 0;
                        current_command2[0] = 0;
                        current_command2[1] = 0;
                        current_command2[2] = 0;

                        // zero out stored command values
                        for (int i=0; i<6; i++){
                            dxl_pos_des[i] = 0;
                            dxl_vel_des[i] = 0;
                            dxl_tff_des[i] = 0;
                            dxl_kp[i] = 0;
                            dxl_kd[i] = 0;
                        }

                    } else { // motor control message
                        
                        if (state==CAN_MODE){
                            unpack_cmd(rxMsg); // unpack command

                            //pc.printf("Received: %d\n\r", rxMsg.id);

                            /*
                            if(rxMsg.id==CAN_TX_DXL1) { 
                                pack_reply(&txDxl1, dxl_IDs_1[0], currentPos1[0], currentVel1[0], Kt*currentCur1[0]); 
                                cansys.write(txDxl1);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL2) { 
                                pack_reply(&txDxl2, dxl_IDs_1[1], currentPos1[1], currentVel1[1], Kt*currentCur1[1]); 
                                cansys.write(txDxl2);
                                wait_us(100);
                                
                                }
                            if(rxMsg.id==CAN_TX_DXL3) { 
                                pack_reply(&txDxl3, dxl_IDs_1[2], currentPos1[2], currentVel1[2], Kt*currentCur1[2]); 
                                cansys.write(txDxl3);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL4) { 
                                pack_reply(&txDxl4, dxl_IDs_2[0], currentPos2[0], currentVel2[0], Kt*currentCur2[0]); 
                                cansys.write(txDxl4);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL5) { 
                                pack_reply(&txDxl5, dxl_IDs_2[1], currentPos2[1], currentVel2[1], Kt*currentCur2[1]); 
                                cansys.write(txDxl5);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL6) { 
                                pack_reply(&txDxl6, dxl_IDs_2[2], currentPos2[2], currentVel2[2], Kt*currentCur2[2]); 
                                cansys.write(txDxl6);
                                wait_us(100);
                                }  
                            */
                            
                        }

                    }

                    can_time = t2.read_us();

                }

            }






            updateBus2();

            // check CAN messages
            if(cansys.read(rxMsg)){ // TODO: move this to a check_CAN function
            
                if((rxMsg.id>=CAN_TX_DXL1)&&(rxMsg.id<=CAN_TX_DXL6)){
                    
                    t2.reset();

                    // Enable messages
                    if((rxMsg.id==CAN_TX_DXL1) & (rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC)){
                        state = CAN_MODE;
                        pc.printf("Entering CAN mode.\n\r");
                        // enable dynamixels
                        for (int i=0; i<num_IDs_1; i++){
                            dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x01); 
                            wait_us(100);
                        }
                        for (int i=0; i<num_IDs_2; i++){
                            dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x01);
                            wait_us(100);
                        }
                    }
                    // Disable messages
                    else if ((rxMsg.id==CAN_TX_DXL1) & (rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD)){
                        state = REST_MODE;
                        pc.printf("Entering rest mode.\n\r");
                        // disable dynamixels
                        for (int i=0; i<num_IDs_1; i++){
                            dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x00);
                            wait_us(100);
                        }
                        for (int i=0; i<num_IDs_2; i++){
                            dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x00);
                            wait_us(100);
                        }

                        // set current commands to zero
                        current_command1[0] = 0;
                        current_command1[1] = 0;
                        current_command1[2] = 0;
                        current_command2[0] = 0;
                        current_command2[1] = 0;
                        current_command2[2] = 0;

                        // zero out stored command values
                        for (int i=0; i<6; i++){
                            dxl_pos_des[i] = 0;
                            dxl_vel_des[i] = 0;
                            dxl_tff_des[i] = 0;
                            dxl_kp[i] = 0;
                            dxl_kd[i] = 0;
                        }

                    } else { // motor control message
                        
                        if (state==CAN_MODE){
                            unpack_cmd(rxMsg); // unpack command

                            //pc.printf("Received: %d\n\r", rxMsg.id);

                            /*
                            if(rxMsg.id==CAN_TX_DXL1) { 
                                pack_reply(&txDxl1, dxl_IDs_1[0], currentPos1[0], currentVel1[0], Kt*currentCur1[0]); 
                                cansys.write(txDxl1);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL2) { 
                                pack_reply(&txDxl2, dxl_IDs_1[1], currentPos1[1], currentVel1[1], Kt*currentCur1[1]); 
                                cansys.write(txDxl2);
                                wait_us(100);
                                
                                }
                            if(rxMsg.id==CAN_TX_DXL3) { 
                                pack_reply(&txDxl3, dxl_IDs_1[2], currentPos1[2], currentVel1[2], Kt*currentCur1[2]); 
                                cansys.write(txDxl3);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL4) { 
                                pack_reply(&txDxl4, dxl_IDs_2[0], currentPos2[0], currentVel2[0], Kt*currentCur2[0]); 
                                cansys.write(txDxl4);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL5) { 
                                pack_reply(&txDxl5, dxl_IDs_2[1], currentPos2[1], currentVel2[1], Kt*currentCur2[1]); 
                                cansys.write(txDxl5);
                                wait_us(100);
                                }
                            if(rxMsg.id==CAN_TX_DXL6) { 
                                pack_reply(&txDxl6, dxl_IDs_2[2], currentPos2[2], currentVel2[2], Kt*currentCur2[2]); 
                                cansys.write(txDxl6);
                                wait_us(100);
                                }  
                            */
                            
                        }

                    }

                    can_time = t2.read_us();

                }

            }

            if (state==CAN_MODE){
                pack_reply(&txDxl1, dxl_IDs_1[0], currentPos1[0], currentVel1[0], Kt*currentCur1[0]); 
                cansys.write(txDxl1);
                wait_us(100);
                pack_reply(&txDxl2, dxl_IDs_1[1], currentPos1[1], currentVel1[1], Kt*currentCur1[1]); 
                cansys.write(txDxl2);
                wait_us(100);
                pack_reply(&txDxl3, dxl_IDs_1[2], currentPos1[2], currentVel1[2], Kt*currentCur1[2]); 
                cansys.write(txDxl3);
                wait_us(100);
                pack_reply(&txDxl4, dxl_IDs_2[0], currentPos2[0], currentVel2[0], Kt*currentCur2[0]); 
                cansys.write(txDxl4);
                wait_us(100);
                pack_reply(&txDxl5, dxl_IDs_2[1], currentPos2[1], currentVel2[1], Kt*currentCur2[1]); 
                cansys.write(txDxl5);
                wait_us(100);
                pack_reply(&txDxl6, dxl_IDs_2[2], currentPos2[2], currentVel2[2], Kt*currentCur2[2]); 
                cansys.write(txDxl6);
                wait_us(100);
            }


            dxl_time = t2.read_us();            
        }

        if ((state==DATA_MODE)&&(print_data_flag==1)) { // print at slower frequency
        // if ((state==CAN_MODE)&&(print_data_flag==1)){
                print_data_flag = 0;
                // print data 
                pc.printf("%.3f, %d, %d,  %2.3f, %2.3f, %2.3f,  %2.3f, %2.3f, %2.3f\n\r",
                    t.read(), dxl_time, can_time, currentPos1[0], currentPos1[1], currentPos1[2], 
                    currentPos2[0], currentPos2[1], currentPos2[2]);
        }



    } 
    
}
