/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include "mbed.h"
#include "dynamixel_XL330.h"
 
// Initialize serial port
Serial pc(PA_9, PA_10, 921600);
 
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
 
// Variables for  dynamixel bus
uint8_t dxl_IDs_1[] = {0x01, 0x02, 0x03};
const int num_IDs_1 = 3;
uint8_t dxl_IDs_2[] = {0x0B, 0x0C, 0x0D};
const int num_IDs_2 = 3;
uint8_t dxl_IDs_3[] = {0x01, 0x02, 0x03};
const int num_IDs_3 = 0;
uint8_t dxl_IDs_4[] = {0x01, 0x02, 0x03};
const int num_IDs_4 = 0;
  

// Set up dynamixel
XL330_bus dxl_bus1(1000000, bus_1_tx, bus_1_rx, bus_1_rts); // baud, tx, rx, rts
XL330_bus dxl_bus2(1000000, bus_2_tx, bus_2_rx, bus_2_rts); // baud, tx, rx, rts
XL330_bus dxl_bus3(1000000, bus_3_tx, bus_3_rx, bus_3_rts); // baud, tx, rx, rts
XL330_bus dxl_bus4(1000000, bus_4_tx, bus_4_rx, bus_4_rts); // baud, tx, rx, rts

//Variables to store values
float currentPos1[num_IDs_1];
float currentPos2[num_IDs_2];
float currentPos3[num_IDs_3];
float currentPos4[num_IDs_4];
float currentVel1[num_IDs_1];
float currentVel2[num_IDs_2];
float currentVel3[num_IDs_3];
float currentVel4[num_IDs_4];
float currentCur1[num_IDs_1];
float currentCur2[num_IDs_2];
float currentCur3[num_IDs_3];
float currentCur4[num_IDs_4];

//Variables to store commands
float currentCommand1[num_IDs_1];
float currentCommand2[num_IDs_2];
float currentCommand3[num_IDs_3];
float currentCommand4[num_IDs_4];

Thread thread1;
Thread thread2;
Thread thread3;
Thread thread4;
Mutex mutex; 
Timer t_test;
int time_pass; 


void updateBus1(){
    // Controller variables
    int32_t dxl_position[3];
    int32_t dxl_velocity[3];
    int16_t dxl_current[3];
    float desired_current[3]; // in A!
    uint16_t current_command[3]; // in mA!
    float dxl_offsets[] = {2.3455f, 0.7440f, 5.5791f};
    float KdJ = 0.8f; // bonus joint damping
    float Kt = 0.60f/1.74f; // effective torque constant, Nm/A
    float Kt_inv = 1.0f/Kt;
    float pulse_to_rad = (2.0f*3.14159f)/4096.0f; // = 0.001534
    float rpm_to_rads = (0.229f*2.0f*3.14159f)/60.0f; // = 0.0239
    float current_limit = 1.75f; // in A, max is 1.75
    float torque_limit = 0.40f; // in Nm, stall is 0.6Nm at 6V
    // current is in mA so no conversion constant is necessary
    t_test.start();
            // Get dynamixel states...sync version
            time_pass =t_test.read_us();
            t_test.reset();
            dxl_bus1.GetMultPositions(dxl_position, dxl_IDs_1, num_IDs_1);
            dxl_bus1.GetMultVelocities(dxl_velocity, dxl_IDs_1, num_IDs_1);
            dxl_bus1.GetMultCurrents(dxl_current, dxl_IDs_1, num_IDs_1);
            
            // convert states
            for(int i=0; i<num_IDs_1; i++){
                currentPos1[i] = (pulse_to_rad*(float)dxl_position[i])-dxl_offsets[i];
                currentVel1[i] = rpm_to_rads*(float)dxl_velocity[i];
                currentCur1[i] = 0.001f*(float)dxl_current[i];
            }
            
            // could limit control signal (non tau_ff) too to prevent some oscillations due to high velocities
            desired_current[0] = 0.0;
            desired_current[1] =  0.0;
            desired_current[2] = 0.0;
            // check against current limit and convert back to uint16_t to send to motors
            desired_current[0] = fmaxf(fminf(desired_current[0],current_limit),-current_limit);
            desired_current[1] = fmaxf(fminf(desired_current[1],current_limit),-current_limit);
            desired_current[2] = fmaxf(fminf(desired_current[2],current_limit),-current_limit);
            current_command[0] = (int16_t)(desired_current[0]*1000.0f); //(0.0f); // need to convert desired_current to mA before type-casting
            current_command[1] = (int16_t)(desired_current[1]*1000.0f); //(0.0f);
            current_command[2] = (int16_t)(desired_current[2]*1000.0f); //(0.0f);
            
            // send commands
            dxl_bus1.SetMultGoalCurrents(dxl_IDs_1, num_IDs_1, current_command); // average of ~2300us to read position, velocity, current, and set current, for 3 motors; 300us to print 3 floats            
 
    
}
void updateBus2(){
// Controller variables
    int32_t dxl_position[3];
    int32_t dxl_velocity[3];
    int16_t dxl_current[3];
    float desired_current[3]; // in A!
    uint16_t current_command[3]; // in mA!
    float dxl_offsets[] = {2.3455f, 0.7440f, 5.5791f};
    float KdJ = 0.8f; // bonus joint damping
    float Kt = 0.60f/1.74f; // effective torque constant, Nm/A
    float Kt_inv = 1.0f/Kt;
    float pulse_to_rad = (2.0f*3.14159f)/4096.0f; // = 0.001534
    float rpm_to_rads = (0.229f*2.0f*3.14159f)/60.0f; // = 0.0239
    float current_limit = 1.75f; // in A, max is 1.75
    float torque_limit = 0.40f; // in Nm, stall is 0.6Nm at 6V
    // current is in mA so no conversion constant is necessary
            // Get dynamixel states...sync version
            dxl_bus2.GetMultPositions(dxl_position, dxl_IDs_2, num_IDs_2);
            dxl_bus2.GetMultVelocities(dxl_velocity, dxl_IDs_2, num_IDs_2);
            dxl_bus2.GetMultCurrents(dxl_current, dxl_IDs_2, num_IDs_2);
            
            // convert states
            for(int i=0; i<num_IDs_1; i++){
                currentPos2[i] = (pulse_to_rad*(float)dxl_position[i])-dxl_offsets[i];
                currentVel2[i] = rpm_to_rads*(float)dxl_velocity[i];
                currentCur2[i] = 0.001f*(float)dxl_current[i];
            }
            
            // could limit control signal (non tau_ff) too to prevent some oscillations due to high velocities
            desired_current[0] = 0.0;
            desired_current[1] =  0.0;
            desired_current[2] = 0.0;
            // check against current limit and convert back to uint16_t to send to motors
            desired_current[0] = fmaxf(fminf(desired_current[0],current_limit),-current_limit);
            desired_current[1] = fmaxf(fminf(desired_current[1],current_limit),-current_limit);
            desired_current[2] = fmaxf(fminf(desired_current[2],current_limit),-current_limit);
            current_command[0] = (int16_t)(desired_current[0]*1000.0f); //(0.0f); // need to convert desired_current to mA before type-casting
            current_command[1] = (int16_t)(desired_current[1]*1000.0f); //(0.0f);
            current_command[2] = (int16_t)(desired_current[2]*1000.0f); //(0.0f);
            
            // send commands
            dxl_bus2.SetMultGoalCurrents(dxl_IDs_2, num_IDs_2, current_command); // average of ~2300us to read position, velocity, current, and set current, for 3 motors; 300us to print 3 floats            
    
}
void updateBus3(){
// Controller variables
    int32_t dxl_position[3];
    int32_t dxl_velocity[3];
    int16_t dxl_current[3];
    float desired_current[3]; // in A!
    uint16_t current_command[3]; // in mA!
    float dxl_offsets[] = {2.3455f, 0.7440f, 5.5791f};
    float KdJ = 0.8f; // bonus joint damping
    float Kt = 0.60f/1.74f; // effective torque constant, Nm/A
    float Kt_inv = 1.0f/Kt;
    float pulse_to_rad = (2.0f*3.14159f)/4096.0f; // = 0.001534
    float rpm_to_rads = (0.229f*2.0f*3.14159f)/60.0f; // = 0.0239
    float current_limit = 1.75f; // in A, max is 1.75
    float torque_limit = 0.40f; // in Nm, stall is 0.6Nm at 6V
    // current is in mA so no conversion constant is necessary
    while (true) {
            // Get dynamixel states...sync version
            dxl_bus3.GetMultPositions(dxl_position, dxl_IDs_3, num_IDs_3);
            dxl_bus3.GetMultVelocities(dxl_velocity, dxl_IDs_3, num_IDs_3);
            dxl_bus3.GetMultCurrents(dxl_current, dxl_IDs_3, num_IDs_3);
            
            // convert states
            for(int i=0; i<num_IDs_1; i++){
                currentPos3[i] = (pulse_to_rad*(float)dxl_position[i])-dxl_offsets[i];
                currentVel3[i] = rpm_to_rads*(float)dxl_velocity[i];
                currentCur3[i] = 0.001f*(float)dxl_current[i];
            }
            
            // could limit control signal (non tau_ff) too to prevent some oscillations due to high velocities
            desired_current[0] = 0.0;
            desired_current[1] =  0.0;
            desired_current[2] = 0.0;
            // check against current limit and convert back to uint16_t to send to motors
            desired_current[0] = fmaxf(fminf(desired_current[0],current_limit),-current_limit);
            desired_current[1] = fmaxf(fminf(desired_current[1],current_limit),-current_limit);
            desired_current[2] = fmaxf(fminf(desired_current[2],current_limit),-current_limit);
            current_command[0] = (int16_t)(desired_current[0]*1000.0f); //(0.0f); // need to convert desired_current to mA before type-casting
            current_command[1] = (int16_t)(desired_current[1]*1000.0f); //(0.0f);
            current_command[2] = (int16_t)(desired_current[2]*1000.0f); //(0.0f);
            
            // send commands
            dxl_bus3.SetMultGoalCurrents(dxl_IDs_3, num_IDs_3, current_command); // average of ~2300us to read position, velocity, current, and set current, for 3 motors; 300us to print 3 floats            
    }
}
void updateBus4(){
// Controller variables
    int32_t dxl_position[3];
    int32_t dxl_velocity[3];
    int16_t dxl_current[3];
    float desired_current[3]; // in A!
    uint16_t current_command[3]; // in mA!
    float dxl_offsets[] = {2.3455f, 0.7440f, 5.5791f};
    float KdJ = 0.8f; // bonus joint damping
    float Kt = 0.60f/1.74f; // effective torque constant, Nm/A
    float Kt_inv = 1.0f/Kt;
    float pulse_to_rad = (2.0f*3.14159f)/4096.0f; // = 0.001534
    float rpm_to_rads = (0.229f*2.0f*3.14159f)/60.0f; // = 0.0239
    float current_limit = 1.75f; // in A, max is 1.75
    float torque_limit = 0.40f; // in Nm, stall is 0.6Nm at 6V
    // current is in mA so no conversion constant is necessary
    while (true) {
            // Get dynamixel states...sync version
            dxl_bus4.GetMultPositions(dxl_position, dxl_IDs_4, num_IDs_4);
            dxl_bus4.GetMultVelocities(dxl_velocity, dxl_IDs_4, num_IDs_4);
            dxl_bus4.GetMultCurrents(dxl_current, dxl_IDs_4, num_IDs_4);
            
            // convert states
            for(int i=0; i<num_IDs_1; i++){
                currentPos4[i] = (pulse_to_rad*(float)dxl_position[i])-dxl_offsets[i];
                currentVel4[i] = rpm_to_rads*(float)dxl_velocity[i];
                currentCur4[i] = 0.001f*(float)dxl_current[i];
            }
            
            // could limit control signal (non tau_ff) too to prevent some oscillations due to high velocities
            desired_current[0] = 0.0;
            desired_current[1] =  0.0;
            desired_current[2] = 0.0;
            // check against current limit and convert back to uint16_t to send to motors
            desired_current[0] = fmaxf(fminf(desired_current[0],current_limit),-current_limit);
            desired_current[1] = fmaxf(fminf(desired_current[1],current_limit),-current_limit);
            desired_current[2] = fmaxf(fminf(desired_current[2],current_limit),-current_limit);
            current_command[0] = (int16_t)(desired_current[0]*1000.0f); //(0.0f); // need to convert desired_current to mA before type-casting
            current_command[1] = (int16_t)(desired_current[1]*1000.0f); //(0.0f);
            current_command[2] = (int16_t)(desired_current[2]*1000.0f); //(0.0f);
            
            // send commands
            dxl_bus4.SetMultGoalCurrents(dxl_IDs_4, num_IDs_4, current_command); // average of ~2300us to read position, velocity, current, and set current, for 3 motors; 300us to print 3 floats            
    }
}

// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {
    // initialize force sensor   
    wait(0.5); 
    
    // dynamixel
    pc.printf("Setting up Dynamixels. 1\n\r");

    // Enable dynamixels and set control mode...individual version
    for (int i=0; i<num_IDs_1; i++){
        dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x00);    
        dxl_bus1.SetRetDelTime(dxl_IDs_1[i],0x32); // 4us delay time?
//        dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
//        dxl_bus.SetControlMode(dxl_IDs[i], EXTEND_POS_CONTROL);
        dxl_bus1.SetControlMode(dxl_IDs_1[i], CURRENT_CONTROL);
        wait(0.1);    
        dxl_bus1.TurnOnLED(dxl_IDs_1[i], 0x01);
        dxl_bus1.SetTorqueEn(dxl_IDs_1[i],0x01); // comment out to disable motors 
        wait(0.1);
    }
    pc.printf("Setting up Dynamixels. 2\n\r");

    for (int i=0; i<num_IDs_2; i++){
        dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x00);    
        dxl_bus2.SetRetDelTime(dxl_IDs_2[i],0x32); // 4us delay time?
//        dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
//        dxl_bus.SetControlMode(dxl_IDs[i], EXTEND_POS_CONTROL);
        dxl_bus2.SetControlMode(dxl_IDs_2[i], CURRENT_CONTROL);
        wait(0.1);    
        dxl_bus2.TurnOnLED(dxl_IDs_2[i], 0x01);
        dxl_bus2.SetTorqueEn(dxl_IDs_2[i],0x01); // comment out to disable motors 
        wait(0.1);
    }
    pc.printf("Setting up Dynamixels. 3\n\r");

    for (int i=0; i<num_IDs_3; i++){
        dxl_bus3.SetTorqueEn(dxl_IDs_3[i],0x00);    
        dxl_bus3.SetRetDelTime(dxl_IDs_3[i],0x32); // 4us delay time?
//        dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
//        dxl_bus.SetControlMode(dxl_IDs[i], EXTEND_POS_CONTROL);
        dxl_bus3.SetControlMode(dxl_IDs_3[i], CURRENT_CONTROL);
        wait(0.1);    
        dxl_bus3.TurnOnLED(dxl_IDs_3[i], 0x01);
        dxl_bus3.SetTorqueEn(dxl_IDs_3[i],0x01); // comment out to disable motors 
        wait(0.1);
    }    
    pc.printf("Setting up Dynamixels. 4\n\r");

    for (int i=0; i<num_IDs_4; i++){
        dxl_bus4.SetTorqueEn(dxl_IDs_4[i],0x00);    
        dxl_bus4.SetRetDelTime(dxl_IDs_4[i],0x32); // 4us delay time?
//        dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
//        dxl_bus.SetControlMode(dxl_IDs[i], EXTEND_POS_CONTROL);
        dxl_bus4.SetControlMode(dxl_IDs_4[i], CURRENT_CONTROL);
        wait(0.1);    
        dxl_bus4.TurnOnLED(dxl_IDs_4[i], 0x01);
        dxl_bus4.SetTorqueEn(dxl_IDs_4[i],0x01); // comment out to disable motors 
        wait(0.1);
    }

    pc.printf("Dynamixels are enabled.\n\r");
    wait(1);
    
    // if (num_IDs_1 != 0){
    //     thread1.start(updateBus1);
    // }
    // if (num_IDs_2 != 0){
    //     thread2.start(updateBus2);
    // }
    // if (num_IDs_3 != 0){
    //     thread3.start(updateBus3);
    // }    
    // if (num_IDs_4 != 0){
    //     thread4.start(updateBus4);
    // }

    while(true){
                    // print data from contact following tests
                    updateBus1();
                    updateBus2();
        pc.printf("Loop_time: %i\r\n", time_pass);
        pc.printf("Pos: %f, %f, %f\r\n", currentPos1[0],currentPos1[1],currentPos1[2]);
        pc.printf("Pos: %f, %f, %f\r\n", currentPos2[0],currentPos2[1],currentPos2[2]);
 
    } // from while(true)
    
    // Disable dynamixel
    //dxl_bus.SetTorqueEn(dxl_IDs[0], 0x00);
    //dxl_bus.SetTorqueEn(dxl_IDs[1], 0x00);
    //dxl_bus.SetTorqueEn(dxl_IDs[2], 0x00);
 
    
}
