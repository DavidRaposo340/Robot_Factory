/* Copyright (c) 2021  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include "channels.h"
#include "robot.h"
#include "PID.h"
#include "proj_types.h"
#include "SPI.h"
#include <TimerOne.h>

byte UsingSimulator;
byte go;


#include <Wire.h>

void sim_loop(void);
void real_loop(void);

void process_serial_packet(char channel, uint32_t value, channels_t& obj);


channels_t serial_channels, udp_channels;

//hw_timer_t * timer_enc = NULL;

#define ENC1_A 6
#define ENC1_B 7

#define ENC2_A 4
#define ENC2_B 5

robot_t robot;

const int M1_dir_pin1 = 2;
const int M1_dir_pin2 = 4;
const int M2_dir_pin1 = 8;
const int M2_dir_pin2 = 12;

const int M1_PWM_pin = TIMER1_A_PIN;
const int M2_PWM_pin = TIMER1_B_PIN;

volatile uint16_t encoder_delta[2];

void setSolenoidPWM(int new_PWM);

#define TOUCHSW_pin 26

void setSolenoidState()
{
  if (robot.solenoid_state) setSolenoidPWM(177);
  else setSolenoidPWM(0);
}

byte readTouchSwitch(void)
{
  return !digitalRead(TOUCHSW_pin);
}

void control(robot_t& robot);

volatile int encoder1_pos = 0;
volatile int encoder2_pos = 0;

void EncodersInterrupt(void)
{
  byte b, new_state;
  static int8_t encoder_table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  static byte encoder_state[2];

  b = PIND;

  new_state = (b >> 6) & 0x03;
  encoder_delta[0] += encoder_table[encoder_state[0] | new_state];
  encoder_state[0] = new_state << 2;

  new_state = (b >> 4) & 0x03;
  encoder_delta[1] += encoder_table[encoder_state[1] | new_state];
  encoder_state[1] = new_state << 2;
}

//uint32_t delta, current, previous, interval = 40000UL;
schedule_t schedule;
//TODO
static void set_M1_PWM(int new_PWM)
{
  int pwm;

  if (new_PWM == 0) {
    pwm = 0;
	digitalWrite(M1_dir_pin1, 0);
	digitalWrite(M1_dir_pin2, 0);  
  } else if (new_PWM > 0) {
    pwm = new_PWM;
	digitalWrite(M1_dir_pin1, 0);
	digitalWrite(M1_dir_pin2, 1);

  } else if (new_PWM < 0) {
    pwm = -new_PWM;
	digitalWrite(M1_dir_pin1, 1);
	digitalWrite(M1_dir_pin2, 0);
  }
  Timer1.setPwmDuty(TIMER1_A_PIN, pwm);
}

static void set_M2_PWM(int new_PWM)
{
  int pwm;
  
  if (new_PWM == 0) {
    pwm = 0;
	digitalWrite(M2_dir_pin1, 0);
	digitalWrite(M2_dir_pin2, 0);
  } else if (new_PWM > 0) {
    pwm = new_PWM;
	digitalWrite(M2_dir_pin1, 0);
	digitalWrite(M2_dir_pin2, 1);
  } else if (new_PWM < 0) {
    pwm = -new_PWM;
	digitalWrite(M2_dir_pin1, 1);
	digitalWrite(M2_dir_pin2, 0);
  }
  Timer1.setPwmDuty(TIMER1_B_PIN, pwm);
}

void setMotorsPWM(int PWM1, int PWM2)
{
  set_M1_PWM(PWM1);
  set_M2_PWM(PWM2);
}


void setSolenoidPWM(int new_PWM)
{
  // TODO
  /*int PWM_max = 177;
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;*/

  //TODO
  //setMotorPWM(M3, new_PWM);
}


void serial_write(uint8_t b)
{
  Serial.write(b);
}




void setup()
{
  // Set the pins as input or output as needed
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  pinMode(TOUCHSW_pin, INPUT_PULLUP);

  pinMode(M1_dir_pin1, OUTPUT);
  pinMode(M1_dir_pin2, OUTPUT);
  pinMode(M2_dir_pin1, OUTPUT);
  pinMode(M2_dir_pin2, OUTPUT);

  UsingSimulator = 0;
  
  schedule.interval = 40000UL;
  robot.dt = 1e-6 * schedule.interval; // in seconds

  Serial.begin(115200);
  serial_channels.init(process_serial_packet, serial_write);
  Serial.println("Init Begin!");

  Timer1.attachInterrupt(EncodersInterrupt);
  Timer1.initialize(50);
  Timer1.pwm(TIMER1_A_PIN, 0);
  Timer1.pwm(TIMER1_B_PIN, 0);
  
  if (!UsingSimulator) {
    Wire.begin();
    //Wire.setClock(400000UL);
  
  }

  // Timer interrupt for the encoder
  //timerAttachInterrupt(timer_enc, &EncodersInterrupt, true);
 
 
 
  // Robot Parameters
  //robot.b = 0.137 / 2;
  robot.r1 = 0.07 / 2;
  robot.r2 = 0.07 / 2;

  robot.dv_max = 5 * robot.dt;  // Linear velocity change per control period
  robot.dw_max = 10 * robot.dt; // Angular velocity change per control period

  robot.state = 0;
  robot.solenoid_state = 1;

  robot.PWM_1_req=50;    ///Adicionado****
  robot.PWM_2_req=50;

  robot.setState(202);

}


void readEncoders(void)
{
  cli();  // Must be done with the interrupts disabled
  robot.enc1 = encoder1_pos;
  robot.enc2 = encoder2_pos;

  encoder1_pos = 0;
  encoder2_pos = 0;
  sei();

  robot.Senc1 += robot.enc1;
  robot.Senc2 += robot.enc2;
}



void process_serial_packet(char channel, uint32_t value, channels_t& obj)
{
  channels_u val;
  val.u = value;

  if (channel == 'o') {  // Clear acumulated encoder
    robot.Senc1 = 0;
    robot.Senc2 = 0;
  
  } else if (channel == 'R')  { 
    robot.enc1 = int16_t((value >> 16) & 0xFFFF);
    robot.enc2 = int16_t((value >> 00) & 0xFFFF);

    robot.Senc1 += robot.enc1;
    robot.Senc2 += robot.enc2;

  } else if (channel == 'I')  {   // IR Sensors + Touch
    robot.TouchSwitch = ((value >> 31) & 1);  
 
  } else if (channel == 'G')  {  // Control
    go = 1;

  } else if (channel == 'v') {  // Set robot linear speed reference
    robot.v_req = val.f;   

  } else if (channel == 'w') {  // Set robot angular speed reference
    robot.w_req = val.f;   

  } else if (channel == 'x') {  // Set robot x position
    robot.x = val.f;   

  } else if (channel == 'y') {  // Set robot y position
    robot.y = val.f;   

  } else if (channel == 't') {  // Set robot theta angle
    robot.theta = val.f;   

  } else if (channel == 's') {  // Set robot state
    robot.setState(value);

  } else if (channel == 'O') {  // Set Requested PWM
    robot.PWM_1_req = int16_t((value >> 16) & 0xFFFF);
    robot.PWM_2_req = int16_t((value >> 0) & 0xFFFF);

  } else if (channel == 'T') {  // Set general parameters T1 and T2
    robot.T1 = (value >> 16) & 0xFFFF;
    robot.T2 = (value >> 0) & 0xFFFF;

  } else if (channel == 'p') { // Set PID parameter
    robot.PID1.Kp = val.f;
    robot.PID2.Kp = val.f;

  } else if (channel == 'i') { // Set PID parameter
    if (fabs(val.f) > 1e-2 ) {
      robot.PID1.Se = robot.PID1.Se * robot.PID1.Ki / val.f;
      robot.PID2.Se = robot.PID2.Se * robot.PID2.Ki / val.f;
    }
    robot.PID1.Ki = val.f;
    robot.PID2.Ki = val.f;

  } else if (channel == 'm') { // Set PID parameter
    robot.PID1.Kd = val.f;
    robot.PID2.Kd = val.f;

  } else if (channel == 'n') { // Set PID parameter
    robot.PID1.Kf = val.f;
    robot.PID2.Kf = val.f;

  } else if (channel == 'z') { // Set Solenoid
    robot.solenoid_state = value;

  }


}

void loop(void)
{
  if (UsingSimulator) {
    sim_loop();
  } else {
    real_loop();
  }
}


void serial_print_format(int value, byte space)
{
  byte b, c;
  b = Serial.print(value);
  for (c = 0; c < space - b; c++) {
     Serial.print(" ");
  }
}

/*
void send_udp_channels(void)
{
  udp_channels.send('I', encodeIRSensors() | (robot.solenoid_state << 30) | (robot.TouchSwitch << 31));
  udp_channels.send('U', robot.PWM_1, robot.PWM_2);
  udp_channels.send('R', robot.enc1, robot.enc2);
  udp_channels.send('S', robot.Senc1, robot.Senc2);
  udp_channels.send('T', robot.T1, robot.T2);
  
  udp_channels.sendFloat('v', robot.ve);
  udp_channels.sendFloat('w', robot.we);
  
  udp_channels.sendFloat('x', robot.x);
  udp_channels.sendFloat('y', robot.y);
  //udp_channels.sendFloat('x', IRLine.pos_right);
  //udp_channels.sendFloat('y', IRLine.pos_left);
  udp_channels.sendFloat('t', robot.theta);

  udp_channels.sendFloat('p', robot.PID1.Kp);
  udp_channels.sendFloat('i', robot.PID1.Ki);
  udp_channels.sendFloat('m', robot.PID1.Kd);
  udp_channels.sendFloat('n', robot.PID1.Kf);

  udp_channels.send('l', byte(IRLine.total / 20), IRLine.cross_count, IRLine.crosses, byte(IRLine.blacks * 10));

  udp_channels.send('P', robot.state, schedule.delta / 100);
  udp_send_buffer();
}
*/

void real_loop(void)
{
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    if (b == '+') robot.solenoid_state = 1;
    if (b == '-') robot.solenoid_state = 0;
    //if (b == '(') {robot.v1_PWM = 50;}
    if (b == '(') {robot.v += 0.1; robot.w = 0;}
    if (b == '/') {robot.v = 0; robot.w = 1.5;}
    if (b == '=') {robot.v = 0; robot.w =-50;}
    //if (b == ')') {robot.v2_PWM = 50;}
    if (b == ')') {robot.v -= -0.1; robot.w = 0;}
    //if (b == '?') {robot.v1_PWM = 0; robot.v2_PWM = 0;} 
    if (b == '?') {robot.v = 0; robot.w = 0;}
    if (b == '\\') robot.state = 0;
    if (b == '*') robot.state = 1;
    serial_channels.StateMachine(b);
  }

  schedule.current = micros();
  schedule.delta = schedule.current - schedule.previous;

  if (schedule.delta >= schedule.interval) {
    schedule.previous = schedule.current;

    readEncoders();



    robot.LastTouchSwitch = robot.TouchSwitch;
    robot.TouchSwitch = readTouchSwitch();

    robot.odometry();
    control(robot);

    // Later: read the real battery voltage
    robot.battery_voltage = 7.4;

    setSolenoidState();

    //robot.accelerationLimit();
    robot.v = robot.v_req;
    robot.w = robot.w_req;

    // Auto Control mode selection:
    // States for 0 to 199 are for PID control
    // States for 200 to 255 are for direct Voltage control
    robot.VWToMotorsVoltage();

    robot.PWM_1 = robot.u1 / robot.battery_voltage * 255;
    robot.PWM_2 = robot.u2 / robot.battery_voltage * 255;

    setMotorsPWM(robot.PWM_1, robot.PWM_2);
    //setMotorsPWM(50, 150);
    

    Serial.print(F(" E1: "));
    serial_print_format(robot.enc1, 4);

    Serial.print(F(" E2: "));
    serial_print_format(robot.enc2, 4);

    Serial.print(F(" T: "));
    serial_print_format(robot.TouchSwitch, 0);

    /*
    Serial.print(F(" V: "));
    serial_print_format(robot.v, 4);
    Serial.print(F(" W: "));
    serial_print_format(robot.w, 4);

    Serial.print(F(" Ve: "));
    serial_print_format(robot.ve, 4);
    Serial.print(F(" We: "));
    serial_print_format(robot.we, 4);    
    */

    Serial.println();
  }

}


void sim_loop(void)
{
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    serial_channels.StateMachine(b);  
  }

  if (go) {
    schedule.previous = schedule.current;
    schedule.current = micros();
    schedule.delta = schedule.current - schedule.previous;
    go = 0;

    robot.odometry();
    control(robot);
    
    //robot.accelerationLimit();
    robot.v = robot.v_req;
    robot.w = robot.w_req;

    robot.VWToMotorsVoltage();
    
    robot.battery_voltage = 7.4;
    robot.PWM_1 = robot.u1 / robot.battery_voltage * 255;
    robot.PWM_2 = robot.u2 / robot.battery_voltage * 255;

	setMotorsPWM(robot.PWM_1, robot.PWM_2);
    
    //Serial.print(WiFi.localIP().toString());   
    //Serial.print(F(" "));
 
    serial_channels.send('s',  robot.state);
    serial_channels.send('M',  round(robot.PWM_1));
    serial_channels.send('Q',  round(robot.PWM_2));
    serial_channels.send('L',  robot.solenoid_state);
    
    //if(udp_on) send_udp_channels();

  }
  
  /*schedule.current = micros();
  schedule.delta = schedule.current - schedule.previous;

  if (schedule.delta >= schedule.interval) {
    schedule.previous = schedule.current;

    if(udp_on) send_udp_channels();
  }*/
}
