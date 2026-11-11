/***************************************************************************
 *   Copyright (C) 2024 by DTU                             *
 *   jcan@dtu.dk                                                    *
 *
 *   Base Teensy firmware
 *   build for Teensy 4.1,
 *   intended for digital control course
 *
 * The MIT License (MIT)  https://mit-license.org/
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE. */

void printLog();

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <IntervalTimer.h>
// Support functions
#include "src/udisplay.h"
#include "src/urobot.h"
#include "src/uusb.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/uencoder.h"

/**
 * Global variables */
// Sample time can not go lower than 300us 0.3ms
const uint32_t sampleTimeUs = 1000; // desired sample time in us !!!! put in 3 hundred ms
// const float ts = sampleTimeUs * 1e-6; // sample time in seconds
// Robot configuration
const float gear = 9.6;
const float wheelRadius = 0.0315; // (m)
const float wheelBase = 0.14;
// pose values
float distA = 0; // trip distance
float pose[4];
float poseVel[2];
bool velSaturated = false;

// added by me
float Kp = 0.15;
float Kpt = 5;
float taui_t = 0.2;
float previous_poseVel[2];
float in_integral_prev;
float out_integral_prev;

// Lead parameters
float Kph = 48.891f;
float tau_lead = 0.0622;
float alpha_lead = 0.217;

// for the heading lead
float error_previous;
float u_lead_previous;

// Previous velocities in the X and Y directions
//  Used by the integrator to calculate the robot's position.
float previous_xVel = 0.0f;
float previous_yVel = 0.0f;
// ////////////////////////////////////////
// define a structure for log elements
typedef struct
{
  float time;
  int state;
  float desiredValue;
  float desiredHeading;
  float motorVel[2];
  float motorVolt[2];
  int32_t encoder[2];
  float motorCurrent[2];
  float battery;
  float pose[4];
  float distA;
  float poseVel[2];
  float inVelLeft;
  float inVelRight;
  float gyro[3];
  float headU;
  float accVel;
  float accTurn;

  // added by me
  float desiredTurnrate;
  float actualTurnrate;
} LogData;
//
// allocate memory for data log in memory block 2 available only through malloc/new
// (specific for this micro processor, a Teensy 4)
const int logsMax = 400000 / sizeof(LogData); // allocates 400kBytes
LogData *logs = (LogData *)malloc(logsMax * sizeof(LogData));
int logsCnt = 0; // next log entry to use / i.e. number of used log entries

void printLog()
{
  LogData *d = logs;
  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());
  // hard-coded settings
  Serial.print("% Sample time ");
  Serial.print(sampleTimeUs);
  Serial.print(" us, Gear ");
  Serial.print(gear);
  Serial.print(", Wheels ");
  Serial.print(wheelRadius);
  Serial.print(" m, PPR ");
  Serial.println(encoder.pulsPerRev);
  Serial.print("% maximum log samples ");
  Serial.print(logsMax);
  Serial.print(" each sized ");
  Serial.print(sizeof(LogData));
  Serial.println(" bytes");
  // Columns
  Serial.println("% 1 time (sec)");
  Serial.println("% 2 state");
  Serial.println("% 3   Desired value");
  Serial.println("% 4-5 motor voltage (left,right) (V)");
  Serial.println("% 6-7 Encoder count (left, right)");
  Serial.println("% 8-9 motor vel (left, right) (rad/s)");
  Serial.println("% 10  Battery voltage (V)");

  // CODE ADDED BY ME
  Serial.println("% 11-13 Gyro values");
  Serial.println("% 14-15  current (left, right) (A)");
  Serial.println("% 16  Pose velocity: velocity (m/s)");
  Serial.println("% 17  Pose velocity: turnrate (rad/s)");
  Serial.println("% 18  Pose: distance(m)");
  Serial.println("% 19  Pose: heading (rad)");
  Serial.println("% 20  Pose: x (m)");
  Serial.println("% 21  Pose: y (m)");
  Serial.println("% 22 Desired heading (deg)");
  Serial.println("% 23 Desired turnrate (rad/s)");
  Serial.println("% 24 Actual turnrate (rad/s)");

  for (int i = 0; i < logsCnt; i++)
  {
    // Serial.print(" ");
    Serial.print(d->time, 4); // 1
    Serial.print(" ");
    Serial.print(d->state); // 2
    Serial.print(" ");
    Serial.print(d->desiredValue, 2); // 3
    Serial.print(" ");
    Serial.print(d->motorVolt[0], 2); // 4 (V)
    Serial.print(" ");
    Serial.print(d->motorVolt[1], 2); // 5 (V)
    Serial.print(" ");
    Serial.print(d->encoder[0]); // 6
    Serial.print(" ");
    Serial.print(d->encoder[1]); // 7
    Serial.print(" ");
    Serial.print(d->motorVel[0], 2); // 8 (rad/s)
    Serial.print(" ");
    Serial.print(d->motorVel[1], 2); // 9 (rad/s)
    Serial.print(" ");
    Serial.print(d->battery, 2); // 10 (V)
    Serial.print(" ");
    Serial.print(d->gyro[0], 2);
    Serial.print(" ");
    Serial.print(d->gyro[1], 2);
    Serial.print(" ");
    Serial.print(d->gyro[2], 2);
    Serial.print(" ");
    Serial.print(d->motorCurrent[0], 2);
    Serial.print(" ");
    Serial.print(d->motorCurrent[1], 2);
    Serial.print(" ");
    Serial.print(d->poseVel[0], 2);
    Serial.print(" ");
    Serial.print(d->poseVel[1], 2);
    Serial.print(" ");
    Serial.print(d->pose[0], 2);
    Serial.print(" ");
    Serial.print(d->pose[1], 2);
    Serial.print(" ");
    Serial.print(d->pose[2], 2);
    Serial.print(" ");
    Serial.print(d->pose[3], 2);
    Serial.print(" ");
    Serial.print(d->desiredHeading, 4);
    Serial.print(" ");
    Serial.print(d->desiredTurnrate, 4);
    Serial.print(" ");
    Serial.print(d->actualTurnrate, 4);
    Serial.println(" ");

    d++;
  }
  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}

// ////////////////////////////////////////

/**
 * Global variables for timing */
bool isSampleTime = false;
float time_sec = 0;

// Timer that controls sample time (interrupt based)
IntervalTimer sampleTimer;

float integrator(float current_in, float prev_integral, float previous_in)
{
  float integral = prev_integral + (current_in + previous_in) * sampleTimeUs * 1e-6f / 2;
  return integral;
}

void timerInterrupt()
{
  isSampleTime = true;
  time_sec += sampleTimeUs / 1e6;
}
/**
 * declaration, that there is a reset function */
void reset();

/**
 * Global variables for state machine */
int state = 0;          // actual sequence state
float endTime = 0;      // for current state
float desiredValue = 0; // desired (reference) value send to controller
float desiredHeading = 0;
float desiredTurnrate = 0; // to change afterward to calculate depending on the desiredHeading

void start()
{ // Start timing
  // reset log and encoders
  logsCnt = 0;
  // change motor PWM frequency for sampling time test
  // should not be above 100000 (100kHz), default is 80kHz.
  motor.setPWMfrq(80000); // before 80000
  time_sec = 0;
  encoder.encoder[0] = 0; // left motor encoder
  encoder.encoder[1] = 0; // right motor encoder
  // Update of display takes too long for fast sampling
  // so, disable during sequence.
  display.useDisplay = false;

  // added lec6
  for (int i = 0; i < 4; i++)
    pose[i] = 0.0f;
  poseVel[0] = 0.0f;
  poseVel[1] = 0.0f;

  // reset turn rate PI integrator: added lec7
  in_integral_prev = 0.0f;
  out_integral_prev = 0.0f;

  // Added lec8: Reset previous velocities before starting a new run.
  // Otherwise, the integrator could use velocity values from the
  // previous run and calculate an incorrect initial position.
  previous_poseVel[0] = 0.0f;
  previous_poseVel[1] = 0.0f;
  previous_xVel = 0.0f;
  previous_yVel = 0.0f;

  // for the heading lead
  error_previous = 0;
  u_lead_previous = 0;
}

void stop(float after)
{ // stop, but allow continued logging for a while
  state = 2;
  desiredValue = 0;
  // end log after an additional short time

  // ADDED lec8: Stop heading corrections
  desiredHeading = pose[1] * 180.0f / PI;
  desiredTurnrate = 0;

  endTime = time_sec + after;
}

void finished()
{ // prepare for a new start
  // stop motors
  motor.motorVoltage[0] = 0; // left motor
  motor.motorVoltage[1] = 0; // right motor
  // set default PWM frequency
  motor.setPWMfrq(80000);
  // start updating display
  display.useDisplay = true;
  // wait for next button press
  state = 0;
  printLog();
}

void sequenceSquare()
{
  bool button;

  switch (state)
  {
  case 0: // Wait for start
    button = digitalReadFast(PIN_START_BUTTON);
    if (button or robot.missionStart)
    {
      start();
      desiredValue = 0;
      desiredHeading = 0;
      endTime = time_sec + 0.020;
      state = 10;
    }
    break;

  case 10: // Start driving forward 0.75 m
    if (time_sec > endTime)
    {
      desiredValue = 0.75;
      endTime = time_sec + 1.0;
      state = 101;
    }
    break;

  case 101: // Start first 90 degree turn
    if (time_sec > endTime)
    {
      desiredValue = 0;
      desiredHeading = 90;
      state = 102;
    }
    break;

  case 102: // Start driving forward 0.75 m
    if (abs(desiredHeading * PI / 180.0f - pose[1]) < 0.05f)
    {
      desiredValue = 0.75;
      endTime = time_sec + 1.0;
      state = 103;
    }
    break;

  case 103: // Start second 90 degree turn
    if (time_sec > endTime)
    {
      desiredValue = 0;
      desiredHeading = 180;
      state = 104;
    }
    break;

  case 104: // Start driving forward 0.75 m
    if (abs(desiredHeading * PI / 180.0f - pose[1]) < 0.05f)
    {
      desiredValue = 0.75;
      endTime = time_sec + 1.0;
      state = 105;
    }
    break;

  case 105: // Start third 90 degree turn
    if (time_sec > endTime)
    {
      desiredValue = 0;
      desiredHeading = 270;
      state = 106;
    }
    break;

  case 106: // Start driving forward 0.75 m
    if (abs(desiredHeading * PI / 180.0f - pose[1]) < 0.05f)
    {
      desiredValue = 0.75;
      endTime = time_sec + 1.0;
      state = 107;
    }
    break;

  case 107: // Start fourth 90 degree turn
    if (time_sec > endTime)
    {
      desiredValue = 0;
      desiredHeading = 360;
      state = 108;
    }
    break;

  case 108: // Start driving forward 0.75 m
    if (abs(desiredHeading * PI / 180.0f - pose[1]) < 0.05f)
    {
      desiredValue = 0.75;
      endTime = time_sec + 1.0;
      state = 109;
    }
    break;

  case 109: // Stop after final forward movement
    if (time_sec > endTime)
    {
      stop(0.3);
    }
    break;

  default: // Finish sequence
    if (time_sec > endTime)
      finished();
    break;
  }
}

void sequence_1m()
{
  bool button;
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  {       // run mission, initial value
  case 0: // State 0 is just inactive, waiting for the start signal.
    button = digitalReadFast(PIN_START_BUTTON);
    if (button or robot.missionStart)
    { // starting by a short time to get zero velocity data to the log
      // reset for new run
      start();
      // Prepare next state
      desiredValue = 0; // reference value to the controller
      // to get start the log with no velocity
      endTime = time_sec + 0.020; // new state to end after 20ms
      state = 10;
    }
    break;
  case 10: // Waiting for first step (zero velocity to log) to finish.
    if (time_sec > endTime)
    {                           // change to next values - drive
      desiredValue = 0.33;      // should be in meters/sec, but is motor voltage for now,
      endTime = time_sec + 3.0; // ~ 0.5 second
      state = 100;
    }
    break;

  case 100: // Maintain last step
    // stop after end time
    if (time_sec > endTime)
    {            // stop, but continue logging for a while (0.3 sec)
      stop(0.3); // actually sets state to 999 (i.e. default:)
    }
    break;
  default: // Hold until finished
    // this state is needed to enable a new start
    if (time_sec > endTime)
      finished();
    break;
  }
}
/**
 * Simple 2-step sequence */
void sequenceTwoSteps()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button;
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  {       // run mission, initial value
  case 0: // State 0 is just inactive, waiting for the start signal.
    button = digitalReadFast(PIN_START_BUTTON);
    if (button or robot.missionStart)
    { // starting by a short time to get zero velocity data to the log
      // reset for new run
      start();
      // Prepare next state
      desiredValue = 0; // reference value to the controller
      // to get start the log with no velocity
      endTime = time_sec + 0.020; // new state to end after 20ms
      state = 10;
    }
    break;
  case 10: // Waiting for first step (zero velocity to log) to finish.
    if (time_sec > endTime)
    {                            // change to next values - drive
      desiredValue = 1.5;        // should be in meters/sec, but is motor voltage for now,
      endTime = time_sec + 0.48; // ~ 0.5 second
      state = 11;
    }
    break;
  case 11: // First step
    // test if ready for next state
    if (time_sec > endTime)
    { // change to next values
      desiredValue = 3;
      endTime = time_sec + 0.50;
      state = 12;
    }
    break;
  case 12: // Second step
    // test if this state is finished
    if (time_sec > endTime)
    { // change to next values
      desiredValue = 6;
      endTime = time_sec + 0.5;
      state = 100;
    }
    break;
  case 100: // Maintain last step
    // stop after end time
    if (time_sec > endTime)
    {            // stop, but continue logging for a while (0.3 sec)
      stop(0.3); // actually sets state to 999 (i.e. default:)
    }
    break;
  default: // Hold until finished
    // this state is needed to enable a new start
    if (time_sec > endTime)
      finished();
    break;
  }
}

void spikeSequence()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button;
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  {       // run mission, initial value
  case 0: // State 0 is just inactive, waiting for the start signal.
    button = digitalReadFast(PIN_START_BUTTON);
    if (button or robot.missionStart)
    { // starting by a short time to get zero velocity data to the log
      // reset for new run
      start();
      // Prepare next state
      desiredValue = 0; // reference value to the controller
      // to get start the log with no velocity
      endTime = time_sec + 0.020; // new state to end after 20ms
      state = 10;
    }
    break;
  case 10: // Waiting for first step (zero velocity to log) to finish.
    if (time_sec > endTime)
    {                            // change to next values - drive
      desiredValue = 6;          // should be in meters/sec, but is motor voltage for now,
      endTime = time_sec + 0.01; // ~ 0.5 second
      state = 11;
    }
    break;
  case 11: // First step
    // test if ready for next state
    if (time_sec > endTime)
    { // change to next values
      desiredValue = 0;
      endTime = time_sec + 0.3;
      state = 12;
    }
    break;
  case 12: // Second step
    // test if this state is finished
    if (time_sec > endTime)
    { // change to next values
      desiredValue = 6;
      endTime = time_sec + 0.01;
      state = 13;
    }
    break;

  case 13: // Second step
    // test if this state is finished
    if (time_sec > endTime)
    { // change to next values
      desiredValue = 0;
      endTime = time_sec + 0.3;
      state = 14;
    }
    break;

  case 14: // Second step
    // test if this state is finished
    if (time_sec > endTime)
    { // change to next values
      desiredValue = 6;
      endTime = time_sec + 0.01;
      state = 100;
    }
    break;
  case 100: // Maintain last step
    // stop after end time
    if (time_sec > endTime)
    {            // stop, but continue logging for a while (0.3 sec)
      stop(0.3); // actually sets state to 999 (i.e. default:)
    }
    break;
  default: // Hold until finished
    // this state is needed to enable a new start
    if (time_sec > endTime)
      finished();
    break;
  }
}

void steadyStateSequence()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button;
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  {       // run mission, initial value
  case 0: // State 0 is just inactive, waiting for the start signal.
    button = digitalReadFast(PIN_START_BUTTON);
    if (button or robot.missionStart)
    { // starting by a short time to get zero velocity data to the log
      // reset for new run
      start();
      // Prepare next state
      desiredValue = 0; // reference value to the controller
      // to get start the log with no velocity
      endTime = time_sec + 0.020; // new state to end after 20ms
      state = 10;
    }
    break;
  case 10: // Waiting for first step (zero velocity to log) to finish.
    if (time_sec > endTime)
    {                         // change to next values - drive
      desiredValue = 6;       // should be in meters/sec, but is motor voltage for now,
      endTime = time_sec + 0; // ~ 0.5 second
      state = 11;
    }
    break;

  case 100: // Maintain last step
    // stop after end time
    if (time_sec > endTime)
    {            // stop, but continue logging for a while (0.3 sec)
      stop(0.3); // actually sets state to 999 (i.e. default:)
    }
    break;
  default: // Hold until finished
    // this state is needed to enable a new start
    if (time_sec > endTime)
      finished();
    break;
  }
}

// float integrator(float current_in, float prev_integral, float previous_in)
float control_tr_velLeft()
{

  float current_turnrate = (encoder.motorVelocity[1] + encoder.motorVelocity[0]) * wheelRadius / (gear * wheelBase);
  float error = desiredTurnrate - current_turnrate;

  float e_kpt = Kpt * error;
  float in_integral = e_kpt * (1 / taui_t);
  float integral = integrator(in_integral, out_integral_prev, in_integral_prev);

  float tr_i = min(integral, 0.25f);
  if (integral < 0)
  {
    tr_i = max(integral, -0.25f);
  }

  in_integral_prev = in_integral;
  out_integral_prev = tr_i;

  float tr_u = tr_i + e_kpt;
  float vel = (wheelBase / 2) * tr_u;
  return vel;
}

float control_heading()
{
  // Calculate heading error (convert desired heading from degrees to radians)
  float error = desiredHeading * PI / 180.0f - pose[1];

  // Proportional heading controller
  float Ts = sampleTimeUs * 1e-6;
  float u_lead = ((tau_lead * (2 / Ts) + 1) * Kph * error + (-tau_lead * (2 / Ts) + 1) * Kph * error_previous - (1 - alpha_lead * tau_lead * (2 / Ts)) * u_lead_previous) / (1 + alpha_lead * tau_lead * (2 / Ts));

  error_previous = error;
  u_lead_previous = u_lead;

  return u_lead;
}

/**
 * make the control */
void controlUpdate()
{ // do control during a mission only.
  // called at every tick
  //

  // ADDED lec8: Heading controller
  // Calculate desired turn rate from heading error
  float u_lead = control_heading();
  desiredTurnrate = u_lead;

  // Turn rate PI controller (lec7)
  float calculated_value = (wheelBase / 2) * desiredTurnrate;

  float velRef[2];
  velRef[0] = desiredValue - calculated_value;
  velRef[1] = desiredValue + calculated_value;

  float voltageLeft = (gear / wheelRadius * velRef[0] + encoder.motorVelocity[0]) * Kp;
  float voltageRight = (gear / wheelRadius * velRef[1] - encoder.motorVelocity[1]) * Kp;

  const float vMax = 9.0;
  voltageLeft = constrain(voltageLeft, -vMax, vMax);
  voltageRight = constrain(voltageRight, -vMax, vMax);
  //
  // For a start, the desiredValue
  // is just used as motor voltage
  //
  // Left motor voltage (-9V to +9V)
  // Before: motor.motorVoltage[0] = -velRef;
  motor.motorVoltage[0] = -voltageLeft;
  // Right motor voltage (-9V to +9V)
  // before: motor.motorVoltage[1] = velRef;
  motor.motorVoltage[1] = voltageRight;
  //
  // in the main loop, there is a motor.tick() that actually send the
  // values to the motor.
}

void updateLog()
{ // step response mode
  if (state > 0 and logsCnt < logsMax)
  { // save data for this sample into dataLog structure
    logs[logsCnt].time = time_sec;
    logs[logsCnt].state = state;
    logs[logsCnt].desiredValue = desiredValue;
    logs[logsCnt].motorVolt[0] = -motor.motorVoltage[0]; // The left motor runs backwards, therefore the sign.
    logs[logsCnt].motorVolt[1] = motor.motorVoltage[1];
    logs[logsCnt].encoder[0] = -encoder.encoder[0];
    logs[logsCnt].encoder[1] = encoder.encoder[1];
    logs[logsCnt].motorVel[0] = -encoder.motorVelocity[0];
    logs[logsCnt].motorVel[1] = encoder.motorVelocity[1];
    logs[logsCnt].battery = robot.batteryVoltage;
    // the last part is offset of current measurement (in Amps)
    // take the initial current readings and add or subtract as needed
    // these values fits Tania: (+0.260A, -0.275A)
    logs[logsCnt].desiredHeading = desiredHeading;
    logs[logsCnt].motorCurrent[0] = -(analogRead(A1) - 2048) * 0.0045 + 0.260;
    logs[logsCnt].motorCurrent[1] = (analogRead(A0) - 2048) * 0.0045 - 0.275;
    logs[logsCnt].poseVel[0] = poseVel[0];
    logs[logsCnt].poseVel[1] = poseVel[1];
    // add more items as needed, NB! also the the struct definition at top.
    // added by me lec6
    logs[logsCnt].pose[0] = pose[0];
    logs[logsCnt].pose[1] = pose[1];
    logs[logsCnt].pose[2] = pose[2];
    logs[logsCnt].pose[3] = pose[3];

    // ADDED by ME
    logs[logsCnt].gyro[0] = imu2.gyro[0];
    logs[logsCnt].gyro[1] = imu2.gyro[1];
    logs[logsCnt].gyro[2] = imu2.gyro[2];

    // ADDED lec8: Save desired and actual turn rates
    logs[logsCnt].desiredTurnrate = desiredTurnrate;
    logs[logsCnt].actualTurnrate = poseVel[1];

    logsCnt++;
  }
}

void setup() // INITIALIZATION
{            // to be able to print to USB interface
  Serial.begin(12000000);
  sampleTimer.begin(timerInterrupt, sampleTimeUs);
  // initialize sensors, safety and display
  robot.setup();   // alive LED, display and battery
  motor.setup();   // motor voltage
  encoder.setup(); // motor encoders to velocity
  encoder.pulsPerRev = 48;
  imu2.setup();             // gyro and accelerometer
  motor.setPWMfrq(0x13000); // 0x13000 = 77824 Hz
  usb.setup();
  display.useDisplay = true;
}

void calcul_pose()
{
  // Convert motor angular velocities (rad/s) to wheel velocities (m/s)
  // The left motor has a negative sign because it rotates in the opposite direction.
  float motorVel_ms_left = -encoder.motorVelocity[0] * wheelRadius / gear;
  float motorVel_ms_right = encoder.motorVelocity[1] * wheelRadius / gear;

  // Calculate forward velocity (m/s) as the average of both wheel velocities.
  poseVel[0] = 0.5 * (motorVel_ms_left + motorVel_ms_right);

  // Calculate turn rate (rad/s) from the difference between wheel velocities.
  poseVel[1] = 1 / wheelBase * (-motorVel_ms_left + motorVel_ms_right);

  // Integrate forward velocity to calculate travelled distance (m).
  pose[0] = integrator(poseVel[0], pose[0], previous_poseVel[0]);

  // Integrate turn rate to calculate the robot's heading (rad).
  pose[1] = integrator(poseVel[1], pose[1], previous_poseVel[1]);

  // Calculate the velocity components along the fixed X and Y axes.
  // The robot's forward velocity is split into X and Y using its heading.
  float xVel = cos(pose[1]) * poseVel[0];
  float yVel = sin(pose[1]) * poseVel[0];

  // Integrate X and Y velocities to calculate the robot's position (m).
  // The previous X and Y velocities are needed for trapezoidal integration.
  pose[2] = integrator(xVel, pose[2], previous_xVel);
  pose[3] = integrator(yVel, pose[3], previous_yVel);

  // Save the current forward velocity and turn rate for the next integration.
  previous_poseVel[0] = poseVel[0];
  previous_poseVel[1] = poseVel[1];

  // Save the current X and Y velocities for the next integration.
  previous_xVel = xVel;
  previous_yVel = yVel;
}

// integrator(float current_in, float prev_integral, float previous_in)

/**
 * Main loop
 * */
void loop(void)
{ // init sample time
  while (true)
  { // main loop
    // loop until time for next sample
    if (isSampleTime) // start of new control cycle
    {                 // time for next tick
      isSampleTime = false;
      // read sensors
      imu2.tick();
      encoder.tick();
      // updatePose();
      // sequenceTwoSteps();
      // spikeSequence();
      sequenceSquare();
      //
      if (state > 0)
      { // Only if started
        // Calculate new motor voltage
        // ADD lec6
        calcul_pose();
        controlUpdate();
      }
      // give value to actuators
      motor.tick();
      // save relevant values
      updateLog();
      // support functions
      robot.tick();   // measure battery voltage etc.
      display.tick(); // update O-LED display
    }
    usb.tick(); // listen to incoming from USB
  }
}

/////////////////////////////////////////////////////////////////
