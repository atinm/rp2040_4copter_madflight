/*#########################################################################################################################

Minimal quadcopter demo program for madflight Arduino ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32 Flight Controller

###########################################################################################################################

See http://madflight.com for detailed description

Required Hardware

    IMU sensor (SPI or I2C)
    RC receiver with 5 channels (CRSF/ELRS preferred)
    4 brushless motors with ESCs

Connecting Hardware

    SPI IMU: connect pin_imu_int, pin_imu_cs, pin_spi0_miso, pin_spi0_mosi, pin_spi0_sclk
    or for I2C IMU: connect pin_imu_int, pin_i2c1_scl, pin_i2c1_sda
    RC receiver: connect pin_ser0_rx to receiver TX pin
    ESCs: pin_out0 ... pin_out3 to the ESC inputs of motor1 ... motor4

Arming/disarming with dedicated switch

    Arm: Set throttle low, then flip arm switch from DISARMED to ARMED.
    Disarm: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch".

Arming/disarming with sticks (when no arm switch is defined, i.e. cfg.rcl_arm_ch == 0 )

    Arm: Pull both sticks toward you, yaw full right, and roll full left
    Disarm: Pull both sticks toward you, yaw full left, and roll full right

Failsafe Features

    - 3-second radio connection timeout: Immediate disarm if no radio connection for 3 seconds
    - 30-second radio command timeout: Gradual ramp-down if no radio commands received for 30 seconds
    - Motor ramp-down: 5-second gradual throttle reduction when disarming with spinning motors

LED State                              Meaning
---------                              -------
OFF                                    Not powered
ON                                     Startup (don't move, running gyro calibration)
Blinking long OFF short ON             DISARMED
Blinking long ON short OFF             ARMED
Blink interval longer than 1 second    imu_loop() is taking too much time
fast blinking                          Something is wrong, connect USB serial for info

MIT license
Copyright (c) 2023-2025 https://madflight.com
##########################################################################################################################*/
#include <Arduino.h>
#include "madflight_config.h" //Edit this header file to setup the pins, hardware, radio, etc. for madflight
#include "madflight.h"

//========================================================================================================================//
//                                               INCLUDES                                                                 //
//========================================================================================================================//

// Function declarations
void imu_watchdog_check();
void imu_interrupt_handler_with_watchdog();

//========================================================================================================================//
//                                               VARIABLES                                                                //
//========================================================================================================================//

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//IMPORTANT: This is a safety feature which keeps props spinning when armed, and hopefully reminds the pilot to disarm!!!
const float armed_min_throttle = 0.10; //Minimum throttle when armed, set to a value between ~0.10 and ~0.25 which keeps the props spinning at minimum speed.

//Flight Mode: Uncommment only one
//#define FLIGHTMODE_RATE   //control rate - stick centered will keep current roll/pitch angle
#define FLIGHTMODE_ANGLE  //control angle - stick centered will return to horizontal - IMPORTANT: execute CLI 'calimu' and 'save' before using this!!!

//Controller parameters (take note of defaults before modifying!):
const float i_limit        = 25.0;      //Integrator saturation level, mostly for safety (default 25.0)
const float maxRoll        = 30.0;      //Max roll angle in degrees for angle mode (maximum ~70 degrees)
const float maxPitch       = 30.0;      //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
const float maxRollRate    = 60.0;      //Max roll rate in deg/sec for rate mode
const float maxPitchRate   = 60.0;      //Max pitch rate in deg/sec for rate mode
const float maxYawRate     = 160.0;     //Max yaw rate in deg/sec for angle and rate mode

//PID Angle Mode
const float Kp_ro_pi_angle  = 0.2;      //Roll/Pitch P-gain
const float Ki_ro_pi_angle  = 0.1;      //Roll/Pitch I-gain
const float Kd_ro_pi_angle  = 0.05;     //Roll/Pitch D-gain
const float Kp_yaw_angle    = 0.6;      //Yaw P-gain
const float Kd_yaw_angle    = 0.1;      //Yaw D-gain

//PID Rate Mode
const float Kp_ro_pi_rate   = 0.15;     //Roll/Pitch rate P-gain
const float Ki_ro_pi_rate   = 0.2;      //Roll/Pitch rate I-gain
const float Kd_ro_pi_rate   = 0.0002;   //Roll/Pitch rate D-gain (be careful when increasing too high, motors will begin to overheat!)
const float Kp_yaw_rate     = 0.3;       //Yaw rate P-gain
const float Ki_yaw_rate     = 0.05;      //Yaw rate I-gain
const float Kd_yaw_rate     = 0.00015;   //Yaw rate D-gain (be careful when increasing too high, motors will begin to overheat!)

//Yaw to keep in ANGLE mode when yaw stick is centered
float yaw_desired = 0;

// Add at the top, after global variables
bool ramp_down_active = false;
unsigned long ramp_down_start = 0;
const unsigned long ramp_down_duration = 5000; // 5 seconds in ms
float ramp_down_start_throttle = 0.0f;

bool allow_arming = false;
bool prev_rcl_armed = false;

// Throttle ramping variables
float current_throttle = 0.0f;
float target_throttle = 0.0f;
const float max_throttle_change_per_second = 0.5f; // Maximum throttle change per second (0.5 = 50% per second)

// 30-second radio timeout failsafe variables
const unsigned long radio_timeout_failsafe = 30000; // 30 seconds in ms
bool radio_timeout_failsafe_active = false;
unsigned long last_radio_command_time = 0;

#ifdef PLATFORMIO
//========================================================================================================================//
//                                                       Forward declarations                                             //
//========================================================================================================================//
void led_Blink();
void control_Rate(bool zero_integrators);
void control_Angle(bool zero_integrators);
void out_KillSwitchAndFailsafe();
void out_Mixer();
#endif

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  //setup madflight components: Serial.begin(115200), imu, rcl, led, etc. See src/madflight/interface.h for full interface description of each component.
  madflight_setup();

  // Uncomment ONE line - select output type
  int out_hz = 50; int min_us = 1000; int max_us = 2000; //Standard PWM: 50Hz, 1000-2000 us
  //int out_hz = 400; int min_us = 950; int max_us = 2000; //Standard PWM: 400Hz, 950-2000 us
  //int out_hz = 2000; int min_us = 125; int max_us = 250; //Oneshot125: 2000Hz, 125-250 us

  // Setup 4 motors for the quadcopter
  out.setupMotor(0, cfg.pin_out0, out_hz, min_us, max_us);
  out.setupMotor(1, cfg.pin_out1, out_hz, min_us, max_us);
  out.setupMotor(2, cfg.pin_out2, out_hz, min_us, max_us);
  out.setupMotor(3, cfg.pin_out3, out_hz, min_us, max_us);

  //set initial desired yaw
  yaw_desired = ahr.yaw;

  // Initialize radio timeout failsafe
  last_radio_command_time = millis();
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  cli.update(); //process CLI commands
}

//========================================================================================================================//
//                                                   IMU UPDATE LOOP                                                      //
//========================================================================================================================//

//This is __MAIN__ function of this program. It is called when new IMU data is available.
void imu_loop() {
  //Blink LED
  led_Blink();

  //Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data
  ahr.update();

  //Get radio commands - Note: don't do this in loop() because loop() is a lower priority task than imu_loop(), so in worst case loop() will not get any processor time.
  rcl.update();

  //PID Controller RATE or ANGLE
  #ifdef FLIGHTMODE_ANGLE
    control_Angle(rcl.throttle == 0); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint  //control_Angle2(rcin_thro_is_low); //Stabilize on pitch/roll setpoint using cascaded method. Rate controller must be tuned well first!
  #else
    control_Rate(rcl.throttle == 0); //Stabilize on rate setpoint
  #endif

  //Updates out.arm, the output armed flag
  out_KillSwitchAndFailsafe(); //Cut all motor outputs if DISARMED or failsafe triggered.

  //Actuator mixing
  out_Mixer(); //Mixes PID outputs and sends command pulses to the motors, if mot.arm == true
}







//========================================================================================================================//
//                      IMU UPDATE LOOP FUNCTIONS - in same order as they are called from imu_loop()                           //
//========================================================================================================================//

void led_Blink() {
  //Blink LED once per second, if LED blinks slower then the loop takes too much time, use CLI 'pimu' to investigate.
  //DISARMED: long off, short on, ARMED: long on, short off
  uint32_t modulus = imu.update_cnt % imu.getSampleRate();
  if( modulus == 0) led.set(!out.armed); //start of pulse
  if( modulus == imu.getSampleRate() / 10)  led.set(out.armed); //end of pulse
}

//returns angle in range -180 to 180
float degreeModulus(float v) {
  if(v >= 180) {
    return fmod(v + 180, 360) - 180;
  }else if(v < -180.0) {
    return fmod(v - 180, 360) + 180;
  }
  return v;
}

void control_Angle(bool zero_integrators) {
  //DESCRIPTION: Computes control commands based on angle error
  /*
   * Basic PID control to stabilize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des. Error
   * is simply the desired state minus the actual state (ex. roll_des - ahr.roll). Two safety features
   * are implemented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle... saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I
   * terms will always start from 0 on takeoff. This function updates the variables PIDroll.PID, PIDpitch.PID, and PIDyaw.PID which
   * can be thought of as 1-D stabilized signals. They are mixed to the configuration of the vehicle in out_Mixer().
   */

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  float roll_des = rcl.roll * maxRoll; //Between -maxRoll and +maxRoll
  float pitch_des = rcl.pitch * maxPitch; //Between -maxPitch and +maxPitch
  float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate roll_PIDand +maxYawRate

  //state vars
  static float integral_roll, integral_pitch, error_yawRate_prev, integral_yawRate;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yawRate = 0;
  }

  //Roll PID
  float error_roll = roll_des - ahr.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahr.gx;
  PIDroll.PID = 0.01 * (Kp_ro_pi_angle*error_roll + Ki_ro_pi_angle*integral_roll - Kd_ro_pi_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch PID
  float error_pitch = pitch_des - ahr.pitch;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = ahr.gy;
  PIDpitch.PID = 0.01 * (Kp_ro_pi_angle*error_pitch + Ki_ro_pi_angle*integral_pitch - Kd_ro_pi_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw PID
  if(-0.02 < rcl.yaw && rcl.yaw < 0.02) {
    //on reset, set desired yaw to current yaw
    if(zero_integrators) yaw_desired = ahr.yaw;

    //Yaw stick centered: hold yaw_desired
    float error_yaw = degreeModulus(yaw_desired - ahr.yaw);
    float desired_yawRate = error_yaw / 0.5; //set desired yawRate such that it gets us to desired yaw in 0.5 second
    float derivative_yaw = desired_yawRate - ahr.gz;
    PIDyaw.PID = 0.01 * (Kp_yaw_angle*error_yaw + Kd_yaw_angle*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //update yaw rate controller
    error_yawRate_prev = 0;
  }else{
    //Yaw stick not centered: stablize on rate from GyroZ
    float error_yawRate = yawRate_des - ahr.gz;
    integral_yawRate += error_yawRate * imu.dt;
    integral_yawRate = constrain(integral_yawRate, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
    float derivative_yawRate = (error_yawRate - error_yawRate_prev) / imu.dt;
    PIDyaw.PID = 0.01 * (Kp_yaw_rate*error_yawRate + Ki_yaw_rate*integral_yawRate + Kd_yaw_rate*derivative_yawRate); //Scaled by .01 to bring within -1 to 1 range

    //Update derivative variables
    error_yawRate_prev = error_yawRate;

    //update yaw controller:
    yaw_desired = ahr.yaw; //set desired yaw to current yaw, the yaw angle controller will hold this value
  }
}

void control_Rate(bool zero_integrators) {
  //Computes control commands based on state error (rate)
  //See explanation for control_Angle(). Everything is the same here except the error is now: desired rate - raw gyro reading.

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  float rollRate_des = rcl.roll * maxRollRate; //Between -maxRoll and +maxRoll
  float pitchRate_des = rcl.pitch * maxPitchRate; //Between -maxPitch and +maxPitch
  float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate and +maxYawRate

  //state vars
  static float integral_roll, error_roll_prev;
  static float integral_pitch, error_pitch_prev;
  static float integral_yaw, error_yaw_prev;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll
  float error_roll = rollRate_des - ahr.gx;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (error_roll - error_roll_prev) / imu.dt;
  PIDroll.PID = 0.01 * (Kp_ro_pi_rate*error_roll + Ki_ro_pi_rate*integral_roll + Kd_ro_pi_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  float error_pitch = pitchRate_des - ahr.gy;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = (error_pitch - error_pitch_prev) / imu.dt;
  PIDpitch.PID = 0.01 * (Kp_ro_pi_rate*error_pitch + Ki_ro_pi_rate*integral_pitch + Kd_ro_pi_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  float error_yaw = yawRate_des - ahr.gz;
  integral_yaw += error_yaw * imu.dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt;
  PIDyaw.PID = 0.01 * (Kp_yaw_rate*error_yaw + Ki_yaw_rate*integral_yaw + Kd_yaw_rate*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update derivative variables
  error_roll_prev = error_roll;
  error_pitch_prev = error_pitch;
  error_yaw_prev = error_yaw;
}

void out_KillSwitchAndFailsafe() {
  static bool was_armed = false;

  // Update last radio command time when we receive new radio data
  if (rcl.connected()) {
    last_radio_command_time = millis();
    radio_timeout_failsafe_active = false; // Reset failsafe if we have connection
  }

  //Change to ARMED when rcl is armed (by switch or stick command)
  // Track rising edge of arm switch
  if (!prev_rcl_armed && rcl.armed && rcl.connected()) {
    allow_arming = true;  // Allow arming only if switch was toggled after boot
  }
  prev_rcl_armed = rcl.armed;

  if (!out.armed && rcl.armed && rcl.connected() && allow_arming) {
    out.armed = true;
    Serial.println("OUT: ARMED");
    ramp_down_active = false;
    radio_timeout_failsafe_active = false;
  }

  // Check for 30-second radio timeout failsafe
  if (out.armed && !radio_timeout_failsafe_active &&
      (millis() - last_radio_command_time) >= radio_timeout_failsafe) {
    radio_timeout_failsafe_active = true;
    Serial.println("OUT: 30-second radio timeout failsafe triggered");
  }

  //Change to DISARMED when rcl is disarmed, or if radio lost connection (3-second timeout)
  if (out.armed && (!rcl.armed || !rcl.connected())) {
    // Only start ramp-down if motors are spinning (throttle > armed_min_throttle)
    float current_thr = armed_min_throttle + (1 - armed_min_throttle) * rcl.throttle;
    if (!ramp_down_active && current_thr > armed_min_throttle + 0.01f) {
      ramp_down_active = true;
      ramp_down_start = millis();
      ramp_down_start_throttle = current_thr;
      Serial.println("OUT: Starting motor ramp-down");
    } else if (!ramp_down_active) {
      // If not spinning, disarm immediately
      out.armed = false;
      if(!rcl.armed) {
        Serial.println("OUT: DISARMED");
      }else{
        Serial.println("OUT: DISARMED due to lost radio connection");
      }
    }
  }

  // Start ramp-down for 30-second timeout failsafe
  if (out.armed && radio_timeout_failsafe_active && !ramp_down_active) {
    float current_thr = armed_min_throttle + (1 - armed_min_throttle) * rcl.throttle;
    ramp_down_active = true;
    ramp_down_start = millis();
    ramp_down_start_throttle = current_thr;
    Serial.println("OUT: Starting motor ramp-down due to 30-second radio timeout");
  }

  // Complete ramp-down after duration
  if (ramp_down_active) {
    unsigned long elapsed = millis() - ramp_down_start;
    if (elapsed >= ramp_down_duration) {
      ramp_down_active = false;
      radio_timeout_failsafe_active = false;
      out.armed = false;
      Serial.println("OUT: DISARMED after ramp-down");
    }
  }
}

float apply_throttle_ramping(float input_throttle) {
  // Update target throttle
  target_throttle = input_throttle;

  // Calculate maximum change allowed per loop iteration
  float max_change_per_loop = max_throttle_change_per_second * imu.dt;

  // Ramp current throttle towards target
  if (current_throttle < target_throttle) {
    current_throttle = min(current_throttle + max_change_per_loop, target_throttle);
  } else if (current_throttle > target_throttle) {
    current_throttle = max(current_throttle - max_change_per_loop, target_throttle);
  }

  return current_throttle;
}

void out_Mixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes PIDroll.PID, PIDpitch.PID, and PIDyaw.PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +PIDroll.PID while the right two motors
   * should have -PIDroll.PID. Front two should have +PIDpitch.PID and the back two should have -PIDpitch.PID etc... every motor has
   * normalized (0 to 1) rcl.throttle command for throttle control. Can also apply direct unstabilized commands from the transmitter with
   * rcl.xxx variables are to be sent to the motor ESCs and servos.
   *
   *Relevant variables:
   *rcl.throttle - direct throttle control
   *PIDroll.PID, PIDpitch.PID, PIDyaw.PID - stabilized axis variables
   *rcl.roll, rcl.pitch, rcl.yaw - direct unstabilized command passthrough
   */
/*
Motor order diagram (Betaflight order)

      front
 CW -->   <-- CCW
     4     2
      \ ^ /
       |X|
      / - \
     3     1
CCW -->   <-- CW

                                        M1234
Pitch up (stick back)   (front+ back-)   -+-+
Roll right              (left+ right-)   --++
Yaw right               (CCW+ CW-)       -++-
*/

  // IMPORTANT: This is a safety feature which keeps props spinning when armed, and hopefully reminds the pilot to disarm!!!
  // Set motor outputs to at least armed_min_throttle, to keep at least one prop spinning when armed. The [out] module will disable motors when out.armed == false

  // Apply throttle ramping to reduce current spikes
  float ramped_throttle = apply_throttle_ramping(rcl.throttle);
  float thr = armed_min_throttle + (1 - armed_min_throttle) * ramped_throttle;

  // Debug output when ramping is active (only print occasionally to avoid spam)
  static unsigned long last_ramp_debug = 0;
  if (abs(ramped_throttle - rcl.throttle) > 0.01f && millis() - last_ramp_debug > 1000) {
    Serial.printf("Throttle ramping: input=%.2f, ramped=%.2f\n", rcl.throttle, ramped_throttle);
    last_ramp_debug = millis();
  }

  // Debug output for radio timeout status (only print occasionally to avoid spam)
  static unsigned long last_timeout_debug = 0;
  if (out.armed && millis() - last_timeout_debug > 5000) { // Every 5 seconds
    unsigned long time_since_last_radio = millis() - last_radio_command_time;
    Serial.printf("Radio timeout status: %lu ms since last command, failsafe active: %d\n",
                  time_since_last_radio, radio_timeout_failsafe_active);
    last_timeout_debug = millis();
  }
  // During ramp-down, override throttle with ramped value
  if (ramp_down_active) {
    unsigned long elapsed = millis() - ramp_down_start;
    float progress = (float)elapsed / (float)ramp_down_duration;
    progress = constrain(progress, 0.0f, 1.0f);
    thr = armed_min_throttle + (1.0f - progress) * (ramp_down_start_throttle - armed_min_throttle);
  }
  if(rcl.throttle == 0 && !ramp_down_active) {
    //if throttle idle, but armed, then run props at low speed without applying PID. This allows for stick commands for arm/disarm.
    if (out.armed) {
      out.set(0, thr);
      out.set(1, thr);
      out.set(2, thr);
      out.set(3, thr);
    } else {
      // disarmed - turn off the props completely, we might have lost contact and want props turned off
      out.set(0, 0.0);
      out.set(1, 0.0);
      out.set(2, 0.0);
      out.set(3, 0.0);
    }
  } else{
    // Quad mixing - if out.armed == false, we've disarmed due to link loss
    if (out.armed) {
      float m1 = thr - PIDpitch.PID - PIDroll.PID - PIDyaw.PID;
      float m2 = thr + PIDpitch.PID - PIDroll.PID + PIDyaw.PID;
      float m3 = thr - PIDpitch.PID + PIDroll.PID + PIDyaw.PID;
      float m4 = thr + PIDpitch.PID + PIDroll.PID - PIDyaw.PID;
      out.set(0, m1); //M1 Back Right CW
      out.set(1, m2); //M2 Front Right CCW
      out.set(2, m3); //M3 Back Left CCW
      out.set(3, m4); //M4 Front Left CW
    } else {
      out.set(0, 0.0);
      out.set(1, 0.0);
      out.set(2, 0.0);
      out.set(3, 0.0);
    }
  }
}
