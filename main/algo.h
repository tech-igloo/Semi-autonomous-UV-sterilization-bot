#ifndef ALGO_H
#define ALGO_H

#include "server.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LEFT_MOTOR_ENABLE 33 
#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_PIN 27                       // PWM pin for left motor
#define RIGHT_MOTOR_PIN 13                      // PWM pin for right motor
#define LEFT_MOTOR_DIRECTION 32 
#define RIGHT_MOTOR_DIRECTION 14

#define resolution 0.1                            //The resolution of points used in the co-ordinate system
#define DEFAULT_LIN_SPEED 1.5                   //Temporary constants I used. To be deleted when encoder feedback is used
#define DEFAULT_ANG_SPEED 10                    //Temporary constants I used. To be deleted when encoder feedback is used

extern int Lpwm;                       //Variables to pass pwm to the LEDC set duty function
extern int Rpwm;  

ledc_channel_config_t motorL, motorR;       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output. Will have to create another for controlling 2 motors

struct point{                                   //Data Structure for storing points
    double x;
    double y;
    double theta;
};

void init_pwm();

void move_forward();
void move_left();
void move_right();
void move_back();
void move_stop();

esp_err_t convert_paths(int n);

#endif