#ifndef ALGO_H
#define ALGO_H

#include "driver/gpio.h"
#include "driver/ledc.h"

#define LEFT_MOTOR_ENABLE 33 
#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_PIN 2   //27                       // PWM pin for left motor
#define RIGHT_MOTOR_PIN 13                      // PWM pin for right motor
#define LEFT_MOTOR_DIRECTION 32 
#define RIGHT_MOTOR_DIRECTION 14

#define LEFT_ENCODERA 36                       //Only signal A of both the motor is being used for now
#define LEFT_ENCODERB 39
#define RIGHT_ENCODERA 34                      //as the direction is set buy us, but for point we might need to use both
#define RIGHT_ENCODERB 35
#define GPIO_ENCODER_PIN_SEL  ((1ULL<<LEFT_ENCODERA) | (1ULL<<RIGHT_ENCODERA))  //64 bit mask
#define ESP_INTR_FLAG_DEFAULT 0

#define ULTRA1 
#define ULTRA2
#define ULTRA3
#define ULTRA4
#define ULTRA5

#define ENCODERresolution 1200              //The resolution of encoder, current using any edge single channel
#define wheeldist_perTick 0.0445           //in centimeters
#define wheelbase 30                   //in centimeters
#define DEFAULT_LIN_SPEED 1.5               // meter/sec    //Temporary constants I used. To be deleted when encoder feedback is used
#define DEFAULT_ANG_SPEED 10                    //Temporary constants I used. To be deleted when encoder feedback is used

extern int Lpwm;                       //Variables to pass pwm to the LEDC set duty function
extern int Rpwm;  
extern int leftRot;
extern int leftTicks;
extern int rightRot;
extern int rightTicks;

extern xQueueHandle gpio_evt_queue;

ledc_channel_config_t motorL, motorR;       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output
gpio_config_t ENCODER, ULTRASONIC;

struct point{                                   //Data Structure for storing points
    double x;
    double y;
    double theta;
};

void init_gpio();
void init_pwm();

void move_forward();
void move_left();
void move_right();
void move_back();
void move_stop();

char determine(int local_flag);
esp_err_t convert_paths(int n);
esp_err_t get_path(int local_flag);
void IRAM_ATTR gpio_encoder_isr_handler(void* arg);

#endif