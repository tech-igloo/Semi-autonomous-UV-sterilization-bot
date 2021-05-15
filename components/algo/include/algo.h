#ifndef ALGO_H
#define ALGO_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"

#define LEFT_MOTOR_DIRECTION_1 2 
#define RIGHT_MOTOR_DIRECTION_1 4
#define LEFT_MOTOR_PIN 25                        // PWM pin for left motor
#define RIGHT_MOTOR_PIN 13                      // PWM pin for right motor
#define LEFT_MOTOR_DIRECTION_2 33 
#define RIGHT_MOTOR_DIRECTION_2 27

#define LEFT_ENCODERA 39                       //Only signal A of both the motor is being used for now
#define LEFT_ENCODERB 34                   //Change the GPIO ENCODER PIN SEL when using all the pins
#define RIGHT_ENCODERA 36                      
#define RIGHT_ENCODERB 35
#define GPIO_ENCODER_PIN_SEL  ((1ULL<<LEFT_ENCODERA) | (1ULL<<RIGHT_ENCODERA))  //64 bit mask
#define ESP_INTR_FLAG_DEFAULT 0

// #define ULTRA1 25
// #define ULTRA2 26
// #define ULTRA3 4
// #define ULTRA4 2
// #define ULTRA5 15
// #define GPIO_ULTRASONIC_PIN_SEL ((1ULL<<ULTRA1) | (1ULL<<ULTRA2) | (1ULL<<ULTRA3) | (1ULL<<ULTRA4) | (1ULL<<ULTRA5))

#define ENCODERresolution 50              //The resolution of encoder, current using 'any edge' single channel
#define wheeldist_perTick 0.00343          //in meters
#define wheelbase 100                     //in meters

#define DEFAULT_LIN_SPEED 0.7              //meter/sec    
#define DEFAULT_ANG_SPEED 0.5               //rad/sec   
#define MIN_LINEAR_SPEED 0.005
#define MIN_ANGULAR_SPEED 0.05
     
#define sampleTime 100000                   //in microsec
#define sampleTimeInSec 0.1

#define ADC_SAMPLING_FREQ 1000000
#define ADC_AVERAGING_FREQ 10

extern int Lpwm;                       //Variables to pass pwm to the LEDC set duty function
extern int Rpwm;  

extern int leftRot;                    //Variable to take care to the encoder feedback
extern int leftTicks;
extern int rightRot;
extern int rightTicks;

extern double left_vel;                //Variable that store instantaneous velocity
extern double right_vel;
extern double prev_disL;
extern double prev_disR;

extern double lin_speed;
extern double ang_speed;

extern int64_t prev_tim;

extern double accumulated_errorL;             //integral term in PID formula
extern double current_errorL;                 //current error in PID formula
extern double prev_errorL;                    //error in the previous time step
extern double accumulated_errorR;             
extern double current_errorR;                 
extern double prev_errorR;                    

extern double xCoor;                          //For auto mode
extern double yCoor;
extern int lastAutoPath;

//extern int pid_flag;                        //flag that signifies that the PID controller's job is done

extern double Kp;                //Common gains for now
extern double Kd;
extern double Ki;


extern xQueueHandle gpio_evt_queue;

ledc_channel_config_t motorL, motorR;       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output
gpio_config_t ENCODER, ULTRASONIC;

struct point{                                   //Data Structure for storing points
    double x;
    double y;
    double theta;
};

void InterruptEncoder(void* arg);
void init_gpio();
void init_pwm();

void move_forward();
void move_left();
void move_right();
void move_back();
void move_stop();

void init_pid();
int pid_velLeft(double actualvel, double desiredvel);
int pid_velRight(double actualvel, double desiredvel);
int mapval(float x, float in_min, float in_max, int out_min, int out_max);

char determine(int local_flag);
esp_err_t convert_paths(int n);
esp_err_t get_path(int local_flag);
void docking_algo(char *pathstring);
void IRAM_ATTR gpio_encoder_isr_handler(void* arg);

void updateParams(double, double);
void actuationAuto();
void normal_motion();
void rotate();
void forward();
void update_stopPoint();
void recalculate();
void sensing();
void forwardSlow(int);
void rotateSlow(int);

#endif