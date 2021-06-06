#ifndef ALGO_H
#define ALGO_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"

/*Various parameters that are used to configure the hardware setup*/
//Motor driver pin configuration
#define LEFT_MOTOR_DIRECTION_1 2 
#define RIGHT_MOTOR_DIRECTION_1 4
#define LEFT_MOTOR_PIN 25                       // PWM pin for left motor
#define RIGHT_MOTOR_PIN 26                      // PWM pin for right motor
#define LEFT_MOTOR_DIRECTION_2 33 
#define RIGHT_MOTOR_DIRECTION_2 27
//Encoder pin configuration
#define LEFT_ENCODERA 39                       //Only signal A of both the motor encoder is used
#define RIGHT_ENCODERA 36                      //only for calulating distance not direction
#define GPIO_ENCODER_PIN_SEL  ((1ULL<<LEFT_ENCODERA) | (1ULL<<RIGHT_ENCODERA))  //64 bit mask
#define ESP_INTR_FLAG_DEFAULT 0
//Robot Wheel, encoders, chassis and velocity parameters
#define ENCODERresolution 50               //The resolution of encoder, current using 'any edge' single channel
#define wheeldist_perTick 0.00343          //meters
#define wheelbase 0.1475                   //meters
#define DEFAULT_LIN_SPEED 0.7              //meter/sec    
#define DEFAULT_ANG_SPEED 0.45             //rad/sec   
#define MIN_LINEAR_SPEED 0.005             //meter/sec 
#define MIN_ANGULAR_SPEED 0.05             //rad/sec 
//For Calling the PID function and sampling the encoder data
#define sampleTime 100000                  //microsec
#define sampleTimeInSec 0.1                //sec
//Ultrasonic sensor pin configuration
#define ULTRA1 12
#define ULTRA2 13
#define ULTRA3 14
#define ULTRA4 32
#define ULTRA5 15
#define GPIO_ULTRASONIC_PIN_SEL ((1ULL<<ULTRA1) | (1ULL<<ULTRA2) | (1ULL<<ULTRA3) | (1ULL<<ULTRA4) | (1ULL<<ULTRA5))
//Battery voltage monitoring ADC configuration
//ADC_CHANNEL_6[GPIO 36]
#define ADC_SAMPLING_FREQ 1000000
#define ADC_AVERAGING_FREQ 10
//Variables to pass pwm to the LEDC set duty function
extern int Lpwm;                       
extern int Rpwm;  
//Variable to store to the encoder feedback
extern int leftRot;                    
extern int leftTicks;
extern int rightRot;
extern int rightTicks;
//ADC Battery voltage monitoring
extern int batteryPercent;
//Variable that store instantaneous velocity
extern double left_vel;                
extern double right_vel;
//To calculate the instantaneous velocity
extern double prev_disL;
extern double prev_disR;
//Desired speed variables
extern double lin_speed;
extern double ang_speed;
extern int64_t prev_tim;   //To sample the encoder and PID at regular intervals
//PID error variables for left and right motor control
extern double accumulated_errorL;             //integral term in PID formula
extern double current_errorL;                 //current error in PID formula
extern double prev_errorL;                    //error in the previous time step
extern double accumulated_errorR;             
extern double current_errorR;                 
extern double prev_errorR;      
//For storing the fetched coordinates from list during auto mode execution
extern double xCoor;                          
extern double yCoor;
extern int lastAutoPath; //Last auto path to idicate the data to docking algo
extern xQueueHandle gpio_evt_queue;  

/*Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output*/
ledc_channel_config_t motorL, motorR;   
/*Data structure for configuring Encoder and Ultrasonic input pins*/    
gpio_config_t ENCODER, ULTRASONIC;
/*Data Structure for storing points*/
struct point{                                   
    double x;
    double y;
    double theta;
};

/*Intialization functions*/
void init_gpio();
void init_pwm();
void IRAM_ATTR gpio_encoder_isr_handler(void* arg);
/*Encoder interrupt update function*/
void InterruptEncoder(void* arg);
/*Motor actuation and PID functions*/
void move_forward();
void move_left();
void move_right();
void move_back();
void move_stop();
int pid_velLeft(double actualvel, double desiredvel);
int pid_velRight(double actualvel, double desiredvel);
int mapval(float x, float in_min, float in_max, int out_min, int out_max);
void init_pid();
/*Path storing and traversing algorithm functions*/
char determine(int local_flag);
esp_err_t convert_paths(int n);
esp_err_t get_path(int local_flag);
void docking_algo(char *pathstring);
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