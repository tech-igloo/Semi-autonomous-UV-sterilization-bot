#ifndef ALGO_H
#define ALGO_H

#include "server.h"

ledc_channel_config_t led_channel;       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output. Will have to create another for controlling 2 motors

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