#include "server.h"
#include "algo.h"

int Lpwm = 0; 
int Rpwm = 0;     //Always remember you can't define the variable in .h file     

int leftRot = 0;
int leftTicks = 0;
int rightRot = 0;
int rightTicks = 0;

double left_vel = 0;
double right_vel = 0;

double prev_disL = 0;
double prev_disR = 0;
int64_t prev_tim = 0;

double current_errorL = 0;
double accumulated_errorL = 0;
double prev_errorL = 0;
double current_errorR = 0;
double accumulated_errorR = 0;
double prev_errorR = 0;


double Kp = 1;                   //common gains for both the PID blocks for now
double Kd = 1/sampleTimeInSec;
double Ki = 1*sampleTimeInSec;

xQueueHandle gpio_evt_queue = NULL;

/*To initialize the motor direction, encoder, and sensor I/O pins*/ 
void init_gpio()
{
    ENCODER.intr_type = GPIO_INTR_ANYEDGE;      //Change according to the resolution, anyedge for quadrature when using both the pins
    ENCODER.mode = GPIO_MODE_INPUT;
    ENCODER.pull_down_en = 0;
    ENCODER.pull_up_en = 0;
    ENCODER.pin_bit_mask = GPIO_ENCODER_PIN_SEL; /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_config(&ENCODER);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(LEFT_ENCODERA, gpio_encoder_isr_handler, (void*) LEFT_ENCODERA);  //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RIGHT_ENCODERA, gpio_encoder_isr_handler, (void*) RIGHT_ENCODERA);
    
    gpio_evt_queue = xQueueCreate(20, sizeof(uint32_t));    //gpio interrupt queue

    gpio_intr_disable(LEFT_ENCODERA);  //Enabled when recording or in auto mode  
    gpio_intr_disable(RIGHT_ENCODERA);

    gpio_set_direction(LEFT_MOTOR_DIRECTION, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_MOTOR_DIRECTION, GPIO_MODE_OUTPUT);
}

/*Used for setting up PWM Channel (Ref: Official Github Repo)*/
void init_pwm()
{
    ledc_timer_config_t led_timer = {
            .duty_resolution = LEDC_TIMER_12_BIT,  // resolution of PWM duty, means PWM value can go from 0 to (2^12-1 = 4095)
            .freq_hz = 10000,                      // frequency of PWM signal
            .speed_mode = LEDC_HIGH_SPEED_MODE,    // High speed mode for immiediate effect
            .timer_num = LEDC_TIMER_0,             // timer index
            .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
        };
    ledc_timer_config(&led_timer);
    
    motorL.channel = LEDC_CHANNEL_0;
    motorL.duty = 0;						//the duty value of the PWM signal
    motorL.gpio_num = LEFT_MOTOR_PIN;			//the GPIO pin connected to Left motor controller PWM port
    motorL.speed_mode = LEDC_HIGH_SPEED_MODE;
    motorL.hpoint = 0;                      //for left aligned PWM
    motorL.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&motorL);

    motorR.channel = LEDC_CHANNEL_1;
    motorR.duty = 0;						//the duty value of the PWM signal
    motorR.gpio_num = RIGHT_MOTOR_PIN;			//the GPIO pin connected to Right motor controller PWM port
    motorR.speed_mode = LEDC_HIGH_SPEED_MODE;
    motorR.hpoint = 0;
    motorR.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&motorR);
}

/*The following are the actuation functions for movement of the bot with velocity feedback control*/
void move_forward()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime) //Running the PID at a specific frequency
    {
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // 9Current_dis-prev_dis)/time VELCOTIY IN m/sec
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disL = left_vel*0.1 + prev_disL;
        prev_disR = right_vel*0.1 + prev_disR;

        Lpwm = pid_velLeft(left_vel, DEFAULT_LIN_SPEED);     //values after being mapped to PWM range
        Rpwm = pid_velRight(right_vel, DEFAULT_LIN_SPEED);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION, 1);    //BOTH IN SAME DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 1);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);   //Update the PWM value to be used by this lED Channel.
    ledc_update_duty(motorL.speed_mode, motorL.channel);      //Use the Updated PWM values
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}                                                                       

void move_left()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN rad/sec
        prev_disL = left_vel*0.1 + prev_disL;
        left_vel = left_vel*2/wheelbase;    //angular velocity
        
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disR = right_vel*0.1 + prev_disR;
        right_vel = right_vel*2/wheelbase;

        Lpwm = pid_velLeft(left_vel, DEFAULT_ANG_SPEED);
        Rpwm = pid_velRight(right_vel, DEFAULT_ANG_SPEED);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION, 1);    //IN OPP DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_right()
{
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN rad/sec
        prev_disL = left_vel*0.1 + prev_disL;
        left_vel = left_vel*2/wheelbase;    //angular velocity
        
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disR = right_vel*0.1 + prev_disR;
        right_vel = right_vel*2/wheelbase;

        Lpwm = pid_velLeft(left_vel, DEFAULT_ANG_SPEED);
        Rpwm = pid_velRight(right_vel, DEFAULT_ANG_SPEED);
        prev_tim = esp_timer_get_time();
    }    

    gpio_set_level(LEFT_MOTOR_DIRECTION, 0);    //IN OPP DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 1);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_back()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN m/sec
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disL = left_vel*0.1 + prev_disL;
        prev_disR = right_vel*0.1 + prev_disR;

        Lpwm = pid_velLeft(left_vel, DEFAULT_LIN_SPEED);
        Rpwm = pid_velRight(right_vel, DEFAULT_LIN_SPEED);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION, 0);    //BOTH IN SAME DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_stop()
{
    ledc_set_duty(motorL.speed_mode, motorL.channel, 0);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, 0);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

/*To reset the PID error variable*/ 
void init_pid(){
    current_errorL = 0;
    accumulated_errorL = 0;
    prev_errorL = 0;
    prev_disL = 0;
    current_errorR = 0;
    accumulated_errorR = 0;
    prev_errorR = 0;
    prev_disR = 0;
}

/*Simple PID funtions, can be upgraded while testing according to the performance. Like integral windup etc.*/
int pid_velLeft(double actualvel, double desiredvel){
    current_errorL = desiredvel - actualvel;
    accumulated_errorL += current_errorL;
    double pid = Kp*current_errorL + Kd*(current_errorL-prev_errorL) + Ki*accumulated_errorL;
    //Main part after this is finding the maximum value of PID(after capping) and mapping it to the PWM 
    if (pid>10)
        pid=10;
    else if (pid<0)
        pid=0;
    prev_errorL = current_errorL;
    return map1(pid, 0, 10, 0, 4095);   //set maximum PWM as the top speed required
}

int pid_velRight(double actualvel, double desiredvel){
    current_errorR = desiredvel - actualvel;
    accumulated_errorR += current_errorR;
    double pid = Kp*current_errorR + Kd*(current_errorR-prev_errorR) + Ki*accumulated_errorR;
    if (pid>10)
        pid=10;
    else if (pid<0)
        pid=0;
    prev_errorR = current_errorR;
    //Main part after this is finding the maximum value of PID(after capping) and mapping it to the PWM 
    return map1(pid, 0, 10, 0, 4095);
}

/*Funtion to map pid value to the PWM range*/
int map1(float x, float in_min, float in_max, int out_min, int out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*Get the character to be used for storing the direction*/
char determine(int local_flag)
{
    char c;
    switch(local_flag){
        case -1:c = 's';break;
        case 0: c = 'f';break;
        case 1: c = 'l';break;
        case 2: c = 'r';break;
        case 3: c = 'b';break;
        default:c = 'a';break;
    }
    return c;
}

/*Algorithm for converting path to co-ordinate based format
  Remove DEFAULT_LIN_SPEED and DEFAULT_ANG_SPEED if the stored format is in distance and not time*/
esp_err_t convert_paths(int n){
    char str[LINE_LEN];
    char aux[500] = "", aux1[500] = "";
    int len = 0, linectr = 0, count_flag = 1;
    char ch, temp[9];
    double val;
    struct point prev = {0,0,0};
    struct point current = prev;
    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    FILE* f_w = fopen("/spiffs/temp.txt", "w");
    if(f_r == NULL){
        ESP_LOGE(TAG, "Error opening file paths.txt\n");
        return ESP_FAIL;
    }
    if(f_w == NULL){
        ESP_LOGE(TAG, "Error opening file temp.txt\n");
        return ESP_FAIL;
    }
    while(!feof(f_r))
    {
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_r);
        // ESP_LOGI(TAG, "%s", str);
        // ESP_LOGI(TAG, "Length: %d", strlen(str));
        strcat(aux, str);
        if (count_flag){
            linectr++;
            count_flag = 0;
        }              
        if (strchr(str, '\n')){      //We only increment the linectr when we have \n, because of a single path having multiple lines
            if(linectr == n) //This before the reseting temp so that if deceted it should retain it
            {
                strcpy(aux1,aux);
                ESP_LOGI(TAG, "%s", aux1);
                //Length calculation for dynamic allocation
                char* temp_token = strtok(aux, "\t");
                while(temp_token!=NULL)
                {
                    ch = temp_token[0];
                    temp_token++;
                    if(ch == 'f' || ch == 'b')
                    len ++;                 
                    temp_token = strtok(NULL, "\t");
                }
                char* result = (char *)calloc((len*2*10+1),sizeof(char));
                strcpy(result, "");
                ESP_LOGI(TAG, "Length: %d", len*4*9+1);
                //Result string storage
                char* token = strtok(aux1, "\t");
                while(token!=NULL)
                {
                    ch = token[0];
                    token++;
                    val = atof(token);    //val is in meters 
                    if(ch == 'f'){
                        current.x = prev.x + val*cos(prev.theta);
                        current.y = prev.y + val*sin(prev.theta);
                        ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                        strcpy(temp, "");
                        snprintf(temp, 9, "%f", current.x);
                        strcat(result, temp);
                        strcat(result, " ");
                        strcpy(temp, "");
                        snprintf(temp, 9, "%f", current.y);
                        strcat(result, temp);
                        strcat(result, " ");
                        current.theta = prev.theta;
                        prev = current;
                    }
                    else if(ch == 'b'){
                        current.x = prev.x - val*cos(prev.theta);  //don't convert as already in radians when using encoders
                        current.y = prev.y - val*sin(prev.theta);
                        ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                        strcpy(temp, "");
                        snprintf(temp, 9, "%f", current.x);
                        strcat(result, temp);
                        strcat(result, " ");
                        strcpy(temp, "");
                        snprintf(temp, 9, "%f", current.y);
                        strcat(result, temp);
                        strcat(result, " ");
                        current.theta = prev.theta;
                        prev = current;
                    }
                    else if(ch == 'r'){
                        //val is in radian only
                        prev.theta = prev.theta + val;
                    }
                    else if(ch == 'l'){
                        prev.theta = prev.theta - val;
                    }
                    token = strtok(NULL, "\t");
                }
                strcat(result, "\n");
                ESP_LOGI(TAG, "%s", result);
                fprintf(f_w, "%s", result);
            }
            else
                fprintf(f_w, "%s", aux);
            count_flag=1;
            strcpy(aux, "");
        }
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/paths.txt");
    rename("/spiffs/temp.txt", "/spiffs/paths.txt");
    return ESP_OK;
}

/*Execute the local_flag th path*/ //auto mode, need to enable the interrupts and use the algorithm
esp_err_t get_path(int local_flag)
{
    char str[LINE_LEN], temp[500] = ""; // Change this temp to calloc
    int linectr = 0, count_flag = 1;
    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    if (f_r == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    while(!feof(f_r))
    {
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_r);

        if(!feof(f_r))
        {
            strcat(temp, str);
            if (count_flag){
                linectr++;
                count_flag = 0;
            }  
                          
            if (strchr(str, '\n')){      //We only increment the linectr when we have \n, because of a single path having multiple lines
                if(linectr == (local_flag+1)) //This before the reseting temp so that if deceted it should retain it
                    break;
                count_flag=1;
                strcpy(temp, "");
            }
        }
    }
    ESP_LOGI(TAG, "%s", temp);
    fclose(f_r);
    //return ESP_OK;
    char* token = strtok(temp, "\t");    //The elements are seperated by "\t"
    auto_flag = 1;                      
    while(token!=NULL)					//iterate through each of the elements
    {
        char ch = token[0];             //Get the first character which denotes the direction to be travelled
        switch(ch){
            case 'f'://move_forward();break;
                        flag = 0; break;	//The flag values are set here. The infinite Loop in Task 1 checks these flag variables and calls the appropriate functions
            case 'l'://move_left();break;
                        flag = 1; break;
            case 'r'://move_right();break;
                        flag = 2; break;
            case 'b'://move_back();break;
                        flag = 3; break;
            default://move_stop();break;
                        flag = 4; break;
        }
        ESP_LOGI(TAG, "Direction: %c", ch);
        token++;                        //Increment the pointer to get the numerical value stored after the first character
        float time = atof(token);       //Convert the value from string to float
        ESP_LOGI(TAG, "Time: %f", time);
        vTaskDelay(time/portTICK_PERIOD_MS);    //Wait for the appropriate time
        token = strtok(NULL, "\t");             //Get the next element
    }
    //move_stop();
    auto_flag = 0;
    return ESP_OK;
}