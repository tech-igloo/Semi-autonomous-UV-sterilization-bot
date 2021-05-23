#include "server.h"
#include "algo.h"

/*Defining the variables declared in alog.h*/
int Lpwm = 0; 
int Rpwm = 0;     
int leftRot = 0;
int leftTicks = 0;
int rightRot = 0;
int rightTicks = 0;
double left_vel = 0;
double right_vel = 0;
double prev_disL = 0;
double prev_disR = 0;
double lin_speed = DEFAULT_LIN_SPEED;
double ang_speed = DEFAULT_ANG_SPEED;
int64_t prev_tim = 0;
double current_errorL = 0;
double accumulated_errorL = 0;
double prev_errorL = 0;
double current_errorR = 0;
double accumulated_errorR = 0;
double prev_errorR = 0;
//Separate PID constants for Left and Right PID control loops
double Kpl = 1;                  
double Kdl = 0.02/sampleTimeInSec;
double Kil = 0.001*sampleTimeInSec;
double Kpr = 1;                   
double Kdr = 0.01/sampleTimeInSec;
double Kir = 0.001*sampleTimeInSec;
double xCoor = 0;
double yCoor = 0;
int lastAutoPath = 0;
xQueueHandle gpio_evt_queue = NULL;           //Queue handle to keep track of the interrupt queue 

/*Static variables used only in this file to perform autonomous mode calculations and execution*/
static double angle_required = 0;             //angle that the bot needs to rotate to align itself with its destination point
static double dist_required = 0;              //distance from stop point that the bot needs to travel to reach the destination point
static double stop_point[2] = {0};            //This is the actual point that is attained as compared to desired(prev_point)
static double prev_point[2] = {0};            //The goal point that robot is trying to reach in that iteration
static double current_point[2] = {0};         //Instantaneous co-ordinates of the bot
static double dist_traversed = 0;             //distance travlled by bot along straight line(+ve for forward and -ve for negative)
static double angle_rotated = 0;              //absolute angle rotated by bot(+ve for clockwise and -ve for anticlockwise) wrt to +ve x axis
static int rotating_flag = 1;                 //flag that signifies whether the bot should rotate or move along a straight line
static int doneFlag = 0;                      //To denote when straight or roatation has been performed
static double prev_time = 0;                  //stores the previous time step, gets updated to current time after sensing
static double time_flag = 0;                  //To represent the last time since the obstacle has been detected
static int detect_flag = 0;                   //To indicate when desired combination of ULTRASONIC sensor has detected a obstacle
static int run = 0;                           //To move onto the next point from list once the previous has been reached
static int obstacle_flag[5] = {0};            //To record ultrasonic value
static int prevleftRot = 0, prevleftTicks = 0;
/*To initialize the motor direction, encoder, sensor I/O pins, and ADC pins*/ 
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
    
    xTaskCreate(InterruptEncoder, "InterruptEncoder", 2048, NULL, 10, NULL);

    gpio_intr_disable(LEFT_ENCODERA);  //Enabled when recording or in auto mode  
    gpio_intr_disable(RIGHT_ENCODERA);

    gpio_set_direction(LEFT_MOTOR_DIRECTION_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_MOTOR_DIRECTION_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEFT_MOTOR_DIRECTION_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_MOTOR_DIRECTION_2, GPIO_MODE_OUTPUT);

    ULTRASONIC.intr_type = 0;      //Change according to the resolution, anyedge for quadrature when using both the pins
    ULTRASONIC.mode = GPIO_MODE_INPUT;
    ULTRASONIC.pull_down_en = 0;
    ULTRASONIC.pull_up_en = 0;
    ULTRASONIC.pin_bit_mask = GPIO_ULTRASONIC_PIN_SEL; /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_config(&ULTRASONIC);

    adc1_config_width(ADC_WIDTH_BIT_12);   // 0-4095 not linear range, testing and filtering req
    adc1_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_11); //ADC1-CH6 corresponds to pin 34
}

/*Used for setting up Motor PWM Channel (Ref: Official Github Repo)*/
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

/*To integrate the Queue in the ISR handler of Encoder interrupts*/ 
void IRAM_ATTR gpio_encoder_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/*A Task created to continuously check the Encoder interrupt Queue and process the data if available,
this function updates the number of rotation and ticks of left and right motor*/ 
void InterruptEncoder(void* arg)
{
    uint32_t io_num;
    ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num)); 
            //printf("leftRot: %d, LeftTicks: %d\n", leftRot, leftTicks); 
            if (io_num == LEFT_ENCODERA){
                leftTicks++;
                if (leftTicks >= ENCODERresolution){
                    leftRot++;                         
                    leftTicks=0;
                }
            }
            else if (io_num == RIGHT_ENCODERA){
                rightTicks++;
                if (rightTicks >= ENCODERresolution){
                    rightRot++;
                    rightTicks=0;
                }
            }
        }
    }
}

/*The following are the actuation functions for movement of the bot with velocity feedback control*/
void move_forward()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime) //Running the PID at a specific frequency
    {
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/sampleTimeInSec;  // 9Current_dis-prev_dis)/time VELCOTIY IN m/sec
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/sampleTimeInSec;
        prev_disL = left_vel*sampleTimeInSec + prev_disL;
        prev_disR = right_vel*sampleTimeInSec + prev_disR;

        Lpwm = pid_velLeft(left_vel, lin_speed);     //values after being mapped to PWM range
        Rpwm = pid_velRight(right_vel, lin_speed);
        //ESP_LOGI(TAG, "Left wheel velocity:%f, right wheel velocity:%f", left_vel, right_vel);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION_1, 1);    //BOTH IN SAME DIRECTION
    gpio_set_level(LEFT_MOTOR_DIRECTION_2, 0);    
    gpio_set_level(RIGHT_MOTOR_DIRECTION_1, 1);
    gpio_set_level(RIGHT_MOTOR_DIRECTION_2, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);   //Update the PWM value to be used by this Channel.
    ledc_update_duty(motorL.speed_mode, motorL.channel);      //Use the Updated PWM values
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}                                                                       

void move_left()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/sampleTimeInSec;  //angular velocity in rad/sec
        prev_disL = left_vel*sampleTimeInSec + prev_disL;
        left_vel = left_vel*2/wheelbase;    //angular velocity of the bot due to left motor
        
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/sampleTimeInSec;
        prev_disR = right_vel*sampleTimeInSec + prev_disR;
        right_vel = right_vel*2/wheelbase;   //angular velocity of the bot due to right motor

        Lpwm = pid_velLeft(left_vel, ang_speed/2);          // Divide by 2 because each motor is providing half the desired 
        Rpwm = pid_velRight(right_vel, ang_speed/2);        // angular velocity to the bot for point rotation
        //ESP_LOGI(TAG, "Left wheel velocity:%f, right wheel velocity:%f", left_vel, right_vel);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION_1, 1);    //IN OPP DIRECTION
    gpio_set_level(LEFT_MOTOR_DIRECTION_2, 0);    
    gpio_set_level(RIGHT_MOTOR_DIRECTION_1, 0);
    gpio_set_level(RIGHT_MOTOR_DIRECTION_2, 1);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_right()
{
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/sampleTimeInSec;  //angular velocity in rad/sec
        prev_disL = left_vel*sampleTimeInSec + prev_disL;
        left_vel = left_vel*2/wheelbase;    //angular velocity of the bot due to left motor
        
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/sampleTimeInSec;
        prev_disR = right_vel*sampleTimeInSec + prev_disR;
        right_vel = right_vel*2/wheelbase;   //angular velocity of the bot due to right motor

        Lpwm = pid_velLeft(left_vel, ang_speed/2);          // Divide by 2 because each motor is providing half the desired angular velocity to the bot
        Rpwm = pid_velRight(right_vel, ang_speed/2);        // for point rotation
        //ESP_LOGI(TAG, "Left wheel velocity:%f, right wheel velocity:%f", left_vel, right_vel);
        prev_tim = esp_timer_get_time();
    }   

    gpio_set_level(LEFT_MOTOR_DIRECTION_1, 0);    //IN OPP DIRECTION
    gpio_set_level(LEFT_MOTOR_DIRECTION_2, 1);   
    gpio_set_level(RIGHT_MOTOR_DIRECTION_1, 1);
    gpio_set_level(RIGHT_MOTOR_DIRECTION_2, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_back()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/sampleTimeInSec;  // (Current_dis-prev_dis)/time=VELCOTIY IN m/sec
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/sampleTimeInSec;
        prev_disL = left_vel*sampleTimeInSec + prev_disL;
        prev_disR = right_vel*sampleTimeInSec + prev_disR;

        Lpwm = pid_velLeft(left_vel, lin_speed);
        Rpwm = pid_velRight(right_vel, lin_speed);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION_1, 0);    
    gpio_set_level(LEFT_MOTOR_DIRECTION_2, 1);    //BOTH IN SAME DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION_1, 0);
    gpio_set_level(RIGHT_MOTOR_DIRECTION_2, 1);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_stop()
{   
    gpio_set_level(LEFT_MOTOR_DIRECTION_1, 0);    
    gpio_set_level(LEFT_MOTOR_DIRECTION_2, 0); 
    gpio_set_level(RIGHT_MOTOR_DIRECTION_1, 0);
    gpio_set_level(RIGHT_MOTOR_DIRECTION_2, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, 0);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, 0);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

/*To reset the PID error variable as well as some automode variables*/ 
void init_pid()
{
    current_errorL = 0; //PID variables
    accumulated_errorL = 0;
    prev_errorL = 0;
    prev_disL = 0;
    current_errorR = 0;
    accumulated_errorR = 0;
    prev_errorR = 0;
    prev_disR = 0;
    leftRot = 0;        //Encoder variables reset when moving onto next function or point
    leftTicks = 0;
    rightRot = 0;
    rightTicks = 0;
    doneFlag = 0;       //To move onto the next point from the list
    prevleftRot = 0;
    prevleftTicks = 0;
}

/*PID control loop performing velocity feedback for left motor*/
int pid_velLeft(double actualvel, double desiredvel)
{
    current_errorL = desiredvel - actualvel;
    accumulated_errorL += current_errorL;
    double pid = Kpl*current_errorL + Kdl*(current_errorL - prev_errorL) + Kil*accumulated_errorL;
    //The maximum and minimum value of the PID needs to be calculated everytime PID constants are changed
    if (pid>2.5)
         pid=2.5;
    else if (pid<0)
         pid=0;
    prev_errorL = current_errorL;
    return mapval(pid, 0, 2.5, 2200, 4095);   //set maximum PWM as the top speed required
}

/*PID control loop performing velocity feedback for left motor*/
int pid_velRight(double actualvel, double desiredvel)
{
    current_errorR = desiredvel - actualvel;
    accumulated_errorR += current_errorR;
    double pid = Kpr*current_errorR + Kdr*(current_errorR - prev_errorR) + Kir*accumulated_errorR;
    //The maximum and minimum value of the PID needs to be calculated everytime PID constants are changed
    if (pid>2.5)
         pid=2.5;
    else if (pid<0)
         pid=0;
    prev_errorR = current_errorR;
    return mapval(pid, 0, 2.5, 2200, 4095);
}

/*Funtion to map pid value to the PWM range*/
int mapval(float x, float in_min, float in_max, int out_min, int out_max) 
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

/*Algorithm for converting path to co-ordinate based format using basic trignometry.
The recorded path in the form of F,L,R, and B is transfomed to x and y coordinates and overwriting them in the path.txt file.*/
esp_err_t convert_paths(int n)
{
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
            if(linectr == n)         
            {
                strcpy(aux1,aux);
                ESP_LOGI(TAG, "%s", aux1);
                //Length calculation for dynamic allocation
                char *temp_token, *temp_token1;
                temp_token = strtok(aux, "\t");
                while(temp_token!=NULL)
                {  
                    temp_token1 = strtok(NULL, "\t");             //Get the next element
                    if (temp_token1 != NULL){
                        ch = temp_token[0];
                        temp_token++;
                        if(ch == 'f' || ch == 'b')
                        len ++; 
                    }
                    else
                        break;
                    temp_token = strtok(NULL, "\t");
                    if (temp_token != NULL){
                        ch = temp_token1[0];
                        temp_token1++;
                        if(ch == 'f' || ch == 'b')
                        len ++; 
                    }
                    else
                        break;
                }
                char* result = (char *)calloc((len*2*10+1+32),sizeof(char));  //+32 for the name
                strcpy(result, "");
                ESP_LOGI(TAG, "Length: %d", len*4*9+1+32);
                //Conversion and storage of path to coordinates in result variable
                char *token, *token1;
                token = strtok(aux1, "\t");
                while(token!=NULL)
                {   
                    token1 = strtok(NULL, "\t");
                    if(token1 != NULL){
                        ch = token[0];
                        token++;
                        val = atof(token);    //val is in meters and radians 
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
                            current.x = prev.x - val*cos(prev.theta);  
                            current.y = prev.y - val*sin(prev.theta);
                            ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                            strcpy(temp, "");               //Each coordinate is separated by a single space
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
                        else if(ch == 'l'){
                            prev.theta = prev.theta + val;
                        }
                        else if(ch == 'r'){
                            prev.theta = prev.theta - val;
                        }
                    }
                    else{
                        strcat(result, token);
                        strcat(result, " ");       //Last space for name
                        break;
                    }
                    token = strtok(NULL, "\t");
                    if (token != NULL){
                        ch = token1[0];
                        token1++;
                        val = atof(token1);    //val is in meters and radians 
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
                            current.x = prev.x - val*cos(prev.theta);  
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
                        else if(ch == 'l'){ 
                            prev.theta = prev.theta + val;  //positive value for left, aangle measured from x axis
                        }
                        else if(ch == 'r'){
                            prev.theta = prev.theta - val;
                    }
                    }
                    else{
                        strcat(result, token1);
                        strcat(result, " ");        //Last space for name
                        break;
                    }
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

void Pathexec_code(void* arg)
{
    ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    while (1)
    {
    //For autonomous mode
        if(auto_flag > 0){
            ESP_ERROR_CHECK(get_path(auto_flag));
            if(!auto_stop_flag)
                ESP_LOGI(TAG, "Path executed successfully");
            else{
                ESP_LOGI(TAG, "Path execution stopped");
                auto_stop_flag = 0;
            }
            docking_flag = 1;   //Only available when out of auto either via stop or after completion
            vTaskDelete(NULL);
        }
    //For docking mode
        else if(docking_enable){
            ESP_ERROR_CHECK(get_path(lastAutoPath));
            if(!auto_stop_flag)
                ESP_LOGI(TAG, "Path executed successfully");
            else{
                ESP_LOGI(TAG, "Path execution stopped");
                auto_stop_flag = 0;
            }
            docking_enable = 0;
            vTaskDelete(NULL);
        }
        else{
            vTaskDelay(10);
            vTaskDelete(NULL);
        }
    }  
}

/*Execute the local_flag'th path. It incorporates the autonomous algorithm which is responsible for replicating the 
recorded path as well docking execution. Some of the features are reading the coordinates from the file,
calling sensing, actuation function, pause and stop of auto path. */
esp_err_t get_path(int local_flag)
{
    char str[LINE_LEN], temp[500] = ""; // Change this temp to calloc
    int linectr = 0, count_flag = 1;
    
    gpio_intr_enable(LEFT_ENCODERA);    //Enabled when in active mode(ready to move)
    gpio_intr_enable(RIGHT_ENCODERA);
    flag = -1; 
    move_stop();

    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    if (f_r == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    /*To get the path in temp variable, in the form of string */
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
    
    fclose(f_r);
    if (docking_enable)
    {
        docking_algo(temp);  //update this fucntion from c file.
    }
    ESP_LOGI(TAG, "Get Path print %s", temp);
    /*To get x,y coordinates of the path*/
    char *token, *token1;
    token = strtok(temp, " ");    //The elements are seperated by " "
    while(token!=NULL)					//iterate through each of the elements
    {
        token1 = strtok(NULL, " ");             //Get the next element
        if (token1 != NULL)
            xCoor = atof(token);       //Convert the value from string to float//printf("token = %s\n",token);
        else
            break;
        token = strtok(NULL, " ");
        if (token != NULL)
            yCoor = atof(token1);       //Convert the value from string to float//printf("token1 = %s\n",token1);
        else
            break;   
        ESP_LOGI(TAG, "From List(X,Y):(%f, %f)",xCoor, yCoor);
        /*All the code to reach the destination with obstacle avoidance*/
        //if(run)
        updateParams(xCoor, yCoor);
        while (!run && !auto_stop_flag)
        {
            if (!auto_pause_flag)
            {
                //sensing();
                actuationAuto();
            }
            else{
                flag = -1;
            }
        }
        ESP_LOGI(TAG, "Current point(X,Y):(%f, %f))",current_point[0],current_point[1]);
        if(auto_stop_flag){  //When /STOPauto is pressed while the path is being executed
            init_pid();     //Not reseting
            break;
        }          
    }
    lastAutoPath = auto_flag;
    auto_flag = 0;

    gpio_intr_disable(LEFT_ENCODERA);  //Disabled interrupt once done  
    gpio_intr_disable(RIGHT_ENCODERA);

    return ESP_OK;
}

/*Reverse the order of coordinates of the last executed path, till the latest x and y coordinates.
This function is called inside get_path() which executes the new path returned by the docking_algo()*/
void docking_algo(char *pathString)
{
    char *token, *token1;
    float pathArray[100] = {0};
    float temopcoord = 0.0;
    int i=0;
    //Getting the last path coordinates till the latest coordinates executed in the pathArray 
    token = strtok(pathString, " ");
    while(token != NULL) {
        token1 = strtok(NULL, " ");             //Get the next element
        if (token1 != NULL)   
            pathArray[i] = atof(token);       //Convert the value from string to float
        else
            break;
        token = strtok(NULL, " ");
        if (token != NULL)     
            pathArray[i+1] = atof(token1);       //Convert the value from string to float
        else
            break;
        if (pathArray[i] == xCoor && pathArray[i+1] == yCoor) {
            break;
        }
        //printf("Array[%d] (X,Y):(%f,%f) \n", i, pathArray[i], pathArray[i+1]);
        i=i+2;
    }

    //To reverse the patharray for reaching home point
    for (size_t j = 0; j < i/2; j=j+2) {
        temopcoord = pathArray[j];
        pathArray[j] = pathArray[i-2-j];
        pathArray[i-2-j] = temopcoord;
        temopcoord = pathArray[j+1];
        pathArray[j+1] = pathArray[i-1-j];
        pathArray[i-1-j] = temopcoord;
    }
    int r=0;
    while (r<i) {
        //printf("(X,Y):(%f,%f) Array after reversing\n", pathArray[r], pathArray[r+1]);
        r=r+2;
    }
    //Converting the array to a string
    char tempvar[9] = "";
    strcpy(pathString, "");
    for (int q = 0; q < i; q++) {
        snprintf(tempvar, 9, "%f", pathArray[q]);
        strcat(pathString, tempvar);
        strcat(pathString, " ");
    }
    strcat(pathString, "\n");
}

/*This function is called every time when new coordinates are read from the list, Its job is to calculate 
the distance and angle required from prev_point to xd and yd. */
void updateParams(double xd, double yd)
{
    dist_required = sqrt(pow(yd - prev_point[1],2)+pow(xd - prev_point[0],2));  //Distance in meters
    /*The angle calculated here is absolute from positive x-axis, range is from +180 to -180. Clockwise -ve and anti +ve*/
    if (xd-prev_point[0] >= 0){
        angle_required = atan((yd-prev_point[1])/(xd-prev_point[0]));
    }
    else{
        angle_required = -atan((xd-prev_point[0])/(yd-prev_point[1]));
        if(yd-prev_point[1] >= 0)
            angle_required = angle_required + M_PI/2;
        else
            angle_required = angle_required - M_PI/2;
    }
    prev_point[0] = xd;
    prev_point[1] = yd;
    ESP_LOGI(TAG, "Start point(X,Y):(%f, %f), dist_required,angle_required:(%f, %f)",prev_point[0],prev_point[1],dist_required,angle_required);  //*57.29
    if(!(angle_required == angle_required))
        doneFlag = 1;
    dist_traversed = 0;
    run = 0;
}

/*Main funtion that responds to the environment whether a obstacle is present or not. 
It also calculate the current_point which is essentially the instantaneous coordinate of the bot 
being updated with the help of encoders. In case of an obstacle it decided with the various cases which 
way to move the bot to avoid collision.*/
void actuationAuto(){
    if (flag == 0)
        dist_traversed = (leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick; //In meters //To know if the target has been reached
    else if (flag == 1 || flag == 2){  //angle_rotated is a static variable and is not being reset to zero in between
        if (flag == 1){ //anti clockwise for positive angle
            //printf("Before:  leftRot: %d, LeftTicks: %d\n", leftRot, leftTicks); 
            angle_rotated = angle_rotated + ((leftRot- prevleftRot)*ENCODERresolution + (leftTicks- prevleftTicks)*wheeldist_perTick*2)/wheelbase; //Angle of the bot in radians
            prevleftRot = leftRot;
            prevleftTicks = leftTicks;
            //printf("After: %d\n", ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick*2)/wheelbase); 
        }
        else{           //Clockwise negative angle
            angle_rotated = angle_rotated - ((leftRot- prevleftRot)*ENCODERresolution + (leftTicks- prevleftTicks)*wheeldist_perTick*2)/wheelbase; //Angle of the bot in radians
            prevleftRot = leftRot;
            prevleftTicks = leftTicks;
        }
    }
    if(angle_rotated > M_PI){
        angle_rotated = angle_rotated - 2*M_PI;
    }
    if(angle_rotated < -M_PI){
        angle_rotated = angle_rotated + 2*M_PI;
    }
    current_point[0] = stop_point[0] + dist_traversed*cos(angle_rotated); //calculates co-ordinates of the current point using the 
    current_point[1] = stop_point[1] + dist_traversed*sin(angle_rotated); //point where the orientation of the bot was last changed


    if(esp_timer_get_time()-prev_time> 500000 ){
        ESP_LOGI(TAG, "Current point(X,Y):(%f, %f), dist_traversed,angle_rotated:(%f, %f), flag: %d",current_point[0],current_point[1],dist_traversed,angle_rotated,flag);
        prev_time = esp_timer_get_time();
    }
    // if((prev_time>=time_flag+0.5) && (prev_time<time_flag+1))  //this for moving forward slowly after sensing 
    //     forwardSlow(1);

    // if(detect_flag==0 && prev_time>=time_flag+1)
    normal_motion();

   // else
    if(detect_flag==1 && prev_time>=time_flag+1){
        //ESP_LOGI(TAG, "after normal_motion");
        if(obstacle_flag[1]==1 && obstacle_flag[3]==0){
	    	time_flag = prev_time;
	    	rotateSlow(-1);
	    	dist_traversed = 0;
	    	update_stopPoint();
    	}
        else if(obstacle_flag[3]==1 && obstacle_flag[1]==0){
    		time_flag = prev_time;
    		rotateSlow(1);
    		dist_traversed = 0;
	    	update_stopPoint();
    	}
        else if(obstacle_flag[1]==1 && obstacle_flag[3]==1){
    		time_flag = prev_time;
    		rotateSlow(1);
    		dist_traversed = 0;
	    	update_stopPoint();
    	}
        else if(obstacle_flag[1]==0 && obstacle_flag[3]==0 && obstacle_flag[2]==1){
    		time_flag = prev_time;
    		rotateSlow(1);
            dist_traversed = 0;
            update_stopPoint();
        }
        else if(obstacle_flag[0]==1 || obstacle_flag[4]==1){
            time_flag = prev_time;
            forwardSlow(5);
    	}
        else if(obstacle_flag[0]==0 && obstacle_flag[4]==0){
    		recalculate();
    		detect_flag = 0;
    	}
    }
}

/*To execute the default path when no obstacles are present, Its job is to first rotate to the desired orientation
the to move forward to the required coordinates. Once the goal is achieved it updates the run flag that fectches the next point*/
void normal_motion(){
    lin_speed = DEFAULT_LIN_SPEED;
    ang_speed = DEFAULT_ANG_SPEED;
        //ESP_LOGI(TAG, "normal_motion");
    if(rotating_flag == 1){
        if(doneFlag == 1){
            ESP_LOGI(TAG, "Rotation Done flag");
            init_pid();
            flag = -1;
            rotating_flag = 0;
        }
        else
            rotate();
    }
    else{
        if(doneFlag == 1){
            ESP_LOGI(TAG, "forward Done flag");
            init_pid();
            flag = -1;
            run=1;          //Move to next point
            update_stopPoint();
            rotating_flag = 1;
        }
        else
            forward();
    }
}

/*To actuate and record the angular motion of the bot, till angle_required is achieved*/
void rotate(){
    if(fabs(angle_required - angle_rotated) <= 0.02)  //set some threshold after which it can move 
        doneFlag = 1;
    else if((angle_required - angle_rotated) > 0 || (angle_required - angle_rotated) < -3.14){
        flag = 1;    //If diff is positive, then left(+ve for anticlockwise)
    }
    else{
        flag = 2; 
    }
}

/*To actuate and record the forward motion of the bot, till dist_required is achieved*/
void forward(){
    flag = 0; 
    if((dist_required - dist_traversed) < 0.0001)  //set some threshold after which it can move 
        doneFlag = 1;
}

/*Updates the stop point when the angle required and distance required have been achieved*/
void update_stopPoint(){
    stop_point[0] = current_point[0];
    stop_point[1] = current_point[1];
}

/*To recalculate the angle and distance required when an obstacle is detected and path has to be altered*/
void recalculate(){
    dist_required = sqrt(pow(prev_point[1]-current_point[1], 2) + pow(prev_point[0]-current_point[0], 2));    
    if(prev_point[0]-current_point[0] >= 0){
        angle_required = atan((prev_point[1]-current_point[1])/(prev_point[0]-current_point[0]))*180/M_PI;
    }
    else{
        angle_required = -atan((prev_point[0]-current_point[0])/(prev_point[1]-current_point[1]))*180/M_PI;
        if(prev_point[1]-current_point[1] >= 0)
            angle_required = angle_required + 90;
        else 
            angle_required = angle_required - 90;
    }
    update_stopPoint();
    dist_traversed = 0;
    init_pid();
    rotating_flag = 1;
}

/*Checks the Ultrasonic sensors and updates the obstacle_flag array and detect flag. */
void sensing(){

    prev_time = esp_timer_get_time()/1000000;    //Getting current time in microseconds and storing in seconds

   	obstacle_flag[0] = !gpio_get_level(ULTRA1);
   	obstacle_flag[1] = !gpio_get_level(ULTRA2);
   	obstacle_flag[2] = !gpio_get_level(ULTRA3);
   	obstacle_flag[3] = !gpio_get_level(ULTRA4);
   	obstacle_flag[4] = !gpio_get_level(ULTRA5);
    
    detect_flag = obstacle_flag[1] | obstacle_flag[2] | obstacle_flag[3];  //comment this when testing only normal motion
}

/*To decrease the forward speed when an obstacle is close*/
void forwardSlow(int num){
    flag = 0;
    lin_speed = MIN_LINEAR_SPEED*num;
}

/*To decrease the angular speed when an obstacle is close*/
void rotateSlow(int num){
    ang_speed = MIN_ANGULAR_SPEED*abs(num);
    if(num > 0){
        flag = 1;    //left
    }
    else{
        flag = 2;    //right
    }
}

