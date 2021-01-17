#include "algo.h"

int Lpwm = 0; 
int Rpwm = 0;     //Always remember you can't define the variable in .h file                         

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

/*The following are dummy functions for movement of the bot*/
void move_forward()
{
    ledc_set_duty(motorL.speed_mode, motorL.channel, 4000);   //Update the PWM value to be used by this lED Channel.
    ledc_update_duty(motorL.speed_mode, motorL.channel);      //Use the Updated PWM values
    ledc_set_duty(motorR.speed_mode, motorR.channel, 4000);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}                                                                       

void move_left()
{
    ledc_set_duty(motorL.speed_mode, motorL.channel, 2000);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, 4000);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_right()
{
    ledc_set_duty(motorL.speed_mode, motorL.channel, 1000);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, 2000);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_back()
{
    ledc_set_duty(motorL.speed_mode, motorL.channel, 400);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, 4000);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_stop()
{
    ledc_set_duty(motorL.speed_mode, motorL.channel, 0);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, 0);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
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
    char str1[LINE_LEN];
    int len = 0, linectr = 0;
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
        //if(!feof(f_r))
        //{
        ESP_LOGI(TAG, "%s", str);
        ESP_LOGI(TAG, "Length: %d", strlen(str));

            linectr++;
            if(linectr == n)
            {
                fseek(f_r, -strlen(str), SEEK_CUR);
                fgets(str1, LINE_LEN, f_r);
                char* temp_token = strtok(str1, "\t");
                while(temp_token!=NULL)
                {
                    ch = temp_token[0];
                    temp_token++;
                    //val = DEFAULT_LIN_SPEED * atof(temp_token)/1000.0;       //temp_token has time in milli seconds
                    if(ch == 'f' || ch == 'b')
                        len ++;      
                    temp_token = strtok(NULL, "\t");
                }
                char* result = (char *)calloc((len*2*10+1),sizeof(char));
                //strcpy(result,"");
                //char* result = (char *)calloc(2048,sizeof(char));
                strcpy(result, "");
                ESP_LOGI(TAG, "Length: %d", len*4*9+1);
                char* token = strtok(str, "\t");
                while(token!=NULL)
                {
                    ch = token[0];
                    token++;
                    val = atof(token);
                    if(ch == 'f'){
                        val = DEFAULT_LIN_SPEED * val/1000.0;
                        current.x = prev.x + val*cos(prev.theta*3.14/180.0);
                        current.y = prev.y + val*sin(prev.theta*3.14/180.0);
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
                        val = DEFAULT_LIN_SPEED * val/1000.0;
                        current.x = prev.x - val*cos(prev.theta*3.14/180.0);
                        current.y = prev.y - val*sin(prev.theta*3.14/180.0);
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
                        val = DEFAULT_ANG_SPEED * val/1000.0;
                        prev.theta = prev.theta + val;
                    }
                    else if(ch == 'l'){
                        val = DEFAULT_ANG_SPEED * val/1000.0;
                        prev.theta = prev.theta - val;
                    }
                    token = strtok(NULL, "\t");
                }
                //strcat(result, "\b");
                strcat(result, "\n");
                ESP_LOGI(TAG, "%s", result);
                fprintf(f_w, "%s", result);
            }
            else
                fprintf(f_w, "%s", str);
        //}
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/paths.txt");
    rename("/spiffs/temp.txt", "/spiffs/paths.txt");
    return ESP_OK;
}

/*Execute the local_flag th path*/
esp_err_t get_path(int local_flag)
{
    char str[LINE_LEN];
    int linectr = 0;
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
            linectr++;
            if(linectr == (local_flag+1)) //1st line contains the number of valid paths, so nth path will be on (n+1)th line
                break;
        }
    }
    ESP_LOGI(TAG, "%s", str);
    fclose(f_r);
    //return ESP_OK;
    char* token = strtok(str, "\t");    //The elements are seperated by "\t"
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