# UV-sterilization-bot

This repo showcase the firmware of a modular semi-autonomous UV sterilization bot built using ESP32. It also contains the simulation scenes from Coppeliasim and Arduino code which was used to mimic encoders feedback, due to the unavailability of the hardware setup. The project is owned by [IOT-unifyAI Tech](https://www.linkedin.com/company/iot-unifyai-tech/about/) and was developed by multiple people working on different aspect of the code. 

## Description

The aim of building the robot was to develop a low-cost modular autonomous differential drive bot that can be used for various applications by switching the attachment on the bot, UV lamp in the present case for sterilization of the hospital ward and ICUs. The autonomous functionality was kept limited to keep the components and cost to a minimum. ESP32 is selected as the main controller because of its cost, speed, and dual-core processing capability. In addition, it features an inbuilt wifi module for running the webserver. [ESP-IDF framework](https://github.com/espressif/esp-idf) is used as the main application for developing the embedded firmware.

Below are some links to detailed documentation and video discussing the selection of components and features as well explaining the ESP32 code.

>Main detailed document

>Code explanation document

>Code explanation video 

## Hardware

The main components of the bot are BLDC Hub motors for differential drive, encoders for localization using dead reckoning, Ultrasonic sensors for obstacle avoidance while in autonomous mode, and ESP32 as the main microcontroller running the web server for remote control, storage of network credentials, and recorded paths using SPIFFS file system, as well as sensing and actuation using GPIO.

## Coppeliasim



## Arduino



## MVP Setup

>Images of the current bot

## Demo

>>demo video
