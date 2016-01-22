# Project Karman Avionics Firmware

This repository contians the firmware for the project karman rocket.

## Setup

The project solution can be opened using the Atmel Studio software. It should be able to be built directly with only a few compiler warnings.

### Installing Atmel Studio 7

Atmel Studio 7 can be downloaded from here: http://www.atmel.com/tools/ATMELSTUDIO.aspx

If you have Visual Studio 2015 Update 1 installed, you may run into issues while starting Atmel Studio. If this occurs, try the workaround described here: http://bit.ly/1OtUPBs

## Software Architecture

We are using FreeRTOS to manage the different MCU operations effectively.
