# IoT Exploration Robot: Arduino Software Solution
## School Assessed Task: VET Integrated Technologies
#### ----------------------------------------------------------------------------------------------
#### Note:
##### Whilst working on this project I will commit all changes to the master branch in order to keep track of my work and for authenticity purposes.
##### I will also create releases at certain stages which you can download in the releases page. I will only ever create releases when the program is in a working condition and I will only do this when major changes have been made.
#### ----------------------------------------------------------------------------------------------
![GitHub Logo](media/IoTER-AC-GitHub-Logo.png)
## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Setup](#setup)

## General info
This project is developed entirely by Liam Price in order to pass the VET IT SAT. The overall function of this software is to be uploaded onto IoT enabled Arduino boards in order to control my IoT exlporation robot, however you may use this software to adapt into your own IoT projects. If your looking for compatiblity for a different type of board such as the ESP-32 chip, look in the branches for the right branch for your board. All ESP-8266 based boards will work on the main 'master' branch. If you want to use the control panel software for this project (windows computers only) go over to my other repo and download what you need there for your control panel. Keep in mind that all control panels that I have built are done in Python, not C/C++
## Technologies
Project is created with:
* Visual Studio + Arduino Extension
* PubSub Client library @ https://github.com/knolleary/pubsubclient/
* Pretty much any IoT enabled development board such as ESP-32 & ESP8266
	
## Setup
To run this project, download it:

```
Go To Releases
Download as ZIP according to the board you are using
If the release readme requires you to make changes to the code such as entering network / MQTT credentials, do that, or it ain't gonna work.
Upload the main .ini file to your Arduino Board
Alternatively download the latest commit from any branch, note that it may not be stable or functional.
```
