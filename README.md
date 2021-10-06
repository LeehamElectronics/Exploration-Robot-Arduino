# IoT Exploration Robot: Arduino Software Solution
## School Assessed Task: VET Integrated Technologies
![GitHub Logo](media/IoTER-AC-GitHub-Logo.png)
## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Setup](#setup)

## General info
This project is developed entirely by Liam Price in order to pass the VET IT School assesed task, however I have documented it here for the public to look at or use too. The overall function of this software is to be uploaded onto IoT enabled Arduino boards in order to control the robot I am making, however you may use this software to adapt into your own IoT projects. I have written the Arduino code to work with ESP32 chips, so if you are using another board, you will need to make some changes to the code. If you want to use the control panel software for this project (windows computers only) go over to my other repo and download what you need there for your control panel. Keep in mind that all control panels that I have built are done in Python, not C/C++ https://github.com/LeehamElectronics/Exploration-Robot-Control-Panel

### 2021 UPDATE:
![Liam Price 740](https://user-images.githubusercontent.com/51737378/136178257-3a163108-259c-4f81-a8f6-b7319d3c3238.jpg)

![18106194T Final Image higher res](https://user-images.githubusercontent.com/51737378/136177903-2d3dfe82-e57e-40a3-ab96-2fb9eda990df.jpg)

#### So this project ended up winning the VCE Top Designs for VET Integrated Technologies! It is currently on display in the Melbourne Museum, you can view it here in a virtual tour: https://museumsvictoria.com.au/melbournemuseum/learning/top-designs-2021/
#### And here is a little article they posted about it: https://museumsvictoria.com.au/melbournemuseum/learning/top-designs-2021/vce-vet-programs/integrated-technologies/
#### And lastly here is an embarrassing interview they posted of me where I say 'umm' more than anything else: https://www.youtube.com/watch?v=ty_FZUM7rPA

Here is a YouTube video with a basic demonstration of the system: [YouTube Video](https://www.youtube.com/watch?v=0TiRYpMsIOc&t=27s) 

And here is a link to the engineering and design folio for this project: [Engineering Folio](https://drive.google.com/file/d/1ohp7j_BZnhxLjLv0mqf6PKU2e-KAmkB1/view?usp=sharing) 
	

## Technologies
Project is created with:
* Visual Studio + Arduino Extension (VisualMicro)
* PubSub Client library @ https://github.com/knolleary/pubsubclient/
* Other required libraries are mentioned in the internal documentation
* Pretty much any IoT enabled development board such as ESP-32
	
## Setup
To run this project, download it:

```
Go To Releases
Download as ZIP according to the board you are using
If the release readme requires you to make changes to the code such as entering network / MQTT credentials, do that, or it ain't gonna work.
Upload the main .ini file to your Arduino Board
Alternatively download the latest commit from any branch, note that it may not be stable or functional.
```
