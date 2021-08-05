/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                     PLEASE READ THE FOLLOWING!                                    *
 * This open source software was written by Liam Price 2020 and is FREE to use...                    *
 * The purpose of this software is to compile and run on an Arduino based IoT device, like an ESP32  *
 *                                                                                                   *
 * This is used to control an 'Exploration Robot' that I designed to be controlled from a computer   *
 * steering wheel joystick, however you can use the code for any robot project, if you want to see   *
 * the Exploration Robot I designed this for software, visit my GitHub Repo for it:                  *
 * https://github.com/LeehamElectronics/Exploration-Robot-Arduino                                    *
 *                                                                                                   *
 * And here is the open source Python Control Panel I made for it:                                   *
 * https://github.com/LeehamElectronics/Exploration-Robot-Control-Panel                              *
 *                                                                                                   *
 * If you'd like to support my open source work, buy me a coffee:                                    *
 * https://www.paypal.com/paypalme/liamproice/														 *
 *																									 *
 * Check out my website and portfolio at https://ldprice.com								         *
 *																									 *
 *                                                                                                   *
 * And if you need help, feel free to email me at liam@ldprice.com                                   *
 * Thanks!                                                                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

 /*

 ██╗░░░░░███████╗███████╗██╗░░██╗░█████╗░███╗░░░███╗██╗░██████╗      ░██████╗░█████╗░███████╗████████╗░██╗░░░░░░░██╗░█████╗░██████╗░███████╗
 ██║░░░░░██╔════╝██╔════╝██║░░██║██╔══██╗████╗░████║╚█║██╔════╝      ██╔════╝██╔══██╗██╔════╝╚══██╔══╝░██║░░██╗░░██║██╔══██╗██╔══██╗██╔════╝
 ██║░░░░░█████╗░░█████╗░░███████║███████║██╔████╔██║░╚╝╚█████╗░      ╚█████╗░██║░░██║█████╗░░░░░██║░░░░╚██╗████╗██╔╝███████║██████╔╝█████╗░░
 ██║░░░░░██╔══╝░░██╔══╝░░██╔══██║██╔══██║██║╚██╔╝██║░░░░╚═══██╗      ░╚═══██╗██║░░██║██╔══╝░░░░░██║░░░░░████╔═████║░██╔══██║██╔══██╗██╔══╝░░
 ███████╗███████╗███████╗██║░░██║██║░░██║██║░╚═╝░██║░░░██████╔╝      ██████╔╝╚█████╔╝██║░░░░░░░░██║░░░░░╚██╔╝░╚██╔╝░██║░░██║██║░░██║███████╗
 ╚══════╝╚══════╝╚══════╝╚═╝░░╚═╝╚═╝░░╚═╝╚═╝░░░░░╚═╝░░░╚═════╝░     ╚═════╝░░╚════╝░╚═╝░░░░░░░░╚═╝░░░░░░╚═╝░░░╚═╝░░╚═╝░░╚═╝╚═╝░░╚═╝╚══════╝

 */

 /* include library */
#include "include/configuration.h"
#include "include/mqttConfiguration.h"
#include "include/gpioConfiguration.h"

#include <WiFiManager.h>          // Use this fork because the original library has issues with HTTP headers on the ESP32: https://github.com/Brunez3BD/WIFIMANAGER-ESP32
#include <WiFi.h>
// UPDATE: WiFiManager Development builds seem to work better and is more stable: https://github.com/tzapu/WiFiManager/tree/development
/*
 * This 'PubSubClient' is actually the MQTT Arduino Library that we are using to make the Arduino
 * board communicate with the Control Panel I made, you might be wondering, why is it called
 * 'PubSubClient' and not ArduinoMQTT? Well there is a more 'official' Arduino MQTT library available,
 * however it is very new at the time of writing and I prefer using these more 'established' libraries
 * for stability.
 */
#include <PubSubClient.h> 
 /* #include <ESP32_Servo.h> having issues? Use the following ESP32 Library instead: */
#include <ESP32Servo.h> /* Get it from https://github.com/madhephaestus/ESP32Servo */
//#include <Arduino.h> /* Surprising? */
#include <analogWrite.h> /* this was required for ESP32 chips to use analogWrite for some reason, look it up https://github.com/ERROPiX/ESP32_AnalogWrite */
/*
 * Used to store information even when the robot is turned off, dont abuse it, only has a set
 * amount of read / write cycles. If you need to use EEPROM A LOT try using a SD Card instead!
 */
#include <EEPROM.h> 
#include <ArduinoJson.h> /* This is a very important library and takes a lot to learn properly, definitely worth it though */ 

/* OLED Display 64x128 setup code, find more examples at https://github.com/olikraus/U8glib_Arduino/blob/master/examples/Bitmap/Bitmap.ino for different OLED's*/
/* If you want to make your own images to display to the OLED LCD, use this: http://en.radzio.dxp.pl/bitmap_converter/ */
#include "U8glib.h" // not using atm because im out of IO on the ESP32 ):

/*
 * The following libraries are needed for the MPU6050 to function:
 *
 * https://github.com/ElectronicCats/mpu6050
 * Connect MPU VCC to Arduino 5V (or if it's a 3.3V board use that voltage instead)
 * Connect GND to GND
 * Connect SCL to Arduino SCL
 * Connect SDA to Arduino SDA
 *
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <math.h>
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 39  // use pin 39 which is 5VN on an ESP32 chip (next to pin 34 GPIO)  CHAJNGED TO PIN 36 OOOOOF

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

boolean update_gyro_pos = true;
float gyro_val_0;
float gyro_val_1;
float gyro_val_2;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

/* EEPROM Flash Memory setup: */
#define EEPROM_SIZE 22 /* don't touch unless your adding more EEPROM functionality... */
/* MQTT Variables Below, make sure to fill them in with your MQTT credentials, or it won't work! */
#define mqtt_user "IoTER"
#define mqtt_password "*!xlVZ3jVOGw"
/* This is the IP address or hostname of your MQTT server... */
const char* mqtt_server = "skynet.ciscofreak.com";

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;

/* Servo Setup */
Servo craneYawServo;
Servo cranePitchServo1;
Servo cranePitchServo2;
Servo craneElbowServo;
Servo craneGripperServo;

/* Global tank control variables */
int steeringFowardMomentumVal = 0;
int steeringLeftMomentumVal = 0;
int steeringRightMomentumVal = 0;
boolean isMomentumLeft = false;
boolean isMomentumRight = false;
boolean isMomentumBackwards = false;
/* crane rotation check: */
boolean isYawRotatingLeft = false;
boolean isYawRotatingRight = false;
/* crane center_axis movement check: */
boolean isPitchMovingUp = false;
boolean isPitchMovingDown = false;
/* crane forarm movement check: */
boolean isElbowMovingUp = false;
boolean isElbowMovingDown = false;
/* crane gripper_axis movement check: */
boolean isGrippperOpening = false;
boolean isGrippperClosing = false;
/* Relay variable logic */
boolean isHeadlightOn = false;

/*
 * Initial crane position variables that are written
 * to servos on startup. These values are replaced by
 * the values stored in EEPROM but I have left values
 * assigned to them anyways in case EEPROM fails...
*/
int craneYawPos = 90;
int cranePitchPos1 = 90;
int cranePitchPos2 = 90;
int craneElbowPos = 90;
int craneGripperPos = 90;

/*
 * Crane movement constraint variables, these are
 * also stored and read from EEPROM and can be
 * virtually adjusted via the Python Control Panel
*/
/***     Rotation axis    ***/
int craneYawConstraintX;
int craneYawConstraintY;
/***     Center axis      ***/
int cranePitchConstraintX;
int cranePitchConstraintY;
/***     Forearm axis     ***/
int craneElbowConstraintX;
int craneElbowConstraintY;
/***    Gripper axis      ***/
int craneGripperConstraintX;
int craneGripperConstraintY;

/*
 * Crane movement speed variables are also
 * stored in EEPROM and are adjustable for
 * conveinence
*/
/***     Rotation axis    ***/
int craneYawMovementSpeed;
/***    Center axis_1     ***/
int cranePitchMovementSpeed;
/***     Forearm axis     ***/
int craneElbowMovementSpeed;
/***    Gripper axis      ***/
int craneGripperMovementSpeed;

/* Time management */
unsigned long craneMovementUpdaterPreviousMillis = 0; // will store last time 'Crane Rotation' was updated
unsigned long sysPwrUsageUpdaterPreviousMillis = 0; // will store last time 'Crane current_value' was updated
unsigned long gyroValUpdaterPreviousMillis = 0; // stores last time gyro pos is updated and sent
long craneMovementUpdaterInterval = 100; // interval at which to update crane's rotation (milliseconds)
long sysPwrUpdaterInterval = 2000; // interval at which to update crane's current usage (milliseconds)
long gyroValUpdaterInterval = 1500; // amount of time between gyro value checks occur and are sent to mqtt broker

/* unproccessed axis values from steering wheel */
float controllerAxis0Raw; // ranges between turning full lock left and full lock right
float controllerAxis1Raw; // foward acceleration
float controllerAxis2Raw; // backwards acceleration

/* OLED Display 64 x 128 (height x width) pixels setup code: */
// U8GLIB_SH1106_128X64 u8g(10, 11, 9, 6, 5);   /* CLK=10, MOS=11, CS=9, DC=6, Reset=5 */


int enable_wifi_portal_button_state = 0; // stores the current state of the button (HIGH or LOW)

/**************************************** RECEIVE DATA FROM MQTT ******************************************/
void callback(char* topic, byte* payload, unsigned int length) {
	Serial.print("Topic is:");
	Serial.println(topic);
	if (strcmp(topic, "/ExpR/in/SW_axs_0/Rs") == 0) {
		// axis 0 is for left and right turning
		String axs_0_temp = String((char*)payload);
		controllerAxis0Raw = axs_0_temp.toFloat();
		// Serial.print("SOFT RIGHT");
		if (isMomentumBackwards == true) {
			Serial.println("SOFT TURN RIGHT MOTOR BACK");
			motor_back();
		}
		else {
			Serial.println("SOFT TURN RIGHT MOTOR FOWARD");
			motor_foward();
		}
		if (controllerAxis0Raw == 1.00) {
			Serial.println("STEERING WHEEL CENTER");
			isMomentumLeft = false;
			isMomentumRight = false;
		}
		else {
			isMomentumRight = true;
		}

		/* accelerate tank with slightly smaler PWM on the right hand side */
		steeringRightMomentumVal = steeringFowardMomentumVal * controllerAxis0Raw;
		accelerate_angle(steeringFowardMomentumVal, steeringLeftMomentumVal, steeringRightMomentumVal);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_0/Rh") == 0) {
		// axis 0 is for left and right turning
		String axs_0_temp = String((char*)payload);
		controllerAxis0Raw = axs_0_temp.toFloat();
		// Serial.print("HARD RIGHT");
		digitalWrite(leftMotorPosPin, HIGH);
		digitalWrite(leftMotorNegPin, LOW);
		digitalWrite(rightMotorPosPin, LOW);
		digitalWrite(rightMotorNegPin, HIGH);
		/* accelerate tank with slightly larger PWM on the right hand side with inversed polarity (full lock "tank steering" standard) */
		steeringRightMomentumVal = steeringFowardMomentumVal * controllerAxis0Raw;
		// Serial.print(global_steering_right);
		isMomentumRight = true;
		accelerate_angle(steeringFowardMomentumVal, steeringLeftMomentumVal, steeringRightMomentumVal);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_0/Ls") == 0) {
		// axis 0 is for left and right turning
		String axs_0_temp = String((char*)payload);
		controllerAxis0Raw = axs_0_temp.toFloat();
		// Serial.print("SOFT LEFT");
		if (isMomentumBackwards == true) {
			Serial.println("SOFT TURN LEFT MOTOR BACK");
			motor_back();
		}
		else {
			Serial.println("SOFT TURN LEFT MOTOR FOWARD");
			motor_foward();
		}
		if (controllerAxis0Raw == 1.00) {
			Serial.println("STEERING WHEEL CENTER!");
			isMomentumLeft = false;
			isMomentumRight = false;
		}
		else {
			isMomentumLeft = true;
		}
		/* accelerate tank with slightly smaler PWM on the left hand side */
		steeringLeftMomentumVal = steeringFowardMomentumVal * controllerAxis0Raw;
		// Serial.print(global_steering_left);
		accelerate_angle(steeringFowardMomentumVal, steeringLeftMomentumVal, steeringRightMomentumVal);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_0/Lh") == 0) {
		// axis 0 is for left and right turning
		String axs_0_temp = String((char*)payload);
		controllerAxis0Raw = axs_0_temp.toFloat();
		// Serial.print("HARD LEFT");
		digitalWrite(leftMotorPosPin, LOW);
		digitalWrite(leftMotorNegPin, HIGH);
		digitalWrite(rightMotorPosPin, HIGH);
		digitalWrite(rightMotorNegPin, LOW);
		/* accelerate tank with slightly larger PWM on the left hand side with inversed polarity (full lock "tank steering" standard) */
		steeringLeftMomentumVal = steeringFowardMomentumVal * controllerAxis0Raw;
		// Serial.print(global_steering_left);
		isMomentumLeft = true;
		accelerate_angle(steeringFowardMomentumVal, steeringLeftMomentumVal, steeringRightMomentumVal);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_1") == 0) {
		// axis 1 is for accelerating fowards
		String axs_1_temp = String((char*)payload);
		controllerAxis1Raw = axs_1_temp.toInt();
		steeringFowardMomentumVal = map(controllerAxis1Raw, -100, 100, 1000, 0);
		Serial.println(steeringFowardMomentumVal);
		isMomentumBackwards = false;
		accelerate(steeringFowardMomentumVal);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_2") == 0) {
		// axis 0 for going backwards
		String axs_2_temp = String((char*)payload);
		controllerAxis2Raw = axs_2_temp.toInt();
		steeringFowardMomentumVal = map(controllerAxis2Raw, -100, 100, 1000, 0);
		Serial.println(steeringFowardMomentumVal);
		isMomentumBackwards = true;
		accelerate_back(steeringFowardMomentumVal);
	}
	else if (strcmp(topic, "/ExpR/in") == 0) {
		Serial.print("Topic match 'in'");
		if (payload[0] == '1') {
			Serial.println("Going foward!");
			manual_foward();
		}
		else if (payload[0] == '2') {
			Serial.println("Going Back!");
			manual_back();
		}
		else if (payload[0] == '3') {
			Serial.println("Going Left!");
			manual_left();
		}
		else if (payload[0] == '4') {
			Serial.println("Going Right!");
			manual_right();
		}
		else if (payload[0] == '5') {
			Serial.println("STOP!");
			manual_stop();
		}
		else if (payload[0] == 'g') {
			Serial.println("sending current crane pos values to control panel");
			send_crane_pos();
		}
		else if (payload[0] == 's') {
			Serial.println("saving current crane pos values as default");
			save_crane_pos();
		}
		else if (payload[0] == 'k') {
			Serial.println("sending current crane constraints to control panel");
			send_crane_constraints();
		}
		else if (payload[0] == 'o') {
			Serial.println("saving current crane constraints into EEPROM");
			save_crane_constraints();
		}
		else if (payload[0] == 'u') {
			Serial.println("saving current crane movement speed into EEPROM");
			save_crane_movement_speed();
		}
		else if (payload[0] == 'y') {
			Serial.println("sending current crane speed values to control panel");
			send_crane_movement_speed_values();
		}
	}
	else if (strcmp(topic, "/ExpR/in/newC") == 0) {
		Serial.print("recieving new crane pos'");
		String json_temp = String((char*)payload);
		Serial.print("Raw JSON Data: ");
		Serial.println(json_temp);

		DynamicJsonBuffer  jsonBuffer(200);

		JsonObject& json_obj = jsonBuffer.parseObject(json_temp);
		if (!json_obj.success()) {
			Serial.println("parseObject() failed");
			return;
		}

		/***     Rotation axis    ***/
		craneYawPos = json_obj["rot"];
		/***    Center axis_1     ***/
		cranePitchPos1 = json_obj["ca_1"];
		/***    Center axis_2     ***/
		cranePitchPos2 = json_obj["ca_2"];
		/***     Forearm axis     ***/
		craneElbowPos = json_obj["fa"];
		/***    Gripper axis      ***/
		craneGripperPos = json_obj["grip"];

		apply_current_crane_pos_values();

	}
	else if (strcmp(topic, "/ExpR/in/ncc") == 0) {
		Serial.print("recieving new crane constraint values'");
		String json_temp = String((char*)payload);
		Serial.print("Raw JSON Data: ");
		Serial.println(json_temp);

		DynamicJsonBuffer  jsonBuffer(200);

		JsonObject& json_obj = jsonBuffer.parseObject(json_temp);
		if (!json_obj.success()) {
			Serial.println("parseObject() failed for crane_constraints");
			return;
		}

		/***     Rotation axis    ***/
		craneYawConstraintX = json_obj["rcx"];
		craneYawConstraintY = json_obj["rcy"];
		/***    Center axis     ***/
		cranePitchConstraintX = json_obj["ccx"];
		cranePitchConstraintY = json_obj["ccy"];
		/***     Forearm axis     ***/
		craneElbowConstraintX = json_obj["fcx"];
		craneElbowConstraintY = json_obj["fcy"];
		/***    Gripper axis      ***/
		craneGripperConstraintX = json_obj["gcx"];
		craneGripperConstraintY = json_obj["gcy"];

	}
	else if (strcmp(topic, "/ExpR/in/ncs") == 0) {
		Serial.print("recieving new crane movement speed values'");
		String json_temp = String((char*)payload);
		Serial.print("Raw JSON Data: ");
		Serial.println(json_temp);

		DynamicJsonBuffer  jsonBuffer(200);

		JsonObject& json_obj = jsonBuffer.parseObject(json_temp);
		if (!json_obj.success()) {
			Serial.println("parseObject() failed for crane_constraints");
			return;
		}

		/***     Rotation axis    ***/
		craneYawMovementSpeed = int(json_obj["rs"]);
		/***    Center axis_1     ***/
		cranePitchMovementSpeed = int(json_obj["cs"]);
		/***     Forearm axis     ***/
		craneElbowMovementSpeed = int(json_obj["fs"]);
		/***    Gripper axis      ***/
		craneGripperMovementSpeed = int(json_obj["gs"]);

	}
	else if (strcmp(topic, "/ExpR/in/nt") == 0) {
	Serial.print("recieving new timings values'");
	String json_temp = String((char*)payload);
	Serial.print("Raw JSON Data: ");
	Serial.println(json_temp);

	DynamicJsonBuffer  jsonBuffer(200);

	JsonObject& json_obj = jsonBuffer.parseObject(json_temp);
	if (!json_obj.success()) {
		Serial.println("parseObject() failed for timings_values");
		return;
	}

	/***      current reading / sending speed:    ***/
	sysPwrUpdaterInterval = long(json_obj["c"]);
	/***      gyro reading / sending speed:       ***/
	gyroValUpdaterInterval = long(json_obj["g"]);

	}
	else if (strcmp(topic, "/ExpR/in/SW_btn_d") == 0) {
		char button_temp = payload[0];
		Serial.println("butn DOWN topic");
		Serial.println(button_temp);
		if (button_temp == '4') {
			isYawRotatingLeft = true;
			Serial.println("crane rot left...");
		}
		else if (button_temp == '5') {
			isYawRotatingRight = true;
			Serial.println("crane rot right...");
		}
		else if (button_temp == '6') {
			light_toggle();
		}
		else if (button_temp == '7') {
			Serial.println("crane open gripper...");
			isGrippperOpening = true;
			isGrippperClosing = false;
		}
		else if (button_temp == 's') {
			Serial.println("crane close gripper...");
			isGrippperOpening = false;
			isGrippperClosing = true;
		}
	}
	else if (strcmp(topic, "/ExpR/in/SW_btn_u") == 0) {
		char button_temp = payload[0];
		Serial.println("butn UP topic");
		Serial.println(button_temp);
		if (button_temp == '4') {
			isYawRotatingLeft = false;
			Serial.println("crane NOT rot left...");
		}
		else if (button_temp == '5') {
			isYawRotatingRight = false;
			Serial.println("crane NOT rot right...");
		}
		else if (button_temp == '6') {
			light_toggle();
		}
		else if (button_temp == '7') {
			Serial.println("crane NOT open gripper...");
			isGrippperOpening = false;
			isGrippperClosing = false;
		}
		else if (button_temp == 's') {
			Serial.println("crane NOT close gripper...");
			isGrippperOpening = false;
			isGrippperClosing = false;
		}
	}
	else if (strcmp(topic, "/ExpR/in/hat") == 0) {
		String temp_local = String(char(payload[0]));
		String hat_temp = temp_local;
		Serial.print("steering wheel HAT: ");
		Serial.println(hat_temp);
		if (hat_temp == "x") {
			Serial.println("crane forearm UP");
			isElbowMovingUp = true;
			isElbowMovingDown = false;
		}
		else if (hat_temp == "y") {
			Serial.println("crane forearm DOWN");
			isElbowMovingDown = true;
			isElbowMovingUp = false;
		}
		else if (hat_temp == "v") {
			Serial.println("crane center_axis UP");
			isPitchMovingDown = false;
			isPitchMovingUp = true;
		}
		else if (hat_temp == "c") {
			Serial.println("crane center_axis DOWN");
			isPitchMovingDown = true;
			isPitchMovingUp = false;
		}
		else if (hat_temp == "b") {
			Serial.println("crane HALT");
			isElbowMovingUp = false;
			isElbowMovingDown = false;
			isPitchMovingDown = false;
			isPitchMovingUp = false;
			isYawRotatingLeft = false;
			isYawRotatingRight = false;
		}
	}
	else {
		Serial.print("Topic not relevant!");
	}
}

void setup() {

	Serial.begin(115200);
	Serial.println("Exploration Robot Systems Starting, Built and Designed By Liam Price 2020");
	Serial.println("Starting LCD Panel...");

	// initialize EEPROM with predefined size
	EEPROM.begin(EEPROM_SIZE);
	delay(10);

	/* initialise all motor outputs */
	pinMode(leftMotorPosPin, OUTPUT);
	pinMode(leftMotorNegPin, OUTPUT);
	pinMode(rightMotorPosPin, OUTPUT);
	pinMode(rightMotorNegPin, OUTPUT);
	pinMode(leftMotorPwmPin, OUTPUT);
	pinMode(rightMotorPwmPin, OUTPUT);
	pinMode(enableWifiPortalPin, INPUT);
	/* Relay outputs */
	pinMode(headLightRelayPin, OUTPUT);
	light_toggle();

	Serial.println("READING EEPROM VALUES");

	/* The following code reads the default crane positions from EEPROM and writes it to the servos */
	craneYawPos = EEPROM.read(0);
	craneElbowPos = EEPROM.read(1);
	cranePitchPos1 = EEPROM.read(2);
	cranePitchPos2 = EEPROM.read(3);
	craneGripperPos = EEPROM.read(4);

	/***     Rotation axis    ***/
	craneYawConstraintX = EEPROM.read(5);
	craneYawConstraintY = EEPROM.read(6);
	/***    Center axis_1     ***/
	cranePitchConstraintX = EEPROM.read(7);
	cranePitchConstraintY = EEPROM.read(8);
	/***     Forearm axis     ***/
	craneElbowConstraintX = EEPROM.read(9);
	craneElbowConstraintY = EEPROM.read(10);
	/***    Gripper axis      ***/
	craneGripperConstraintX = EEPROM.read(11);
	craneGripperConstraintY = EEPROM.read(12);

	/***     Rotation axis    ***/
	craneYawMovementSpeed = EEPROM.read(13);
	/***    Center axis_1     ***/
	cranePitchMovementSpeed = EEPROM.read(14);
	/***     Forearm axis     ***/
	craneElbowMovementSpeed = EEPROM.read(15);
	/***    Gripper axis      ***/
	craneGripperMovementSpeed = EEPROM.read(16);

	/* Now we will initialize the servo outputs */
	craneYawServo.attach(25);
	cranePitchServo1.attach(27);
	cranePitchServo2.attach(32);
	craneElbowServo.attach(26);
	craneGripperServo.attach(33);

	/* NETWORKING SETUP BELOW */
	//set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
	wifiManager.setAPCallback(configModeCallback);

	Serial.println("Connecting to WiFi...");
	//if it does not connect it starts an access point with the specified name
	//here  "AutoConnectAP"
	//and goes into a blocking loop awaiting configuration
	if (!wifiManager.autoConnect(PORTAL_SSID, PORTAL_PWD)) {
		Serial.println("failed to connect and hit timeout");
		//reset and try again, or maybe put it to deep sleep
		ESP.restart();
		delay(1000);
	}

	//if you get here you have connected to the WiFi
	Serial.println("connected...yeey :)");

	/* Attempt to connect to MQTT server */
	client.setServer(MQTT_SERVER, MQTT_PORT);
	client.setCallback(callback);

	/* This will cause the crane to move into initial position, we will make this happen AFTER it connects to network. */
	apply_current_crane_pos_values();

	Serial.println("Networking Setup done, displaying LCD update, will no init the gyro");
	delay(100);

	/* This following code will setup our MPU6050 Gyroscope by joining a I2C bus */
	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// initialize MPU6050 Gyro
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		Serial.println(F(")..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

}

void loop() {
	if (!client.connected()) {
		/* If we loose connection to MQTT server try to reconnect */
		Serial.println("Not connected to MQTT, trying to connect to broker...");
		reconnect();
	}
	unsigned long currentMillis = millis();
	if (currentMillis - craneMovementUpdaterPreviousMillis >= craneMovementUpdaterInterval) {
		// save the last time we checked the cranes rotation status
		craneMovementUpdaterPreviousMillis = currentMillis;

		if (isYawRotatingLeft == true) {
			if (craneYawPos >= craneYawConstraintY) {
				isYawRotatingLeft = false;
				Serial.println("Crane Rotation Left MAX");
			}
			else {
				craneYawPos = craneYawPos + craneYawMovementSpeed;
				craneYawServo.write(craneYawPos);
				Serial.println(craneYawPos);
			}
		}
		if (isYawRotatingRight == true) {
			if (craneYawPos <= craneYawConstraintX) {
				isYawRotatingRight = false;
				Serial.println("Crane Rotation Right MAX");
				craneYawPos = craneYawConstraintX;
				craneYawServo.write(craneYawPos);
			}
			else {
				craneYawPos = craneYawPos - craneYawMovementSpeed;
				if (craneYawPos <= craneYawConstraintX) {
					isYawRotatingRight = false;
					Serial.println("Crane Rotation Right MAX");
					craneYawPos = craneYawConstraintX;
				}
				else {
					craneYawServo.write(craneYawPos);
					Serial.println(craneYawPos);
				}
			}
		}

		if (isElbowMovingUp == true) {
			if (craneElbowPos >= craneElbowConstraintY) {  // contraint for forearm movement upwards...
				isElbowMovingUp = false;
				Serial.println("crane forearm_upward MAX");
			}
			else {
				craneElbowPos = craneElbowPos + craneElbowMovementSpeed;
				craneElbowServo.write(craneElbowPos);
				Serial.println(craneElbowPos);
			}
		}
		else if (isElbowMovingDown == true) {
			if (craneElbowPos <= craneElbowConstraintX) {  // contraint for forearm movement down...
				isElbowMovingDown = false;
				Serial.println("crane forearm_down MAX");
			}
			else {
				craneElbowPos = craneElbowPos - craneElbowMovementSpeed;
				craneElbowServo.write(craneElbowPos);
				Serial.println(craneElbowPos);
			}
		}
		if (isPitchMovingUp == true) {
			if (cranePitchPos1 >= cranePitchConstraintY) {  // contraint for center_axis movement upwards...
				isPitchMovingUp = false;
				Serial.println("crane center_axis_upward MAX");
			}
			else {
				cranePitchPos1 = cranePitchPos1 + cranePitchMovementSpeed;
				cranePitchServo1.write(cranePitchPos1);
				Serial.println(cranePitchPos1);

				// support servo:
				cranePitchPos2 = cranePitchPos2 - cranePitchMovementSpeed;
				cranePitchServo2.write(cranePitchPos2);
			}
		}
		else if (isPitchMovingDown == true) {
			if (cranePitchPos2 >= cranePitchConstraintX) {  // contraint for center_axis movement down...
				isPitchMovingDown = false;
				Serial.println("crane center_axis_down MAX");
			}
			else {
				cranePitchPos1 = cranePitchPos1 - cranePitchMovementSpeed;
				cranePitchServo1.write(cranePitchPos1);
				Serial.println(cranePitchPos1);

				// support servo:
				cranePitchPos2 = cranePitchPos2 + cranePitchMovementSpeed;
				cranePitchServo2.write(cranePitchPos2);
			}
		}

		if (isGrippperClosing == true) {
			if (craneGripperPos >= craneGripperConstraintY) {  // contraint for center_axis movement upwards...
				isGrippperClosing = false;
				Serial.println("crane gripper closing MAX");
			}
			else {
				craneGripperPos = craneGripperPos + craneGripperMovementSpeed;
				craneGripperServo.write(craneGripperPos);
				Serial.println(craneGripperPos);
			}
		}
		else if (isGrippperOpening == true) {
			if (craneGripperPos <= craneGripperConstraintX) {  // contraint for center_axis movement down...
				isGrippperOpening = false;
				Serial.println("crane gripper opening MAX");
			}
			else {
				craneGripperPos = craneGripperPos - craneGripperMovementSpeed;
				craneGripperServo.write(craneGripperPos);
				Serial.println(craneGripperPos);
			}
		}

	}
	if (currentMillis - sysPwrUsageUpdaterPreviousMillis >= sysPwrUpdaterInterval) {
		sysPwrUsageUpdaterPreviousMillis = currentMillis;
		/*
		 * This code is executed every time 'sysPwrUsageUpdaterPreviousMillis' is reached, which then
		 * reads the current being used by the robot and sends it to the control panel as a float.
		 */
		Serial.print("Uploading current usage value... ");
		float average = 0;
		for (int i = 0; i < 100; i++) {
			average = average + (.044 * analogRead(sysPwrReaderPin) - 3.78) / 100;

			//5A mode, if 20A or 30A mode, need to modify this formula to 
			//(.19 * analogRead(A0) -25) for 20A mode and 
			//(.044 * analogRead(A0) -3.78) for 30A mode

		}
		Serial.println(average);
		String temp = String(average);
		Serial.print("formatted current value: ");
		Serial.println(temp);

		client.publish("/ExpR/out/current_main", (char*)temp.c_str());


	}

	if (currentMillis - gyroValUpdaterPreviousMillis >= gyroValUpdaterInterval) {
		gyroValUpdaterPreviousMillis = currentMillis;
		/*
		 * This code is executed every time 'gyro_value_previousMillis' is reached, which then
		 * reads the gyro values of the robot and sends it to the control panel as a float.
		 */
		Serial.print("Uploading gyro usage value... ");
		send_crane_gyro_values();
	}

	/* This code checks to see if the wifi portal button was pressed */
	enable_wifi_portal_button_state = digitalRead(enableWifiPortalPin);
	if (enable_wifi_portal_button_state == HIGH) {
		Serial.println("WiFi Portal button has been hit!!!");
		WiFi.disconnect(true);
		//reset settings - for testing
		//wifiManager.resetSettings();
		//sets timeout until configuration portal gets turned off
		//useful to make it all retry or go to sleep
		//in seconds
		//wifiManager.setTimeout(120);
		//WITHOUT THIS THE AP DOES NOT SEEM TO WORK PROPERLY WITH SDK 1.5 , update to at least 1.5.11883
		//WiFi.mode(WIFI_STA);
		if (!wifiManager.startConfigPortal("ExpR WiFi Connect Manual", "CiscoExpR123")) {
			Serial.println("failed to connect and hit timeout");
			delay(3000);
			//reset and try again, or maybe put it to deep sleep
			ESP.restart();
			delay(5000);
		}
		//if you get here you have connected to the WiFi
		Serial.println("connected...yeey :)");
		
	}

	/* The following code will update the gyro values locally */
	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
		if (mpuInterrupt && fifoCount < packetSize) {
			// try to get out of the infinite loop 
			fifoCount = mpu.getFIFOCount();
		}
		// other program behavior stuff here
		// .
		// .
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data

	}


	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();
	if (fifoCount < packetSize) {
		//Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
		// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	}
	// check for overflow (this should never happen unless our code is too inefficient)
	else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		//  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

		// read a packet from FIFO
		while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;
		}
			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			gyro_val_0 = round(ypr[0] * 180 / M_PI);
			gyro_val_1 = round(ypr[1] * 180 / M_PI);
			gyro_val_2 = round(ypr[2] * 180 / M_PI);
	}


	client.loop();
}

//gets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager* myWiFiManager) {
	Serial.println("Entered config mode");
	//WiFi.disconnect(true);
	Serial.println(WiFi.softAPIP());
	//if you used auto generated SSID, print it
	Serial.println(myWiFiManager->getConfigPortalSSID());
	// Make LED turn on representing we are in WiFi config mode!
	// red_led_ticker.attach(.2, red_led_tick_func);
}

/********************************************* ACCELERATE *****************************************************/
void accelerate(int f)
{
	// PWM code:
	if (isMomentumLeft == true) {
		Serial.println("ACCELERARTING WITH LEFT PRE VALUES");
		steeringLeftMomentumVal = f * controllerAxis0Raw;
		steeringRightMomentumVal = f;
		if (f == 1000) {
			Serial.println("max RM speed");
			digitalWrite(rightMotorPwmPin, HIGH);
			analogWrite(leftMotorPwmPin, steeringLeftMomentumVal);
		}
		else {
			analogWrite(rightMotorPwmPin, steeringRightMomentumVal);
			analogWrite(leftMotorPwmPin, steeringLeftMomentumVal);
		}
	}
	else if (isMomentumRight == true) {
		Serial.println("ACCELERARTING WITH RIGHT PRE VALUES");
		steeringRightMomentumVal = f * controllerAxis0Raw;
		steeringLeftMomentumVal = f;
		if (f == 1000) {
			Serial.println("max LM speed");
			digitalWrite(leftMotorPwmPin, HIGH);
			analogWrite(rightMotorPwmPin, steeringRightMomentumVal);
		}
		else {
			analogWrite(rightMotorPwmPin, steeringRightMomentumVal);
			analogWrite(leftMotorPwmPin, steeringLeftMomentumVal);
		}
	}
	else {
		Serial.println("ACCELERARTING WITH CENTER FOWARD");
		motor_foward();
		if (f == 1000) {
			Serial.println("max BM speed!");
			digitalWrite(rightMotorPwmPin, HIGH);
			digitalWrite(leftMotorPwmPin, HIGH);
		}
		else {
			analogWrite(leftMotorPwmPin, f);
			analogWrite(rightMotorPwmPin, f);
		}
	}

}

/********************************************* ACCELERATE BACK *****************************************************/
void accelerate_back(int f)
{
	// PWM code:
	if (isMomentumLeft == true) {
		Serial.println("ACCELERARTING backwards WITH LEFT PRE VALUES");
		steeringLeftMomentumVal = f * controllerAxis0Raw;
		steeringRightMomentumVal = f;
		if (f == 1000) {
			Serial.println("max RM speed");
			digitalWrite(rightMotorPwmPin, HIGH);
			analogWrite(leftMotorPwmPin, steeringLeftMomentumVal);
		}
		else {
			analogWrite(rightMotorPwmPin, steeringRightMomentumVal);
			analogWrite(leftMotorPwmPin, steeringLeftMomentumVal);
		}
	}
	else if (isMomentumRight == true) {
		Serial.println("ACCELERARTING backwards WITH RIGHT PRE VALUES");
		steeringRightMomentumVal = f * controllerAxis0Raw;
		steeringLeftMomentumVal = f;
		if (f == 1000) {
			Serial.println("max LM speed");
			digitalWrite(leftMotorPwmPin, HIGH);
			analogWrite(rightMotorPwmPin, steeringRightMomentumVal);
		}
		else {
			analogWrite(leftMotorPwmPin, steeringLeftMomentumVal);
			analogWrite(rightMotorPwmPin, steeringRightMomentumVal);
		}

	}
	else {
		Serial.println("ACCELERARTING WITH CENTER BACKWARD");
		motor_back();
		if (f > 950) {
			Serial.println("max BM speed!");
			digitalWrite(rightMotorPwmPin, HIGH);
			digitalWrite(leftMotorPwmPin, HIGH);
		}
		else {
			analogWrite(leftMotorPwmPin, f);
			analogWrite(rightMotorPwmPin, f);
		}
	}
}

/********************************************* ACCELERATE ANGLE *****************************************************/
void accelerate_angle(int f, int l, int r)
{
	// PWM code:
	if (l == 1000) {
		Serial.println("max LM speed");
		digitalWrite(leftMotorPwmPin, HIGH);
	}
	else {
		analogWrite(leftMotorPwmPin, l);
	}
	if (r == 1000) {
		Serial.println("max RM speed");
		digitalWrite(rightMotorPwmPin, HIGH);
	}
	else {
		analogWrite(rightMotorPwmPin, r);
	}
}

/********************************************* CENTRE *****************************************************/
void turn_centre(void)
{
	Serial.println("turn_centre Function Running...");
}
/********************************************* RIGHT *****************************************************/
void turn_right(void)
{
	Serial.println("turn_right Function Running...");
}
/********************************************* LEFT *****************************************************/
void turn_left(void)
{
	Serial.println("turn_left Function Running...");
}
/********************************************* MANUAL FOWARD *****************************************************/
void manual_foward(void)
{
	Serial.println("manual_foward Function Running...");
	digitalWrite(leftMotorPosPin, HIGH);
	digitalWrite(leftMotorNegPin, LOW);
	digitalWrite(rightMotorPosPin, HIGH);
	digitalWrite(rightMotorNegPin, LOW);
	digitalWrite(leftMotorPwmPin, HIGH);
	digitalWrite(rightMotorPwmPin, HIGH);
}
/********************************************* MANUAL BACK	 *****************************************************/
void manual_back(void)
{
	Serial.println("manual_foward Function Running...");
	digitalWrite(leftMotorPosPin, LOW);
	digitalWrite(leftMotorNegPin, HIGH);
	digitalWrite(rightMotorPosPin, LOW);
	digitalWrite(rightMotorNegPin, HIGH);
	digitalWrite(leftMotorPwmPin, HIGH);
	digitalWrite(rightMotorPwmPin, HIGH);
}
/********************************************* MANUAL LEFT *****************************************************/
void manual_left(void)
{
	Serial.println("manual_left Function Running...");
	digitalWrite(leftMotorPosPin, LOW);
	digitalWrite(leftMotorNegPin, HIGH);
	digitalWrite(rightMotorPosPin, HIGH);
	digitalWrite(rightMotorNegPin, LOW);
	digitalWrite(leftMotorPwmPin, HIGH);
	digitalWrite(rightMotorPwmPin, HIGH);
}
/********************************************* MANUAL RIGHT *****************************************************/
void manual_right(void)
{
	Serial.println("manual_right Function Running...");
	digitalWrite(leftMotorPosPin, HIGH);
	digitalWrite(leftMotorNegPin, LOW);
	digitalWrite(rightMotorPosPin, LOW);
	digitalWrite(rightMotorNegPin, HIGH);
	digitalWrite(leftMotorPwmPin, HIGH);
	digitalWrite(rightMotorPwmPin, HIGH);
}
/********************************************* MANUAL STOP *****************************************************/
void manual_stop(void)
{
	Serial.println("manual_stop Function Running...");
	digitalWrite(leftMotorPosPin, LOW);
	digitalWrite(leftMotorNegPin, LOW);
	digitalWrite(rightMotorPosPin, LOW);
	digitalWrite(rightMotorNegPin, LOW);
	digitalWrite(leftMotorPwmPin, HIGH);
	digitalWrite(rightMotorPwmPin, HIGH);
}
/********************************************* FOWARD *****************************************************/
void motor_foward(void)
{
	digitalWrite(leftMotorPosPin, HIGH);
	digitalWrite(leftMotorNegPin, LOW);
	digitalWrite(rightMotorPosPin, HIGH);
	digitalWrite(rightMotorNegPin, LOW);
}
/********************************************* FOWARD *****************************************************/
void motor_back(void)
{
	digitalWrite(leftMotorPosPin, LOW);
	digitalWrite(leftMotorNegPin, HIGH);
	digitalWrite(rightMotorPosPin, LOW);
	digitalWrite(rightMotorNegPin, HIGH);
}
/* Sending current crane position values to MQTT control panel */
void send_crane_pos(void)
{
	Serial.println("send_crane_pos func running...");
	String temp_r_pos = String(craneYawPos);
	String temp_fa_pos = String(craneElbowPos);
	String temp_ca_pos = String(cranePitchPos1);
	String temp_ca_2_pos = String(cranePitchPos2);
	String temp_claw_pos = String(craneGripperPos);


	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& crane_pos_output = jsonBuffer.createObject();
	crane_pos_output["rot"] = temp_r_pos;
	crane_pos_output["ca_1"] = temp_ca_pos;
	crane_pos_output["ca_2"] = temp_ca_2_pos;
	crane_pos_output["fa"] = temp_fa_pos;
	crane_pos_output["grip"] = temp_claw_pos;

	String jsonStr;
	crane_pos_output.printTo(jsonStr);
	Serial.print("RAW JSON as a String ready for transmision: ");
	Serial.println(jsonStr);
	client.publish("/ExpR/out/crane_pos", (char*)jsonStr.c_str());
}
void send_crane_constraints(void)
{
	Serial.println("send_crane_constraints func running...");

	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& crane_con_output = jsonBuffer.createObject();
	/***     Rotation axis    ***/
	crane_con_output["rx"] = craneYawConstraintX;
	crane_con_output["ry"] = craneYawConstraintY;
	/***     Center axis      ***/
	crane_con_output["cx"] = cranePitchConstraintX;
	crane_con_output["cy"] = cranePitchConstraintY;
	/***     Forearm axis     ***/
	crane_con_output["fx"] = craneElbowConstraintX;
	crane_con_output["fy"] = craneElbowConstraintY;
	/***    Gripper axis      ***/
	crane_con_output["gx"] = craneGripperConstraintX;
	crane_con_output["gy"] = craneGripperConstraintY;

	String jsonStr;
	crane_con_output.printTo(jsonStr);
	Serial.print("RAW JSON as a String ready for transmision: ");
	Serial.println(jsonStr);
	client.publish("/ExpR/out/crane_con", (char*)jsonStr.c_str());
}
void send_crane_movement_speed_values(void)
{
	Serial.println("send_crane_movement_speed_values func running...");

	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& crane_speed_output = jsonBuffer.createObject();
	/***     Rotation axis    ***/
	crane_speed_output["rs"] = craneYawMovementSpeed;
	/***     Center axis      ***/
	crane_speed_output["cs"] = cranePitchMovementSpeed;
	/***     Forearm axis     ***/
	crane_speed_output["fs"] = craneElbowMovementSpeed;
	/***    Gripper axis      ***/
	crane_speed_output["gs"] = craneGripperMovementSpeed;

	String jsonStr;
	crane_speed_output.printTo(jsonStr);
	Serial.print("RAW JSON as a String ready for transmision: ");
	Serial.println(jsonStr);
	client.publish("/ExpR/out/crane_speed", (char*)jsonStr.c_str());
}
void save_crane_pos(void)
{
	Serial.println("save_crane_pos to EEPROM Flash mem");
	Serial.print("current ROTATION = ");
	Serial.println(craneYawPos);
	EEPROM.write(0, int(craneYawPos));
	EEPROM.write(1, int(craneElbowPos));
	EEPROM.write(2, int(cranePitchPos1));
	EEPROM.write(3, int(cranePitchPos2));
	EEPROM.write(4, int(craneGripperPos));
	EEPROM.commit();
	Serial.println("FINISHED!!!!");
	int test_var = EEPROM.read(0);
	Serial.print("TEST VAR FROM EEPROM READ = ");
	Serial.println(test_var);

}
void save_crane_constraints(void)
{
	Serial.println("save_crane_constraints to EEPROM Flash mem");
	/***     Rotation axis    ***/
	EEPROM.write(5, craneYawConstraintX);
	EEPROM.write(6, craneYawConstraintY);
	/***    Center axis     ***/
	EEPROM.write(7, cranePitchConstraintX);
	EEPROM.write(8, cranePitchConstraintY);
	/***     Forearm axis     ***/
	EEPROM.write(9, craneElbowConstraintX);
	EEPROM.write(10, craneElbowConstraintY);
	/***    Gripper axis      ***/
	EEPROM.write(11, craneGripperConstraintX);
	EEPROM.write(12, craneGripperConstraintY);

	EEPROM.commit();
}
void save_crane_movement_speed(void)
{
	Serial.println("save_crane_movement_speed to EEPROM Flash mem");
	/***     Rotation axis    ***/
	EEPROM.write(13, craneYawMovementSpeed);
	/***    Center axis     ***/
	EEPROM.write(14, cranePitchMovementSpeed);
	/***     Forearm axis     ***/
	EEPROM.write(15, craneElbowMovementSpeed);
	/***    Gripper axis      ***/
	EEPROM.write(16, craneGripperMovementSpeed);

	EEPROM.commit();
}
void apply_current_crane_pos_values(void)
{
	Serial.println("moving crane servos to current position variables");
	craneYawServo.write(craneYawPos);
	craneElbowServo.write(craneElbowPos);
	cranePitchServo1.write(cranePitchPos1);
	cranePitchServo2.write(cranePitchPos2);
	craneGripperServo.write(craneGripperPos);
}
/********************************************* light_toggle *****************************************************/
void light_toggle(void)
{
	Serial.println("light_toggle");
	if (isHeadlightOn == false) {
		isHeadlightOn = true;
		Serial.println("light ON");
		digitalWrite(headLightRelayPin, HIGH);
		digitalWrite(headLightRelayPin, HIGH);
	}
	else {
		isHeadlightOn = false;
		Serial.println("light OFF");
		digitalWrite(headLightRelayPin, LOW);
		digitalWrite(headLightRelayPin, LOW);
	}
}
void send_crane_gyro_values(void)
{
	Serial.println("send_crane_gyro_values func running...");
	Serial.print("ypr\t");
	Serial.print(gyro_val_0);
	Serial.print("\t");
	Serial.print(gyro_val_1);
	Serial.print("\t");
	Serial.print(gyro_val_2);

	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& crane_gyro_output = jsonBuffer.createObject();
	/***     yaw axis    ***/
	crane_gyro_output["y"] = gyro_val_0;
	/***     pitch axis      ***/
	crane_gyro_output["p"] = gyro_val_1;
	/***     rotation axis     ***/
	crane_gyro_output["r"] = gyro_val_2;

	String jsonStr;
	crane_gyro_output.printTo(jsonStr);
	Serial.print("RAW JSON as a String ready for transmision: ");
	Serial.println(jsonStr);
	client.publish("/ExpR/out/tank_gyro", (char*)jsonStr.c_str());
}
/********************************** RECONNECT TO MQTT ******************************************/
void reconnect() {
	// Loop until we're reconnected
	if (!client.connected()) {
		Serial.print("Attempting MQTT connection...");
		// Attempt to connect
		if (client.connect("ExpR-ESP32", mqtt_user, mqtt_password)) {
			Serial.println("connected");
			/* General input to tank from control panel: */
			client.subscribe("/ExpR/in");
			client.subscribe("/ExpR/in/newC"); // we recieve new crane_positions values on this topic
			client.subscribe("/ExpR/in/ncc"); // we recieve new crane_constraints values on this topic
			client.subscribe("/ExpR/in/ncs"); // we recieve new crane_speed values on this topic
			client.subscribe("/ExpR/in/nt"); // we recieve new timing values on this topic
			/* DPAD / hat on steering wheel input: */
			client.subscribe("/ExpR/in/hat");
			/* Steering wheel button up/down topics: */
			client.subscribe("/ExpR/in/SW_btn_d");
			client.subscribe("/ExpR/in/SW_btn_u");
			/* Steering wheel axis topics */
			client.subscribe("/ExpR/in/SW_axs_0/Rs");
			client.subscribe("/ExpR/in/SW_axs_0/Ls");
			client.subscribe("/ExpR/in/SW_axs_0/Rh");
			client.subscribe("/ExpR/in/SW_axs_0/Lh");
			client.subscribe("/ExpR/in/SW_axs_1");
			client.subscribe("/ExpR/in/SW_axs_2");
		}
		else {
			Serial.print("failed, rc=");
			Serial.print(client.state());
			Serial.println(" try again in 1 second");
			// Wait 2 seconds before retrying
			delay(2000);
			// attempt to connect to local MQTT server:

		}
	}
}

/* Startup image for OLED display */
const uint8_t start_logo[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x7F, 0x80, 0x7F, 0xFE, 0x7F, 0xFE, 0x78, 0x78, 0x07, 0x80, 0x78, 0x1E, 0x1F, 0xE0, 0x00,
0x00, 0x7F, 0x80, 0x7F, 0xFE, 0x7F, 0xFE, 0x78, 0x78, 0x07, 0x80, 0x78, 0x1E, 0x1F, 0xE0, 0x00,
0x00, 0x1E, 0x00, 0x1E, 0x06, 0x1E, 0x06, 0x78, 0x78, 0x1F, 0xE0, 0x7E, 0x7E, 0x78, 0x78, 0x00,
0x00, 0x1E, 0x00, 0x1E, 0x06, 0x1E, 0x06, 0x78, 0x78, 0x1F, 0xE0, 0x7E, 0x7E, 0x78, 0x78, 0x00,
0x00, 0x1E, 0x00, 0x1E, 0x00, 0x1E, 0x00, 0x78, 0x78, 0x78, 0x78, 0x7F, 0xFE, 0x78, 0x78, 0x00,
0x00, 0x1E, 0x00, 0x1E, 0x00, 0x1E, 0x00, 0x78, 0x78, 0x78, 0x78, 0x7F, 0xFE, 0x78, 0x78, 0x00,
0x00, 0x1E, 0x00, 0x1E, 0x18, 0x1E, 0x18, 0x78, 0x78, 0x78, 0x78, 0x7F, 0xFE, 0x78, 0x00, 0x00,
0x00, 0x1E, 0x00, 0x1E, 0x18, 0x1E, 0x18, 0x78, 0x78, 0x78, 0x78, 0x7F, 0xFE, 0x78, 0x00, 0x00,
0x00, 0x1E, 0x00, 0x1F, 0xF8, 0x1F, 0xF8, 0x7F, 0xF8, 0x78, 0x78, 0x79, 0x9E, 0x1F, 0x80, 0x00,
0x00, 0x1E, 0x00, 0x1F, 0xF8, 0x1F, 0xF8, 0x7F, 0xF8, 0x78, 0x78, 0x79, 0x9E, 0x1F, 0x80, 0x00,
0x00, 0x1E, 0x06, 0x1E, 0x18, 0x1E, 0x18, 0x78, 0x78, 0x7F, 0xF8, 0x78, 0x1E, 0x01, 0xE0, 0x00,
0x00, 0x1E, 0x06, 0x1E, 0x18, 0x1E, 0x18, 0x78, 0x78, 0x7F, 0xF8, 0x78, 0x1E, 0x01, 0xE0, 0x00,
0x00, 0x1E, 0x1E, 0x1E, 0x00, 0x1E, 0x00, 0x78, 0x78, 0x78, 0x78, 0x78, 0x1E, 0x78, 0x78, 0x00,
0x00, 0x1E, 0x1E, 0x1E, 0x00, 0x1E, 0x00, 0x78, 0x78, 0x78, 0x78, 0x78, 0x1E, 0x78, 0x78, 0x00,
0x00, 0x1E, 0x1E, 0x1E, 0x06, 0x1E, 0x06, 0x78, 0x78, 0x78, 0x78, 0x78, 0x1E, 0x78, 0x78, 0x00,
0x00, 0x1E, 0x1E, 0x1E, 0x06, 0x1E, 0x06, 0x78, 0x78, 0x78, 0x78, 0x78, 0x1E, 0x78, 0x78, 0x00,
0x00, 0x7F, 0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x78, 0x78, 0x78, 0x78, 0x78, 0x1E, 0x1F, 0xE0, 0x00,
0x00, 0x7F, 0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x78, 0x78, 0x78, 0x78, 0x78, 0x1E, 0x1F, 0xE0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x1F, 0xE0, 0x07, 0xE0, 0x7F, 0xFE, 0x7F, 0xF8, 0x78, 0x1E, 0x07, 0x80, 0x7F, 0xF8, 0x7F, 0xFE,
0x1F, 0xE0, 0x07, 0xE0, 0x7F, 0xFE, 0x7F, 0xF8, 0x78, 0x1E, 0x07, 0x80, 0x7F, 0xF8, 0x7F, 0xFE,
0x78, 0x78, 0x1E, 0x78, 0x1E, 0x1E, 0x67, 0x98, 0x78, 0x1E, 0x1F, 0xE0, 0x1E, 0x1E, 0x1E, 0x06,
0x78, 0x78, 0x1E, 0x78, 0x1E, 0x1E, 0x67, 0x98, 0x78, 0x1E, 0x1F, 0xE0, 0x1E, 0x1E, 0x1E, 0x06,
0x78, 0x78, 0x78, 0x1E, 0x1E, 0x06, 0x07, 0x80, 0x78, 0x1E, 0x78, 0x78, 0x1E, 0x1E, 0x1E, 0x00,
0x78, 0x78, 0x78, 0x1E, 0x1E, 0x06, 0x07, 0x80, 0x78, 0x1E, 0x78, 0x78, 0x1E, 0x1E, 0x1E, 0x00,
0x78, 0x00, 0x78, 0x1E, 0x1E, 0x18, 0x07, 0x80, 0x78, 0x1E, 0x78, 0x78, 0x1E, 0x1E, 0x1E, 0x18,
0x78, 0x00, 0x78, 0x1E, 0x1E, 0x18, 0x07, 0x80, 0x78, 0x1E, 0x78, 0x78, 0x1E, 0x1E, 0x1E, 0x18,
0x1F, 0x80, 0x78, 0x1E, 0x1F, 0xF8, 0x07, 0x80, 0x79, 0x9E, 0x78, 0x78, 0x1F, 0xF8, 0x1F, 0xF8,
0x1F, 0x80, 0x78, 0x1E, 0x1F, 0xF8, 0x07, 0x80, 0x79, 0x9E, 0x78, 0x78, 0x1F, 0xF8, 0x1F, 0xF8,
0x01, 0xE0, 0x78, 0x1E, 0x1E, 0x18, 0x07, 0x80, 0x79, 0x9E, 0x7F, 0xF8, 0x1E, 0x78, 0x1E, 0x18,
0x01, 0xE0, 0x78, 0x1E, 0x1E, 0x18, 0x07, 0x80, 0x79, 0x9E, 0x7F, 0xF8, 0x1E, 0x78, 0x1E, 0x18,
0x78, 0x78, 0x78, 0x1E, 0x1E, 0x00, 0x07, 0x80, 0x1E, 0x78, 0x78, 0x78, 0x1E, 0x1E, 0x1E, 0x00,
0x78, 0x78, 0x78, 0x1E, 0x1E, 0x00, 0x07, 0x80, 0x1E, 0x78, 0x78, 0x78, 0x1E, 0x1E, 0x1E, 0x00,
0x78, 0x78, 0x1E, 0x78, 0x1E, 0x00, 0x07, 0x80, 0x1E, 0x78, 0x78, 0x78, 0x1E, 0x1E, 0x1E, 0x06,
0x78, 0x78, 0x1E, 0x78, 0x1E, 0x00, 0x07, 0x80, 0x1E, 0x78, 0x78, 0x78, 0x1E, 0x1E, 0x1E, 0x06,
0x1F, 0xE0, 0x07, 0xE0, 0x7F, 0x80, 0x1F, 0xE0, 0x1E, 0x78, 0x78, 0x78, 0x7E, 0x1E, 0x7F, 0xFE,
0x1F, 0xE0, 0x07, 0xE0, 0x7F, 0x80, 0x1F, 0xE0, 0x1E, 0x78, 0x78, 0x78, 0x7E, 0x1E, 0x7F, 0xFE,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

};

/* Default for OLED display */
const uint8_t side_view_tank[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xF8, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xF8, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFC, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFC, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFC, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x83, 0xF8, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0x80, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x00, 0x3C, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0xF7, 0x80, 0x00, 0x03, 0xFF, 0xE4, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x1F, 0xFF, 0xFF, 0x80, 0x00, 0xFC, 0x00, 0x06, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x1F, 0xFF, 0xFF, 0x80, 0x3F, 0x00, 0x00, 0x03, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0x80, 0x01, 0xF0, 0x00, 0x00, 0x00,
0x00, 0x7F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x3C, 0x00, 0x00, 0x00,
0x00, 0x7F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x0F, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0xE0, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x38, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x07, 0x8E, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x07, 0xE3, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0xC7, 0xF8, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xF3, 0xF8, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF9, 0x98, 0xF8, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF9, 0x0C, 0x03, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF9, 0x07, 0xFE, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF9, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x80, 0xF8, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0xE7, 0x80, 0xF8, 0x70, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x7F, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x80, 0xF8, 0x18, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x80, 0xF8, 0x0C, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x80, 0x00, 0x06, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF1, 0x80, 0x00, 0x02, 0x00, 0x00, 0x00,
0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00,
0x00, 0x00, 0x3F, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x7F, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xFF, 0xC0, 0x00, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x7F, 0xE7, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xFD, 0x80, 0x00, 0xFF, 0x00, 0x00, 0x00,
0x00, 0x00, 0x3F, 0xCF, 0xB6, 0xFB, 0xEF, 0xFF, 0xFF, 0xF7, 0x80, 0x00, 0x03, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x1D, 0xFF, 0xDF, 0xFF, 0x7D, 0xBD, 0xBA, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF7, 0x6F, 0xBF, 0xB7, 0xFE, 0x03, 0xFF, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x7E, 0xD5, 0x4A, 0x00, 0x00, 0x00, 0x40, 0xBF, 0xFF, 0xFF, 0x80, 0x00, 0x00,
0x00, 0x00, 0x01, 0xFF, 0xEF, 0x7F, 0x80, 0x00, 0x01, 0xFF, 0xF6, 0xAE, 0xAD, 0xC0, 0x00, 0x00,
0x00, 0x00, 0x1F, 0xF6, 0xFB, 0xAF, 0x80, 0x00, 0x01, 0x00, 0x7F, 0xFB, 0xFF, 0xFF, 0x80, 0x00,
0x00, 0x00, 0x7E, 0xBF, 0x6F, 0xFB, 0xFF, 0xFF, 0xFF, 0x00, 0x6A, 0xBD, 0x5B, 0xFF, 0xE0, 0x00,
0x00, 0x03, 0xFB, 0xF6, 0xFD, 0xEF, 0x80, 0x00, 0x00, 0x00, 0x7F, 0xDF, 0xFD, 0xBF, 0xF0, 0x00,
0x00, 0x03, 0xFB, 0xDF, 0xB6, 0xDD, 0x80, 0x00, 0x00, 0x01, 0xFE, 0xFB, 0xFF, 0xE6, 0xF8, 0x00,
0x00, 0x03, 0x5A, 0xEA, 0xFE, 0xEF, 0xC0, 0x00, 0x00, 0x03, 0xD7, 0x5C, 0xED, 0xFA, 0xB8, 0x00,
0x00, 0x01, 0xEF, 0x7B, 0xDE, 0xB3, 0xC0, 0x00, 0x00, 0x06, 0xED, 0xFE, 0xAC, 0x97, 0xF8, 0x00,
0x00, 0x00, 0x7F, 0xAE, 0x7F, 0x5D, 0x40, 0x00, 0x00, 0x07, 0xB5, 0x57, 0x6E, 0xFB, 0xF8, 0x00,
0x00, 0x00, 0x3F, 0xF7, 0xEE, 0xF6, 0xC0, 0x00, 0x00, 0x06, 0x5D, 0xFE, 0xB3, 0xFF, 0x60, 0x00,
0x00, 0x00, 0x07, 0xFB, 0x3F, 0x5F, 0xC0, 0x00, 0x00, 0x07, 0xD6, 0xBB, 0x6D, 0xFF, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x7F, 0xEF, 0xEF, 0x80, 0x00, 0x00, 0x07, 0xFE, 0xFD, 0xBF, 0xFF, 0x00, 0x00,
0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

};

/* Systems ready image for OLED display */
const uint8_t systems_ready_tank[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x02, 0x0E, 0x0E, 0x00, 0x7B, 0x8E, 0x79, 0xFC, 0xFC, 0xE0, 0xE3, 0xC0, 0x00, 0x0F, 0xF8, 0x00,
0x06, 0x04, 0x04, 0x00, 0x89, 0x04, 0x89, 0x24, 0x44, 0x60, 0xC4, 0x40, 0x01, 0xFF, 0xF8, 0x00,
0x05, 0x04, 0x04, 0x00, 0x88, 0x88, 0x88, 0x20, 0x40, 0x51, 0x44, 0x40, 0x0F, 0xFF, 0xFC, 0x00,
0x09, 0x04, 0x04, 0x00, 0x40, 0x50, 0x40, 0x20, 0x50, 0x51, 0x42, 0x00, 0x0F, 0xFF, 0xFC, 0x00,
0x09, 0x04, 0x04, 0x00, 0x20, 0x20, 0x20, 0x20, 0x70, 0x51, 0x41, 0x00, 0x0F, 0xFF, 0xFC, 0x00,
0x1F, 0x84, 0x04, 0x00, 0x10, 0x20, 0x10, 0x20, 0x50, 0x4A, 0x40, 0x80, 0x0F, 0xFF, 0xFC, 0x00,
0x10, 0x84, 0x04, 0x00, 0x88, 0x20, 0x88, 0x20, 0x40, 0x4A, 0x44, 0x40, 0x03, 0xFF, 0xF8, 0x00,
0x20, 0x44, 0x24, 0x20, 0x88, 0x20, 0x88, 0x20, 0x44, 0x44, 0x44, 0x40, 0x03, 0xFF, 0x80, 0x00,
0x70, 0xEF, 0xEF, 0xE0, 0xF0, 0x70, 0xF0, 0x70, 0xFC, 0xE4, 0xE7, 0x80, 0x06, 0x01, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x03, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x02, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x02, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x02, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x80, 0x00, 0x03, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0x80, 0x00, 0x01, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xC3, 0xFC, 0x00, 0xC0, 0x00,
0x3E, 0x1F, 0x81, 0x07, 0xE3, 0x8E, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xEE, 0x06, 0x00, 0x60, 0x00,
0x11, 0x08, 0x83, 0x02, 0x11, 0x04, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xF8, 0x02, 0x00, 0x20, 0x00,
0x11, 0x08, 0x02, 0x82, 0x08, 0x88, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xE0, 0x02, 0x00, 0xFF, 0x00,
0x11, 0x0A, 0x04, 0x82, 0x08, 0x50, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xC0, 0x02, 0x00, 0xFF, 0x00,
0x1E, 0x0E, 0x04, 0x82, 0x08, 0x20, 0x00, 0x0F, 0xF8, 0x01, 0xFF, 0x80, 0x06, 0x00, 0x3F, 0x00,
0x12, 0x0A, 0x0F, 0xC2, 0x08, 0x20, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x20, 0x00,
0x11, 0x08, 0x08, 0x42, 0x08, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x80, 0xE0, 0x00,
0x11, 0x08, 0x90, 0x22, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x80, 0xFC, 0x00,
0x38, 0xDF, 0xB8, 0x77, 0xE0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0xFC, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0xFC, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0xFC, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0xE0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFC, 0x20, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xE0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xE0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xF9, 0xFF,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xF3, 0xFF,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xE1,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC7, 0xCD,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x81, 0xFF, 0xFF, 0xFF, 0xFF, 0xDF, 0xBD,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x07, 0xFF, 0xFF, 0xFF, 0x1F, 0x7F,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x3F, 0xFF, 0xFE, 0x3E, 0xF2,
0x20, 0x20, 0x00, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x01, 0xF0, 0x00, 0x07, 0xFF, 0xFC, 0xFB, 0x6E,
0x20, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x03, 0xFF, 0xF8, 0xF9, 0xDC,
0x20, 0x21, 0xCD, 0x00, 0x8A, 0xC2, 0x1C, 0x70, 0x00, 0x7F, 0xF0, 0x03, 0xFF, 0xFD, 0xDD, 0xB8,
0x20, 0x20, 0x2A, 0x80, 0xF1, 0x22, 0x22, 0x88, 0x01, 0xF3, 0xFF, 0x03, 0xFF, 0xEF, 0x9E, 0x70,
0x20, 0x21, 0xEA, 0x80, 0x81, 0x02, 0x20, 0xF0, 0x07, 0xCF, 0xFF, 0xF3, 0xFF, 0xC6, 0x0F, 0xE0,
0x20, 0x22, 0x28, 0x80, 0x81, 0x02, 0x22, 0x80, 0x0F, 0x3F, 0xFF, 0xFF, 0xFF, 0xAF, 0x80, 0x00,
0x3E, 0x31, 0xE8, 0x80, 0x83, 0x83, 0x1C, 0x70, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0x1F, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xFF, 0xFF, 0xFF, 0xFF, 0x3B, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xFF, 0xC7, 0xFF, 0xFF, 0xF8, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x7F, 0x81, 0xFF, 0xFF, 0xE6, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x00, 0xFF, 0xFF, 0xDE, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFE, 0x00, 0x0F, 0xFF, 0xBD, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFC, 0x00, 0x00, 0x7E, 0xFB, 0x80, 0x00,
0x1C, 0x71, 0xC7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xFB, 0x00, 0x00,
0x22, 0x8A, 0x28, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFB, 0xE6, 0x00, 0x00,
0x02, 0x98, 0x29, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF5, 0x5C, 0x00, 0x00,
0x0C, 0xA8, 0xCA, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF7, 0x3C, 0x00, 0x00,
0x10, 0xC9, 0x0C, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE2, 0x78, 0x00, 0x00,
0x20, 0x8A, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF4, 0xF0, 0x00, 0x00,
0x3E, 0x73, 0xE7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xE0, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};