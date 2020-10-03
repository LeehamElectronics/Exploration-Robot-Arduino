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
 * https://www.paypal.com/paypalme/liamproice/                                                       *
 *                                                                                                   *
 * And if you need help, feel free to email me at liamisprice@gmail.com                              *
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
#include <WiFi.h>
#include <WiFiMulti.h>
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
#include <Arduino.h> /* Surprising? */
#include <analogWrite.h> /* this was required for ESP32 chips to use analogWrite for some reason, look it up */
/*
 * Used to store information even when the robot is turned off, dont abuse it, only has a set
 * amount of read / write cycles. If you need to use EEPROM A LOT try using a SD Card instead!
 */
#include <EEPROM.h> 
#include <ArduinoJson.h> /* This is a very important library and takes a lot to learn properly, definitely worth it though */ 

 /* EEPROM Flash Memory setup: */
#define EEPROM_SIZE 17 // don't touch unless your adding more EEPROM functionality...
/* MQTT Variables Below, make sure to fill them in with your MQTT credentials, or it won't work! */
#define mqtt_user "USERNAME"
#define mqtt_password "USE_A_STRONG_PASSWORD"
/* This is the IP address or hostname of your MQTT server... */
const char* mqtt_server = "REDACTED";

WiFiClient espClient;
PubSubClient client(espClient);

/* Wifi Multi code */
WiFiMulti wifiMulti;
boolean connectioWasAlive = true;

/* Servo Setup */
Servo crane_rot_axis;
Servo crane_fa_axis;
Servo crane_ca_axis;
Servo crane_ca_2_axis;
Servo crane_gripper_servo;

/* Global tank control variables */
int global_foward = 0;
int global_steering_left = 0;
int global_steering_right = 0;
boolean going_left = false;
boolean going_right = false;
boolean going_back = false;
/* crane rotation check: */
boolean is_rotating_left = false;
boolean is_rotating_right = false;
/* crane forarm movement check: */
boolean is_FA_moving_up = false;
boolean is_FA_moving_down = false;
/* crane center_axis movement check: */
boolean is_CA_moving_up = false;
boolean is_CA_moving_down = false;
/* crane gripper_axis movement check: */
boolean is_grippper_opening = false;
boolean is_grippper_closing = false;
/* Relay variable logic */
boolean is_light_on = false;

/*
 * Initial crane position variables that are written
 * to servos on startup. These values are replaced by
 * the values stored in EEPROM but I have left values
 * assigned to them anyways in case EEPROM fails...
*/
int current_rotation_pos = 90;
int current_FA_pos = 90;
int current_CA_pos = 90;
int current_CA_2_pos = 95;
int current_gripper_pos = 90;

/*
 * Crane movement constraint variables, these are
 * also stored and read from EEPROM and can be
 * virtually adjusted via the Python Control Panel
*/
/***     Rotation axis    ***/
int crane_rot_constraint_x;
int crane_rot_constraint_y;
/***     Center axis      ***/
int crane_cent_constraint_x;
int crane_cent_constraint_y;
/***     Forearm axis     ***/
int crane_forarm_constraint_x;
int crane_forarm_constraint_y;
/***    Gripper axis      ***/
int crane_gripp_constraint_x;
int crane_gripp_constraint_y;

/*
 * Crane movement speed variables are also
 * stored in EEPROM and are adjustable for
 * conveinence
*/
/***     Rotation axis    ***/
int crane_rot_speed;
/***    Center axis_1     ***/
int crane_cent_speed;
/***     Forearm axis     ***/
int crane_forarm_speed;
/***    Gripper axis      ***/
int crane_gripp_speed;

/* Time management */
unsigned long previousMillis = 0;        // will store last time 'Crane Rotation' was updated
const long interval = 100;           // interval at which to update crane's rotation (milliseconds)

/* unproccessed axis values from steering wheel */
float axs_0_value;
float axs_1_value;
float axs_2_value;

/******* Pinout Menu for wiring up robot *******
 *                                             *
 * Here for reference, the actual servo attach *
 * code is located in the Setup code setup()   *
 * crane_rot_axis.attach(25);                  *
 * crane_fa_axis.attach(26);                   *
 * crane_ca_axis.attach(27);                   *
 * crane_ca_2_axis.attach(32);                 *
 * crane_gripper_servo.attach(33);             *
************************************************/
#define head_light_relay 23
/* pin outs for motor driver + PWM */
int left_motor_p = 16; // motor 1 pin 1
int left_motor_n = 17; // motor 1 pin 2
int right_motor_p = 18; // motor 2 pin 1
int right_motor_n = 19; // motor 2 pin 2
int left_pwm = 21; // motor 1 pwm pin
int right_pwm = 22; // motor 2 pwm pin

/**************************************** RECEIVE DATA FROM MQTT ******************************************/
void callback(char* topic, byte* payload, unsigned int length) {
	Serial.print("Topic is:");
	Serial.println(topic);
	if (strcmp(topic, "/ExpR/in/SW_axs_0/Rs") == 0) {
		// axis 0 is for left and right turning
		String axs_0_temp = String((char*)payload);
		axs_0_value = axs_0_temp.toFloat();
		// Serial.print("SOFT RIGHT");
		if (going_back == true) {
			Serial.println("SOFT TURN RIGHT MOTOR BACK");
			motor_back();
		}
		else {
			Serial.println("SOFT TURN RIGHT MOTOR FOWARD");
			motor_foward();
		}
		if (axs_0_value == 1.00) {
			Serial.println("STEERING WHEEL CENTER");
			going_left = false;
			going_right = false;
		}
		else {
			going_right = true;
		}

		/* accelerate tank with slightly smaler PWM on the right hand side */
		global_steering_right = global_foward * axs_0_value;
		accelerate_angle(global_foward, global_steering_left, global_steering_right);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_0/Rh") == 0) {
		// axis 0 is for left and right turning
		String axs_0_temp = String((char*)payload);
		axs_0_value = axs_0_temp.toFloat();
		// Serial.print("HARD RIGHT");
		digitalWrite(left_motor_p, HIGH);
		digitalWrite(left_motor_n, LOW);
		digitalWrite(right_motor_p, LOW);
		digitalWrite(right_motor_n, HIGH);
		/* accelerate tank with slightly larger PWM on the right hand side with inversed polarity (full lock "tank steering" standard) */
		global_steering_right = global_foward * axs_0_value;
		// Serial.print(global_steering_right);
		going_right = true;
		accelerate_angle(global_foward, global_steering_left, global_steering_right);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_0/Ls") == 0) {
		// axis 0 is for left and right turning
		String axs_0_temp = String((char*)payload);
		axs_0_value = axs_0_temp.toFloat();
		// Serial.print("SOFT LEFT");
		if (going_back == true) {
			Serial.println("SOFT TURN LEFT MOTOR BACK");
			motor_back();
		}
		else {
			Serial.println("SOFT TURN LEFT MOTOR FOWARD");
			motor_foward();
		}
		if (axs_0_value == 1.00) {
			Serial.println("STEERING WHEEL CENTER!");
			going_left = false;
			going_right = false;
		}
		else {
			going_left = true;
		}
		/* accelerate tank with slightly smaler PWM on the left hand side */
		global_steering_left = global_foward * axs_0_value;
		// Serial.print(global_steering_left);
		accelerate_angle(global_foward, global_steering_left, global_steering_right);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_0/Lh") == 0) {
		// axis 0 is for left and right turning
		String axs_0_temp = String((char*)payload);
		axs_0_value = axs_0_temp.toFloat();
		// Serial.print("HARD LEFT");
		digitalWrite(left_motor_p, LOW);
		digitalWrite(left_motor_n, HIGH);
		digitalWrite(right_motor_p, HIGH);
		digitalWrite(right_motor_n, LOW);
		/* accelerate tank with slightly larger PWM on the left hand side with inversed polarity (full lock "tank steering" standard) */
		global_steering_left = global_foward * axs_0_value;
		// Serial.print(global_steering_left);
		going_left = true;
		accelerate_angle(global_foward, global_steering_left, global_steering_right);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_1") == 0) {
		// axis 1 is for accelerating fowards
		String axs_1_temp = String((char*)payload);
		axs_1_value = axs_1_temp.toInt();
		global_foward = map(axs_1_value, -100, 100, 1000, 0);
		Serial.println(global_foward);
		going_back = false;
		accelerate(global_foward);
	}
	else if (strcmp(topic, "/ExpR/in/SW_axs_2") == 0) {
		// axis 0 for going backwards
		String axs_2_temp = String((char*)payload);
		axs_2_value = axs_2_temp.toInt();
		global_foward = map(axs_2_value, -100, 100, 1000, 0);
		Serial.println(global_foward);
		going_back = true;
		accelerate_back(global_foward);
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
		current_rotation_pos = json_obj["rot"];
		/***    Center axis_1     ***/
		current_CA_pos = json_obj["ca_1"];
		/***    Center axis_2     ***/
		current_CA_2_pos = json_obj["ca_2"];
		/***     Forearm axis     ***/
		current_FA_pos = json_obj["fa"];
		/***    Gripper axis      ***/
		current_gripper_pos = json_obj["grip"];

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
		crane_rot_constraint_x = json_obj["rcx"];
		crane_rot_constraint_y = json_obj["rcy"];
		/***    Center axis     ***/
		crane_cent_constraint_x = json_obj["ccx"];
		crane_cent_constraint_y = json_obj["ccy"];
		/***     Forearm axis     ***/
		crane_forarm_constraint_x = json_obj["fcx"];
		crane_forarm_constraint_y = json_obj["fcy"];
		/***    Gripper axis      ***/
		crane_gripp_constraint_x = json_obj["gcx"];
		crane_gripp_constraint_y = json_obj["gcy"];

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
		crane_rot_speed = int(json_obj["rs"]);
		/***    Center axis_1     ***/
		crane_cent_speed = int(json_obj["cs"]);
		/***     Forearm axis     ***/
		crane_forarm_speed = int(json_obj["fs"]);
		/***    Gripper axis      ***/
		crane_gripp_speed = int(json_obj["gs"]);

	}
	else if (strcmp(topic, "/ExpR/in/SW_btn_d") == 0) {
		char button_temp = payload[0];
		Serial.println("butn DOWN topic");
		Serial.println(button_temp);
		if (button_temp == '4') {
			is_rotating_left = true;
			Serial.println("crane rot left...");
		}
		else if (button_temp == '5') {
			is_rotating_right = true;
			Serial.println("crane rot right...");
		}
		else if (button_temp == '6') {
			light_toggle();
		}
		else if (button_temp == '7') {
			Serial.println("crane open gripper...");
			is_grippper_opening = true;
			is_grippper_closing = false;
		}
		else if (button_temp == 's') {
			Serial.println("crane close gripper...");
			is_grippper_opening = false;
			is_grippper_closing = true;
		}
	}
	else if (strcmp(topic, "/ExpR/in/SW_btn_u") == 0) {
		char button_temp = payload[0];
		Serial.println("butn UP topic");
		Serial.println(button_temp);
		if (button_temp == '4') {
			is_rotating_left = false;
			Serial.println("crane NOT rot left...");
		}
		else if (button_temp == '5') {
			is_rotating_right = false;
			Serial.println("crane NOT rot right...");
		}
		else if (button_temp == '6') {
			light_toggle();
		}
		else if (button_temp == '7') {
			Serial.println("crane NOT open gripper...");
			is_grippper_opening = false;
			is_grippper_closing = false;
		}
		else if (button_temp == 's') {
			Serial.println("crane NOT close gripper...");
			is_grippper_opening = false;
			is_grippper_closing = false;
		}
	}
	else if (strcmp(topic, "/ExpR/in/hat") == 0) {
		String temp_local = String(char(payload[0]));
		String hat_temp = temp_local;
		Serial.print("steering wheel HAT: ");
		Serial.println(hat_temp);
		if (hat_temp == "x") {
			Serial.println("crane forearm UP");
			is_FA_moving_up = true;
			is_FA_moving_down = false;
		}
		else if (hat_temp == "y") {
			Serial.println("crane forearm DOWN");
			is_FA_moving_down = true;
			is_FA_moving_up = false;
		}
		else if (hat_temp == "v") {
			Serial.println("crane center_axis UP");
			is_CA_moving_down = false;
			is_CA_moving_up = true;
		}
		else if (hat_temp == "c") {
			Serial.println("crane center_axis DOWN");
			is_CA_moving_down = true;
			is_CA_moving_up = false;
		}
		else if (hat_temp == "b") {
			Serial.println("crane HALT");
			is_FA_moving_up = false;
			is_FA_moving_down = false;
			is_CA_moving_down = false;
			is_CA_moving_up = false;
			is_rotating_left = false;
			is_rotating_right = false;
		}
	}
	else {
		Serial.print("Topic not relevant!");
	}
}

void setup() {
	Serial.begin(9600);
	// initialize EEPROM with predefined size
	EEPROM.begin(EEPROM_SIZE);
	delay(10);
	/*
	* Replace your network information here, you can add as many networks
	* as you want and the strongest signal will be automatically chosen
	* for you! How cool is that?
	*/
	wifiMulti.addAP("SSID", "PASSWORD");
	wifiMulti.addAP("SSID", "PASSWORD");
	wifiMulti.addAP("SSID", "PASSWORD");

	Serial.println("Connecting Wifi...");
	if (wifiMulti.run() == WL_CONNECTED) {
		Serial.println("");
		Serial.println("WiFi connected");
		Serial.println("IP address: ");
		Serial.println(WiFi.localIP());
	}
	client.setServer(mqtt_server, 53);
	client.setCallback(callback);

	/* initialise all motor outputs */
	pinMode(left_motor_p, OUTPUT);
	pinMode(left_motor_n, OUTPUT);
	pinMode(right_motor_p, OUTPUT);
	pinMode(right_motor_n, OUTPUT);
	pinMode(left_pwm, OUTPUT);
	pinMode(right_pwm, OUTPUT);
	/* Relay outputs */
	pinMode(head_light_relay, OUTPUT);

	/* The following code reads the default crane positions from EEPROM and writes it to the servos */
	current_rotation_pos = EEPROM.read(0);
	current_FA_pos = EEPROM.read(1);
	current_CA_pos = EEPROM.read(2);
	current_CA_2_pos = EEPROM.read(3);
	current_gripper_pos = EEPROM.read(4);

	/***     Rotation axis    ***/
	crane_rot_constraint_x = EEPROM.read(5);
	crane_rot_constraint_y = EEPROM.read(6);
	/***    Center axis_1     ***/
	crane_cent_constraint_x = EEPROM.read(7);
	crane_cent_constraint_y = EEPROM.read(8);
	/***     Forearm axis     ***/
	crane_forarm_constraint_x = EEPROM.read(9);
	crane_forarm_constraint_y = EEPROM.read(10);
	/***    Gripper axis      ***/
	crane_gripp_constraint_x = EEPROM.read(11);
	crane_gripp_constraint_y = EEPROM.read(12);

	/***     Rotation axis    ***/
	crane_rot_speed = EEPROM.read(13);
	/***    Center axis_1     ***/
	crane_cent_speed = EEPROM.read(14);
	/***     Forearm axis     ***/
	crane_forarm_speed = EEPROM.read(15);
	/***    Gripper axis      ***/
	crane_gripp_speed = EEPROM.read(16);

	crane_rot_axis.attach(25);
	crane_fa_axis.attach(26);
	crane_ca_axis.attach(27);
	crane_ca_2_axis.attach(32);
	crane_gripper_servo.attach(33);

	apply_current_crane_pos_values();
}

void loop() {
	if (wifiMulti.run() != WL_CONNECTED) {
		Serial.println("WiFi not connected!");
		delay(1000);
	}
	if (wifiMulti.run() == WL_CONNECTED) {
		if (!client.connected()) {
			Serial.println("Not connected to MQTT, trying to connect to broker...");
			reconnect();
		}
	}
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis >= interval) {
		// save the last time we checked the cranes rotation status
		previousMillis = currentMillis;

		if (is_rotating_left == true) {
			if (current_rotation_pos >= crane_rot_constraint_y) {
				is_rotating_left = false;
				Serial.println("Crane Rotation Left MAX");
			}
			else {
				current_rotation_pos = current_rotation_pos + crane_rot_speed;
				crane_rot_axis.write(current_rotation_pos);
				Serial.println(current_rotation_pos);
			}
		}
		if (is_rotating_right == true) {
			if (current_rotation_pos <= crane_rot_constraint_x) {
				is_rotating_right = false;
				Serial.println("Crane Rotation Right MAX");
				current_rotation_pos = crane_rot_constraint_x;
				crane_rot_axis.write(current_rotation_pos);
			}
			else {
				current_rotation_pos = current_rotation_pos - crane_rot_speed;
				if (current_rotation_pos <= crane_rot_constraint_x) {
					is_rotating_right = false;
					Serial.println("Crane Rotation Right MAX");
					current_rotation_pos = crane_rot_constraint_x;
				}
				else {
					crane_rot_axis.write(current_rotation_pos);
					Serial.println(current_rotation_pos);
				}
			}
		}

		if (is_FA_moving_up == true) {
			if (current_FA_pos >= crane_forarm_constraint_y) {  // contraint for forearm movement upwards...
				is_FA_moving_up = false;
				Serial.println("crane forearm_upward MAX");
			}
			else {
				current_FA_pos = current_FA_pos + crane_forarm_speed;
				crane_fa_axis.write(current_FA_pos);
				Serial.println(current_FA_pos);
			}
		}
		else if (is_FA_moving_down == true) {
			if (current_FA_pos <= crane_forarm_constraint_x) {  // contraint for forearm movement down...
				is_FA_moving_down = false;
				Serial.println("crane forearm_down MAX");
			}
			else {
				current_FA_pos = current_FA_pos - crane_forarm_speed;
				crane_fa_axis.write(current_FA_pos);
				Serial.println(current_FA_pos);
			}
		}
		if (is_CA_moving_up == true) {
			if (current_CA_pos >= crane_cent_constraint_y) {  // contraint for center_axis movement upwards...
				is_CA_moving_up = false;
				Serial.println("crane center_axis_upward MAX");
			}
			else {
				current_CA_pos = current_CA_pos + crane_cent_speed;
				crane_ca_axis.write(current_CA_pos);
				Serial.println(current_CA_pos);

				// support servo:
				current_CA_2_pos = current_CA_2_pos - crane_cent_speed;
				crane_ca_2_axis.write(current_CA_2_pos);
			}
		}
		else if (is_CA_moving_down == true) {
			if (current_CA_2_pos >= crane_cent_constraint_x) {  // contraint for center_axis movement down...
				is_CA_moving_down = false;
				Serial.println("crane center_axis_down MAX");
			}
			else {
				current_CA_pos = current_CA_pos - crane_cent_speed;
				crane_ca_axis.write(current_CA_pos);
				Serial.println(current_CA_pos);

				// support servo:
				current_CA_2_pos = current_CA_2_pos + crane_cent_speed;
				crane_ca_2_axis.write(current_CA_2_pos);
			}
		}

		if (is_grippper_closing == true) {
			if (current_gripper_pos >= crane_gripp_constraint_y) {  // contraint for center_axis movement upwards...
				is_grippper_closing = false;
				Serial.println("crane gripper closing MAX");
			}
			else {
				current_gripper_pos = current_gripper_pos + crane_gripp_speed;
				crane_gripper_servo.write(current_gripper_pos);
				Serial.println(current_gripper_pos);
			}
		}
		else if (is_grippper_opening == true) {
			if (current_gripper_pos <= crane_gripp_constraint_x) {  // contraint for center_axis movement down...
				is_grippper_opening = false;
				Serial.println("crane gripper opening MAX");
			}
			else {
				current_gripper_pos = current_gripper_pos - crane_gripp_speed;
				crane_gripper_servo.write(current_gripper_pos);
				Serial.println(current_gripper_pos);
			}
		}

	}

	client.loop();
}

/********************************************* ACCELERATE *****************************************************/
void accelerate(int f)
{
	// PWM code:
	if (going_left == true) {
		Serial.println("ACCELERARTING WITH LEFT PRE VALUES");
		global_steering_left = f * axs_0_value;
		global_steering_right = f;
		if (f == 1000) {
			Serial.println("max RM speed");
			digitalWrite(right_pwm, HIGH);
			analogWrite(left_pwm, global_steering_left);
		}
		else {
			analogWrite(right_pwm, global_steering_right);
			analogWrite(left_pwm, global_steering_left);
		}
	}
	else if (going_right == true) {
		Serial.println("ACCELERARTING WITH RIGHT PRE VALUES");
		global_steering_right = f * axs_0_value;
		global_steering_left = f;
		if (f == 1000) {
			Serial.println("max LM speed");
			digitalWrite(left_pwm, HIGH);
			analogWrite(right_pwm, global_steering_right);
		}
		else {
			analogWrite(right_pwm, global_steering_right);
			analogWrite(left_pwm, global_steering_left);
		}
	}
	else {
		Serial.println("ACCELERARTING WITH CENTER FOWARD");
		motor_foward();
		if (f == 1000) {
			Serial.println("max BM speed!");
			digitalWrite(right_pwm, HIGH);
			digitalWrite(left_pwm, HIGH);
		}
		else {
			analogWrite(left_pwm, f);
			analogWrite(right_pwm, f);
		}
	}

}

/********************************************* ACCELERATE BACK *****************************************************/
void accelerate_back(int f)
{
	// PWM code:
	if (going_left == true) {
		Serial.println("ACCELERARTING backwards WITH LEFT PRE VALUES");
		global_steering_left = f * axs_0_value;
		global_steering_right = f;
		if (f == 1000) {
			Serial.println("max RM speed");
			digitalWrite(right_pwm, HIGH);
			analogWrite(left_pwm, global_steering_left);
		}
		else {
			analogWrite(right_pwm, global_steering_right);
			analogWrite(left_pwm, global_steering_left);
		}
	}
	else if (going_right == true) {
		Serial.println("ACCELERARTING backwards WITH RIGHT PRE VALUES");
		global_steering_right = f * axs_0_value;
		global_steering_left = f;
		if (f == 1000) {
			Serial.println("max LM speed");
			digitalWrite(left_pwm, HIGH);
			analogWrite(right_pwm, global_steering_right);
		}
		else {
			analogWrite(left_pwm, global_steering_left);
			analogWrite(right_pwm, global_steering_right);
		}

	}
	else {
		Serial.println("ACCELERARTING WITH CENTER BACKWARD");
		motor_back();
		if (f > 950) {
			Serial.println("max BM speed!");
			digitalWrite(right_pwm, HIGH);
			digitalWrite(left_pwm, HIGH);
		}
		else {
			analogWrite(left_pwm, f);
			analogWrite(right_pwm, f);
		}
	}
}

/********************************************* ACCELERATE ANGLE *****************************************************/
void accelerate_angle(int f, int l, int r)
{
	// PWM code:
	if (l == 1000) {
		Serial.println("max LM speed");
		digitalWrite(left_pwm, HIGH);
	}
	else {
		analogWrite(left_pwm, l);
	}
	if (r == 1000) {
		Serial.println("max RM speed");
		digitalWrite(right_pwm, HIGH);
	}
	else {
		analogWrite(right_pwm, r);
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
	digitalWrite(left_motor_p, HIGH);
	digitalWrite(left_motor_n, LOW);
	digitalWrite(right_motor_p, HIGH);
	digitalWrite(right_motor_n, LOW);
	digitalWrite(left_pwm, HIGH);
	digitalWrite(right_pwm, HIGH);
}
/********************************************* MANUAL BACK	 *****************************************************/
void manual_back(void)
{
	Serial.println("manual_foward Function Running...");
	digitalWrite(left_motor_p, LOW);
	digitalWrite(left_motor_n, HIGH);
	digitalWrite(right_motor_p, LOW);
	digitalWrite(right_motor_n, HIGH);
	digitalWrite(left_pwm, HIGH);
	digitalWrite(right_pwm, HIGH);
}
/********************************************* MANUAL LEFT *****************************************************/
void manual_left(void)
{
	Serial.println("manual_left Function Running...");
	digitalWrite(left_motor_p, LOW);
	digitalWrite(left_motor_n, HIGH);
	digitalWrite(right_motor_p, HIGH);
	digitalWrite(right_motor_n, LOW);
	digitalWrite(left_pwm, HIGH);
	digitalWrite(right_pwm, HIGH);
}
/********************************************* MANUAL RIGHT *****************************************************/
void manual_right(void)
{
	Serial.println("manual_right Function Running...");
	digitalWrite(left_motor_p, HIGH);
	digitalWrite(left_motor_n, LOW);
	digitalWrite(right_motor_p, LOW);
	digitalWrite(right_motor_n, HIGH);
	digitalWrite(left_pwm, HIGH);
	digitalWrite(right_pwm, HIGH);
}
/********************************************* MANUAL STOP *****************************************************/
void manual_stop(void)
{
	Serial.println("manual_stop Function Running...");
	digitalWrite(left_motor_p, LOW);
	digitalWrite(left_motor_n, LOW);
	digitalWrite(right_motor_p, LOW);
	digitalWrite(right_motor_n, LOW);
	digitalWrite(left_pwm, HIGH);
	digitalWrite(right_pwm, HIGH);
}
/********************************************* FOWARD *****************************************************/
void motor_foward(void)
{
	digitalWrite(left_motor_p, HIGH);
	digitalWrite(left_motor_n, LOW);
	digitalWrite(right_motor_p, HIGH);
	digitalWrite(right_motor_n, LOW);
}
/********************************************* FOWARD *****************************************************/
void motor_back(void)
{
	digitalWrite(left_motor_p, LOW);
	digitalWrite(left_motor_n, HIGH);
	digitalWrite(right_motor_p, LOW);
	digitalWrite(right_motor_n, HIGH);
}
/* Sending current crane position values to MQTT control panel */
void send_crane_pos(void)
{
	Serial.println("send_crane_pos func running...");
	String temp_r_pos = String(current_rotation_pos);
	String temp_fa_pos = String(current_FA_pos);
	String temp_ca_pos = String(current_CA_pos);
	String temp_ca_2_pos = String(current_CA_2_pos);
	String temp_claw_pos = String(current_gripper_pos);


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
	crane_con_output["rx"] = crane_rot_constraint_x;
	crane_con_output["ry"] = crane_rot_constraint_y;
	/***     Center axis      ***/
	crane_con_output["cx"] = crane_cent_constraint_x;
	crane_con_output["cy"] = crane_cent_constraint_y;
	/***     Forearm axis     ***/
	crane_con_output["fx"] = crane_forarm_constraint_x;
	crane_con_output["fy"] = crane_forarm_constraint_y;
	/***    Gripper axis      ***/
	crane_con_output["gx"] = crane_gripp_constraint_x;
	crane_con_output["gy"] = crane_gripp_constraint_y;

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
	crane_speed_output["rs"] = crane_rot_speed;
	/***     Center axis      ***/
	crane_speed_output["cs"] = crane_cent_speed;
	/***     Forearm axis     ***/
	crane_speed_output["fs"] = crane_forarm_speed;
	/***    Gripper axis      ***/
	crane_speed_output["gs"] = crane_gripp_speed;

	String jsonStr;
	crane_speed_output.printTo(jsonStr);
	Serial.print("RAW JSON as a String ready for transmision: ");
	Serial.println(jsonStr);
	client.publish("/ExpR/out/crane_speed", (char*)jsonStr.c_str());
}
void save_crane_pos(void)
{
	Serial.println("save_crane_pos to EEPROM Flash mem");
	EEPROM.write(0, current_rotation_pos);
	EEPROM.write(1, current_FA_pos);
	EEPROM.write(2, current_CA_pos);
	EEPROM.write(3, current_CA_2_pos);
	EEPROM.write(4, current_gripper_pos);
	EEPROM.commit();
}
void save_crane_constraints(void)
{
	Serial.println("save_crane_constraints to EEPROM Flash mem");
	/***     Rotation axis    ***/
	EEPROM.write(5, crane_rot_constraint_x);
	EEPROM.write(6, crane_rot_constraint_y);
	/***    Center axis     ***/
	EEPROM.write(7, crane_cent_constraint_x);
	EEPROM.write(8, crane_cent_constraint_y);
	/***     Forearm axis     ***/
	EEPROM.write(9, crane_forarm_constraint_x);
	EEPROM.write(10, crane_forarm_constraint_y);
	/***    Gripper axis      ***/
	EEPROM.write(11, crane_gripp_constraint_x);
	EEPROM.write(12, crane_gripp_constraint_y);

	EEPROM.commit();
}
void save_crane_movement_speed(void)
{
	Serial.println("save_crane_movement_speed to EEPROM Flash mem");
	/***     Rotation axis    ***/
	EEPROM.write(13, crane_rot_speed);
	/***    Center axis     ***/
	EEPROM.write(14, crane_cent_speed);
	/***     Forearm axis     ***/
	EEPROM.write(15, crane_forarm_speed);
	/***    Gripper axis      ***/
	EEPROM.write(16, crane_gripp_speed);

	EEPROM.commit();
}
void apply_current_crane_pos_values(void)
{
	Serial.println("moving crane servos to current position variables");
	crane_rot_axis.write(current_rotation_pos);
	crane_fa_axis.write(current_FA_pos);
	crane_ca_axis.write(current_CA_pos);
	crane_ca_2_axis.write(current_CA_2_pos);
	crane_gripper_servo.write(current_gripper_pos);
}
/********************************************* light_toggle *****************************************************/
void light_toggle(void)
{
	Serial.println("light_toggle");
	if (is_light_on == false) {
		is_light_on = true;
		Serial.println("light ON");
		digitalWrite(head_light_relay, HIGH);
		digitalWrite(head_light_relay, HIGH);
	}
	else {
		is_light_on = false;
		Serial.println("light OFF");
		digitalWrite(head_light_relay, LOW);
		digitalWrite(head_light_relay, LOW);
	}
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
			client.subscribe("/ExpR/in/newC");
			client.subscribe("/ExpR/in/ncc");
			client.subscribe("/ExpR/in/ncs");
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
		}
	}
}
