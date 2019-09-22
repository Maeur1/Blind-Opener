#include <Arduino.h>
/**
 * Author Teemu Mäntykallio
 * Initializes the library and turns the motor in alternating directions.
*/

#define EN_PIN    16
#define DIR_PIN   4  //			19			55	//direction
#define STEP_PIN  5  //			18			54	//step
#define MOSI_PIN 13
#define MISO_PIN 12
#define CS_PIN 1
#define SCK_PIN 14
#define MOTOR_STEPS 200 // Number of steps per rotation
#define MICROSTEPS 256
#define MAX_DELTA MICROSTEPS
#define MIN_DELTA -MICROSTEPS
#define ESP_LED 2

#include "BasicStepperDriver.h"
#include <TMC2130Stepper.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h> // required for settings file to make it readable
#include <FS.h>
#include <ESPmanager.h>

ESP8266WebServer HTTP(80);
BasicStepperDriver stepper = BasicStepperDriver(MOTOR_STEPS, DIR_PIN, STEP_PIN);
TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

const char * defaultSSID = "DefaultSSID";
const char * defaultPSK = "DefaultPassword";
const char * logging_file = "/temp.csv";
const char * mqtt_server = "hubpi.local";
const char * topic_set = "esp8266/blinds/set";
const char * topic_position = "esp8266/blinds/position";
const char * topic_set_position = "esp8266/blinds/set_position";
const int blinds_top = -1474560;
const int blinds_bot = 0;

int taken = 0; //Steps taken so far by the stepper motor
int target = 0; //Steps wanted to be taken by the stepper motor 
int position = taken;
bool publish_position = false;

ESPmanager settings(HTTP, SPIFFS, "ESPManager", defaultSSID , defaultPSK);

void motorStatus() 
{
	String stall = driver.stallguard() ? "STALLED" : "NOT STALLED";
	String status = String(driver.DRV_STATUS(), BIN);
	String high = digitalRead(EN_PIN) ? " HIGH" : " LOW";
	String chopper = String(driver.CHOPCONF(), BIN);
	String lost_steps = String(driver.LOST_STEPS(), BIN);
	String cool = String(driver.COOLCONF(), BIN);
	File data = SPIFFS.open(logging_file, "r");
	HTTP.send(200, "application/json", 
	"STATUS REG: " + status + 
	"\nSTALL: " + stall + 
	"\nEN PIN: " + high + 
	"\nCHOPPER CONF: " + chopper + 
	"\nCOOL CONF: " + cool + 
	"\nLOST_STEPS: " + lost_steps + "\n" +
	data.readString());
	data.close();
}

void motorUp() {
	target = blinds_top;
	publish_position = true;
}

void motorDown() {
	target = blinds_bot;
	publish_position = true;
}

void motorStop(){
	target = taken;
	publish_position = false;
}

void ledOff(){
	digitalWrite(ESP_LED, HIGH);
	HTTP.send(200, "application/json", "LED Turned OFF");
}

void ledOn(){
	digitalWrite(ESP_LED, LOW);
	HTTP.send(200, "application/json", "LED Turned ON");
}

void mqtt_callback(char* topic, byte* payload, unsigned int length){
	String response = "";
	for(int i = 0; i < length; i++){
		response += (char)payload[i];
	}
	if(String(topic) == topic_set){
		if(response == "OPEN"){
			motorUp();
		} else if(response == "CLOSE") {
			motorDown();
		} else if(response == "STOP"){
			motorStop();
		} else if(response == "ledoff"){
			ledOff();
		} else if(response == "ledon"){
			ledOn();
		}
	} else if(String(topic) == topic_set_position) {
		target = map(response.toInt(), 0, 100, 0, blinds_top);
	}
}

void stepperLoop(){
	int delta = target - taken;
	delta = max(delta, MIN_DELTA);
	delta = min(delta, MAX_DELTA);
	if(!delta){
		digitalWrite(EN_PIN, HIGH);
		if(publish_position) {
			position = map(taken, blinds_top, 0, 100, 0);
			client.publish(topic_position, String(position).c_str());
			publish_position = false;
		}
	} else {
		digitalWrite(EN_PIN, LOW);
		stepper.rotate(delta);
		taken += delta;
		position = map(taken, blinds_top, 0, 100, 0);
		client.publish(topic_position, String(position).c_str());
	}
}

void setup() {
	SPIFFS.begin();
	SPIFFS.remove(logging_file);
	pinMode(ESP_LED, OUTPUT);
	digitalWrite(ESP_LED, HIGH);
	Serial.begin(115200);
	while(!Serial);
	Serial.println("Start...");

	HTTP.on("/motorstatus", motorStatus);
	HTTP.on("/ledoff", ledOff);
	HTTP.on("/ledon", ledOn);

	settings.begin();
	HTTP.begin();
	client.setServer(mqtt_server, 1883);
	client.setCallback(mqtt_callback);

	driver.begin(); 			// Initiate pins and registeries
	driver.SilentStepStick2130(1000); 	// Set stepper current to 1000mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
	driver.stealthChop(1); 	// Enable extremely quiet stepping
	driver.coolstep_min_speed(50);
	driver.sg_stall_value(26);

	driver.microsteps(MICROSTEPS);

	stepper.begin(5000, MICROSTEPS);
	
	digitalWrite(EN_PIN, HIGH);
	digitalWrite(ESP_LED, LOW);
	motorDown();
}

void loop() {
	settings.handle();
	HTTP.handleClient();
	if(!client.connected()){
		client.connect("mayur-blinds");
		client.subscribe(topic_set);
		client.subscribe(topic_set_position);
	}
	stepperLoop();
	client.loop();
}