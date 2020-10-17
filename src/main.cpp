#include <Arduino.h>

// ESP8266 Usable Pins: 16, 15, 14, 13, 12, 5, 4, 3, 2, 1, 0

#define EN_PIN 16
#define DIR_PIN 15
#define STEP_PIN 1
#define MOSI_PIN 13
#define MISO_PIN 12
#define CS_PIN 15
#define SCK_PIN 14
#define MOTOR_STEPS 200 // Number of steps per rotation
#define MICROSTEPS 128
#define MAX_DELTA MICROSTEPS / 4
#define MIN_DELTA -MICROSTEPS / 4
#define DEADZONE 600
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
#include <AS5600.h>

ESP8266WebServer HTTP(80);
BasicStepperDriver stepper = BasicStepperDriver(MOTOR_STEPS, DIR_PIN, STEP_PIN);
TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);
WiFiClient espClient;
PubSubClient client(espClient);
AS5600 encoder;

const char *defaultSSID = "DefaultSSID";
const char *defaultPSK = "DefaultPassword";
const char *logging_file = "/temp.csv";
const char *mqtt_server = "hub.local";
const char *topic_set = "esp8266/blinds/set";
const char *topic_position = "esp8266/blinds/position";
const char *topic_feedback = "esp8266/blinds/feedback";
const char *topic_set_position = "esp8266/blinds/set_position";
const long encoder_top = 700;
const long encoder_bottom = -179000;
const long max_power = -80000;
long revolutions = 0;
double output;
long lastOutput;
long lastOvercurrent = 0;

long target = 0; //Steps wanted to be taken by the stepper motor
long position = 0;
bool publish_position = false;

ESPmanager settings(HTTP, SPIFFS, "ESPManager", defaultSSID, defaultPSK);

void motorStatus()
{
	String stall = driver.stallguard() ? "STALLED" : "NOT STALLED";
	String status = String(driver.DRV_STATUS(), BIN);
	String high = digitalRead(EN_PIN) ? " HIGH" : " LOW";
	String chopper = String(driver.getCurrent());
	String lost_steps = String(stepper.getRPM());
	String cool = String(position);
	File data = SPIFFS.open(logging_file, "r");
	HTTP.send(200, "application/json",
			  "STATUS REG: " + status +
				  "\nSTALL: " + stall +
				  "\nEN PIN: " + high +
				  "\nMOTOR CURRENT: " + chopper +
				  "\nENCODER POS: " + cool +
				  "\nMOTOR SPEED: " + lost_steps + "\n" +
				  data.readString());
	data.close();
}

void motorUp()
{
	target = encoder_top;
	publish_position = true;
}

void motorDown()
{
	target = encoder_bottom;
	publish_position = true;
}

void motorStop()
{
	target = position;
	publish_position = false;
}

void ledOff()
{
	digitalWrite(ESP_LED, HIGH);
	HTTP.send(200, "application/json", "LED Turned OFF");
}

void ledOn()
{
	digitalWrite(ESP_LED, LOW);
	HTTP.send(200, "application/json", "LED Turned ON");
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
	String response = "";
	for (int i = 0; i < length; i++)
	{
		response += (char)payload[i];
	}
	if (String(topic) == topic_set)
	{
		if (response == "OPEN")
		{
			motorUp();
		}
		else if (response == "CLOSE")
		{
			motorDown();
		}
		else if (response == "STOP")
		{
			motorStop();
		}
		else if (response == "ledoff")
		{
			ledOff();
		}
		else if (response == "ledon")
		{
			ledOn();
		}
		else if (response == "RESET")
		{
			target = encoder_top;
			publish_position = true;
		}
	}
	else if (String(topic) == topic_set_position)
	{
		target = map(response.toInt(), 0, 100, encoder_bottom, encoder_top);
	}
	else if (String(topic) == topic_feedback)
	{
		position = map(response.toInt(), 0, 100, 0, encoder_top);
		target = position;
		publish_position = true;
	}
}

void stepperLoop()
{
	int delta = target - position;
	if (delta < DEADZONE && delta > -DEADZONE)
	{
		stepper.stop();
		delta = 0;
	}
	delta = max(delta, MIN_DELTA);
	delta = min(delta, MAX_DELTA);
	output = encoder.getPosition();
	if ((lastOutput - output) > 2047)
		revolutions++;
	if ((lastOutput - output) < -2047)
		revolutions--;
	position = revolutions * 4096 + output;
	lastOutput = output;
	if (!delta)
	{
		digitalWrite(EN_PIN, HIGH);
		if (publish_position)
		{
			long realPos = target == encoder_bottom ? encoder_bottom : position;
			client.publish(topic_position, String(map(realPos, encoder_top, encoder_bottom, 100, 0)).c_str());
			publish_position = false;
		}
	}
	else
	{
		int power, speed;
		if (position < max_power && target > max_power)
		{
			power = 1500;
			speed = 30;
		}
		else if (position > max_power && target > max_power)
		{
			power = 1250;
			speed = 30;
		}
		else
		{
			power = 800;
			speed = 90;
		}
		stepper.begin(speed, MICROSTEPS);
		driver.SilentStepStick2130(power);
		driver.hold_delay(5);
		driver.hold_current(0);
		driver.DRV_STATUS();
		if (driver.drv_err())
		{
			digitalWrite(EN_PIN, HIGH);
			lastOvercurrent = millis();
		}
		if (millis() - lastOvercurrent > 5000)
		{
			digitalWrite(EN_PIN, LOW);
			stepper.rotate(delta);
			client.publish(topic_position, String(map(position, encoder_top, encoder_bottom, 100, 0)).c_str());
			publish_position = true;
		}
	}
}

void setup()
{
	SPIFFS.begin();
	SPIFFS.remove(logging_file);
	pinMode(ESP_LED, OUTPUT);
	pinMode(EN_PIN, OUTPUT);
	digitalWrite(ESP_LED, HIGH);
	Serial.begin(115200);
	while (!Serial)
		;
	Serial.println("Start...");

	HTTP.on("/motorstatus", motorStatus);
	HTTP.on("/ledoff", ledOff);
	HTTP.on("/ledon", ledOn);

	settings.begin();
	HTTP.begin();
	client.setServer(mqtt_server, 1883);
	client.setCallback(mqtt_callback);

	driver.begin();		   // Initiate pins and registeries
	driver.stealthChop(1); // Enable extremely quiet stepping
	driver.coolstep_min_speed(50);
	driver.sg_stall_value(26);
	driver.microsteps(MICROSTEPS);

	digitalWrite(EN_PIN, HIGH);
	digitalWrite(ESP_LED, LOW);
	motorUp();
	ledOff();
}

void loop()
{
	settings.handle();
	HTTP.handleClient();
	if (!client.connected())
	{
		client.connect("mayur-blinds");
		client.subscribe(topic_set);
		client.subscribe(topic_set_position);
		client.subscribe(topic_feedback);
	}
	stepperLoop();
	client.loop();
}