/*
  Developed by Cam King
  Controls the finger using an input encoder. Records encoder input, encoder output, and ms per click. 
*/

#include <ESP32Servo.h>
#include <ESP32Encoder.h>

#define NUM_JOINTS 5
#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 20

// The serial to read signals from. Typically Serial2 for RPi, Serial for USB (helpful for debugging)
// You must also uncomment SigSerial.begin() if this is set to be not Serial
#define SigSerial Serial

#define PWM_NEUTRAL 91

// Pins
#define ENCODER_PIN_A 35 // Index
#define ENCODER_PIN_B 34
#define ESC_PIN 12

#define ENCODER2_PIN_A 39 // Middle
#define ENCODER2_PIN_B 36
#define ESC2_PIN 13

#define ENCODER3_PIN_A 33 //Ring
#define ENCODER3_PIN_B 32
#define ESC3_PIN 14

#define ENCODER4_PIN_A 26 //Pinky
#define ENCODER4_PIN_B 25
#define ESC4_PIN 27

#define ENCODER5_PIN_A 21 //Thumb
#define ENCODER5_PIN_B 19
#define ESC5_PIN 22

int angles[NUM_JOINTS];

bool readCommand();

Servo ESC; // An object is defined for ESC.
Servo ESC2;
Servo ESC3;
Servo ESC4;
Servo ESC5;

ESP32Encoder encoder;
ESP32Encoder feedback;
ESP32Encoder feedback2;
ESP32Encoder feedback3;
ESP32Encoder feedback4;
ESP32Encoder feedback5;

int32_t pwmValue;
unsigned long rate;

int32_t lastPos;
int32_t currentPos;

int32_t pos2;
int32_t pos3;
int32_t pos4;
int32_t pos5;

unsigned long lastTick;
unsigned long currentTick;

void setup()
{
	Serial.begin(115200);
	ESP32Encoder::useInternalWeakPullResistors = DOWN;
	encoder.attachSingleEdge(23, 22);
	encoder.setCount(PWM_NEUTRAL);

	feedback.attachSingleEdge(ENCODER_PIN_A, ENCODER_PIN_B);
	feedback.setCount(0);

	feedback2.attachSingleEdge(ENCODER2_PIN_A, ENCODER2_PIN_B);
	feedback2.setCount(0);

	feedback3.attachSingleEdge(ENCODER3_PIN_A, ENCODER3_PIN_B);
	feedback3.setCount(0);

	feedback4.attachSingleEdge(ENCODER4_PIN_A, ENCODER4_PIN_B);
	feedback4.setCount(0);

	feedback5.attachSingleEdge(ENCODER5_PIN_A, ENCODER5_PIN_B);
	feedback5.setCount(0);

	SigSerial.setTimeout(SERIAL_TIMEOUT);

	ESC.attach(ESC_PIN); // PWM signal output from pin 13.
	ESC2.attach(ESC2_PIN);
	ESC3.attach(ESC3_PIN);
	ESC4.attach(ESC4_PIN);
	ESC5.attach(ESC5_PIN);
	angles[0] = PWM_NEUTRAL;
	angles[1] = PWM_NEUTRAL;
	angles[2] = PWM_NEUTRAL;
	angles[3] = PWM_NEUTRAL;
	angles[4] = PWM_NEUTRAL;
	delay(5000);
	ESC.write(PWM_NEUTRAL);
	ESC2.write(PWM_NEUTRAL);
	ESC3.write(PWM_NEUTRAL);
	ESC4.write(PWM_NEUTRAL);
	ESC5.write(PWM_NEUTRAL);
}

//encoder.getCount() works, but feedback.getCount() always equals zero, which makes it hard to print...
// moveing forward, try downloading this file again, git init, check if it works, and make mods from there.
//it was working 2 hours ago... and I don't think I changed anything in this file...

void loop()
{
	currentPos = feedback.getCount();
	pos2 = feedback2.getCount();
	pos3 = feedback3.getCount();
	pos4 = feedback4.getCount();
	pos5 = feedback5.getCount();
	currentTick = millis();
	rate = currentTick - lastTick;
	Serial.println("PWM:" + String(angles[0])
		+ " PWM2:" + String(angles[1])
		+ " PWM3:" + String(angles[2])
		+ " PWM4:" + String(angles[3])
		+ " PWM5:" + String(angles[4])
		+ " - Pos:" + String((int32_t)currentPos)
		+ " - Pos2:" + String((int32_t)pos2)
		+ " - Pos3:" + String((int32_t)pos3)
		+ " - Pos4:" + String((int32_t)pos4)
		+ " - Pos5:" + String((int32_t)pos5));

	if (SigSerial.available() > 0){
		if(readCommand()){
			ESC.write(angles[0]);
			ESC2.write(angles[1]);
			ESC3.write(angles[2]);
			ESC4.write(angles[3]);
			ESC5.write(angles[4]);
		} else {
			ESC.write(PWM_NEUTRAL);
			ESC2.write(PWM_NEUTRAL);
			ESC3.write(PWM_NEUTRAL);
			ESC4.write(PWM_NEUTRAL);
			ESC5.write(PWM_NEUTRAL);
			Serial.println("Failed to parse. Stopping");
			angles[0] = PWM_NEUTRAL;
			angles[1] = PWM_NEUTRAL;
			angles[2] = PWM_NEUTRAL;
			angles[3] = PWM_NEUTRAL;
			angles[4] = PWM_NEUTRAL;
		}
	}
	//  Serial.println ("feedback.getCount(): " + String((int32_t)feedback.getCount()) + " - encoder.getCount: " + String((int32_t)encoder.getCount()));

	lastTick = currentTick;
	lastPos = currentPos;
}

/**
 * Reads command from SigSerial. 
 * Command must be of the form <#,#,...,#>, where the
 * number of numers is at least NUM_JOINTS.
 * 
 * Returns: true if read successful, false otherwise
 * 
 * Side effects: angles modified. This may occur even if false
 * is returned, so if the old angles are needed save them before
 * calling.
 */
bool readCommand()
{
	if (SigSerial.available() <= 0)
		return false;
	String command = SigSerial.readStringUntil('\n');
	if (!command.startsWith("<") || !command.endsWith(">"))
		return false;
	const char *input = command.c_str() + 1; //Skip opening '<'
	char *endPtr;
	int joint = 0;
	while (joint < NUM_JOINTS)
	{
		int angle = (int)strtol(input, &endPtr, 10);
		if (endPtr == input)
		{
			Serial.println("Failed to parse number");
			return false;
		}
		angles[joint] = angle;
		input = endPtr + 1; //Skip the comma
		joint += 1;
	}
	return true;
}
