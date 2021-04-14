/* 2021-03-27 Cam King
	OHSU Helping Hand - finding zero, using saftey nets, and control using encoder as joystick for 1 finger.
	Notes: - Make sure to add some lube if gears are dry
		 - if the encoder jumpers fall off immedietly pull the orange pwm wire from the board and restart
		 - the feedback encoder should have green on top, purple on bottom
		 - make sure to unplug battery when not using
		 - if anything seems to be going wrong pull the orange pwm wire so it does not break itself
		 - if using encoder joystick, clockwise opens (increase pwm) and counterclockwise closes (decreases)
*/


#include <ESP32Servo.h>		//https://github.com/madhephaestus/ESP32Servo
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder

#define FIND_ZERO 0

#define PWM_NEUTRAL 91

#define NUM_JOINTS 1
#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 20
// The serial to read signals from. Typically Serial2 for RPi, Serial for USB (helpful for debugging)
// You must also uncomment SigSerial.begin() if this is set to be not Serial
#define SigSerial Serial

// Angle where finger is fully closed, as sum of angles of all joints. Min is 0. Max Pi can give is 265.
#define MAX_ANGLE 200

int arr1[NUM_JOINTS] = {0};
int arr2[NUM_JOINTS] = {0};
int* target_angles = arr1;
int* setpoints = arr2; // Must hold semRpi to access
bool readCommand();

TaskHandle_t getRpiVals;
//http://exploreembedded.com/wiki/Mutex_Semaphore_02%3A_Recursive_Locks
SemaphoreHandle_t semRpi = NULL;

double kp, ki, kd;

bool printing;

//initialize esc & feedback/joystick encoders
Servo ESC;
//ESP32Encoder control_encoder;
//ESP32Encoder feedback_encoder;

//initialize variables
int32_t pwmValue;
int32_t lastPos, currentPos;
const int32_t maxPos = 90;
int32_t state;
int32_t directionChange;
int32_t addedMovement;
int32_t addOpen, addClose;

int32_t longAdjust, longmidAdjust, shortmidAdjust, shortAdjust;

unsigned long rate;
unsigned long currentLoopTime, deltaTime;

#define NUM_JOINT_STRUCTS 1

struct JointPin {
	int encoderA;
	int encoderB;
	int esc;
};

struct Joint {
    double setpoint;
	int ticks; // # ticks from start
	double errSum; // Running total of errors (for integral)
	double lastErr; // = prevOutput - prevInput (for derivative)
	int lastTick; //the time when tick last changed
	int output; // PWM to write
	int lastOutput;
	ESP32Encoder encoder;
	Servo esc;
	
	bool checkBounds() {
	    //Safety Net: once zero is found, has bounds from positions 0 (fully extended) & 110 (fully closed)
	    //	*Note: Because the springs are helping to close the finger there is no noticable metric to identify
	    //	when resisting closure, however as long as the gear position stays as low as 110 the fishing
	    //	line does not tangle i.e. slack doesn't become an issue.
	    if ((this->ticks > 85 && this->output <= PWM_NEUTRAL) ||
			(this->ticks < 0 && this->output >= PWM_NEUTRAL))
	    {
		    this->output = PWM_NEUTRAL;
		    return false;
	    }
	    return true;
    }
    
    bool isObstructed() {
		//Safety Net: trying to extend but cant
		if(this->lastOutput == PWM_NEUTRAL){
			this->lastTick = currentLoopTime;
			return false;
		} else return currentLoopTime - this->lastTick >= 1200;
	}
	
	double computePID() {
		/*Compute all the working error variables*/
		double error = this->setpoint - this->ticks;

		this->errSum += (error * deltaTime);
		double dErr = (error - this->lastErr) / deltaTime;

		/*Compute PID output*/
		output = PWM_NEUTRAL - kp * error + ki * this->errSum + kd * dErr;

		/*Remember some variables for next time*/
		this->lastErr = error;

		return output;
	}
};

Joint index_finger{};

Joint joints[NUM_JOINT_STRUCTS] = {index_finger};

const JointPin pins[NUM_JOINT_STRUCTS] = {
	JointPin{39, 36, 13}
};

void setup()
{
	printing = false;
	//initialize serial for feedback
	Serial.begin(115200);
	if (&SigSerial != &Serial) {
		Serial.println("Starting SigSerial");
		SigSerial.begin(115200);
	}
	SigSerial.setTimeout(SERIAL_TIMEOUT);
	
	ESP32Encoder::useInternalWeakPullResistors = DOWN;

	for(int i = 0; i < NUM_JOINT_STRUCTS; i++){
		Joint& j = joints[i];
		JointPin p = pins[i];
		
		j.setpoint = 0;
		j.ticks = 0;
		j.errSum = 0;
		j.lastErr = 0;
		j.lastTick = millis();
		j.output = PWM_NEUTRAL;
		j.lastOutput = PWM_NEUTRAL;
		
		j.encoder = ESP32Encoder();
		Serial.println("Attaching encoder to pins " + String(p.encoderA) + " and " + String(p.encoderB));
		j.encoder.attachSingleEdge(p.encoderA, p.encoderB);
		j.encoder.setCount(0);
		
		j.esc = Servo();
		j.esc.attach(p.esc);
		j.esc.write(j.output);
		
		// This is before task creation and so is thread-safe		
		setpoints[i] = 0;
	}

	// I have no idea what these are
	longAdjust = 0;
	longmidAdjust = 7, shortmidAdjust = 5;
	shortAdjust = 7; //bounds found in state 3 of findZero()

	state = 0;		//intialize state (tracks states in findZero function)

	tunePid(.2, 0, 0);

	initSemaphore(semRpi);
	//function, name, stack in words, input param, priority(higher is higher), task handle, core
	xTaskCreatePinnedToCore(getRpiValsCode,
							"GetRpiVals",
							10000,
							NULL,
							0,
							&getRpiVals,
							0);

	if (!FIND_ZERO)
	{
		Serial.println("****zero set****");
		state = 6;
	}
	Serial.println(joints[0].output);
	delay(5000); // ESCs sing to us
}

void loop()
{
	//loop sends pwm to the ESC as checking for errors(obstructions/out of bound movement)
	//loop tracks when feedback encoder moves (current/lastTick) and position of feedback encoder (current/lastPos)
	
	takeSem();
	for(int i = 0; i < NUM_JOINT_STRUCTS; i++){
		Joint& j = joints[i];
		j.setpoint = setpoints[i];
	}
	giveSem();

	// Technically, we should be setting this as close to when we read the
	// encoder as possible so we accurately compute time, but the error for each
	// joint should be constant, which means the time delta should still be
	// accurate
	unsigned long lastLoopTime = currentLoopTime;
    currentLoopTime = millis();
    deltaTime = currentLoopTime - lastLoopTime;

	/*if (FIND_ZERO)
	{
		findZero();
	}*/
	
	for(int i = 0; i < NUM_JOINT_STRUCTS; i++){
	    Joint& j = joints[i];
	    
	    int prevTicks = j.ticks;
	    j.ticks = j.encoder.getCount();
	    Serial.println("Ticks:"+String(j.ticks));
	    if(j.ticks != prevTicks){
	        j.lastTick = currentLoopTime;
	    }
	    
	    if(!j.isObstructed()){
	        j.computePID();
	        if(!j.checkBounds()){
	        	Serial.println("Bounds error on joint " + String(i));
	        }
	    } else {
	        j.output = PWM_NEUTRAL;
	        Serial.println("Obstruction error on joint " + String(i));
	        // TODO: Not calling PID in this case will screw up ID terms
	    }
	    
	    j.esc.write(j.output);
	    j.lastOutput = j.output;
	       
	}
	
	//printStatus();
	delay(100);	 //TODO: Make delay a delayUntil and hardcode delta, and consider shortening 
}

// Only printing for first joint because I don't have time to care about the second.
void printStatus() {
	Joint& j = joints[0];
	Serial.print("PWM:" + String(j.output));

	Serial.print("\tPos:" + String((int)j.ticks));
	// Serial.printf("\tRate: %8d", (unsigned long)rate);
	// Serial.print("\tstate: " + String((int32_t)state));
	// Serial.print("\tdchng: " + String((int32_t)directionChange));

	// Serial.print("\ttarget angle: " + String(target_angles[0]));

	// TODO: isn't this the same as position?
	//Serial.print("\tinput:");
	//Serial.print((double)input);
	
	// TODO: isn't this the same as PWM?
	/*Serial.print(" ");
	Serial.print("\toutput:");
	Serial.print((double)output);*/
	
	Serial.print("\tSetpoint:");
	Serial.print(j.setpoint);
	
	Serial.println("");
	// Serial.print("kd*dErr:");
	// Serial.print((double)(kd * dErr));
	// Serial.println(" ");
	/*https://dreamonward.com/2020/07/25/arduino-serial-plotter-labels/ */
}

void initSemaphore(SemaphoreHandle_t sem)
{
	vSemaphoreCreateBinary(semRpi);
	if (semRpi == NULL)
	{
		Serial.println("Failed to create semaphore.");
		delay(10000);
		exit(-1);
	}
}

// Seperate function in case method changes
// Given an angle from the Pi, find what the setpoint should be (in units of encoder position)
inline int toSetpoint(int angle){
	return min(angle, MAX_ANGLE) * maxPos / MAX_ANGLE;
}

inline void takeSem(){
	// Block until acquired
	// TODO: is there a reason not to set the max wait time to the max value?
	while(xSemaphoreTake(semRpi, 500) == pdFALSE) {};
}

inline void giveSem(){
	xSemaphoreGive(semRpi);
}

void getRpiValsCode(void *parameter)
{
	while (true) {
		if (readCommand()) {
			for(int i = 0; i < NUM_JOINTS; i++){
				target_angles[i] = toSetpoint(target_angles[i]);
			}
			// Since only this core will write to setpoints, this read is thread-safe without locking the sem
			int* temp = setpoints;
			takeSem();
			setpoints = target_angles;		
			giveSem();
			target_angles = temp;	
		}
	}
}

inline void tunePid(double Kp, double Ki, double Kd)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

/**
	 Reads command from SigSerial.
	 Command must be of the form <#,#,...,#>, where the
	 number of numers is at least NUM_JOINTS.

	 Returns: true if read successful, false otherwise

	 Side effects: target_angles modified. This may occur even if false
	 is returned, so if the old angles are needed save them before
	 calling.
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
			Serial.println("Failed to parse number: " + String(input));
			return false;
		}
		target_angles[joint] = angle;
		input = endPtr + 1; //Skip the comma
		joint += 1;
	}
	return true;
}

/*
void findZero()
{
	if (state <= 5)
	{
		if (state == 0)
		{
			rate = 9999;	 //rate must start high to not immediately begin state 2
			pwmValue = PWM_NEUTRAL; //initialize at no movement
			ESC.write(PWM_NEUTRAL);
			delay(4000); //allows time for esc to boot (4sec)
			Serial.println("****esc ready****");
			state = 1; //transition to state 1
		}

		if (pwmValue >= 88 && state == 1)
		{ //increments pwm till 87, the increment is very fast but nessesary for esc
			pwmValue--;
		}

		if (rate <= 47 && state == 1)
		{
			//once the finger is closing at a rate of 47ms per click finger stops
			//it then notes the position and engages state 2
			pwmValue = PWM_NEUTRAL;
			directionChange = feedback_encoder.getCount();
			state = 2;
		}

		if (pwmValue <= 104 && state == 2)
		{ //increments pwm till 87
			pwmValue++;
		}

		if (rate > 30 && state == 2 && directionChange - 5 > feedback_encoder.getCount())
		{
			//once the finger is opening at a rate over 30ms per click finger stops and state 3 begins
			pwmValue = PWM_NEUTRAL;
			state = 3;
		}

		if (state == 3)
		{
			//The amount of distance the finger travels before hitting the end point of state 2 determines further adjustment of finger position.
			//This is because the finger closes at a relitively incosisten rate but opens far more consistantly.

			addedMovement = (directionChange - feedback_encoder.getCount()); //finds how far the finger traveled from change indirection

			if (addedMovement > 85)
			{ //if long the finger will go to correct location
				Serial.println("****long****");
				addOpen = 0;
				addClose = 0;
				addedMovement = longAdjust; //as the bounds can very from amount of grease/different fingers the values are adjusted in setup
			}
			else if (addedMovement <= 85 && addedMovement > 60)
			{ //if longmedium close predefined amount
				Serial.println("****longmid****");
				addOpen = 0;
				addClose = 1;
				addedMovement = longmidAdjust;
			}
			else if (addedMovement <= 60 && addedMovement > 35)
			{ //if shorrmedium close
				Serial.println("****shortmid****");
				addOpen = 0;
				addClose = 1;
				addedMovement = shortmidAdjust;
			}
			else if (addedMovement <= 35)
			{ //if only a small amount no movement is expected
				Serial.println("****short****");
				addOpen = 0;
				addClose = 1;
				addedMovement = shortAdjust;
			}
			directionChange = feedback_encoder.getCount();
			state = 4;
		}

		//starting state 4 we now have the current location and where we must go
		if (state == 4 && pwmValue >= 87 && addClose == 1)
		{ //closes if determined in state 3
			pwmValue--;
		}

		if (state == 4 && pwmValue <= 104 && addOpen == 1)
		{ //open if determined in state 3
			pwmValue++;
		}

		if (state == 4 && (directionChange + addedMovement) == feedback_encoder.getCount())
		{ //once desired position is reached the movement stops
			pwmValue = PWM_NEUTRAL;
			state = 5;
		}

		if (state == 5 && deltaTime >= 400)
		{ //the last state waits .4s as the momementum can carry thru subsiquent ticks
			Serial.println("****zero found****");
			rate = 0; //when pwmValue is forced to PWM_NEUTRAL, the rate is sent to zero for checkObstruction() to work
			feedback_encoder.setCount(0);
			state = 6;
		}
	}

		currentPos = feedback_encoder.getCount();

		calcRate();

		ESC.write(pwmValue); //send pwm signal to esc

		lastTick = currentTick;
		lastPos = currentPos;
}*/
