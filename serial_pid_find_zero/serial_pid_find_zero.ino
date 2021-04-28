/* 2021-03-27 Cam King
	OHSU Helping Hand - finding zero, using safety nets, and control using encoder as joystick for 1 finger.
	Notes: - Make sure to add some lube if gears are dry
		 - if the encoder jumpers fall off immediately pull the orange pwm wire from the board and restart
		 - the feedback encoder should have green on top, purple on bottom
		 - make sure to unplug battery when not using
		 - if anything seems to be going wrong pull the orange pwm wire so it does not break itself
		 - if using encoder joystick, clockwise opens (increase pwm) and counterclockwise closes (decreases)
*/

// 0 or 1. 0 for the ESP to control the fingers, 1 for the ESP to control the wrist
#define ESP 0

#include <ESP32Servo.h>		//https://github.com/madhephaestus/ESP32Servo
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder

#define FIND_ZERO 0

//todo: individially define for each motor.
#define PWM_NEUTRAL 91

// todo: rename. NUM_JOINTS is the number of joints read from the serial monitor
#if ESP == 0
#define NUM_JOINTS 5
#else
#define NUM_JOINTS 3
#endif

#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 20
// The serial to read signals from. Typically Serial2 for RPi, Serial for USB (helpful for debugging)
// You must also uncomment SigSerial.begin() if this is set to be not Serial
#define SigSerial Serial2

// Angle where finger is fully closed, as sum of angles of all joints. Min is 0. Max Pi can give is 265.
#define MAX_ANGLE 200

//todo: figure out why these aren't initialized to zero. 
int arr1[NUM_JOINTS] = {0};
int arr2[NUM_JOINTS] = {0};
int* target_angles = arr1;
int* setpoints = arr2; // Must hold semRpi to access

TaskHandle_t getRpiVals;
//http://exploreembedded.com/wiki/Mutex_Semaphore_02%3A_Recursive_Locks
SemaphoreHandle_t semRpi = NULL;

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

enum command_type
{
	fail,
	set_position,
	tune_pid,
	change_view
};
command_type readCommand();
command_type readPosition();
command_type readPidTunings();
command_type readViewType();

enum print_type_t
{
	print_index,
	print_middle,
	print_ring,
	print_little,
	print_thumb_flex,
	print_all
};
print_type_t print_type; 

struct JointPin {
	int encoderA;
	int encoderB;
	int esc;
};

struct Joint {
	double kp, ki, kd;
	double setpoint;
	int ticks; // # ticks from start
	double errSum; // Running total of errors (for integral)
	double lastErr; // = prevOutput - prevInput (for derivative)
	int lastTick; //the time when tick last changed
	int output; // PWM to write
	int lastOutput;
	ESP32Encoder encoder;
	Servo esc;
	boolean reverseBounds = false;
	
	bool checkBounds() {
	    //Safety Net: once zero is found, has bounds from positions 0 (fully extended) & 110 (fully closed)
	    //	*Note: Because the springs are helping to close the finger there is no noticable metric to identify
	    //	when resisting closure, however as long as the gear position stays as low as 110 the fishing
	    //	line does not tangle i.e. slack doesn't become an issue.
	    bool pushOutOfBounds;
	    if (!reverseBounds) {
			pushOutOfBounds = (this->ticks > 85 && this->output < PWM_NEUTRAL) 
	    		|| (this->ticks < 0 && this->output > PWM_NEUTRAL);
	    } else {
	    	pushOutOfBounds = (this->ticks > 85 && this->output > PWM_NEUTRAL) 
	    		|| (this->ticks < 0 && this->output < PWM_NEUTRAL);
	    }
	    
	    if (pushOutOfBounds)
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
		} else return currentLoopTime - this->lastTick >= 800;
	}
	
	double computePID() {
		/*Compute all the working error variables*/
		double error = this->setpoint - this->ticks;
		
		this->errSum += (error * deltaTime);
		double dErr = (error - this->lastErr) / deltaTime;

		/*Compute PID output*/
		output = PWM_NEUTRAL - this->kp * error + this->ki * this->errSum + this->kd * dErr;
		
		/*Remember some variables for next time*/
		this->lastErr = error;

		return output;
	}

	inline void tunePid(double Kp, double Ki, double Kd)
	{
		this->kp = Kp;
		this->ki = Ki;
		this->kd = Kd;
	}
};

#if ESP == 0
	//todo: rename? this is the number of fingers that get "wired up"
	#define NUM_JOINT_STRUCTS 5
	
	Joint index_finger{};
	Joint middle_finger{};
	Joint ring_finger{};
	Joint little_finger{};
	Joint thumb_flex{};
	
	Joint joints[NUM_JOINT_STRUCTS] = {index_finger, middle_finger, ring_finger, little_finger, thumb_flex};
	
	const JointPin pins[NUM_JOINT_STRUCTS] = {
		JointPin{35, 34, 12}, //index 
		JointPin{39, 36, 13}, //middle
		JointPin{33, 32, 14},  //ring
		JointPin{26, 25, 27},  //little
		JointPin{21, 19, 22}  //Thumb flex
	};
#else //ESP == 1
	#define NUM_JOINT_STRUCTS 3

	Joint wrist_left{};
	Joint wrist_right{};
	Joint thumb_ab_ad{};

	Joint joints[NUM_JOINT_STRUCTS] = {wrist_left, wrist_right, thumb_ab_ad};

	const JointPin pins[NUM_JOINT_STRUCTS] = {
		JointPin{39, 36, 13},
		JointPin{35, 34, 12},
		JointPin{32, 33, 14}
	};
#endif

void setup()
{
	printing = false;
	print_type = print_all; 
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

	// momentum adjustments for state 3 in findZero()
	// longAdjust = 0;
	// longmidAdjust = 7, shortmidAdjust = 5;
	// shortAdjust = 7; 

	state = 0;		//intialize state (tracks states in findZero function)

#if ESP == 0
	joints[0].tunePid(.23, -0.001, 0); //index
	joints[1].tunePid(.23, -0.001, 0); //middle
	joints[2].tunePid(.23, -0.001, 0); //ring
	joints[3].tunePid(.23, -0.001, 0); //little
	joints[4].tunePid(.23, 0, 0); //thumb flex
#else
	joints[0].tunePid(-.18, 0.00001, 0); //wrist left
	joints[0].reverseBounds = true;
	joints[1].tunePid(-.18, 0.00001, 0); //wrist right
	joints[1].reverseBounds = true;
	joints[2].tunePid(-.26, 0.00005, 0); //thumb ab ad
	joints[2].reverseBounds = true;
#endif

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
	delay(10000); // ESCs sing to us
	currentLoopTime = millis();
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
	    // Serial.println("Ticks:"+String(j.ticks));
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
	
	printStatus();
	delay(5);	 //TODO: Make delay a delayUntil and hardcode delta, and consider shortening 
}

// Only printing for first joint because I don't have time to care about the second.
void printStatus() {
	Serial.print("0 is neutral. ");

	if(print_type == print_all){
		for (int i = 0; i < NUM_JOINT_STRUCTS; i++)
		{
			Serial.print(" PWM" + String(i) + ":" + String(joints[i].output));
		}

		for (int i = 0; i < NUM_JOINT_STRUCTS; i++)
		{
			Serial.print(" Pos" + String(i) + ":" + String((int)joints[i].ticks));
		}
		// Serial.printf("\tRate: %8d", (unsigned long)rate);
		// Serial.print("\tstate: " + String((int32_t)state));
		// Serial.print("\tdchng: " + String((int32_t)directionChange));

		// Serial.print("\ttarget angle: " + String(target_angles[0]));

		for(int i=0; i<NUM_JOINT_STRUCTS; i++){
			Serial.print(" Setpoint" + String(i) + ":" + String((int)joints[i].setpoint));
		}

		Serial.println("");
		// Serial.print("kd*dErr:");
		// Serial.print((double)(kd * dErr));
		// Serial.println(" ");
		/*https://dreamonward.com/2020/07/25/arduino-serial-plotter-labels/ */
	} else {
		int finger_num = -1; 
		
		if(print_type == print_index){
			finger_num = 0; 
		} else if(print_type == print_middle){
			finger_num = 1;
		} else if(print_type == print_ring){
			finger_num = 2;
		} else if (print_type == print_little){
			finger_num = 3;
		} else if (print_type == print_thumb_flex) {
			finger_num = 4;
		} else if(finger_num == -1){
			Serial.print("Error: invalid print_type"); 
			return;
		}

		Serial.print(" PWM" + String(finger_num) + ":" + String(joints[finger_num].output));

		Serial.print(" Pos" + String(finger_num) + ":" + String((int)joints[finger_num].ticks));

		Serial.print(" Setpoint" + String(finger_num) + ":" + String((int)joints[finger_num].setpoint));

		Serial.println("");
	}
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
	// return min(angle, MAX_ANGLE) * maxPos / MAX_ANGLE;
	return angle;
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
		if (readCommand() == set_position) {
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



/**
	 Reads command from SigSerial.
	 Command must be of the form <#,#,...,#>, where the
	 number of numers is at least NUM_JOINTS.

	 Returns: true if read successful, false otherwise

	 Side effects: target_angles modified. This may occur even if false
	 is returned, so if the old angles are needed save them before
	 calling.
*/
command_type readCommand()
{
	if (SigSerial.available() <= 0){
		return fail;
	}
	String command = SigSerial.readStringUntil('\n');
	if(command.startsWith("PID:")){
		Serial.println("reading PID tunings...");
		return readPidTunings(command);
	} else if (command.startsWith("<") && command.endsWith(">")){
		// Serial.println("reading new positions...");
		return readPosition(command);
	} else if (command.startsWith("view:")){
		Serial.println("reading new view...");
		return readViewType(command);
	}
	
	return fail;
}

command_type readViewType(String command){
	String input = command.substring(5); //Skip 'view:'

	if(input.equals("a")){
		print_type = print_all; 
	} else if(input.equals("index")){
		print_type = print_index;
	} else if(input.equals("middle")){
		print_type = print_middle;
	} else if (input.equals("ring")){
		print_type = print_ring;
	} else if(input.equals("little")){
		print_type = print_little; 
	} else if(input.equals("thumb_flex") || input.equals("thumb flex")){
		print_type = print_thumb_flex; 
	} else {
		return fail; 
	}
	Serial.println("changing view...");

	return change_view;
}

command_type readPosition(String command){
	const char *input = command.c_str() + 1; //Skip opening '<'
	char *endPtr;
	int joint = 0;
	while (joint < NUM_JOINTS)
	{
		int angle = (int)strtol(input, &endPtr, 10);
		if (endPtr == input)
		{
			Serial.println("Failed to parse number: " + String(input));
			return fail;
		}
		target_angles[joint] = angle;
		input = endPtr + 1; //Skip the comma
		joint += 1;
	}
	return set_position;
}

/**
	 Reads command from SigSerial.
	 Command must be of the form PID:<joint num>, <kp>, <ki>, <kd> where each variable is a long. 
	 kp will be divided by 100, and ki will be divided by -1000. 
	 
	 Returns: true if read successful, false otherwise

	 Side effects: the PID for joint_num is modified to the given values. 
*/
command_type readPidTunings(String command)
{
	const char *input = command.c_str() + 4; //Skip opening "PID:"
	char *endPtr;
	
	int joint_num = (int)strtol(input, &endPtr, 10); //first argument is joint to tune. 
	if (endPtr == input)
	{
		Serial.println("Failed to parse joint num: " + String(input));
		return fail;
	}

	input = endPtr + 1; //Skip the comma

	double pid_vals[3]; // There are 3 pid vals, kp, ki, kd
	
	int pid_num = 0; 
	while (pid_num < 3)
	{
		int tuning_val = (int)strtol(input, &endPtr, 10);
		if (endPtr == input)
		{
			Serial.println("Failed to parse PID tunings: " + String(input));
			return fail;
		}
		pid_vals[pid_num] = tuning_val;
		input = endPtr + 1; //Skip the comma
		pid_num += 1;
	}
	Serial.println(
		"kp: " + String(pid_vals[0]/100) + 
		"\tki: " + String(pid_vals[1]/(-1000)) + 
		"\tkd: " + String(pid_vals[2]));

	#if ESP==0
		joints[joint_num].tunePid(pid_vals[0]/100, -pid_vals[1]/10000, pid_vals[2]); 
	#else
		joints[joint_num].tunePid(-pid_vals[0]/100, pid_vals[1]/10000, -pid_vals[2]); 	
	#endif
	
	return tune_pid;
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
