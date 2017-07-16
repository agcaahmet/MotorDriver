/*
 Name:		MotorDriver.ino
 Created:	7/16/2017 4:04:43 PM
 Author:	ahmet
*/

#include <Agenda.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PWM.h>

#include "config.h"


Agenda scheduler;
OneWire oneWire(PIN_TEMP_SENSOR);
DallasTemperature sensors(&oneWire);


DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(SERIAL_SPEED);

	//Start System in IDLE State
	systemState = SYSTEM_STATE_IDLE;

	initPWM();
	initTempSensor();

	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_MOTOR, OUTPUT);
	pinMode(PIN_RELAY, OUTPUT);
	digitalWrite(PIN_MOTOR, LOW);
	digitalWrite(PIN_RELAY, LOW);



	scheduler.insert(taskLed, 500000);
	scheduler.insert(taskTempSensor, PERIOD_TASK_TEMP_SENSOR);

	scheduler.insert(taskThrottle, PERIOD_TASK_THROTTLE);
	scheduler.insert(taskFSM, PERIOD_TASK_FSM);
	scheduler.insert(task1Hz, 1000000);

}

// the loop function runs over and over again until power down or reset
void loop() {

	scheduler.update(); 
}
void initTempSensor()
{
	//Set status to Fail first,
	tempSensorStatus = tempSensorStatus_InitFail;
	Serial.println("Temp sensor DS18B20 initialization started....");

	// Start up the library
	sensors.begin();

	// Grab a count of devices on the wire
	int numberOfDevices = sensors.getDeviceCount();

	// locate devices on the bus
	Serial.print("Locating devices...");

	Serial.print("Found ");
	Serial.print(numberOfDevices, DEC);
	Serial.println(" devices.");



	// report parasite power requirements
	Serial.print("Parasite power is: ");
	if (sensors.isParasitePowerMode()) Serial.println("ON");
	else Serial.println("OFF");

	// Loop through each device, print out address
	for (int i = 0; i<numberOfDevices; i++)
	{
		// Search the wire for address
		if (sensors.getAddress(tempDeviceAddress, i))
		{
			Serial.print("Found device ");
			Serial.print(i, DEC);
			Serial.print(" with address: ");
			for (uint8_t i = 0; i < 8; i++)
			{
				if (tempDeviceAddress[i] < 16) Serial.print("0");
				Serial.print(tempDeviceAddress[i], HEX);
			}

			Serial.println();

			Serial.print("Setting resolution to ");
			Serial.println(TEMP_RESOLUTION, DEC);

			// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
			sensors.setResolution(tempDeviceAddress, TEMP_RESOLUTION);

			Serial.print("Resolution actually set to: ");
			Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
			Serial.println();



			//Serial.println("Setting Read Type to ASYNC..");
			// set async read operation, bool waitForConversion to false
			sensors.setWaitForConversion(false);

			Serial.print("Read Type actually set to: ");
			bool tempBool = sensors.getWaitForConversion();
			if (sensors.getWaitForConversion()) Serial.print("SYNC Read");
			else Serial.print("ASYNC Read");
			Serial.println();

			tempSensorStatus = tempSensorStatus_OK;

		}
		else {
			Serial.print("Found ghost device at ");
			Serial.print(i, DEC);
			Serial.print(" but could not detect address. Check power and cabling");


			tempSensorStatus = tempSensorStatus_InitFail;
		}
	}

	Serial.println("Temp sensor DS18B20 initialization end.");


}
void initPWM()
{
	//initialize all timers except for 0, to save time keeping functions
	InitTimersSafe();

	//sets the frequency for the specified pin
	bool success = SetPinFrequencySafe(PIN_MOTOR, PWM_FREQUENCY);

	if (success)
	{
		Serial.print("PWM frequency set to ");
		Serial.println(PWM_FREQUENCY);
	}
		
	else
	{
		Serial.print("PWM Frequency could not be set to ");
		Serial.println(PWM_FREQUENCY);
	}
}
void taskLed()
{
	test_task_counter++;

	if (test_task_counter % 2 == 0)
	{
		digitalWrite(PIN_LED, HIGH);
	}
	else
	{
		digitalWrite(PIN_LED, LOW);
	}


	//Serial.print("State:");
	//Serial.println(systemState);
}
void taskTempSensor()
{
	if (tempSensorStatus != tempSensorStatus_InitFail)
	{

		// call sensors.requestTemperatures() to issue a global temperature 
		// request to all devices on the bus
		//Serial.print("Requesting temperatures...");
		sensors.requestTemperatures(); // Send the command to get temperatures
									   //Serial.println("DONE");

									   // Search the wire for address
		if (sensors.getAddress(tempDeviceAddress, 0))
		{


			// It responds almost immediately. Let's print out the data
			// method 2 - faster
			sensor_temp = sensors.getTempC(tempDeviceAddress);
			//Serial.print("Temp:");
			//Serial.print(sensor_temp);

			if (sensor_temp < TEMP_VALID_MIN || sensor_temp > TEMP_VALID_MAX)
			{
				//Serial.println("Sensor Read Temp Fail");
				tempSensorStatus = tempSensorStatus_ReadFail;
			}

			if (!temp_limit_exceeded)
			{
				if (sensor_temp > (OPERATION_TEMP_LIMIT + OPERATION_TEMP_LIMIT_HYSTERESIS))
					temp_limit_exceeded = true;
			}
			else
			{
				if (sensor_temp < (OPERATION_TEMP_LIMIT - OPERATION_TEMP_LIMIT_HYSTERESIS))
					temp_limit_exceeded = false;
			}




			lastTime_TempSensor = millis();
		}
		//else ghost device! Check your power requirements and cabling

		if (millis() - lastTime_TempSensor > TEMP_SENSOR_FAIL_THRESHOLD_TIME)
		{
			tempSensorStatus = tempSensorStatus_ReadFail;
		}
	}
}
void taskThrottle()
{
	unsigned long start_time = millis();
	throttleVoltage = analogRead(PIN_THROTTLE) * 5.0 / 1024.0;

	if (throttleVoltage < THROTTLE_VOLTAGE_VALID_MIN || throttleVoltage > THROTTLE_VOLTAGE_VALID_MAX)
	{
		Serial.println("Throttle Voltage Out of Limits Fail");
		throttleStatus = throttleStatus_Fail;
	}
	else
	{
		throttleStatus = throttleStatus_OK;
	}

	motor_pwm_command= mapping(throttleVoltage, PWM_THROTTLE_VOLTAGE_MIN, PWM_THROTTLE_VOLTAGE_MAX, PWM_COMMAND_MIN, PWM_COMMAND_MAX);
	


	//Serial.print("Throttle:");
	//Serial.print(throttleVoltage);
	//Serial.print("    PWM:");
	//Serial.println(motor_pwm_command);
	
	//Serial.println(millis() - start_time);

}
void taskFSM()
{
	if (systemState == SYSTEM_STATE_IDLE)
	{

		digitalWrite(PIN_RELAY, LOW);
		pwmWrite(PIN_MOTOR, 0);

		if (tempSensorStatus == tempSensorStatus_OK &&
			throttleStatus == throttleStatus_OK &&
			!temp_limit_exceeded &&
			throttleVoltage > PWM_THROTTLE_VOLTAGE_MIN &&
			throttleVoltage < PWM_THROTTLE_VOLTAGE_SAFE_START)
		{
			systemState = SYSTEM_STATE_RUNNING;
		}
	}
	else if (systemState == SYSTEM_STATE_RUNNING)
	{
		digitalWrite(PIN_RELAY, HIGH);
		pwmWrite(PIN_MOTOR, motor_pwm_command);

		if (tempSensorStatus != tempSensorStatus_OK ||
			throttleStatus != throttleStatus_OK ||
			temp_limit_exceeded)
		{
			systemState = SYSTEM_STATE_IDLE;
		} 
		else if (throttleVoltage < PWM_THROTTLE_VOLTAGE_MIN)
		{
			systemState = SYSTEM_STATE_STOPPED;
			stopped_state_time = millis();
		}
	}

	else if (systemState == SYSTEM_STATE_STOPPED)
	{
		digitalWrite(PIN_RELAY, HIGH);
		pwmWrite(PIN_MOTOR, 0);

		if (tempSensorStatus != tempSensorStatus_OK ||
			throttleStatus != throttleStatus_OK  ||
			temp_limit_exceeded ||
			(millis()- stopped_state_time) > STOPPED_STATE_TIME_THRESHOLD)
		{
			systemState = SYSTEM_STATE_IDLE;
		}
		else if (throttleVoltage > PWM_THROTTLE_VOLTAGE_MIN)
		{
			systemState = SYSTEM_STATE_RUNNING;
		}

	}


}
float mapping(float input, float inputMin, float inputMax, float outputMin, float outputMax)
{
	float result = 0;

	if (input <= inputMin)
	{
		result = outputMin;
	}
	else if (input >= inputMax)
	{
		result = outputMax;
	}
	else
	{
		result = outputMin + (input - inputMin) / (inputMax - inputMin) * (outputMax - outputMin);
	}
	return result;
}
void task1Hz()
{
	if (tempSensorStatus == tempSensorStatus_InitFail)
		initTempSensor();
}