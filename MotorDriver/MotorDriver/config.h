#pragma once

#define PIN_LED		13
#define PIN_THROTTLE  A0
#define PIN_MOTOR     3
#define PIN_RELAY     4
#define PIN_TEMP_SENSOR		8
#define TEMP_RESOLUTION     9
#define SERIAL_SPEED	115200
#define PWM_FREQUENCY   20000

#define TEMP_VALID_MIN   -50
#define TEMP_VALID_MAX   150

#define PWM_THROTTLE_VOLTAGE_SAFE_START    1.2
#define PWM_THROTTLE_VOLTAGE_MIN   0.92
#define PWM_THROTTLE_VOLTAGE_MAX   4.5

#define PWM_COMMAND_MIN   0
#define PWM_COMMAND_MAX   200

#define THROTTLE_VOLTAGE_VALID_MIN   0.4
#define THROTTLE_VOLTAGE_VALID_MAX   4.8

#define PERIOD_TASK_TEMP_SENSOR    250000
#define TEMP_SENSOR_FAIL_THRESHOLD_TIME  4*PERIOD_TASK_TEMP_SENSOR/1000

#define PERIOD_TASK_FSM       10000

#define PERIOD_TASK_THROTTLE 20000

#define STOPPED_STATE_TIME_THRESHOLD   1000


uint8_t motor_pwm_command = 0;
float sensor_temp = 0;
unsigned long test_task_counter = 0;
unsigned long lastTime_TempSensor = 0;
unsigned long stopped_state_time = 0;

double throttleVoltage = 0;

enum tempSensorStatusType
{
	tempSensorStatus_OK = 1,
	tempSensorStatus_InitFail = 0,
	tempSensorStatus_ReadFail = 2,
}tempSensorStatus;


enum throttleStatusType
{
	throttleStatus_OK = 1,
	throttleStatus_Fail = 0,
}throttleStatus;



enum systemStateType
{
	SYSTEM_STATE_IDLE = 0,
	SYSTEM_STATE_RUNNING = 1,
	SYSTEM_STATE_STOPPED = 2,
}systemState;
