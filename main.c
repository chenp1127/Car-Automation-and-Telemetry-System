/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "light.h"
#include "temp.h"

#include "stdio.h"
#include "string.h"
#include "math.h"

//------------ Values as required by the assignment 2------------//
#define MODE_CHANGE_TIME 1000				// 1 second to decide if going to reverse
#define TEMP_HIGH_THRESHOLD 28.0			// As per lab manual requirement
#define ACC_THRESHOLD 25.6  // = 0.4g		// (1g = 64. Therefore 0.4g = 25.6)
#define OBSTACLE_NEAR_THRESHOLD 1500   		// Valid when LIGHT_RANGE_4000
#define MAX_LUX 3891.0						// Max. Value when LIGHT_RANGE_4000 is 3891 (As given in the light sensor datasheet and as tested)


//------------ Values required for temperature sensor optimisation-----------//
#define TEMP_SCALAR 10.0					// J26 as default (TS0 and TS1 - both jumpers are inserted)
#define SAMPLE_TRIGGER 667					// Too little sample = lesser accuracy. Too much samples = less sensitive to changes

#define FAST_MAINTENANCE_REFRESH 100		// Time between sensor update values in maintenance mode
#define ANIMATION_FRAME_DURATION 100


//------------ Basic Functions Feature Prototype------------//
void mode_toggling_decision_polling(void);
void mode_toggling_decision_interrupt(void);

void initial_stationary_task (void);
void stationary_task (void);
void oled_clear_stationary(void);

void reverse_task (void);
void pca_control_intensity(void);
void obstacle_detection_action(void);

void forward_task (void);
void temp_acc_warnings(void);
void segment_display(uint32_t segment_time);
void uart_transmit_at_f (void);

void temperature_update_function(void);
void acc_reading_function(void);
void oled_sensor_display(void);

void blink_blue (uint32_t blue_time_period);
void blink_red (uint32_t red_time_period);


//------------ Enhanced Feature Function Prototype --------------//
void advanced_password_prompt(void);
void advanced_check_password(void);
void advanced_wrong_password_action(void);
void advanced_correct_password_action(void);

void advanced_remote_maintenance(void);  						// Maintenance_telemetry ('t') or Maintenance_Reboot ('r')
void advanced_remote_maintenance_telemetry(void);

void advanced_remote_maintenance_reboot(void); 					// go to mode enhanced -> system reboot animation
void oled_refresh_control(void);
void reboot_animation_remote(void); 									// Go back initial_stationary
void reboot_animation_abort(void); 									// Go back initial_stationary


//----------- Enhanced Feature: Remote Diagnostic System Variables Declaration -------------//
//volatile uint8_t string_uart_admin[100] = {};   		// For Maintenance in Telemetry Mode
volatile uint8_t uart_data = 0;    						// Uart3 interrupt triggered, when user enter 'p'
volatile uint8_t warning_count = 0;
volatile uint8_t warning_not_horizontal = 0;			// 1 means not on a level surface
volatile uint32_t remote_telemetry_count = 0;


//----------- Enhanced Feature: Hacker Prevention System Variable Declaration -------------//
volatile uint8_t hacker_flag = 0;


//------SYSTIC Timing Variable-----//
volatile uint32_t ms_ticks = 0;
volatile uint8_t reset_initial_msticks = 1;
volatile uint32_t initial_msticks = 0;
volatile uint32_t current_msticks = 0;

// ------ MODE Toggling Timing Variable ------ //
volatile uint8_t reset_reference_msticks_sw3 = 0;
volatile uint32_t reference_msticks_sw3 = 0;
volatile uint32_t current_msticks_sw3 = 0;
volatile uint32_t counter_sw3 = 0;

// ------ RGB-Red Timing Variable ------ //
volatile uint8_t reset_reference_msticks_red = 1;
volatile uint32_t reference_msticks_red = 0;
volatile uint32_t current_msticks_red = 0;

// ------ Blue-Red Timing Variable ------ //
volatile uint8_t reset_reference_msticks_blue = 1;
volatile uint32_t reference_msticks_blue = 0;
volatile uint32_t current_msticks_blue = 0;

// ------ 7 Segment Timing Variable ------ //
volatile uint8_t reset_reference_msticks_segment = 1;
volatile uint32_t reference_msticks_segment = 0;
volatile uint32_t current_msticks_segment = 0;

// ------ Temperature Sensor Variables ----//
volatile float temperature_reading = 25.0;	// Standard room temperature
volatile uint8_t reset_reference_msticks_temp = 1;
volatile uint32_t reference_msticks_temp = 0;
volatile uint32_t current_msticks_temp = 0;

// ------ Maintenance Timing Variable ------ //
volatile uint8_t reset_reference_msticks_fast_update = 1;
volatile uint32_t reference_msticks_fast_update = 0;
volatile uint32_t current_msticks_fast_update = 0;

//---- OLED Timing Variable ------ //
volatile uint8_t reset_reference_msticks_oled = 1;
volatile uint32_t reference_msticks_oled = 0;
volatile uint32_t current_msticks_oled = 0;

//------- String variables ------//
volatile uint8_t string_oled_acc[30] = {};
volatile uint8_t string_oled_temp[30] = {};
volatile uint8_t string_oled_light[30] = {};
volatile uint8_t string_oled_period[30] = {};

volatile uint8_t string_oled_temp_threshold[30] = {};
volatile uint8_t string_oled_acc_threshold[30] = {};


//------- UART variables ------//
volatile uint8_t string_uart_sensor_values[100] = {};
volatile uint8_t string_uart_stationary[100] = {};
volatile uint8_t string_uart_forward[100] = {};
volatile uint8_t string_uart_reverse[100] = {};

volatile uint8_t string_uart_temp_threshold[100] = {};
volatile uint8_t string_uart_acc_threshold[100] = {};
volatile uint8_t string_uart_light_threshold[100] = {};
volatile uint8_t string_uart_admin[300] = {};

volatile uint32_t transmission_counter = 0;


//------- Maintenance related ------//
volatile uint32_t uart_password_count = 0;
volatile uint32_t oled_frame_count = 0;
volatile uint8_t reboot_type = 0;	// 0 means reboot by the remote technician, 1 means reboot by the user to release hacker control


//------RGB Flag Declaration------//
volatile uint8_t red_flag = 0;
volatile uint8_t blue_flag = 0;

volatile uint8_t red_threshold_flag = 0;
volatile uint8_t blue_threshold_flag = 0;
volatile uint8_t obstacle_threshold_flag = 0; // 0 means obstacles is not near - no warning

//---- 7 segment variables ---------//
volatile uint8_t segment_array[16]= {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', '8', 'C', '0', 'E', 'F'}; // All characters appear in capital letters
volatile uint8_t segment_counter = 0;

//----- Acceleration Declaration ------//
volatile int8_t acc_x = 0;
volatile int8_t acc_y = 0;
volatile int8_t acc_z = 0;
// For value offset of accelerometer
volatile int8_t acc_xoff = 0;
volatile int8_t acc_yoff = 0;
volatile int8_t acc_zoff = 0;

//------ Temperature Reading interrupt -------//
volatile uint32_t t1_reading = 0;
volatile uint32_t t2_reading = 0;
volatile float temp_period = 0.0;
volatile uint32_t number_of_samples = 0;

volatile uint8_t reset_reference_t = 1;
volatile uint32_t reference_msticks_t = 0;
volatile uint32_t current_msticks_t = 0;

//------ Mode initialization once Flag -------//
volatile uint8_t overall_stationary_flag = 0;
volatile uint8_t overall_forward_flag = 0;
volatile uint8_t overall_reverse_flag = 0;

//------ Light Sensor Variables ------//
volatile uint8_t reset_reference_msticks_light = 1;
volatile uint32_t reference_msticks_light = 0;
volatile uint32_t current_msticks_light = 0;
volatile uint32_t light_lux=0;

typedef enum
{
	INITIAL_STATIONARY,
	STATIONARY,
	FORWARD,
	REVERSED,
	ENHANCED

}modes_name;

modes_name modes = INITIAL_STATIONARY;

void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
		PINSEL_ConfigPin(&PinCfg);

	PinCfg.Pinnum = 8;
		PINSEL_ConfigPin(&PinCfg);

	PinCfg.Pinnum = 9;
		PINSEL_ConfigPin(&PinCfg);

	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
		PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
		PinCfg.Funcnum = 2;
		PinCfg.Pinnum = 10;     //SDA2
		PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	  	PinCfg.Pinnum = 11;     //SCL2
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

// Function to setup the UART3
void ready_uart(void)
{
	// PINSEL Configuration        P0.0 AND P0.1 UART3
	PINSEL_CFG_Type PinCfg;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Funcnum = 2;
		PinCfg.Pinnum = 0;
		PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
		PinCfg.Pinnum = 1;
		PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	// Initialise and enable the UART. Not enabling the UART will lead to a hard fault
	UART_CFG_Type UartCfg;
	    UartCfg.Baud_rate = 115200;
	    UartCfg.Databits = UART_DATABIT_8;
	    UartCfg.Parity = UART_PARITY_NONE;
	    UartCfg.Stopbits = UART_STOPBIT_1;

	// supply power & setup working parameters for UART3
	UART_Init(LPC_UART3, &UartCfg);

	// enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
}


void init_GPIO(void)
{

	PINSEL_CFG_Type PinCfg;

	// Initialize SW3 as EINT0 (P2.10 AS Function 1. P2.10 is not GPIO)
		PinCfg.Funcnum = 1;		// EINT0, not GPIO
		PinCfg.Portnum = 2;
		PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize RGB-RED P2.0
		PinCfg.Funcnum = 0;		// GPIO
		PinCfg.Portnum = 2;
		PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize RGB-BLUE P0.26
		PinCfg.Portnum = 0;		// GPIO
		PinCfg.Pinnum = 26;
		PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize Temperature Sensor P0.2
		PinCfg.Portnum = 0;		// GPIO
		PinCfg.Pinnum = 2;
		PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	//Initialize Joystick centre P0.17
		PinCfg.Portnum = 0;		// GPIO
		PinCfg.Pinnum = 17;
		PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	//Initialize Rotary one direction P0.25
		PinCfg.Portnum = 0;		// GPIO
		PinCfg.Pinnum = 25;
		PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);


	//GPIO Interfaces Input//
    GPIO_SetDir(0, 1<<10, 0);       //SW3 - Not used as GPIO, so not necessary
    GPIO_SetDir(0, 1<<2, 0);        //Temperature Sensor
    GPIO_SetDir(0, 1<<17, 0);       //Joystick up
    GPIO_SetDir(0, 1<<25, 0);       //Rotary one direction


    //GPIO Interfaces Output//
	GPIO_SetDir(2, 1<<0,  1);        //RGB-Red as output
	GPIO_SetDir(0, 1<<26,  1);		// RGB-Blue as output
}


void SysTick_Handler(void)
{
	ms_ticks++;
}

static uint32_t Get_Ticks(void)
{
	return ms_ticks;
}

void initial_stationary_task (void)
{
	led7seg_setChar(' ', FALSE);
	GPIO_ClearValue(2,1);   //Red - RGB OFF
	red_flag = 0;

	GPIO_ClearValue(0,1<<26); // Blue - RGB OFF
	blue_flag = 0;
	pca9532_setLeds(0x0000,0xFFFF);
	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(0,0,"Stationary",OLED_COLOR_WHITE,OLED_COLOR_BLACK);


	reset_reference_msticks_segment = 0;    // To ensure the segment shows 0 immediately when going to STATIONARY mode
	reference_msticks_segment = 0;			// To ensure the segment shows 0 immediately when going to STATIONARY mode

	reboot_type = 0;						// Default non-hacker reboot animation
	modes = STATIONARY;
    return;
}

void stationary_task (void)
{
	if (overall_stationary_flag == 1)
	{
		sprintf(string_uart_stationary,"Entering Stationary Mode.\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_stationary,strlen(string_uart_stationary),BLOCKING);

		oled_putString(0,0,"Stationary        ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);

		GPIO_ClearValue(2,1);   //Red - RGB OFF
		red_flag = 0;

		GPIO_ClearValue(0,1<<26); // Blue - RGB OFF
		blue_flag = 0;

		pca9532_setLeds(0x0000,0xFFFF);
		led7seg_setChar(' ', FALSE);

		blue_threshold_flag = 0; 		// Blink_Blue Stops
		red_threshold_flag = 0;  		// Blink_Red Stops
		obstacle_threshold_flag = 0;    // For Reverse Mode - 0 indicates no obstacle detection

		oled_clear_stationary();
		overall_stationary_flag = 0;
	}

	return;
}

void forward_task (void)
{

	if(overall_forward_flag == 1)
	{
		segment_counter = 0;
		sprintf(string_uart_forward,"Entering Forward Mode.\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_forward,strlen(string_uart_forward),BLOCKING);
		oled_putString(0,0,"Forward        ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);

		acc_reading_function();				// Reading accelerometer
		temperature_update_function();	   	// Applying temperature formula
		oled_sensor_display();  		   	// Display accelerometer and temperature reading

		pca9532_setLeds(0x0000,0xFFFF);

		overall_forward_flag = 0;
	}

	segment_display(1000);					// 1 second update on 7 segment each time

	temp_acc_warnings();  // Display warning on OLED

	if (red_threshold_flag == 1 && blue_threshold_flag == 0) blink_red(333);

	if (blue_threshold_flag == 1 && red_threshold_flag == 0 ) blink_blue(333);

	if (red_threshold_flag == 1 && blue_threshold_flag == 1) // Setting appropriate flag for synchronization of red and blue blinking
	{
		reset_reference_msticks_red = 1; red_flag = 0; red_threshold_flag = 2;
		reset_reference_msticks_blue = 1; blue_flag = 0; blue_threshold_flag = 2;
	}
	if (red_threshold_flag >= 2 || blue_threshold_flag >= 2)  // Actual blinking when 2 or above. Instructions meant for blinking of both colours only
	{
		blink_red(333);
		blink_blue(333);
	}

	return;
}

void obstacle_detection_action(void)
{

	if(light_lux < OBSTACLE_NEAR_THRESHOLD)
	{
		obstacle_threshold_flag = 1;
	}
	else
	{
		obstacle_threshold_flag = 0;	// No obstacle near, therefore stop warning signals. Also can become 0 when going to stationary
	}

	if (obstacle_threshold_flag == 1)
	{
		oled_putString(0,30,"Obstacle Near.",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		sprintf(string_uart_light_threshold,"Obstacle Near.\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_light_threshold,strlen(string_uart_light_threshold),BLOCKING);
	}
	else
	{
		oled_putString(0,30,"                ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);	// Clear obstacle warning on oled
	}


	return;
}

void pca_control_intensity(void)
{
	if (light_lux > (14.0/16.0*MAX_LUX) && light_lux <= (15.0/16.0*MAX_LUX))
		pca9532_setLeds (0xFEFF, 0xFFFF);
	else if (light_lux > (13.0/16.0*MAX_LUX) && light_lux <= (14.0/16.0*MAX_LUX))
		pca9532_setLeds (0xFCFF, 0xFFFF);
	else if (light_lux > (12.0/16.0*MAX_LUX) && light_lux <= (13.0/16.0*MAX_LUX))
		pca9532_setLeds (0xF8FF, 0xFFFF);
	else if (light_lux > (11.0/16.0*MAX_LUX) && light_lux <= (12.0/16.0*MAX_LUX))
		pca9532_setLeds (0xF0FF, 0xFFFF);
	else if (light_lux > (10.0/16.0*MAX_LUX) && light_lux <= (11.0/16.0*MAX_LUX))
		pca9532_setLeds (0xE0FF, 0xFFFF);
	else if (light_lux > (9.0/16.0*MAX_LUX) && light_lux <= (10.0/16.0*MAX_LUX))
		pca9532_setLeds (0xC0FF, 0xFFFF);
	else if (light_lux > (8.0/16.0*MAX_LUX) && light_lux <= (9.0/16.0*MAX_LUX))
		pca9532_setLeds (0x80FF, 0xFFFF);
	else if (light_lux > (7.0/16.0*MAX_LUX) && light_lux <= (8.0/16.0*MAX_LUX))
		pca9532_setLeds (0x00FF, 0xFFFF);
	else if (light_lux > (6.0/16.0*MAX_LUX) && light_lux <= (7.0/16.0*MAX_LUX))
		pca9532_setLeds (0x00FE, 0xFFFF);
	else if (light_lux > (5.0/16.0*MAX_LUX) && light_lux <= (6.0/16.0*MAX_LUX))
		pca9532_setLeds (0x00FC, 0xFFFF);
	else if (light_lux > (4.0/16.0*MAX_LUX) && light_lux <= (5.0/16.0*MAX_LUX))
		pca9532_setLeds (0x00F8, 0xFFFF);
	else if (light_lux > (3.0/16.0*MAX_LUX) && light_lux <= (4.0/16.0*MAX_LUX))
		pca9532_setLeds (0x00F0, 0xFFFF);
	else if (light_lux > (2.0/16.0*MAX_LUX) && light_lux <= (3.0/16.0*MAX_LUX))
		pca9532_setLeds (0x00E0, 0xFFFF);
	else if (light_lux > (1.0/16.0*MAX_LUX) && light_lux <= (2.0/16.0*MAX_LUX))
		pca9532_setLeds (0x00C0, 0xFFFF);
	else if (light_lux > (0.0/16.0*MAX_LUX) && light_lux <= (1.0/16.0*MAX_LUX))
	    pca9532_setLeds (0x0080, 0xFFFF);
	else
		pca9532_setLeds (0xFFFF, 0xFFFF);

	return;
}

void reverse_task (void)
{
	if(overall_reverse_flag == 1)
	{
		sprintf(string_uart_reverse,"Entering Reverse Mode.\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_reverse,strlen(string_uart_reverse),BLOCKING);
		oled_putString(0,0,"Reverse        ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		overall_reverse_flag = 0;
	}

	current_msticks_light = Get_Ticks();

	if(reset_reference_msticks_light == 1)
	{
		reset_reference_msticks_light = 0;
		reference_msticks_light = current_msticks_light;
	}

	else
	{	// Light sensor is required to update every 1 sec meaning also that obstacle detection happens every 1 second
		if(current_msticks_light - reference_msticks_light >= 1000)
		{
			light_lux = light_read();
			obstacle_detection_action();
			pca_control_intensity();

			reset_reference_msticks_light = 1;
		}
	}
	return;
}

void mode_toggling_decision_interrupt(void)
{
	if (uart_password_count == 0) 				// Not attempting remote maintenance mode
	{
		if (modes == STATIONARY)
		{
			counter_sw3++;
			if (counter_sw3 == 1){reset_reference_msticks_sw3 = 1;}
		}

		else if((modes == FORWARD)||(modes == REVERSED))
		{
			modes = STATIONARY;
		}
	}

	return;
}

void mode_toggling_decision_polling(void)
{
	if(modes == STATIONARY)
	{
		if (counter_sw3 > 0)
		{
			current_msticks_sw3 = Get_Ticks();
			if (reset_reference_msticks_sw3)
			{
				reset_reference_msticks_sw3 = 0;
				reference_msticks_sw3 = current_msticks_sw3;
			}
			else
			{
				if (current_msticks_sw3 - reference_msticks_sw3 >= MODE_CHANGE_TIME)	// 1 second by default (MODE_CHANGE_TIME = 1000 ms)
				{
					if(counter_sw3 == 1) {modes = FORWARD;}
					else if (counter_sw3 == 2) {modes = REVERSED;}
					//else if (counter_sw3 >2){ }	// Lab manual did not state what happens if SW3 is pressed more than 2 times within 1 second
					counter_sw3 = 0; // reset reference time
				}
			}
		}
	}
	return;
}

void advanced_password_prompt(void)
{
	if (modes != STATIONARY)
	{
		sprintf(string_uart_admin, "Remote maintenance is not possible while vehicle is moving.\r\nPlease wait for vehicle to become stationary before trying again\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
	}

	if (modes == STATIONARY)
	{
		sprintf(string_uart_admin, "Please enter administrator password for remote maintenance (Hidden characters): ");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		uart_password_count = 1;	// System is ready to check for password entry
	}
	return;
}

void advanced_check_password(void)
{
	// Checking the user password entry versus the pre-set password: "dearestee2028" and pressing enter at the end
	if(uart_password_count == 1 && uart_data == 'd') uart_password_count++;
	else if(uart_password_count == 2 && uart_data == 'e') uart_password_count++;
	else if(uart_password_count == 3 && uart_data == 'a') uart_password_count++;
	else if(uart_password_count == 4 && uart_data == 'r') uart_password_count++;
	else if(uart_password_count == 5 && uart_data == 'e') uart_password_count++;
	else if(uart_password_count == 6 && uart_data == 's') uart_password_count++;
	else if(uart_password_count == 7 && uart_data == 't') uart_password_count++;
	else if(uart_password_count == 8 && uart_data == 'e') uart_password_count++;
	else if(uart_password_count == 9 && uart_data == 'e') uart_password_count++;
	else if(uart_password_count == 10 && uart_data == '2') uart_password_count++;
	else if(uart_password_count == 11 && uart_data == '0') uart_password_count++;
	else if(uart_password_count == 12 && uart_data == '2') uart_password_count++;
	else if(uart_password_count == 13 && uart_data == '8') uart_password_count++;
	else if(uart_password_count == 14 && uart_data == '\r') uart_password_count++;
	else if(uart_password_count > 0 && uart_password_count < 15) uart_password_count = 999;		// User has attempted to enter a password by pressing 'p' but the password is wrong
																								// uart_password_count >= 15 (and not 999) means password is correct
	return;
}



void advanced_wrong_password_action(void)
{
	if (uart_password_count == 999 && uart_data == '\r')	// Incorrect password
	{
		sprintf(string_uart_admin, "\r\nWrong password. Press 'p' to enter password again for remote maintenance.\r\r\n\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		uart_password_count = 0;	// System remains in stationary mode and waits for further pressing of the 'p' character
	}
	return;
}


void advanced_correct_password_action(void)
{
	if (uart_password_count == 15)	// Correct password - Password is (13 characters + 1 carriage return + 1)
	{
		sprintf(string_uart_admin, "\r\nWelcome to remote maintenance.\r\nPress 't' to get telemetry data.\r\nPress 'r' to reboot the vehicle system.\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		sprintf(string_uart_sensor_values, "-------------------------------------------------------------------------------------------------------------\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_sensor_values,strlen(string_uart_sensor_values),BLOCKING);
		uart_password_count++;		// Makes system go to remote maintenance mode (uart_password_count = 16)
		oled_putString(0,0, "Maintenance         ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	}
	return;
}

void maintenance_fast_update(void)
{
	current_msticks_fast_update = Get_Ticks();
	if(reset_reference_msticks_fast_update)
	{
		reset_reference_msticks_fast_update = 0;
		reference_msticks_fast_update = current_msticks_fast_update;
	}

	else
	{
		if((current_msticks_fast_update - reference_msticks_fast_update) >= FAST_MAINTENANCE_REFRESH)	// Do an update of sensor values every FAST_MAINTENANCE_REFRESH milliseconds
		{

			// Reading the 3 sensors
			acc_reading_function();
			temperature_update_function();
			light_lux = light_read();

			// Sensor value display
			sprintf(string_oled_light,"LIGHT:%5u              ",light_lux);
			oled_putString(0, 10, (uint8_t*)string_oled_light,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			sprintf(string_oled_temp,"TEMP:  %5.2f             ",temperature_reading);
			oled_putString(0, 20, (uint8_t*)string_oled_temp,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			sprintf(string_oled_acc,"ACC X:%5.2f g              ",acc_x/64.0);
			oled_putString(0, 30, (uint8_t*)string_oled_acc,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			sprintf(string_oled_acc,"ACC Y:%5.2f g              ",acc_y/64.0);
			oled_putString(0, 40, (uint8_t*)string_oled_acc,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			sprintf(string_oled_acc,"ACC Z:%5.2f g              ",acc_z/64.0);
			oled_putString(0, 50, (uint8_t*)string_oled_acc,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

			// Sensor warning display
			if ((abs(acc_z) > (64+ACC_THRESHOLD)) || (abs(acc_z) < (64-ACC_THRESHOLD))) oled_putString(86, 50, "!",OLED_COLOR_WHITE,OLED_COLOR_BLACK);		// ! will not blink because it occurs immediately after the previous clearing
			else oled_putString(86, 50, " ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);																				// Due to the vehicle always experience an acceleration due to gravity, there is no need to blink
			if (light_lux < OBSTACLE_NEAR_THRESHOLD) oled_putString(86, 10, "!",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			else oled_putString(86, 10, " ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			if (temperature_reading > TEMP_HIGH_THRESHOLD) oled_putString(86, 20, "!",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			else oled_putString(86, 20, " ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			if (abs(acc_x) > ACC_THRESHOLD) oled_putString(86, 30, "!",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			else oled_putString(86, 30, " ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			if (abs(acc_y) > ACC_THRESHOLD) oled_putString(86, 40, "!",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			else oled_putString(86, 40, " ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);


			reset_reference_msticks_fast_update = 1;
		}
	}
	return;
}

void advanced_remote_maintenance_telemetry(void)
{
	if (uart_data == 't')		// telemetry data
	{
		acc_reading_function();
		temperature_update_function();
		light_lux = light_read();

		remote_telemetry_count++;	// To keep track of the number of remote requests for telemetry data since hard reset of system

		sprintf(string_uart_admin, "\r\n-------------------------------------------------------------------------------------------------------------\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);

		sprintf(string_uart_admin, "\r\n\e[34mGetting Realtime Data from Vehicle (Request Number: %05u):\e[37;0m\r\n\r\n",remote_telemetry_count);
		UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);

		//-------------System Check Light Sensor : Collision Detection-----------//
		if (light_lux < OBSTACLE_NEAR_THRESHOLD)
		{
			sprintf(string_uart_admin, "     LIGHT SENSOR:\e[31m                                               %u Lux\e[37;0m\r\r\n",light_lux);       // 31 indicates Warning Detection : Red
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
			warning_count++;
		}
		else
		{
			sprintf(string_uart_admin, "     LIGHT SENSOR:\e[32m                                               %u Lux\e[37;0m\r\r\n",light_lux);       // 32 indicates Status ok : Green
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		}

		//---------- System Check Temperature Threshold ---------//
		if(temperature_reading >TEMP_HIGH_THRESHOLD)
		{
			sprintf(string_uart_admin, "     TEMPERATURE SENSOR:\e[31m                                         %.2f Degrees Celcius\e[37;0m\r\r\n",temperature_reading);
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
			warning_count++;
		}
		else
		{
			sprintf(string_uart_admin, "     TEMPERATURE SENSOR:\e[32m                                         %.2f Degrees Celcius\e[37;0m\r\r\n",temperature_reading);
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		}

		//------------- System Acceleration Check --------------//
		if(abs(acc_x) > ACC_THRESHOLD)
		{
			sprintf(string_uart_admin, "     ACCELEROMETER SENSOR: X-AXIS in g (Raw values in bracket): \e[31m%5.2f g (%d)\e[37;0m\r\r\n",acc_x/64.0,acc_x);
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
			warning_count++;
		}
		else
		{
			sprintf(string_uart_admin, "     ACCELEROMETER SENSOR: X-AXIS in g (Raw values in bracket): \e[32m%5.2f g (%d)\e[37;0m\r\r\n",acc_x/64.0,acc_x);
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		}

		if(abs(acc_y) > ACC_THRESHOLD)
		{
			sprintf(string_uart_admin, "     ACCELEROMETER SENSOR: Y-AXIS in g (Raw values in bracket): \e[31m%5.2f g (%d)\e[37;0m\r\r\n",acc_y/64.0,acc_y);
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
			warning_count++;
		}

		else
		{
			sprintf(string_uart_admin, "     ACCELEROMETER SENSOR: Y-AXIS in g (Raw values in bracket): \e[32m%5.2f g (%d)\e[37;0m\r\r\n",acc_y/64.0,acc_y);
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		}

		if((abs(acc_z) > (64+ACC_THRESHOLD)) || (abs(acc_z) < (64-ACC_THRESHOLD))) // By default, 1 g due to gravity. If ACC_THRESHOLD is 0.4, Warning if less than 0.6 g or more than 1.4 g
		{
			sprintf(string_uart_admin, "     ACCELEROMETER SENSOR: Z-AXIS in g (Raw values in bracket): \e[31m%5.2f g (%d) \e[37;0m<Default: 1 g due to gravity>\e[37;0m\r\r\n",acc_z/64.0,acc_z);
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
			warning_not_horizontal = 1;		// Vehicle is not on a horizontal surface
			warning_count++;
		}
		else
		{
			sprintf(string_uart_admin, "     ACCELEROMETER SENSOR: Z-AXIS in g (Raw values in bracket): \e[32m%5.2f g (%d) \e[37;0m<Default: 1 g due to gravity>\e[37;0m\r\r\n",acc_z/64.0,acc_z);
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
			warning_not_horizontal = 0;		// Vehicle on a horizontal surface
		}

		if (warning_count == 0)
		{
			sprintf(string_uart_admin, "\r\n     \e[42;30m ALL STATUS OK \e[40;37;0m\r\n");
			UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		}
		else
		{
			if (warning_not_horizontal == 0)
			{
				sprintf(string_uart_admin, "\r\n     \e[41;30m NUMBER OF WARNING(S): %u \e[40;37;0m\r\n",warning_count);   // Display Number of Warning
				UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
			}
			else
			{
				sprintf(string_uart_admin, "\r\n     \e[41;30m NUMBER OF WARNING(S): %u \e[40;37;0m     \e[41;30m VEHICLE IS ON A DANGEROUS SLOPE \e[40;37;0m\r\n",warning_count);   // Display Number of Warning
				UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
			}


		}

		sprintf(string_uart_admin, "\r\n-------------------------------------------------------------------------------------------------------------\e[37;0m\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);

		warning_count = 0;

	}
	return;
}

void advanced_remote_maintenance_reboot(void)
{
	if (uart_data == 'r')		// Reboot system
	{
		sprintf(string_uart_admin, "\r\n\e[33mSystem is rebooting. Sensors will be re-calibrated.\r\nAnimation started on the OLED.\r\nPlease wait till system sends the stationary mode message.\e[37;0m\r\n\r\n");
		UART_Send(LPC_UART3, (uint8_t *)string_uart_admin,strlen(string_uart_admin),BLOCKING);
		oled_frame_count = 0;
		uart_password_count = 0;
		modes = ENHANCED;
	}
	return;
}

void advanced_remote_maintenance(void)
{
	if (uart_password_count == 16)	// remote maintenance mode
	{

		advanced_remote_maintenance_telemetry();	// if user presses 't'
		advanced_remote_maintenance_reboot();		// if user presses 'r'
	}
	return;
}

void reboot_animation_remote(void)
{
	switch (oled_frame_count)
	{
		default: 	oled_clearScreen(OLED_COLOR_BLACK); oled_frame_count++; break;
		case 1:		oled_putString(0,0,"SAVING SETTINGS ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 2: 	oled_putString(0,10,">              <",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 3: 	oled_putString(0,10,">>            <<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 4: 	oled_putString(0,10,">>>          <<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 5: 	oled_putString(0,10,">>>>        <<<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 6: 	oled_putString(0,10,">>>>>      <<<<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 7: 	oled_putString(0,10,">>>>>>    <<<<<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 8: 	oled_putString(0,10,">>>>>> OK <<<<<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 9: 	oled_putString(0,20,"UPDATING SYSTEM ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 10: 	oled_putString(0,30,">              <",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 11: 	oled_putString(0,30,">>            <<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 12: 	oled_putString(0,30,">>>          <<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 13: 	oled_putString(0,30,">>>>        <<<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 14: 	oled_putString(0,30,">>>>>      <<<<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 15: 	oled_putString(0,30,">>>>>>    <<<<<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 16: 	oled_putString(0,30,">>>>>> OK <<<<<<",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 17: 	oled_putString(0,40,"STARTING REBOOT ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 18: 	oled_putString(0,50,".               ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 19: 	oled_putString(0,50,"..              ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 20: 	oled_putString(0,50,"...             ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 21: 	oled_putString(0,50,"....            ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 22: 	oled_putString(0,50,".....           ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 23: 	oled_putString(0,50,".....o          ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 24: 	oled_putString(0,50,".....oo         ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 25: 	oled_putString(0,50,".....ooo        ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 26: 	oled_putString(0,50,".....oooo       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 27: 	oled_putString(0,50,".....ooooo      ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 28: 	oled_putString(0,50,".....oooooO     ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 29: 	oled_putString(0,50,".....oooooOO    ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 30: 	oled_putString(0,50,".....oooooOOO   ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 31: 	oled_putString(0,50,".....oooooOOOO  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 32: 	oled_putString(0,50,".....oooooOOOOO ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 33: 	oled_putString(0,50,".....oooooOOOOOO",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 34: 	oled_putString(0,50,".....oooooOOOOOO",OLED_COLOR_WHITE,OLED_COLOR_BLACK); modes = INITIAL_STATIONARY; break;
	}
	return;
}

void reboot_animation_abort(void)
{
	switch (oled_frame_count)
	{
		default: 	oled_clearScreen(OLED_COLOR_BLACK); oled_frame_count++; break;
		case 1:		oled_putString(0,0, "DISCONNECTING!! ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 2: 	oled_putString(0,10,"     <  >       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 3: 	oled_putString(0,10,"    <    >      ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 4: 	oled_putString(0,10,"   <      >     ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 5: 	oled_putString(0,10,"  <        >    ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 6: 	oled_putString(0,10," <          >   ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 7: 	oled_putString(0,10,"<            >  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 8: 	oled_putString(0,10,"<DISCONNECTED>  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 9: 	oled_putString(0,20,"BLOCKING ACCESS ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 10: 	oled_putString(0,30,"     <  >       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 11: 	oled_putString(0,30,"    <    >      ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 12: 	oled_putString(0,30,"   <      >     ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 13: 	oled_putString(0,30,"  <        >    ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 14: 	oled_putString(0,30," <          >   ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 15: 	oled_putString(0,30,"<            >  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 16: 	oled_putString(0,30,"<BLOCKED MODE>  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 17: 	oled_putString(0,40,"STARTING REBOOT ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 18: 	oled_putString(0,50,".               ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 19: 	oled_putString(0,50,"..              ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 20: 	oled_putString(0,50,"...             ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 21: 	oled_putString(0,50,"....            ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 22: 	oled_putString(0,50,".....           ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 23: 	oled_putString(0,50,".....o          ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 24: 	oled_putString(0,50,".....oo         ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 25: 	oled_putString(0,50,".....ooo        ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 26: 	oled_putString(0,50,".....oooo       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 27: 	oled_putString(0,50,".....ooooo      ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 28: 	oled_putString(0,50,".....oooooO     ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 29: 	oled_putString(0,50,".....oooooOO    ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 30: 	oled_putString(0,50,".....oooooOOO   ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 31: 	oled_putString(0,50,".....oooooOOOO  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 32: 	oled_putString(0,50,".....oooooOOOOO ",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 33: 	oled_putString(0,50,".....oooooOOOOOO",OLED_COLOR_WHITE,OLED_COLOR_BLACK); oled_frame_count++; break;
		case 34: 	oled_putString(0,50,".....oooooOOOOOO",OLED_COLOR_WHITE,OLED_COLOR_BLACK); modes = INITIAL_STATIONARY; break;
	}
	return;
}



void oled_refresh_control(void)
{
	current_msticks_oled = Get_Ticks();
	if(reset_reference_msticks_oled)
	{
		reset_reference_msticks_oled = 0;
		reference_msticks_oled = current_msticks_oled;
	}

	else
	{
		if((current_msticks_oled - reference_msticks_oled) >= ANIMATION_FRAME_DURATION)
		{
			if (reboot_type == 0) reboot_animation_remote();		// Genuine maintenance. Saving changes
			else if (reboot_type == 1) {hacker_flag = 1; reboot_animation_abort();}	// Releasing from hacker control. Discarding changes. Disable remote maintenance

			reset_reference_msticks_oled = 1;
		}
	}

	return;
}

void UART3_IRQHandler (void)
{
	UART_Receive(LPC_UART3, &uart_data,1,BLOCKING);

	if(hacker_flag == 0) // if no hacker
	{
		if (uart_password_count == 0 && uart_data == 'p')	// User is attempting to go into remote maintenance mode. Password will be required
		{
			advanced_password_prompt();
			return;		// Subsequent characters will be the password entry when this handler is called again
		}

		advanced_check_password();			// determine is uart_password_count == 999
		advanced_wrong_password_action();	// what to do if uart_password_count == 999
		advanced_correct_password_action();	// what to do if uart_password_count == 15 or 16
		advanced_remote_maintenance();
	}

	return;
}

//P2.10
void EINT0_IRQHandler (void)
{
	LPC_SC->EXTINT = 0b0001; // Clears EINT0, while leaving the other EINT unchanged
	mode_toggling_decision_interrupt();

	return;
}

void EINT3_IRQHandler (void)
{

	if ((LPC_GPIOINT ->IO0IntStatF>>17) & 0X1)		// Centre Joystick
	{
		if (uart_password_count == 16)	// remote maintenance mode 'by' hacker
		{
			reboot_type = 1;	// Hacker in system
			oled_frame_count = 0;
			uart_password_count = 0;
			modes = ENHANCED;
		}
		LPC_GPIOINT ->IO0IntClr = 1<<17;
	}

	else if ((LPC_GPIOINT ->IO0IntStatF>>25) & 0X1)		// Rotary one direction
		{
			hacker_flag = 0;	// No hacker. Allow password entry again
			LPC_GPIOINT ->IO0IntClr = 1<<25;
		}

	// Temperature sensor interrupt triggered //
	else if ((LPC_GPIOINT ->IO0IntStatR>>2) & 0X1)
	{
		number_of_samples++;
		if (number_of_samples >= SAMPLE_TRIGGER)
		{
			t2_reading = Get_Ticks();
			temp_period = ((t2_reading - t1_reading) / (float)(SAMPLE_TRIGGER-1));
			t1_reading = t2_reading;
			number_of_samples = 0;
		}
		LPC_GPIOINT ->IO0IntClr = 1<<2;
	}
	return;
}


void blink_red (uint32_t red_time_period)
{
	current_msticks_red = Get_Ticks();

	if(reset_reference_msticks_red)
	{
		reset_reference_msticks_red = 0;
		reference_msticks_red = current_msticks_red;
	}

	else
	{
		if((current_msticks_red - reference_msticks_red) >= red_time_period)
		{
			if(red_flag == 0)
			{
				GPIO_SetValue(2,1);
				red_flag = 1;
			}
			else
			{
				GPIO_ClearValue(2,1);
				red_flag = 0;
			}
			reset_reference_msticks_red = 1;
		}
	}
	return;
}
void blink_blue (uint32_t blue_time_period)
{
	current_msticks_blue = Get_Ticks();
	if(reset_reference_msticks_blue)
	{
		reset_reference_msticks_blue = 0;
		reference_msticks_blue = current_msticks_blue;
	}

	else
	{
		if((current_msticks_blue - reference_msticks_blue) >= blue_time_period)
		{
			if(blue_flag == 0)
			{
				GPIO_SetValue(0,1<<26);
				blue_flag = 1;
			}
			else
			{
				GPIO_ClearValue(0,1<<26);
				blue_flag = 0;
			}
			reset_reference_msticks_blue = 1;
		}
	}
	return;
}


void oled_sensor_display(void)
{
	sprintf(string_oled_acc,"ACC X:      ");
	oled_putString(0, 20, (uint8_t*)string_oled_acc,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

	sprintf(string_oled_acc,"%.2f   ", acc_x/64.0);
	oled_putString(40, 20, (uint8_t*)string_oled_acc,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

	sprintf(string_oled_temp,"TEMP: %5.2f       ",temperature_reading);
	oled_putString(0, 30, (uint8_t*)string_oled_temp,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

	return;
}

void oled_clear_stationary(void) // clear display by line whenever go back stationary mode
{
	oled_putString(0, 10, "                       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 20, "                       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 30, "                       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 40, "                       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 50, "                       ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	return;
}

void acc_reading_function(void)
{
	acc_read(&acc_x, &acc_y, &acc_z);

	acc_x = (acc_x-acc_xoff);
	acc_y = (acc_y-acc_yoff);
	acc_z = acc_z-acc_zoff+64;

	return;
}

void temperature_update_function(void)
{
	temperature_reading = ((temp_period*1000.0)/TEMP_SCALAR)-273.15;
	return;
}

void uart_transmit_at_f (void) // only happen when 7segment display 'F'
{
	sprintf(string_uart_sensor_values, "%03d_TEMP_%.2f_ACC_%.2f_%.2f_%.2f \r\n",transmission_counter,temperature_reading,(float)(acc_x/64.0),(float)(acc_y/64.0),(float)(acc_z/64.0));
	UART_Send(LPC_UART3, (uint8_t *)string_uart_sensor_values,strlen(string_uart_sensor_values),BLOCKING);
	transmission_counter++;
}

void temp_acc_warnings(void)
{
	// For Temperature
	if (temperature_reading >= TEMP_HIGH_THRESHOLD && red_threshold_flag == 0)
	{
		red_threshold_flag = 1; // Blink Red Must Happen
		sprintf(string_oled_temp_threshold,"Temp.too High");
		oled_putString(0, 40, (uint8_t*)string_oled_temp_threshold,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	}
	// For Accelerometer
	if(abs(acc_x) >= ACC_THRESHOLD && blue_threshold_flag == 0)
	{
		blue_threshold_flag = 1;
		sprintf(string_oled_acc_threshold,"Airbag Deployed");
	    oled_putString(0, 50, (uint8_t*)string_oled_acc_threshold,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	}
	return;
}

void segment_display(uint32_t segment_time)
{
	current_msticks_segment = Get_Ticks();

	if(reset_reference_msticks_segment == 1)
	{
		reset_reference_msticks_segment = 0;
		reference_msticks_segment = current_msticks_segment;
	}
	else
	{
		if((current_msticks_segment - reference_msticks_segment) >= segment_time)
		{
		   led7seg_setChar(segment_array[segment_counter], FALSE);
		   acc_reading_function();
		   temperature_update_function();

		   switch(segment_counter)
		   {
			   case 5:	// 5
				   oled_sensor_display();
				   break;
			   case 10:	// A
				   oled_sensor_display();
				   break;
			   case 15:	// F
				   oled_sensor_display();
				   break;
			   default:
				   // oled_sensor_display(); // for testing so that oled update every second
				   break;
		   }

		   segment_counter++;
		   if (segment_counter == 16)
		   {
			   if((red_threshold_flag) >= 1)	// Blink red is happening, when both blue and red blinks, red message send first
			   {
				   sprintf(string_uart_temp_threshold,"Temp. too High\r\n");
				   UART_Send(LPC_UART3, (uint8_t *)string_uart_temp_threshold,strlen(string_uart_temp_threshold),BLOCKING);
			   }

			   if(blue_threshold_flag >= 1)	// Blink blue is happening
			   {
				   sprintf(string_uart_acc_threshold,"Airbag Deployed \r\n");
				   UART_Send(LPC_UART3, (uint8_t *)string_uart_acc_threshold,strlen(string_uart_acc_threshold),BLOCKING);
			   }

			   uart_transmit_at_f();	// transmit  sensor values to UART only when 7segment display 'F'
			   segment_counter = 0;
		   }
		   reset_reference_msticks_segment = 1;
		}
	}
    return;
}

int main (void)
{
	// Protocol Initialisation
    init_i2c();
    init_ssp();
    init_GPIO();
    ready_uart();

    // Device initialisation
    pca9532_init();
    acc_init();
    oled_init();
    led7seg_init();
    joystick_init();

    // System Core Configuration - Return of function ignored - Assumed to always be working
    SysTick_Config(SystemCoreClock/1000);


	LPC_GPIOINT->IO0IntEnR |= 1<<2;			//Enable temperature sensor P0.2 as GPIO Interrupt
	LPC_GPIOINT->IO0IntEnF |= 1<<17;        //Enable Joystick centre P0.17 as as GPIO Interrupt
	LPC_GPIOINT->IO0IntEnF |= 1<<25;        //Enable Rotary one direction P0.25 as as GPIO Interrupt

	LPC_SC->EXTMODE = 0b0001;		// Make EINT0 edge-sensitive
	LPC_SC->EXTPOLAR = 0b0000;		// EINT0 is detecting falling edge (As soon as active low SW3 is pressed)
	// UART interrupt source is always assumed to be from the receive side (not transit, not error)
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);

	NVIC_SetPriorityGrouping(4);

	NVIC_SetPriority(SysTick_IRQn,NVIC_EncodePriority(4,0,0));	// Highest priority for timing
	NVIC_SetPriority(EINT3_IRQn,NVIC_EncodePriority(4,1,0));
	NVIC_SetPriority(UART3_IRQn,NVIC_EncodePriority(4,2,0));
	NVIC_SetPriority(EINT0_IRQn,NVIC_EncodePriority(4,3,0));	// Lowest priority for SW3

	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_ClearPendingIRQ(UART3_IRQn);

    NVIC_EnableIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(UART3_IRQn);

	light_enable();
	light_setRange(LIGHT_RANGE_4000);	// As per project requirement
	temp_init(&Get_Ticks);
	acc_read(&acc_xoff,&acc_yoff,&acc_zoff);	//Initial offset values for acc

    while (1)
    {
		mode_toggling_decision_polling();

		switch(modes)
		{
			case INITIAL_STATIONARY:
				initial_stationary_task();
				overall_stationary_flag = 1;
				overall_forward_flag = 1;
				break;

			case STATIONARY:
				stationary_task();
				overall_forward_flag = 1;
				overall_reverse_flag = 1;
				if (uart_password_count == 16)	maintenance_fast_update(); // remote maintenance mode
				break;

			case FORWARD:
				overall_stationary_flag = 1;
				forward_task();
				break;

			case REVERSED:
				overall_stationary_flag = 1;
				reverse_task();
				break;

			case ENHANCED:
				oled_refresh_control();
				break;

			default:
				break;
		} // End of switch case

    }

    return 0;
 }





