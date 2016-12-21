/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "stdio.h"
#include "LPC17xx.h"
#include "string.h"
#include "printf_lib.h"
#include "io.hpp"
#include "event_groups.h"
#include "storage.hpp"
#include "time.h"
#include <ctime>
#include "lpc_pwm.hpp"
#include "queue.h"
#include "task.h"
#include "scheduler_task.hpp"
#include "FreeRTOS.h"
#include "adc0.h"

/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
bool startup = 0;
typedef enum {
shared_TimerQueueId,
} sharedHandle_t;
class timerset: public scheduler_task {
public:
	timerset(uint8_t priority) : scheduler_task("timerset", 2000, priority){
		QueueHandle_t timer_queue = xQueueCreate(1, sizeof(int));
		addSharedObject(shared_TimerQueueId, timer_queue);
	}
	bool run(void *p) {
		while(1){
			int timer;
			if (LPC_GPIO1->FIOPIN & (1 << 10)){ //P1[9] SW0
				timer = 0;
			}
			else if (LPC_GPIO1->FIOPIN & (1 << 9)){ //P1[10] SW1
				timer = timer + 60;
			}
			else if (LPC_GPIO1->FIOPIN & (1<<14)){ //P1[14] SW2
				xQueueSend(getSharedObject(shared_TimerQueueId), &timer, 1000);
				vTaskSuspend(0);
			}

			LD.setNumber(timer/60);
			vTaskDelay(250);
		}
	}
	bool init(void){
		LPC_PINCON -> PINSEL2 &= ~(18 << 0);
		LPC_PINCON -> PINSEL2 &= ~(20 << 0);
		LPC_PINCON -> PINSEL2 &= ~(28 << 0);
		LPC_GPIO1->FIODIR &= ~(9 << 0); //set P1[9] input, SW0
		LPC_GPIO1->FIODIR &= ~(10 << 1); //set P1[10] as input, SW1
		LPC_GPIO1->FIODIR &= ~(14 << 1); //set P1[14] as input, SW2
		return true;
	}
};

class countdown: public scheduler_task {
public:
	countdown(uint8_t priority) :
			scheduler_task("countdown", 2000, priority) {

	}
	bool run(void *p) {
		while(1){
		bool send;
		QueueHandle_t qid = getSharedObject(shared_TimerQueueId);
		int count;
		send = xQueueReceive(qid, &count, 0);
		while (count >= -1 && send){
			if (count > 60){
				LD.setNumber((count/60) - 1);
			}

			if (count <= 60){
				LD.setNumber(count);
			}
			if (count % 60 >= 48){
				LPC_GPIO1->FIOSET=(1<<0);
				LPC_GPIO1->FIOSET=(1<<1);
				LPC_GPIO1->FIOSET=(1<<4);
				LPC_GPIO1->FIOSET=(1<<8);
			}
			else if (count % 60 >= 36 && count % 60 < 48){
				LPC_GPIO1->FIOCLR=(1<<0);
				LPC_GPIO1->FIOSET=(1<<1);
				LPC_GPIO1->FIOSET=(1<<4);
				LPC_GPIO1->FIOSET=(1<<8);
			}
			else if (count % 60 >= 24 && count % 60 < 36){
				LPC_GPIO1->FIOCLR=(1<<0);
				LPC_GPIO1->FIOCLR=(1<<1);
				LPC_GPIO1->FIOSET=(1<<4);
				LPC_GPIO1->FIOSET=(1<<8);
			}
			else if (count % 60 >= 12 && count % 60 < 24){
				LPC_GPIO1->FIOCLR=(1<<0);
				LPC_GPIO1->FIOCLR=(1<<1);
				LPC_GPIO1->FIOCLR=(1<<4);
				LPC_GPIO1->FIOSET=(1<<8);
			}
			else if (count % 60 < 12){
				LPC_GPIO1->FIOCLR=(1<<0);
				LPC_GPIO1->FIOCLR=(1<<1);
				LPC_GPIO1->FIOCLR=(1<<4);
				LPC_GPIO1->FIOCLR=(1<<8);
			}
			if(count == 0 && send) {
				puts("Alarm is off and alarm task will be suspended");
				startup = 1;
				vTaskSuspend(0);

			}
			count = count - 1;
			vTaskDelay(1000);
			printf("Clock is %i \n", count);

		}
		}
	}
	bool init(void) {
		LPC_PINCON -> PINSEL2 &= ~(0 << 0); // set P1.0 as GPIO, LED0
		LPC_PINCON -> PINSEL2 &= ~(2 << 0); // set P1.1 as GPIO, LED1
		LPC_PINCON -> PINSEL2 &= ~(8 << 0); // set P1.4 as GPIO, LED2
		LPC_PINCON -> PINSEL2 &= ~(16 << 0); //set P1.8 as GPIO, LED3
		LPC_GPIO2->FIODIR |= (0 << 1);
		LPC_GPIO2->FIODIR |= (1 << 1);
		LPC_GPIO2->FIODIR |= (4 << 1);
		LPC_GPIO2->FIODIR |= (8 << 1);
		return true;
	}
};

typedef enum {
	shared_SensorQueueId,
} sharedHandleId_t;

class IR_sensor1: public scheduler_task {
public:
	IR_sensor1(uint8_t priority) :
			scheduler_task("IR_sensor1", 2000, priority) {
		//We save the queue handle by using addSharedObject()
		QueueHandle_t IR_sentData1 = xQueueCreate(1, sizeof(int));//make the queue of size 1
		addSharedObject(shared_SensorQueueId, IR_sentData1);
	}
	bool init(void) {
		LPC_PINCON->PINSEL1 |= (1 << 20); // ADC-3 is on P0.26, select this as ADC0.3
		return true;
	}
	bool run(void *p) {
		while (1) {
			int dist_sensor1 = adc0_get_reading(3); // Read the value of ADC-3
			if (startup){
			xQueueSend(getSharedObject(shared_SensorQueueId), &dist_sensor1,
					portMAX_DELAY);
			}
			puts("IR_Sensor 1 is sent");
			printf("IR_Sensor 1 Value is %i\n", dist_sensor1);
			vTaskDelay(200);
			if(SW.getSwitch(4)) {
				vTaskSuspend(0);
			}
			return true;
		}
	}
};

class IR_sensor2: public scheduler_task {
public:
	IR_sensor2(uint8_t priority) :
			scheduler_task("IR_sensor2", 2000, priority) {
		//We save the queue handle by using addSharedObject()
		QueueHandle_t IR_sentData2 = xQueueCreate(1, sizeof(int));//make the queue of size 1
		addSharedObject(shared_SensorQueueId, IR_sentData2);
	}
	bool init(void) {
		LPC_PINCON->PINSEL3 |= (3 << 30); // ADC-5 is on P1.31, select this as ADC0.5
		return true;
	}
	bool run(void *p) {
		while (1) {
			int dist_sensor2 = adc0_get_reading(5); // Read the value of ADC-5
			if (startup){
			xQueueSend(getSharedObject(shared_SensorQueueId), &dist_sensor2,
					portMAX_DELAY);
			}
			puts("IR Sensor 2 data is sent");
			printf("IR_Sensor 2 is %i\n", dist_sensor2);
			vTaskDelay(200);
			if(SW.getSwitch(4)) {
				vTaskSuspend(0);
			}
			return true;
		}
	}

};

class buzzer: public scheduler_task {
public:
	buzzer(uint8_t priority) :
			scheduler_task("buzzer", 2048, priority) {
	}
	bool init(void) {
		LPC_PINCON->PINSEL4 &= ~(3 << 10);
		LPC_PINCON->PINSEL4 |= (1 << 10); //P2.5 PWM = PWM1[6]
		return true;
	}
	bool run(void *p) {
		PWM buzzer1(PWM::pwm6, 5000); //PWM6 = PWM1-6 which is located in P2[5]
		if (startup) {					//Once the timer goes off, it will start ringing but once the button 4 is pressed, the buzzer will be off and task will end.
			if (SW.getSwitch(4)) {
				buzzer1.set(0);
				vTaskSuspend(0);
			}
			init();
			buzzer1.set(50);
			vTaskDelay(200);
		}
		return true;
	}
};
class state_machineTask: public scheduler_task {
public:
	state_machineTask(uint8_t priority) :
			scheduler_task("state_machine", 2048, priority) {

	}
	bool init(void) {
		LPC_PINCON->PINSEL4 &= ~(3 << 4); // P2.2 PWM reset to 0 PWM3
		LPC_PINCON->PINSEL4 |= (1 << 4); //Set to bit 01
		LPC_PINCON->PINSEL4 &= ~(3 << 8); // P2.4 PWM PWM5
		LPC_PINCON->PINSEL4 |= (1 << 8); //set bit to 01

		LPC_PINCON->PINSEL3 &= ~(3 << 6); //P1.19 bit 7:6 is reseted
		LPC_PINCON->PINSEL3 |= (0 << 6);	//initialize the GPIO function
		LPC_GPIO1->FIODIR |= (1 << 19); //Make the direction of P1.19 as output

		LPC_PINCON->PINSEL3 &= ~(3 << 8); //P1.20 bit 9:8 is reseted
		LPC_PINCON->PINSEL3 |= (0 << 8); //Initialize the GPIO function
		LPC_GPIO1->FIODIR |= (1 << 20); //Make the direction of P1.20 as output

		LPC_PINCON->PINSEL3 &= ~(3 << 12); //P1.22 bit 13:12 is reseted
		LPC_PINCON->PINSEL3 |= (0 << 12); //Initialize the GPIO function
		LPC_GPIO1->FIODIR |= (1 << 22); //Make the direction of P1.22 as output

		LPC_PINCON->PINSEL3 &= ~(3 << 14); //P1.23 bit 15:14 is reseted
		LPC_PINCON->PINSEL3 |= (0 << 14); //Initialize the GPIO function
		LPC_GPIO1->FIODIR |= (1 << 23); //Make the direction of P1.23 as output
		return true;
	}
	void PWM_motorInit() {
		PWM motor1(PWM::pwm3, 1000); //PWM3 = PWM1-3 which is located in P2[2]
		PWM motor3(PWM::pwm3, 1000);		//Left Side
		PWM motor2(PWM::pwm5, 1000);//PWM5 = PWM1-5 which is located in P2[4]
		PWM motor4(PWM::pwm5, 1000);		//Right side
	}

	void move_forward() {
		LPC_GPIO1->FIOSET = (1 << 19);//Set the GPIO pin connected to IN1 in H-bridge to be 1
		LPC_GPIO1->FIOCLR = (0 << 20);//Set the GPIO pin connected to IN2 in H-bridge to be 0
		LPC_GPIO1->FIOSET = (1 << 22);//Set the GPIO pin connected to IN3 in H-bridge to be 1
		LPC_GPIO1->FIOCLR = (1 << 23);//Set the GPIO pin connected to IN4 in H-bridge to be 0
		PWM motor1(PWM::pwm3, 1000);//PWM3 = PWM1-3 which is located in P2[2]
		PWM motor3(PWM::pwm3, 1000);		//Left Side
		PWM motor2(PWM::pwm5, 1000);//PWM5 = PWM1-5 which is located in P2[4]
		PWM motor4(PWM::pwm5, 1000);		//Right side
		motor1.set(75);
		motor2.set(75);
		motor3.set(75);
		motor4.set(75);
	}

	/*
	 * IN1 = 5V, IN2 = GND (Forward)
	 * IN1 = GND, IN2 = 5V (Reverse)
	 */
	void move_left() {
		LPC_GPIO1->FIOCLR = (1 << 19);//Set the GPIO pin connected to IN1 in H-bridge to be 0
		LPC_GPIO1->FIOSET = (1 << 20);//Set the GPIO pin connected to IN2 in H-bridge to be 1
		LPC_GPIO1->FIOSET = (1 << 22);//Set the GPIO pin connected to IN3 in H-bridge to be 1
		LPC_GPIO1->FIOCLR = (1 << 23);//Set the GPIO pin connected to IN4 in H-bridge to be 0
		PWM motor1(PWM::pwm3, 1000);//PWM3 = PWM1-3 which is located in P2[2]
		PWM motor3(PWM::pwm3, 1000);		//Left Side
		PWM motor2(PWM::pwm5, 1000);//PWM5 = PWM1-5 which is located in P2[4]
		PWM motor4(PWM::pwm5, 1000);		//Right side
		motor1.set(75);
		motor2.set(75);
		motor3.set(75);
		motor4.set(75);
	}
	/*
	 * IN1 = 5V, IN2 = GND (Forward)
	 * IN1 = GND, IN2 = 5V (Reverse)
	 */
	void move_right() {
		LPC_GPIO1->FIOSET = (1 << 19);//Set the GPIO pin connected to IN1 in H-bridge to be 1
		LPC_GPIO1->FIOCLR = (1 << 20);//Set the GPIO pin connected to IN2 in H-bridge to be 0
		LPC_GPIO1->FIOCLR = (1 << 22);//Set the GPIO pin connected to IN3 in H-bridge to be 0
		LPC_GPIO1->FIOSET = (1 << 23);//Set the GPIO pin connected to IN4 in H-bridge to be 1
		PWM motor1(PWM::pwm3, 1000);//PWM3 = PWM1-3 which is located in P2[2]
		PWM motor3(PWM::pwm3, 1000);		//Left Side
		PWM motor2(PWM::pwm5, 1000);//PWM5 = PWM1-5 which is located in P2[4]
		PWM motor4(PWM::pwm5, 1000);		//Right side
		motor1.set(75);
		motor2.set(75);
		motor3.set(75);
		motor4.set(75);
	}


	bool run(void *p) {
		init();
		PWM_motorInit();

		int distance1 = adc0_get_reading(3); // Read the value of ADC-3
		int distance2 = adc0_get_reading(5);// Read the value of ADC-5

		QueueHandle_t IR_getData1 = getSharedObject(shared_SensorQueueId);
		QueueHandle_t IR_getData2 = getSharedObject(shared_SensorQueueId);
		if (xQueueReceive(IR_getData1, &distance1,
				portMAX_DELAY) && xQueueReceive(IR_getData2, &distance2, portMAX_DELAY)) {
			puts("Received the IR_Data for both sensors \n");
			if (distance1 > 900 || distance2 > 900) //sensor 1 = Left IR Sensor  sensor 2 = Right IR Sensor
			{	//sensor 1 and sensor 2
				if (distance1 > distance2) {
					move_right();
					puts("Moving right");
				} else {
					move_left();
					puts("Moving left");
				}
			} else {
				move_forward();
				puts("Moving forward");
			}
		}
/*
		else {
			move_forward();
			puts("IR is not detecting any obstacles within its range");
		}
*/
		return true;
	}
};

int main(void)
{
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

/*  scheduler_add_task(new orient_compute(PRIORITY_LOW));
    scheduler_add_task(new orient_process(PRIORITY_LOW));*/
    //task_watchdog = xEventGroupCreate();
    //scheduler_add_task(new consumer_task(PRIORITY_MEDIUM));
    //scheduler_add_task(new producer_task(PRIORITY_MEDIUM));
   // scheduler_add_task(new watchdog_task(PRIORITY_HIGH));
	scheduler_add_task(new timerset(PRIORITY_HIGH));
	scheduler_add_task(new countdown(PRIORITY_MEDIUM));
	scheduler_add_task(new buzzer(PRIORITY_LOW));
    scheduler_add_task(new IR_sensor1(PRIORITY_LOW));
    scheduler_add_task(new IR_sensor2(PRIORITY_LOW));
    scheduler_add_task(new state_machineTask(PRIORITY_LOW));
    //scheduler_add_task(new spi_task(PRIORITY_HIGH));
    //scheduler_add_task(new EINT_task(PRIORITY_HIGH));
   // scheduler_add_task(new UART_task(PRIORITY_HIGH));
    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
    //scheduler_add_task(new gpio_task(PRIORITY_HIGH));
    //scheduler_add_task(new led_task(PRIORITY_HIGH));
    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */

    #if 1
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
