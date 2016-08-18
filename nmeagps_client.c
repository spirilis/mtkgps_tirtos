/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty_min.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
//#include <ti/drivers/uart/UARTCC26XX.h>
// #include <ti/drivers/Watchdog.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplaySharp.h>

/* Board Header files */
#include "Board.h"
#include <mtkgps.h>
#include <stdlib.h>

#define TASKSTACKSIZE   2048

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
Task_Struct task1Struct;
Char task1Stack[1024];

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_INPUT_EN | PIN_GPIO_OUTPUT_DIS,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/* Init Sharp96 LCD display */
Display_Handle initSharp()
{
	Display_Params dP;
	Display_Params_init(&dP);

	dP.lineClearMode = DISPLAY_CLEAR_BOTH;

	Display_Handle dH = Display_open(Display_Type_LCD, &dP);

	if (!dH) {
		System_printf("ERROR: Unable to open Sharp display.\n");
		System_flush();
		while(1) Task_sleep(1000);
	}

	DisplaySharpColor_t dCol = {ClrBlack, ClrWhite};
	Display_control(dH, DISPLAYSHARP_CMD_SET_COLORS, (void *)&dCol);
	Display_clear(dH);
	Display_print0(dH, 0, 0, "INIT DPY");

	return dH;  // should be a pointer so this is OK to return even though dH was allocated in the function's stack
}


/*
 * gpsClientFxn - Utilizing the MTKGPS client API, poll the GPS for GGA information
 */
Void heartbeatLcd(UArg, UArg);

volatile uint32_t DebugMask, totalReads = 0;

Void gpsClientFxn(UArg arg0, UArg arg1)
{
    Display_Handle dpy = initSharp();

    Semaphore_Handle dpySem;
	Semaphore_Params sP;
	Error_Block eB;
	Semaphore_Params_init(&sP);
	sP.mode = Semaphore_Mode_BINARY;
	dpySem = Semaphore_create(1, &sP, &eB);

	Task_Params rptTP;
	Task_Params_init(&rptTP);
    rptTP.arg0 = (UArg)dpySem;
    rptTP.arg1 = (UArg)dpy;
    rptTP.stackSize = 1024;
    rptTP.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)heartbeatLcd, &rptTP, NULL);

    if (!MTKGPS_open(Board_UART0, 9600)) {
    	Semaphore_pend(dpySem, BIOS_WAIT_FOREVER);
    	Display_print0(dpy, 0, 0, "MTKGPS_open ERR");
    	Semaphore_post(dpySem);
    	return;
    }

    Mailbox_Handle gpsInbox = MTKGPS_Register_listener(1, NMEA_GPGGA | NMEA_GPVTG, 2, NULL, 0);
    if (gpsInbox == NULL) {
    	MTKGPS_close();
    	Semaphore_pend(dpySem, BIOS_WAIT_FOREVER);
    	Display_print0(dpy, 0, 0, "RegLstnr ERR");
    	Semaphore_post(dpySem);
    	return;
    }

    NmeaSentence_t data;
    UInt32 totalGGA = 0;

    while (1) {
    	MTKGPS_read(gpsInbox, &data);
    	totalReads++;
    	switch (data.parsedMessageID) {
    	case NMEA_GPGGA:
    		totalGGA++;
    		if (Semaphore_pend(dpySem, BIOS_NO_WAIT)) {
    			Display_print1(dpy, 0, 0, "GPSFix: %s", data.dataField[5]);
    			Display_print1(dpy, 1, 0, "#SAT: %s", data.dataField[6]);
    			Display_print1(dpy, 2, 0, "#GGA: %d", totalGGA);
    			Semaphore_post(dpySem);
    		}
    		break;
    	case NMEA_GPVTG:
    		if (Semaphore_pend(dpySem, BIOS_NO_WAIT)) {
    			if (data.dataField[8][0] != 'N') {
    				Display_print1(dpy, 5, 0, "KM/H: %s", data.dataField[6]);
    			} else {
    				Display_print0(dpy, 5, 0, "               ");
    			}
    			Semaphore_post(dpySem);
    		}
    		break;
    	} /* switch(data.parsedMessageID) */
    } /* while(1) */
}

Void heartbeatLcd(UArg arg0, UArg arg1)
{
	Semaphore_Handle dpySem = (Semaphore_Handle)arg0;
	Display_Handle dpy = (Display_Handle)arg1;
	static unsigned long toggle = 0;

	while (1) {
		Task_sleep(500000 / Clock_tickPeriod);
		Semaphore_pend(dpySem, BIOS_WAIT_FOREVER);
		toggle ^= 1;
		PIN_setOutputValue(ledPinHandle, Board_GLED, toggle);
		Display_print1(dpy, 9, 0, "%d", toggle);
		Semaphore_post(dpySem);
	}
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    // Board_initI2C();
    // Board_initSPI();
    Board_initUART();
    // Board_initWatchdog();
    MTKGPS_init();

    /* Construct GPS Client Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000000 / Clock_tickPeriod;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)gpsClientFxn, &taskParams, NULL);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_setOutputValue(ledPinHandle, Board_LED1, 0);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
