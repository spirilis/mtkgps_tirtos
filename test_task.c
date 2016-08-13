/*
 * ** TESTED on CC1310 LaunchPad **
 *
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

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "Board.h"
#include <mtkgps.h>

#define TASKSTACKSIZE   2048

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
Task_Struct task1Struct;
Char task1Stack[512];

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 *  gpsUartFxn
 *
 *  Example TI-RTOS task for polling the UART and processing NMEA sentences.  This is
 *  a testbed for the eventual mtkgpslib-native RTOS task which will perform a similar role.
 *  In this testbed, a secondary RTOS task is created which reports some (semaphore-gated) stats
 *  every 15 seconds to the CIO (System_printf) interface.
 *
 *  I have determined by modifying the sleep delay on this reporting task that the CIO interface
 *  does appear to have a direct effect of causing the gpsUartFxn task to lose UART data, guaranteeing
 *  at least 1 NMEA sentence is read as invalid each time the reporting task does its thing.
 *
 *  This has been tested on the CC1310 LaunchPad with CC1310_LAUNCHXL.h modified so the RX, TX pins are
 *  swapped (to account for the fact that my Quectel L80 boosterpack uses J1.3 for GPS_TXD/MCU_RXD, but
 *  the CC1310 LaunchPad has its J1.3/J1.4 pins swapped in its physical layout relative to what the
 *  silkscreen says).  This should work well on the CC2650 LaunchPad or CC1350 LaunchPad as long as the
 *  correct DIO's are used for the GPSPwr control pin and UART pins are configured correctly.
 *
 *  As the task uses the PIN_ interface, it's limited to TI Chameleon chipsets like the CC13xx and CC26xx
 *  however, the MSP432 and other TI-RTOS platforms may be supported by changing all the GPIO PIN_ operations
 *  with function calls relevant to that platform.
 */
char nmeabuf[256];
volatile uint32_t totalNmea = 0, totalNmeaFailed = 0, totalNmeaKnownMsg = 0;
Void mtkgpsReportStats(UArg, UArg);

Void gpsUartFxn(UArg arg0, UArg arg1)
{
	Semaphore_Handle rptSem;
	Semaphore_Params sP;
	Error_Block eB;
	Semaphore_Params_init(&sP);
	sP.mode = Semaphore_Mode_BINARY;
	rptSem = Semaphore_create(1, &sP, &eB);

	Task_Params rptTP;
	Task_Params_init(&rptTP);
    rptTP.arg0 = (UArg)rptSem;
    rptTP.stackSize = 512;
    rptTP.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)mtkgpsReportStats, &rptTP, NULL);


    UART_Handle uart;
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 9600;

    // UART PINS HAVE BEEN CUSTOM SWITCHED TO COMPENSATE FOR INCORRECT BOOSTERPACK LAYOUT ON CC1310 LAUNCHPAD
	// It just so happens that my own Quectel L80-based BoosterPack, soon to be available via Tindie, has its
	// GPS power-domain control pin assigned to the same DIO on the CC1310 LaunchPad as the onboard Red LED, hence
	// the use of "Board_RLED" as its GPIO identifier.
    PIN_setOutputValue(ledPinHandle, Board_RLED, 0);  // GPSPwr=OFF
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
    	System_abort("Error opening the UART");
    }
    // CC13xx/CC26xx custom UART control feature - RETURN PARTIAL
    UART_control(uart, UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE, NULL);
    PIN_setOutputValue(ledPinHandle, Board_RLED, 1);  // GPSPwr=ON

    char tmpbuf[64];
    size_t npos = 0, t, j;
    int i;
    Bool semAcq;

    System_printf("Start:\n"); System_flush();
    while (1) {
    	i = UART_read(uart, tmpbuf, 64);
    	if (i != UART_ERROR) {
    		for (t=0; t < i; t++) {
    			if (npos > 0) {
					nmeabuf[npos] = tmpbuf[t];
					npos++;
    			}
    			if (npos == 0 && tmpbuf[t] == '$') {
    				nmeabuf[npos] = '$';
    				npos++;
    			}
    			if (tmpbuf[t] == '\n' && npos > 2 && nmeabuf[npos-2] == '\r' && nmeabuf[0] == '$') {
    				semAcq = Semaphore_pend(rptSem, 50);
    				if (semAcq) totalNmea++;

    				// nmeabuf contains potentially valid NMEA string, process
    				nmeabuf[npos] = '\0';
    				//System_printf("Processing: %s\n", nmeabuf); System_flush();
    				NmeaSentence_t * nsen = mtkgps_parse_sentence((uint8_t *)nmeabuf);
    				if (nsen != NULL) {
						//System_printf("Talker ID: %s\n", nsen->Talker);
						//System_printf("Message ID: %s\n", nsen->MessageID);
						//System_flush();
						//j = 0;
						//while (nsen->dataField[j] != NULL) {
							//System_printf("Field #%d: %s\n", j+1, nsen->dataField[j]);
						//	j++;
						//}
						//System_flush();
						//System_printf("Checksum: %d\n", nsen->Checksum);
						//System_flush();
    					NmeaStreamMsgID_t nId = mtkgps_identify_msgid(nsen);
    					if (semAcq && nId != NMEA_UNKNOWN) totalNmeaKnownMsg++;
						nsen->Talker[0] = '\0';  // Free the NmeaSentence_t object
    				} else {
    					if (semAcq) totalNmeaFailed++;
    					//System_printf("mtkgps error: %s\n", mtkgps_parse_strerror(mtkgps_parse_errno)); System_flush();
    				}

    				// all done; reset npos
    				if (semAcq) Semaphore_post(rptSem);
    				npos = 0;
    			}
    		}
    	}
    }
}

Void mtkgpsReportStats(UArg arg0, UArg arg1)
{
	Semaphore_Handle rptSem = (Semaphore_Handle)arg0;
	uint32_t tot, fail, msgknown;

	while (1) {
		Task_sleep(15000000 / Clock_tickPeriod);
		Semaphore_pend(rptSem, BIOS_WAIT_FOREVER);
		tot = totalNmea;
		totalNmea = 0;
		fail = totalNmeaFailed;
		totalNmeaFailed = 0;
		msgknown = totalNmeaKnownMsg;
		totalNmeaKnownMsg = 0;
		Semaphore_post(rptSem);

		System_printf("%d NMEA sentences processed, %d known messages, %d failed\n", tot, msgknown, fail);
		System_flush();
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

    /* Construct GPS UART Task  thread */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)gpsUartFxn, &taskParams, NULL);

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
