/**
 * @file rtostask.c
 * @brief MTK3339 GPS Parsing Library - TI-RTOS management tasks
 * @headerfile <mtkgps.h>
 * @details TI-RTOS code library for managing a live MTK3339 GPS using RTOS Mailboxes
 *
 * @author Eric Brundick
 * @date 2016
 * @version 100
 * @copyright (C) 2016 Eric Brundick spirilis at linux dot com
 *  @n Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
 *  @n (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
 *  @n publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
 *  @n do so, subject to the following conditions:
 *  @n
 *  @n The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *  @n
 *  @n THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *  @n OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 *  @n BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
 *  @n OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
#include <ti/drivers/uart/UARTCC26XX.h>

#include <string.h>
#include <mtkgps.h>



static Mailbox_Handle registrationQueue;  // Handles incoming client listener registrations
static Mailbox_Handle writeQueue;         // Handles incoming GPS write requests from clients
static Semaphore_Handle writeReady;       // GPS Writer task indicates to main GPS manager that we're available for writes
static Semaphore_Handle newWrite;         // GPS manager indicates to GPS Writer task that a write is available
static Semaphore_Handle readReady;        // GPS Read callback function indicates to main GPS manager that a buffer is available to read

Void MTKGPS_RTOS_task(UArg, UArg);
Void MTKGPS_UART_ReadFxn(UART_Handle, void *, size_t);
Void MTKGPS_RTOS_Write_task(UArg, UArg);

Void MTKGPS_init()
{
	// Init the registrationQueue
	Mailbox_Handle iH;
	Mailbox_Params iP;
	Error_Block eB;

	Error_init(&eB);
	Mailbox_Params_init(&iP);
	iH = Mailbox_create(sizeof(NmeaListenRegistration_t), GPS_MANAGER_MAX_PENDING_CLIENT_REGISTRATIONS, &iP, &eB);
	if (!iH) {
		System_printf("MTKGPS_init(): Error creating registration Mailbox: %s\n", eB.msg);
		System_flush();
		System_abort("MTKGPS_init(): Forcing RTOS abort");
		return;
	}
	registrationQueue = iH;

	// Init the writeQueue (for client tasks to write to the GPS)
	Error_init(&eB);
	Mailbox_Params_init(&iP);
	iH = Mailbox_create(sizeof(NmeaSentence_t), GPS_MANAGER_MAX_PENDING_GPS_WRITES, &iP, &eB);
	if (!iH) {
		System_printf("MTKGPS_init(): Error creating GPS write Mailbox: %s\n", eB.msg);
		System_flush();
		System_abort("MTKGPS_init(): Forcing RTOS abort");
		return;
	}
	writeQueue = iH;

	// Init the writeReady semaphore (checked before accepting GPS write mailbox entries)
	Semaphore_Handle sH;
	Semaphore_Params sP;

	Error_init(&eB);
	Semaphore_Params_init(&sP);
	sP.mode = Semaphore_Mode_BINARY;
	sH = Semaphore_create(1, &sP, &eB);
	if (!sH) {
		System_printf("MTKGPS_init(): Error creating GPS write ready semaphore: %s\n", eB.msg);
		System_flush();
		System_abort("MTKGPS_init(): Forcing RTOS abort");
		return;
	}
	writeReady = sH;

	Error_init(&eB);
	Semaphore_Params_init(&sP);
	sP.mode = Semaphore_Mode_BINARY;
	sH = Semaphore_create(1, &sP, &eB);
	if (!sH) {
		System_printf("MTKGPS_init(): Error creating GPS new-write semaphore: %s\n", eB.msg);
		System_flush();
		System_abort("MTKGPS_init(): Forcing RTOS abort");
		return;
	}
	newWrite = sH;

	Error_init(&eB);
	Semaphore_Params_init(&sP);
	sP.mode = Semaphore_Mode_BINARY;
	sH = Semaphore_create(1, &sP, &eB);
	if (!sH) {
		System_printf("MTKGPS_init(): Error creating GPS ready-ready semaphore: %s\n", eB.msg);
		System_flush();
		System_abort("MTKGPS_init(): Forcing RTOS abort");
		return;
	}
	readReady = sH;
}

//! @brief Client API function registers this task to receive streaming GPS NMEA messages matching messageMask (OR of 1+ NmeaStreamMsgId_t values)
//! @returns A Mailbox_Handle which may be Mailbox_pend()'d to retrieve incoming streaming GPS messages already parsed out.
Mailbox_Handle MTKGPS_Register_listener(UInt32 clientId, UInt32 messageMask, UInt32 queueDepth, Event_Handle triggerEvent, UInt triggerEventId)
{
	if (queueDepth < 1)
		queueDepth = NMEA_MAILBOX_DEFAULT_QUEUEDEPTH;

	Mailbox_Handle iH;
	Mailbox_Params iP;
	Error_Block eB;

	Error_init(&eB);
	Mailbox_Params_init(&iP);
	if (triggerEvent) {
		iP.readerEvent = triggerEvent;
		iP.readerEventId = triggerEventId;
	}
	iH = Mailbox_create(sizeof(NmeaSentence_t), queueDepth, &iP, &eB);
	if (!iH) {
		System_printf("MTKGPS_Register_listener: Error creating Mailbox: %s\n", eB.msg);
		System_flush();
		return NULL;
	}

	// Compose NmeaListenRegistration and post it to the main task's mailbox
	NmeaListenRegistration_t reg;
	reg.clientId = clientId;
	reg.messageMask = messageMask;
	reg.inbox = iH;

	if (registrationQueue) {
		Mailbox_post(registrationQueue, &reg, BIOS_WAIT_FOREVER);
	} else {
		return NULL;  // GPS task not ready?
	}

	return iH;
}

//! @brief Client API function for posting a new NMEA write operation to the GPS manager task
Void MTKGPS_write(NmeaSentence_t *insent)
{
	Mailbox_post(writeQueue, insent, BIOS_WAIT_FOREVER);
}

Void MTKGPS_read(Mailbox_Handle mH, NmeaSentence_t *s)
{
	Mailbox_pend(mH, s, BIOS_WAIT_FOREVER);
	mtkgps_repointer(s);
}

#define MTKGPS_RTOS_TASK_STACK_SIZE 1024
static Char _mtkgps_rtos_task_stack[MTKGPS_RTOS_TASK_STACK_SIZE];
static Task_Handle mtkgps_rtos_task, mtkgps_rtos_write_task;
static UART_Handle uart;  // Two tasks reference this

//! @brief Creates the GPS Manager RTOS task which performs all the work.  GPS management becomes active at this point.
//! @details This function, along with the RTOS task, manage the GPS but do NOT power it on, off,
//!          or perform any resets on the device.  Use MTKGPS_close() to halt the RTOS task in order
//!          to perform any poweron/off/reset operations outside of the MTKGPS library.
//! @returns 1 if task creation succeeded, 0 if task creation failed
Bool MTKGPS_open(UInt uartInstance, UInt32 initialBaudrate)
{
	// Create the RTOS task with UART info as arguments
    Task_Params taskParams;

    /* Construct GPS Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = (UArg) uartInstance;
    taskParams.arg1 = (UArg) initialBaudrate;
    taskParams.stackSize = MTKGPS_RTOS_TASK_STACK_SIZE;
    taskParams.stack = &_mtkgps_rtos_task_stack;
    mtkgps_rtos_task = Task_create((Task_FuncPtr)MTKGPS_RTOS_task, &taskParams, NULL);
    if (mtkgps_rtos_task == NULL) {
    	return 0;
    }
    return 1;
}

//! @brief Kill all running GPS manager and GPS write tasks.  The client registrations remain valid across close/re-opens.
Bool MTKGPS_close()
{
	Bool retval = 0;

	if (mtkgps_rtos_write_task) {
		Task_delete(&mtkgps_rtos_write_task);
		retval = 1;
	}
	if (mtkgps_rtos_task) {
		Task_delete(&mtkgps_rtos_task);
		retval = 1;
	}
	return retval;
}

static NmeaListenRegistration_t mtkgps_client_registration[GPS_MANAGER_MAX_CLIENT_LISTENER];
static volatile Char mtkgps_uart_write_buffer[NMEA_SENTENCE_MAX_SIZE+1];
#define GPS_MANAGER_WRITE_TASK_STACK_SIZE 384
static Char _mtkgps_uart_write_task_stack[GPS_MANAGER_WRITE_TASK_STACK_SIZE];
static Char mtkgps_uart_read_buffer[2][64];
static volatile Char *mtkgps_readbuf;
static volatile UInt32 mtkgps_readcount;

volatile uint32_t totalNmea = 0, totalValidNmea = 0, totalReadOverruns = 0;


//! @brief The MTKGPS RTOS task manages all GPS I/O and marshalling of data.
Void MTKGPS_RTOS_task(UArg arg0, UArg arg1)
{
	System_printf("totalNmea = %p, totalValidNmea = %p, totalReadOverruns = %p\n", &totalNmea, &totalValidNmea, &totalReadOverruns); System_flush();
	// Step 1: Open UART
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = MTKGPS_UART_ReadFxn;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readTimeout = GPS_MANAGER_UART_READ_TIMEOUT / Clock_tickPeriod;
    uartParams.baudRate = (UInt32)arg1;

	uart = UART_open((UInt)arg0, &uartParams);
	if (uart == NULL) {
    	System_abort("MTKGPS_RTOS_task: Error opening the UART");
    }
    // CC13xx/CC26xx custom UART control feature - RETURN PARTIAL
	#ifdef UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE
    UART_control(uart, UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE, NULL);
	#endif

    // Prepare Read process
    Semaphore_pend(readReady, BIOS_NO_WAIT);
    mtkgps_readbuf = &mtkgps_uart_read_buffer[0][0];
    UART_read(uart, (Void *)mtkgps_readbuf, 64);  // Our callback will handle this.

    // Step 2: Create the UART writer task, which will manage & flow-control submissions to the UART
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = GPS_MANAGER_WRITE_TASK_STACK_SIZE;
    taskParams.stack = &_mtkgps_uart_write_task_stack;
	mtkgps_rtos_write_task = Task_create((Task_FuncPtr)MTKGPS_RTOS_Write_task, &taskParams, NULL);

    // Step 3: Main loop, process UART data and queue requests.
    size_t npos = 0, t, j;
    int i;
    Char *tmpbuf;
	NmeaSentence_t nsen, *nptr;
	NmeaStreamMsgID_t nId;
	nsen.headPointer = &nsen;

	while (1) {
    	// Step 1: Read UART, process data
		if (Semaphore_pend(readReady, 100 / Clock_tickPeriod)) {
			// Grab a local copy of these variables before the Interrupt strikes again
			tmpbuf = (Char *)mtkgps_readbuf;
			i = mtkgps_readcount;
    		for (t=0; t < i; t++) {
    			if (npos > 0) {
					nsen.sentence[npos] = tmpbuf[t];
					npos++;
					if (npos > NMEA_SENTENCE_MAX_SIZE) {
						npos = 0;  // Overflow; reset to nothing
					}
    			}
    			if (npos == 0 && tmpbuf[t] == '$') {
    				nsen.sentence[npos] = '$';
    				npos++;
    			}
    			if (tmpbuf[t] == '\n' && npos > 2 && nsen.sentence[npos-2] == '\r' && nsen.sentence[0] == '$') {
    				// nmeabuf contains potentially valid NMEA string, process
    				nsen.sentence[npos] = '\0';
    				totalNmea++;

    				nptr = mtkgps_parse_sentence(&nsen.sentence[0], &nsen);  // nptr should return as the same value as &nsen if this succeeds
    				if (nptr != NULL) {
    					totalValidNmea++;
    					nId = mtkgps_identify_msgid(nptr);
    					nptr->parsedMessageID = nId;

    					// Poll list of client registrations, post message to their inbox queue if relevant
    					for (j=0; j < GPS_MANAGER_MAX_CLIENT_LISTENER; j++) {
    						if (mtkgps_client_registration[j].inbox != NULL) {
    							if (mtkgps_client_registration[j].messageMask & (uint32_t)nId) {
    								// This client is listening for this type of message.  Post to their mailbox.
    								Mailbox_post(mtkgps_client_registration[j].inbox, nptr, BIOS_NO_WAIT);
    							}
    						}
    					}

    					// Perform any other GPS manager-esque processing of the GPS messages here.
    				}
    				// all done; reset npos
    				npos = 0;
    			} /* if we have a full, valid NMEA sentence in the buffer */
    		} /* keep processing tmpbuf[] entries from the last UART read */
    		mtkgps_readcount = 0;  // Signal to MTKGPS_UART_ReadFxn that we're done with the buffer
    	}

    	// Step 2: Look for any incoming GPS write requests - if we're ready.
    	if (Semaphore_pend(writeReady, BIOS_NO_WAIT) && Mailbox_pend(writeQueue, &nsen, BIOS_NO_WAIT)) {
    		// Compose a proper NMEA sentence and submit it to the UART writer task
    		mtkgps_repointer(&nsen);
    		if (mtkgps_synthesize_sentence(&nsen, (Char *)mtkgps_uart_write_buffer) >= NMEA_SENTENCE_MIN_SIZE) {
        		// Submit to UART writer task
        		Semaphore_post(newWrite);

        		// Interpret the type of message just written to see if, for example, it's a UART baudrate change that
        		// requires our attention (closing & re-opening the UART).
        		nId = mtkgps_identify_msgid(nptr);
        		// TODO: Support identifying any of the PMTK Baudrate Change commands
    		}
    	}

    	// Step 3: Look for any incoming GPS listener registrations
    	NmeaListenRegistration_t incomingRegistration;
    	if (Mailbox_pend(registrationQueue, &incomingRegistration, BIOS_NO_WAIT)) {
    		// First, check if this clientId has already registered.
    		i = 0;  // i is used as a flag indicating whether we're re-registering an already-registered clientId
    		for (j=0; j < GPS_MANAGER_MAX_CLIENT_LISTENER; j++) {
    			if (mtkgps_client_registration[j].inbox != NULL && mtkgps_client_registration[j].clientId == incomingRegistration.clientId) {
    				// Then just replace the messageMask and inbox.
    				mtkgps_client_registration[j].messageMask = incomingRegistration.messageMask;
    				mtkgps_client_registration[j].inbox = incomingRegistration.inbox;
    				i = 1;
    				break;
    			}
    		}
    		// If not, look for a new slot and record it.
    		if (!i) {
    			for (j=0; j < GPS_MANAGER_MAX_CLIENT_LISTENER; j++) {
    				if (mtkgps_client_registration[j].inbox == NULL) {
        				mtkgps_client_registration[j].clientId = incomingRegistration.clientId;
        				mtkgps_client_registration[j].messageMask = incomingRegistration.messageMask;
        				mtkgps_client_registration[j].inbox = incomingRegistration.inbox;
        				break;
    				}
    			}
    			// If there is no available slot, just keep going without recording it.  There's no way to report this back to
    			// the caller.
    			if (j == GPS_MANAGER_MAX_CLIENT_LISTENER) {
					System_printf("MTKGPS_RTOS_task(): Attempted to record new listener at clientId=%d but no registration slots available!\n", incomingRegistration.clientId);
					System_flush();
    			}
    		}
    	}
    }
}

//! @brief The RTOS Read callback accepts reads from the UART driver and flags the GPS manager
Void MTKGPS_UART_ReadFxn(UART_Handle uH, void *inbuf, size_t rdsz)
{
	void *newbuf;
	static volatile UInt readBufferIndex;
	static UInt32 currentBufferLen = 0;

	currentBufferLen += rdsz;

	if (mtkgps_readcount && currentBufferLen < 64) {
		// Keep attempting to fill the current buffer if main task isn't done processing the other
		newbuf = (char *)inbuf + currentBufferLen;
	} else {
		if (mtkgps_readcount) {
			totalReadOverruns++;
		}
		// Post current filled buffer to main task
		mtkgps_readbuf = &mtkgps_uart_read_buffer[readBufferIndex][0];
		mtkgps_readcount = currentBufferLen;

		// Flip to the other buffer for the next read operation
		readBufferIndex = !readBufferIndex;
		newbuf = &mtkgps_uart_read_buffer[readBufferIndex][0];
		currentBufferLen = 0;
	}

	UART_read(uH, newbuf, 64-currentBufferLen);  // Keep it going
	Semaphore_post(readReady);
}

//! @brief The RTOS Write task handles blocking UART_write()'s without impacting GPS read performance.
Void MTKGPS_RTOS_Write_task(UArg arg0, UArg arg1)
{
	Semaphore_pend(newWrite, BIOS_NO_WAIT);  // Clear semaphore once

	while (1) {
		if (Semaphore_pend(newWrite, BIOS_WAIT_FOREVER)) {  // Wait for a new incoming write
			UART_write(uart, (const char *)mtkgps_uart_write_buffer, strlen((const char *)mtkgps_uart_write_buffer));
			Semaphore_post(writeReady);  // Post that we're available for another write request
		}
	}
}
