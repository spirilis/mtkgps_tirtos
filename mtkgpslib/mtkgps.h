/**
 * @file mtkgps.h
 * @brief MTK3339 GPS Parsing Library
 * @details TI-RTOS code library for parsing NMEA sentences belonging to an MTK3339 GPS chipset
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

#ifndef MTKGPSLIB_MTKGPS_H_
#define MTKGPSLIB_MTKGPS_H_

#include <stdint.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Event.h>

/*
 * MTK GPS API for TI-RTOS
 *
 * UART I/O
 * - Library manages UART baudrate
 * - Library runs a task autonomously parsing GPS messages
 * - Other tasks may use an API to register a queue to receive GPS messages of a particular type
 * - Library has a master task queue for submitting requests to the manager task
 * - Client tasks provide a Semaphore_Handle as part of the task request which may be pended to
 *   block until task completion
 * - Library will support low-power modes and recovery
 */


/* NmeaStreamMsgID_t - Identify known message types that stream from the GPS, making these eligible
 * for automated alerting to other tasks via Queue notification.
 */
typedef enum {
	NMEA_GPRMC = 1 << 0,
	NMEA_GPVTG = 1 << 1,
	NMEA_GPGGA = 1 << 2,
	NMEA_GPGSA = 1 << 3,
	NMEA_GPGSV = 1 << 4,
	NMEA_GPGLL = 1 << 5,
	NMEA_GPTXT = 1 << 6,
	NMEA_PMTK_SYS_MSG = 1 << 7,
	NMEA_PMTK_TXT_MSG = 1 << 8,
	NMEA_PMTK_ACK = 1 << 9,
	NMEA_PMTK_DT_FIX_CTL = 1 << 10,
	NMEA_PMTK_DT_DGPS_MODE = 1 << 11,
	NMEA_PMTK_DT_SBAS_ENABLED = 1 << 12,
	NMEA_PMTK_DT_NMEA_OUTPUT = 1 << 13,
	NMEA_PMTK_DT_RELEASE = 1 << 14,
	NMEA_PMTK_LOG = 1 << 15,
	NMEA_PMTK_LSC = 1 << 16,
	NMEA_PQ_EPE = 1 << 17,
	NMEA_ECEFPOSVEL = 1 << 18,
	NMEA_PQ_ODOMETER = 1 << 19,
	NMEA_GNDTM = 1 << 20,
	NMEA_PQ_VELOCITY = 1 << 21,
	NMEA_UNKNOWN = 1 << 31
} NmeaStreamMsgID_t;


//! @brief Max # of unique NmeaSentence_t objects that can be in use at one time
//! @details Client tasks must "free" an NmeaSentence_t by setting the .Talker[0] byte to 0.
//!          This is not actually used with the RTOS task.  It provides a single instance of
//!          NmeaSentence_t for its own purposes.
#define NMEA_SENTENCE_POOL_SIZE 4

//! @brief Maximum # of Data Fields (delimited by ',') allowed in an NMEA sentence
#define NMEA_SENTENCE_MAX_DATA_FIELDS 19
//! @brief Maximum size of the Talker ID (typically a prefix in front of Message ID)
#define NMEA_SENTENCE_MAX_TALKER_ID_LENGTH 5
//! @brief Minimum size of a valid NMEA sentence
#define NMEA_SENTENCE_MIN_SIZE 9
//! @brief Maximum size of a single NMEA sentence for buffer-size purposes
#define NMEA_SENTENCE_MAX_SIZE 127
//! @brief Default buffer size of GPS inbox Mailbox's (in # of NmeaSentence_t objects)
#define NMEA_MAILBOX_DEFAULT_QUEUEDEPTH 4
//! @brief Default buffer size of GPS client registration Mailbox
#define GPS_MANAGER_MAX_PENDING_CLIENT_REGISTRATIONS 2
//! @brief GPS manager RTOS task tunable: Max uS to wait on a UART read
#define GPS_MANAGER_UART_READ_TIMEOUT 20000
//! @brief GPS manager RTOS task: Maximum # of client registrations
#define GPS_MANAGER_MAX_CLIENT_LISTENER 4
//! @brief GPS manager RTOS task: Maximum # of pending GPS writes
#define GPS_MANAGER_MAX_PENDING_GPS_WRITES 2

/* GPS NMEA sentence
 *
 */
typedef struct {
	void *headPointer;
	char Talker[NMEA_SENTENCE_MAX_TALKER_ID_LENGTH+1];
	char *MessageID;
	NmeaStreamMsgID_t parsedMessageID;
	char *dataField[NMEA_SENTENCE_MAX_DATA_FIELDS];
	char sentence[NMEA_SENTENCE_MAX_SIZE+1];
	uint8_t Checksum;
} NmeaSentence_t;

/* RTOS task listen requests
 */
typedef struct {
	uint32_t clientId;     // Some unique ID per client RTOS task
	uint32_t messageMask;  // OR'd mask of NmeaStreamMsgID_t entries
	Mailbox_Handle inbox;  // Client allocates a Mailbox which GPS mgmt task will use to post new data
} NmeaListenRegistration_t;

typedef enum {
	NMEA_PARSE_OK = 0,
	NMEA_PARSE_ERR_TOO_SHORT = 1,
	NMEA_PARSE_ERR_TOO_LONG,
	NMEA_PARSE_ERR_NO_FREE_OBJECTS, // Refers to the internal NmeaSentence_t array supplying the return value for mtkgps_parse_sentence()
	NMEA_PARSE_ERR_INVALID_STARTCHAR, // Missing the initial '$'
	NMEA_PARSE_ERR_UNKNOWN_TALKER,
	NMEA_PARSE_ERR_MISSING_ASTERISK,
	NMEA_PARSE_ERR_MISSING_INITIAL_COMMA,
	NMEA_PARSE_ERR_NO_CHECKSUM,
	NMEA_PARSE_ERR_CHECKSUMS_DONT_MATCH,
	NMEA_PARSE_ERR_TOO_MANY_DATAFIELDS
} NmeaParseError_t;

extern NmeaParseError_t mtkgps_parse_errno;



/* API */

NmeaStreamMsgID_t mtkgps_identify_msgid(NmeaSentence_t *);
NmeaSentence_t * mtkgps_parse_sentence(const char *, NmeaSentence_t *);
const char * mtkgps_parse_strerror(NmeaParseError_t);  //! @brief Interpret mtkgps_parse_errno as a string
size_t mtkgps_synthesize_sentence(NmeaSentence_t *, char *);
void mtkgps_repointer(NmeaSentence_t *);

// RTOS task management and I/O functions
Void MTKGPS_init();  // Init basic semaphores/mailboxes used by the GPS manager RTOS task
Bool MTKGPS_open(UInt uartInstance, UInt32 initialBaudrate);  // Define the initial baudrate expected by the GPS module.
Bool MTKGPS_close();  // Halt the GPS manager RTOS task, e.g. before resetting the GPS device.
Mailbox_Handle MTKGPS_Register_listener(uint32_t clientId, uint32_t messageMask, uint32_t queueDepth, Event_Handle triggerEvent, UInt triggerEventId);
Void MTKGPS_write(NmeaSentence_t *); //! @brief Submit NmeaSentence_t to the writeQueue and wait forever if queue is full
Void MTKGPS_read(Mailbox_Handle, NmeaSentence_t *); //! @brief Wait forever for the next NMEA sentence.  mtkgps_repointer() is run before returning.

/*
 * The MTKGPS Client API allows client tasks utilizing the GPS to register themselves as a listener for a select series of
 * NMEA message types, to receive relevant messages over its Mailbox queue, and to submit NMEA commands to the GPS module in order
 * to configure it.
 *
 * The MTKGPS RTOS task manages all incoming UART data and services incoming Mailbox queue requests for new client registrations and new
 * commands submitted to the GPS.
 *
 * When a client registers itself with the GPS manager task, the MTKGPS_Register_listener() function is used.  This function creates a new
 * Mailbox instance for that client task and submits the registration request to the GPS manager.  The submission involves a private Mailbox
 * and the MTKGPS_Register_listener() function uses BIOS_WAIT_FOREVER when submitting this request, so be aware this function might block
 * indefinitely if the GPS manager task is frozen and no longer servicing its Mailbox queues.
 *
 * As the GPS manager task receives new GPS messages, it will poll its list of client ID + message Masks to see who should receive the message.
 * It posts to each client's Mailbox using BIOS_NO_WAIT so that if a client task has not serviced its queue fast enough, it will drop messages
 * (but if multiple client tasks have registered, it will only drop message for those tasks whose mailbox queues are full).
 *
 */

#endif /* MTKGPSLIB_MTKGPS_H_ */
