/**
 * @file mtkgps.c
 * @brief MTK3339 GPS Parsing Library
 * @headerfile <mtkgps.h>
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

#include <string.h>
#include <mtkgps.h>
#include <xdc/runtime/System.h>

/*
 * typedef enum {
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
 */

//! @brief Structure used for mtkgps_identify_msgid() lookup table
typedef struct {
	char TalkerID[8];
	size_t TalkerID_Length;
	uint32_t StreamMsgID_Bitmask;
} NmeaStreamMap_TalkerID_t;

//! @brief TalkerID-to-MessageID lookup table used for mtkgps_identify_msgid()
//! @details This map, combined with the NmeaMap_MessageID[] array, represent a crude hash table
//!          used to optimize Message ID lookups
const NmeaStreamMap_TalkerID_t NmeaMap_TalkerID[] = {
	{"GP", 2, NMEA_GPRMC | NMEA_GPVTG | NMEA_GPGGA | NMEA_GPGSA | NMEA_GPGSV | NMEA_GPGLL | NMEA_GPTXT},
	{"PMTK", 4, NMEA_PMTK_SYS_MSG | NMEA_PMTK_TXT_MSG | NMEA_PMTK_ACK | NMEA_PMTK_DT_FIX_CTL | NMEA_PMTK_DT_DGPS_MODE \
			    | NMEA_PMTK_DT_SBAS_ENABLED | NMEA_PMTK_DT_NMEA_OUTPUT | NMEA_PMTK_DT_RELEASE | NMEA_PMTK_LOG | NMEA_PMTK_LSC},
	{"PQ", 2, NMEA_PQ_EPE | NMEA_PQ_ODOMETER | NMEA_PQ_VELOCITY},
	{"ECEF", 4, NMEA_ECEFPOSVEL},
	{"GN", 2, NMEA_GNDTM},
	{"", 0, 0}
};

//! @brief MessageID-to-NmeaStreamMsgID_t bit index lookup table for mtkgps_identify_msgid()
const char *NmeaMap_MessageID[] = {
		"RMC", "VTG", "GGA", "GSA", "GSV", "GLL", "TXT", "010", "011", "001", "500", "501", "513", "514", "705", "LOG", "LSC", "EPE", "POSVEL", "ODO", "DTM", "VEL",
		"", "", "", "", "", "", "", "", "", "" // index 22-31 defined but empty to avoid buffer overruns
};

//! @brief Performance optimization - bitmask lookup table
const uint32_t Bitmask_32[] = {
	0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080,
	0x00000100, 0x00000200, 0x00000400, 0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000,
	0x00010000, 0x00020000, 0x00040000, 0x00080000, 0x00100000, 0x00200000, 0x00400000, 0x00800000,
	0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000
};

/*!
 * @brief Identify a streaming-read style of message from a parsed NmeaSentence_t
 * @details Used to quickly identify information streamed from the GPS to the MCU, this function performs a lookup
 *          of the NMEA data to identify the type of data being presented in the NmeaSentence_t.  This boils down
 *          to a simple enum integer which is evaluated easily using a switch() statement.  As the enum values just
 *          happen to be bitfields, it is easy to compare against a bitmask of "allowed" message IDs.  It's a crude
 *          hash table.
 * @return enum NmeaStreamMsgID_t value (corresponding to a 32-bit unsigned integer bitfield)
 */
NmeaStreamMsgID_t mtkgps_identify_msgid(NmeaSentence_t *insent)
{
	int i, bidx;
	uint32_t mask;

	// Search for TalkerID within NmeaMap_TalkerID[]
	i=0;
	while (NmeaMap_TalkerID[i].StreamMsgID_Bitmask != 0) {
		if (!strcmp(insent->Talker, NmeaMap_TalkerID[i].TalkerID)) {
			mask = NmeaMap_TalkerID[i].StreamMsgID_Bitmask;
			// Search for MessageID within NmeaMap_MessageID[] matching TalkerID bitmask first
			for (bidx = 0; bidx < 32; bidx++) {
				if (mask & Bitmask_32[bidx]) {
					if (!strcmp(insent->MessageID, NmeaMap_MessageID[bidx])) {
						return (NmeaStreamMsgID_t)Bitmask_32[bidx];
					}
				}
			}
		}
		i++;
	}

	return NMEA_UNKNOWN;
}


//! @brief Internal pool of NmeaSentence_t objects presented to client tasks upon mtkgps_parse_sentence()
static NmeaSentence_t parsedSentencePool[NMEA_SENTENCE_POOL_SIZE];

//! @brief Variable holds the last error from the parser
NmeaParseError_t mtkgps_parse_errno;

/*!
 * @brief Parse NMEA string sentence into an NmeaSentence_t object
 * @details Take a single NMEA sentence, 1 line, optionally including the \r and/or \n, and parse it out into
 *          a TalkerID, MessageID, array of strings representing data fields, and checksum.  The NMEA sentence
 *          is passed by the caller as a pointer to uint8_t array and it is destructively modified in the parsing
 *          of the data fields.  The NmeaSentence_t.dataField[] array point directly into the supplied buffer.
 * @return A pointer to an NmeaSentence_t stored within a finite-size local pool, or NULL if there was an error or
 *         if this is not a valid NMEA sentence.
 */
NmeaSentence_t * mtkgps_parse_sentence(const char *insent, NmeaSentence_t *dest)
{
	char tmpc;
	size_t slen = strlen(insent);
	size_t i = 0, pst = 0, didx = 0; // i, pst are generic markers; didx keeps track of dataField[] entries
	NmeaSentence_t *outs;
	uint8_t computedChecksum = 0;

	mtkgps_parse_errno = NMEA_PARSE_OK;

	if (slen < NMEA_SENTENCE_MIN_SIZE) {  // Talker+Message usually >=5, plus 4 - the $, the * and the two checksum digits
		mtkgps_parse_errno = NMEA_PARSE_ERR_TOO_SHORT;
		return NULL;
	}

	if (slen > NMEA_SENTENCE_MAX_SIZE) {
		mtkgps_parse_errno = NMEA_PARSE_ERR_TOO_LONG;
		return NULL;
	}

	if (dest == NULL) {
		// Find an unused parsedSentencePool object
		for (i=0; i < NMEA_SENTENCE_POOL_SIZE; i++) {
			if (parsedSentencePool[i].Talker[0] == '\0') {
				break;
			}
		}
		if (i == NMEA_SENTENCE_POOL_SIZE) {
			mtkgps_parse_errno = NMEA_PARSE_ERR_NO_FREE_OBJECTS;
			return NULL;
		}
		outs = &parsedSentencePool[i];
	} else {
		outs = dest;
	}

	// Copy (if necessary) raw NMEA sentence data to NmeaSentence_t.sentence buffer
	if ( (void *)outs->sentence != (void *)insent ) {  // Validate whether this actually needs copying
		System_printf("Copying %p to %p...\n", insent, outs->sentence); System_flush();
		memcpy(outs->sentence, insent, slen);  // Copy sentence to local buffer
	}
	char *cinsent = &outs->sentence[0];

	if (cinsent[0] != '$') {
		mtkgps_parse_errno = NMEA_PARSE_ERR_INVALID_STARTCHAR;
		return NULL;
	}

	if (cinsent[slen-1] == '\n') {
		slen--;
	}
	if (cinsent[slen-1] == '\r') {
		slen--;
	}

	i = 0;
	while (NmeaMap_TalkerID[i].StreamMsgID_Bitmask != 0) {
		if (!strncmp(cinsent+1, NmeaMap_TalkerID[i].TalkerID, NmeaMap_TalkerID[i].TalkerID_Length)) {
			memcpy(outs->Talker, cinsent+1, NmeaMap_TalkerID[i].TalkerID_Length);
			pst = NmeaMap_TalkerID[i].TalkerID_Length;
			outs->Talker[pst] = '\0';
			pst++;
			break;
		}
		i++;
	}

	if (pst == 0) {
		mtkgps_parse_errno = NMEA_PARSE_ERR_UNKNOWN_TALKER;
		return NULL;
	}

	// Compute XOR checksum for later verification
	for (i=1; i<slen; i++) {
		if (cinsent[i] != '*') {
			computedChecksum ^= insent[i];
		} else {
			break;
		}
	}
	if (i == slen) { // Invalid NMEA sentence if we're at the end of the line and haven't found the '*'!
		mtkgps_parse_errno = NMEA_PARSE_ERR_MISSING_ASTERISK;
		outs->Talker[0] = '\0';
		return NULL;
	}

	// pst is the Message ID start index for now
	i = pst;
	while (cinsent[i] != ',' && i < slen) {
		i++;
	}
	if (i == slen) {  // Reached end of string before finding our ','
		mtkgps_parse_errno = NMEA_PARSE_ERR_MISSING_INITIAL_COMMA;
		outs->Talker[0] = '\0';  // free buffer before bailing
		return NULL;
	}
	cinsent[i] = '\0';
	outs->MessageID = cinsent+pst;
	pst = ++i;  // Advance past the first ',' (now a '\0' terminating Message ID) for our data fields

	outs->dataField[didx] = cinsent+i;
	while (didx < NMEA_SENTENCE_MAX_DATA_FIELDS) {
		tmpc = cinsent[i];
		while (tmpc != ',' && tmpc != '*') {
			tmpc = cinsent[++i];
		}
		cinsent[i] = '\0';
		didx++;
		if (tmpc == '*') {  // End of data fields; parse checksum, verify checksum, then return valid NmeaSentence_t *
			for (pst=didx; pst < NMEA_SENTENCE_MAX_DATA_FIELDS; pst++) {
				outs->dataField[pst] = NULL;  // Clear unused dataField pointers
			}
			// Parse checksum
			if ( (slen-i) < 2 ) {
				// Invalid sentence because we don't have the checksum!
				mtkgps_parse_errno = NMEA_PARSE_ERR_NO_CHECKSUM;
				outs->Talker[0] = '\0';
				return NULL;
			}
			i++;
			tmpc = cinsent[i];
			if (tmpc >= '0' && tmpc <= '9') {
				pst = tmpc - '0';
			} else if (tmpc >= 'A' && tmpc <= 'F') {
				pst = tmpc - 'A' + 10;
			} else if (tmpc >= 'a' && tmpc <= 'f') {
				pst = tmpc - 'a' + 10;
			}
			outs->Checksum = pst << 4;
			i++;
			tmpc = cinsent[i];
			if (tmpc >= '0' && tmpc <= '9') {
				pst = tmpc - '0';
			} else if (tmpc >= 'A' && tmpc <= 'F') {
				pst = tmpc - 'A' + 10;
			} else if (tmpc >= 'a' && tmpc <= 'f') {
				pst = tmpc - 'a' + 10;
			}
			outs->Checksum |= pst;

			// Verify checksum
			if (computedChecksum != outs->Checksum) {
				mtkgps_parse_errno = NMEA_PARSE_ERR_CHECKSUMS_DONT_MATCH;
				outs->Talker[0] = '\0';
				return NULL;
			}

			return outs;  // Successful; return NmeaSentence_t object.
		}
		i++;  // Advance past the '\0' terminator
		outs->dataField[didx] = cinsent+i;
	}
	if (didx == NMEA_SENTENCE_MAX_DATA_FIELDS) { // Invalid NMEA string - too many data fields!
		mtkgps_parse_errno = NMEA_PARSE_ERR_TOO_MANY_DATAFIELDS;
		outs->Talker[0] = '\0';
		return NULL;
	}
	return NULL;  // shouldn't ever get here
}

//! @brief Re-write pointer values within an NmeaSentence_t after copying via Mailbox
//! @details This is necessary after pulling an NmeaSentence_t from a Mailbox since the Mailbox_post/pend
//!          mechanism internally performs a buffer copy, thus rendering the pointer addresses invalid.
void mtkgps_repointer(NmeaSentence_t *s)
{
	int i;

	void *newHead = s;
	void *oldHead = s->headPointer;

	s->MessageID = (char *)((uint8_t *)newHead + ((uint8_t *)s->MessageID - (uint8_t *)oldHead));
	for (i=0; i < NMEA_SENTENCE_MAX_DATA_FIELDS; i++) {
		if (s->dataField[i] != NULL) {
			s->dataField[i] = (char *)((uint8_t *)newHead + ((uint8_t *)s->dataField[i] - (uint8_t *)oldHead));
		}
	}
	s->headPointer = newHead;
}

const char * mtkgps_parse_strerror(NmeaParseError_t pe)
{
	switch (pe) {
	case NMEA_PARSE_OK:
		return "OK";
	case NMEA_PARSE_ERR_TOO_SHORT:
		return "TOO_SHORT";
	case NMEA_PARSE_ERR_TOO_LONG:
		return "TOO_LONG";
	case NMEA_PARSE_ERR_NO_FREE_OBJECTS:
		return "NO_FREE_OBJECTS";
	case NMEA_PARSE_ERR_INVALID_STARTCHAR:
		return "INVALID_STARTCHAR";
	case NMEA_PARSE_ERR_UNKNOWN_TALKER:
		return "UNKNOWN_TALKER";
	case NMEA_PARSE_ERR_MISSING_ASTERISK:
		return "MISSING_ASTERISK";
	case NMEA_PARSE_ERR_MISSING_INITIAL_COMMA:
		return "MISSING_INITIAL_COMMA";
	case NMEA_PARSE_ERR_NO_CHECKSUM:
		return "NO_CHECKSUM";
	case NMEA_PARSE_ERR_CHECKSUMS_DONT_MATCH:
		return "CHECKSUMS_DONT_MATCH";
	case NMEA_PARSE_ERR_TOO_MANY_DATAFIELDS:
		return "TOO_MANY_DATAFIELDS";
	default:
		return "ERR_UNRECOGNIZED";
	}
}

size_t mtkgps_synthesize_sentence(NmeaSentence_t *nobj, char *buf)
{
	size_t len = 0, i = 0, j, k;
	uint8_t cksum = 0;

	buf[len] = '$';
	len++;
	if (nobj->parsedMessageID != 0 && nobj->parsedMessageID != NMEA_UNKNOWN) {
		i = 0;
		while (NmeaMap_TalkerID[i].StreamMsgID_Bitmask != 0) {
			if (NmeaMap_TalkerID[i].StreamMsgID_Bitmask & nobj->parsedMessageID) {
				j = 0;
				while (NmeaMap_TalkerID[i].TalkerID[j] != '\0') {
					buf[len] = NmeaMap_TalkerID[i].TalkerID[j];
					len++;
					j++;
				}
				// Locate the index corresponding to this bitmask
				for (j=0; j < 32; j++) {
					if (Bitmask_32[j] & nobj->parsedMessageID) {
						k = 0;
						while (NmeaMap_MessageID[j][k] != '\0') {
							buf[len] = NmeaMap_MessageID[j][k];
							k++;
							len++;
						}
						break;
					}
				}
				break;
			}
			i++;
		}
	} else {
		while (nobj->Talker[i] != '\0' && i < NMEA_SENTENCE_MAX_TALKER_ID_LENGTH) {
			buf[len] = nobj->Talker[i];
			len++;
			i++;
		}
		i = 0;
		while (nobj->MessageID[i] != '\0') {
			buf[len] = nobj->MessageID[i];
			len++;
			i++;
		}
	}
	for (j=0; j < NMEA_SENTENCE_MAX_DATA_FIELDS; j++) {
		if (nobj->dataField[j] == NULL) {
			break;
		}
		buf[len] = ',';
		len++;
		i = 0;
		while (nobj->dataField[j][i] != '\0') {
			buf[len] = nobj->dataField[j][i];
			len++;
			i++;
		}
	}

	// Compute checksum
	for (i=1; i < len; i++) {
		cksum ^= (uint8_t)buf[i];
	}
	buf[len] = '*';
	len++;
	j = cksum >> 4;
	if (j > 9) {
		buf[len] = 'A' + j - 10;
	} else {
		buf[len] = '0' + j;
	}
	len++;
	j = cksum & 0x0F;
	if (j > 9) {
		buf[len] = 'A' + j - 10;
	} else {
		buf[len] = '0' + j;
	}
	len++;
	buf[len] = '\0';

	return len;
}
