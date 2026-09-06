/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(KYOSHO_RX_A7105_INO)

#include "iface_a7105.h"

// Derived from the transmit side in Kyosho_a7105.ino, FHSS/Syncro path:
//   - calc_fh_channels(32) and hopping_frequency_no++ once per packet, so the TX walks its
//     32 entry table one channel at a time, one packet per channel
//   - packet_period 3852us, KYOSHO_FORCE_ID_FHSS carries a real captured ID and table
//   - A7105_WriteData(37, ...) while the register table sets FIFO length 0x25, so 38 bytes go
//     over the air and byte 37 is stale FIFO content
//   - normal packet 0x58: TX id in 1..4, 0xFF in 5..8, channels from byte 9, and the index of
//     the *next* hop stuffed into the spare high bits of bytes 34 and 36
//   - bind packet 0xBC: half the RF table per packet, half selected by byte 9, sent on 0x0D for
//     the low half and 0x8C for the high half, type 0x05 or 0x07 alternating in byte 27
//
// The TX therefore announces both where and when its next packet will be. This receiver holds a
// schedule against that timeline instead of polling on a free running grid: it arms on the
// announced channel immediately, sleeps until the packet is nearly due, then watches finely
// across the arrival window. A miss costs exactly one channel and one period, so it stays on the
// TX schedule rather than drifting away from it.

#define KYOSHO_RX_PACKET_SIZE	38		// what the register table FIFO length puts on the air
#define KYOSHO_RX_NUMFREQ		32		// calc_fh_channels(32)
#define KYOSHO_RX_NUMCHAN		13		// bytes 9..34, byte 35/36 is a trailer and not a channel
#define KYOSHO_RX_PERIOD		3852	// packet_period on the TX
#define KYOSHO_RX_EARLY			600		// start watching this long before the packet is due
#define KYOSHO_RX_LATE			1000	// give up on it this long after
#define KYOSHO_RX_POLL			300		// how finely to watch inside that window
#define KYOSHO_RX_SEARCH_POLL	500		// polling step while hunting for the TX
#define KYOSHO_RX_SEARCH_DWELL	300		// ~150ms per channel, over one full pass of the table
#define KYOSHO_RX_LOST			64		// consecutive misses before dropping back to searching

enum {
	KYOSHO_RX_BIND,
	KYOSHO_RX_SEARCH,		// not locked: sit on a channel long enough for the TX to come past
	KYOSHO_RX_TRACK			// locked: follow the hop index the TX announces
};

// The callback is not a reliable clock: when the main loop blocks on telemetry the scheduler
// runs late and fires several callbacks back to back (Short CB in the debug log). Counting them
// would collapse the arrival window, so the schedule is held against micros() instead.
static uint32_t KYOSHO_RX_due;		// when the next packet is expected
static uint8_t KYOSHO_RX_misses;	// consecutive packets not seen
static uint16_t KYOSHO_RX_dwell;	// polls done on the current channel while searching
static uint8_t KYOSHO_RX_hop_ok;	// 0xFF once the announced hop index has proven itself
static uint8_t KYOSHO_RX_last_idx;	// hop index announced by the previous packet caught
static uint16_t KYOSHO_RX_raw, KYOSHO_RX_bad, KYOSHO_RX_d1, KYOSHO_RX_d2, KYOSHO_RX_dx;

static void __attribute__((unused)) KYOSHO_RX_build_telemetry_packet()
{
	uint32_t bits = 0;
	uint8_t bitsavailable = 0;
	uint8_t idx = 0;

	packet_in[idx++] = RX_LQI; // 0 - 130
	packet_in[idx++] = RX_RSSI;
	packet_in[idx++] = 0; // start channel
	packet_in[idx++] = KYOSHO_RX_NUMCHAN; // number of channels in packet
	for (uint8_t i = 0; i < KYOSHO_RX_NUMCHAN; i++) {
		// convert_channel_ppm() on the TX, so 860-2140 with the high nibble left free
		uint32_t val = packet[9+i*2] | (((packet[10+i*2])&0x0F) << 8);
		if (val < 860)
			val = 860;
		val = min(((val-860)<<3)/5, 2047);	// to Multi 0-2047

		bits |= val << bitsavailable;
		bitsavailable += 11;
		while (bitsavailable >= 8) {
			packet_in[idx++] = bits & 0xff;
			bits >>= 8;
			bitsavailable -= 8;
		}
	}
}

// Sleep until the arrival window opens, never less than one poll and never past one period
static uint16_t __attribute__((unused)) KYOSHO_RX_sleep()
{
	int32_t t = (int32_t)(KYOSHO_RX_due - KYOSHO_RX_EARLY - micros());
	if (t < KYOSHO_RX_POLL)
		return KYOSHO_RX_POLL;
	if (t > KYOSHO_RX_PERIOD)
		return KYOSHO_RX_PERIOD;
	return t;
}

// Tune to an entry of the RF table and start listening on it
static void __attribute__((unused)) KYOSHO_RX_listen(uint8_t idx)
{
	hopping_frequency_no = idx;
	A7105_WriteReg(A7105_0F_PLL_I, hopping_frequency[idx]);
	A7105_Strobe(A7105_RX);
}

// 0 = nothing yet, 1 = good packet waiting in the FIFO, 2 = one arrived but was corrupted
static uint8_t __attribute__((unused)) KYOSHO_RX_check()
{
	uint8_t mode = A7105_ReadReg(A7105_00_MODE);
	if (mode & 0x01)
		return 0;					// still armed, nothing has come in
	if (mode & (1 << 5 | 1 << 6))
	{ // CRC or FEC failed, the A7105 goes idle so put it straight back to listening
		KYOSHO_RX_bad++;
		A7105_Strobe(A7105_RX);
		return 2;
	}
	return 1;
}

// Consume a packet from the FIFO while locked. Returns 1 if it was one of ours.
static uint8_t __attribute__((unused)) KYOSHO_RX_read()
{
	uint8_t next, d;

	A7105_ReadData(KYOSHO_RX_PACKET_SIZE);
	KYOSHO_RX_raw++;
	if (memcmp(&packet[1], rx_id, 4) != 0 || packet[0] != 0x58)
	{ // not ours, keep listening on this channel
		A7105_Strobe(A7105_RX);
		return 0;
	}

	if ((telemetry_link & 0x7F) == 0)
	{
		int rssi = min(A7105_ReadReg(A7105_1D_RSSI_THOLD),160);
		RX_RSSI = map16b(rssi, 160, 8, 0, 128);
		KYOSHO_RX_build_telemetry_packet();
		telemetry_link = 1;
		#ifdef SEND_CPPM
			if(sub_protocol>0)
				telemetry_link |= 0x80;	// Disable telemetry output
		#endif
	}

	// Index of the next hop, from the spare high bits of the last 2 channel slots
	next = ((packet[34] >> 4) | (packet[36] & 0xF0)) & (KYOSHO_RX_NUMFREQ - 1);
	if (KYOSHO_RX_last_idx != 0xFF)
	{ // how far did the TX move between the 2 packets we caught?
		d = (next - KYOSHO_RX_last_idx) & (KYOSHO_RX_NUMFREQ - 1);
		if (d == 1)			KYOSHO_RX_d1++;
		else if (d == 2)	KYOSHO_RX_d2++;
		else				KYOSHO_RX_dx++;
	}
	KYOSHO_RX_last_idx = next;

	if (KYOSHO_RX_hop_ok != 0xFF)
	{ // only follow the announced index once it has been seen tracking our own hopping
		if (next == ((hopping_frequency_no + 1) & (KYOSHO_RX_NUMFREQ - 1)))
		{
			if (++KYOSHO_RX_hop_ok >= 20)
				KYOSHO_RX_hop_ok = 0xFF;
		}
		else
			KYOSHO_RX_hop_ok = 0;
		next = (hopping_frequency_no + 1) & (KYOSHO_RX_NUMFREQ - 1);
	}

	KYOSHO_RX_listen(next);			// arm on the next channel now, the TX is heading there
	KYOSHO_RX_due = micros() + KYOSHO_RX_PERIOD;
	rx_data_started = true;
	KYOSHO_RX_misses = 0;
	pps_counter++;
	return 1;
}

void KYOSHO_RX_init()
{
	uint8_t i;
	A7105_Init();
	// The Kyosho register table is a TX one, enable the automatic RSSI measurement needed on a RX
	A7105_WriteReg(A7105_01_MODE_CONTROL, 0x42 | (1<<5));
	packet_count = 0;
	KYOSHO_RX_due = micros();
	KYOSHO_RX_misses = 0;
	KYOSHO_RX_dwell = 0;
	KYOSHO_RX_hop_ok = 0;
	KYOSHO_RX_last_idx = 0xFF;
	KYOSHO_RX_raw = KYOSHO_RX_bad = KYOSHO_RX_d1 = KYOSHO_RX_d2 = KYOSHO_RX_dx = 0;
	rx_data_started = false;
	rx_disable_lna = IS_POWER_FLAG_on;
	A7105_SetTxRxMode(rx_disable_lna ? TXRX_OFF : RX_EN);

	if (IS_BIND_IN_PROGRESS) {
		packet_sent = 0;		// halves of the RF table received so far
		phase = KYOSHO_RX_BIND;
		A7105_Strobe(A7105_RX);
	}
	else {
		uint16_t temp = KYOSHO_RX_EEPROM_OFFSET;
		for (i = 0; i < 4; i++)
			rx_id[i] = eeprom_read_byte((EE_ADDR)temp++);
		for (i = 0; i < KYOSHO_RX_NUMFREQ; i++)
			hopping_frequency[i] = eeprom_read_byte((EE_ADDR)temp++);
		phase = KYOSHO_RX_SEARCH;
		KYOSHO_RX_listen(0);
	}
}

uint16_t KYOSHO_RX_callback()
{
	uint16_t temp;
	uint8_t i;

#ifndef FORCE_KYOSHO_TUNING
	A7105_AdjustLOBaseFreq(1);
#endif
	if (rx_disable_lna != IS_POWER_FLAG_on) {
		rx_disable_lna = IS_POWER_FLAG_on;
		A7105_SetTxRxMode(rx_disable_lna ? TXRX_OFF : RX_EN);
	}

	// packets per second, and how far the TX hop index moved between the packets we caught
	if (phase != KYOSHO_RX_BIND && millis() - pps_timer >= 1000) {
		pps_timer = millis();
		debugln("%d pps, %d raw, %d bad, hop %d, d1 %d d2 %d dx %d", pps_counter, KYOSHO_RX_raw, KYOSHO_RX_bad, KYOSHO_RX_hop_ok, KYOSHO_RX_d1, KYOSHO_RX_d2, KYOSHO_RX_dx);
		RX_LQI = pps_counter / 2;		// a healthy link is 1000000/3852 = 260 packets per second
		pps_counter = 0;
		KYOSHO_RX_raw = KYOSHO_RX_bad = KYOSHO_RX_d1 = KYOSHO_RX_d2 = KYOSHO_RX_dx = 0;
	}

	switch(phase) {
	case KYOSHO_RX_BIND:
		if(IS_BIND_DONE)
		{
			KYOSHO_RX_init();	// Abort bind
			break;
		}
		if (KYOSHO_RX_check() == 1) {
			A7105_ReadData(KYOSHO_RX_PACKET_SIZE);
			// BC, TX id, FF FF FF FF, table half, 00, 16 RF channels, 05 FHSS or 07 Syncro
			if (packet[0] == 0xBC && packet[9] <= 0x01 && packet[10] == 0x00
				&& (packet[27] == 0x05 || packet[27] == 0x07))
			{
				if (packet_sent && memcmp(rx_id, &packet[1], 4) != 0)
					packet_sent = 0;			// another TX started binding, restart from scratch
				memcpy(rx_id, &packet[1], 4);	// TX id actually
				memcpy(&hopping_frequency[packet[9]<<4], &packet[11], 16);
				packet_sent |= 1 << packet[9];
				debugln("bind half %d", packet[9]);
				if (packet_sent == 0x03)
				{ // both halves of the RF table received, save tx info to eeprom
					temp = KYOSHO_RX_EEPROM_OFFSET;
					for (i = 0; i < 4; i++)
						eeprom_write_byte((EE_ADDR)temp++, rx_id[i]);
					for (i = 0; i < KYOSHO_RX_NUMFREQ; i++)
						eeprom_write_byte((EE_ADDR)temp++, hopping_frequency[i]);
					debugln("done");
					BIND_DONE;
					KYOSHO_RX_init();	// Restart protocol
					break;
				}
			}
		}
		// the TX alternates halves packet by packet and sends each one on its own channel
		A7105_WriteReg(A7105_0F_PLL_I, (packet_count++ & 1) ? 0x0D : 0x8C);
		A7105_Strobe(A7105_RX);
		return 10000;

	case KYOSHO_RX_SEARCH:
		if (KYOSHO_RX_check() == 1 && KYOSHO_RX_read())
		{ // caught it, from here on we know where and when the next one will be
			phase = KYOSHO_RX_TRACK;
			return KYOSHO_RX_sleep();
		}
		if (++KYOSHO_RX_dwell >= KYOSHO_RX_SEARCH_DWELL)
		{ // the TX passes over every channel once per sweep of the table, so try the next one
			KYOSHO_RX_dwell = 0;
			KYOSHO_RX_listen((hopping_frequency_no + 1) & (KYOSHO_RX_NUMFREQ - 1));
		}
		return KYOSHO_RX_SEARCH_POLL;

	case KYOSHO_RX_TRACK:
		if (KYOSHO_RX_check() == 1 && KYOSHO_RX_read())
			return KYOSHO_RX_sleep();			// on schedule, wait for the next one
		if ((int32_t)(micros() - (KYOSHO_RX_due + KYOSHO_RX_LATE)) < 0)
			return KYOSHO_RX_POLL;				// still inside the arrival window

		// Missed one. The TX moves on by exactly one channel every period, so follow it there
		// instead of guessing: a lost packet then costs one packet, not the lock.
		KYOSHO_RX_due += KYOSHO_RX_PERIOD;
		if (++KYOSHO_RX_misses >= KYOSHO_RX_LOST)
		{
			debugln("lost");
			rx_data_started = false;
			KYOSHO_RX_misses = 0;
			KYOSHO_RX_last_idx = 0xFF;
			KYOSHO_RX_dwell = 0;
			phase = KYOSHO_RX_SEARCH;
			return KYOSHO_RX_SEARCH_POLL;
		}
		KYOSHO_RX_listen((hopping_frequency_no + 1) & (KYOSHO_RX_NUMFREQ - 1));
		return KYOSHO_RX_sleep();
	}
	return KYOSHO_RX_PERIOD; // never reached
}

#endif
