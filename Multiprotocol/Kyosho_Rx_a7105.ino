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

#define KYOSHO_RX_TXPACKET_SIZE	38	// FIFO length configured by the register table
#define KYOSHO_RX_NUMFREQ		32	// full hopping table, sent by the TX in 2 halves of 16
#define KYOSHO_RX_NUMCHAN		14

enum {
	KYOSHO_RX_BIND,
	KYOSHO_RX_DATA
};

static void __attribute__((unused)) KYOSHO_RX_build_telemetry_packet()
{
	uint32_t bits = 0;
	uint8_t bitsavailable = 0;
	uint8_t idx = 0;

	packet_in[idx++] = RX_LQI; // 0 - 130
	packet_in[idx++] = RX_RSSI;
	packet_in[idx++] = 0; // start channel
	packet_in[idx++] = KYOSHO_RX_NUMCHAN; // number of channels in packet
	// pack channels
	for (uint8_t i = 0; i < KYOSHO_RX_NUMCHAN; i++) {
		uint32_t val = packet[9+i*2] | (((packet[10+i*2])&0x0F) << 8);
		if (val < 860)
			val = 860;
		// convert ppm (860-2140) to Multi (0-2047)
		val = min(((val-860)<<3)/5, 2047);

		bits |= val << bitsavailable;
		bitsavailable += 11;
		while (bitsavailable >= 8) {
			packet_in[idx++] = bits & 0xff;
			bits >>= 8;
			bitsavailable -= 8;
		}
	}
}

static uint8_t KYOSHO_RX_hop_trust;	// 0xFF once the hop index embedded by the TX has been validated
static uint8_t KYOSHO_RX_missed;	// consecutive hops without a packet
static uint8_t KYOSHO_RX_next_ch;	// channel to hop to at the next hop, 0xFF if not known

static uint8_t __attribute__((unused)) KYOSHO_RX_data_ready()
{
	// check if FECF+CRCF Ok
	return !(A7105_ReadReg(A7105_00_MODE) & (1 << 5 | 1 << 6 | 1 << 0));
}

void KYOSHO_RX_init()
{
	uint8_t i;
	A7105_Init();
	// The Kyosho register table is a TX one, enable the automatic RSSI measurement needed on a RX
	A7105_WriteReg(A7105_01_MODE_CONTROL, 0x42 | (1<<5));
	hopping_frequency_no = 0;
	packet_count = 0;
	KYOSHO_RX_hop_trust = 0;
	KYOSHO_RX_missed = 0;
	KYOSHO_RX_next_ch = 0xFF;
	rx_data_started = false;
	rx_disable_lna = IS_POWER_FLAG_on;
	A7105_SetTxRxMode(rx_disable_lna ? TXRX_OFF : RX_EN);
	A7105_Strobe(A7105_RX);

	if (IS_BIND_IN_PROGRESS) {
		packet_sent = 0;			// halves of the RF table already received
		phase = KYOSHO_RX_BIND;
	}
	else {
		uint16_t temp = KYOSHO_RX_EEPROM_OFFSET;
		for (i = 0; i < 4; i++)
			rx_id[i] = eeprom_read_byte((EE_ADDR)temp++);
		for (i = 0; i < KYOSHO_RX_NUMFREQ; i++)
			hopping_frequency[i] = eeprom_read_byte((EE_ADDR)temp++);
		phase = KYOSHO_RX_DATA;
	}
}

uint16_t KYOSHO_RX_callback()
{
	static int8_t read_retry;
	uint16_t temp;
	uint8_t i;

#ifndef FORCE_KYOSHO_TUNING
	A7105_AdjustLOBaseFreq(1);
#endif
	if (rx_disable_lna != IS_POWER_FLAG_on) {
		rx_disable_lna = IS_POWER_FLAG_on;
		A7105_SetTxRxMode(rx_disable_lna ? TXRX_OFF : RX_EN);
	}

	switch(phase) {
	case KYOSHO_RX_BIND:
		if(IS_BIND_DONE)
		{
			KYOSHO_RX_init();	// Abort bind
			break;
		}
		if (KYOSHO_RX_data_ready()) {
			A7105_ReadData(KYOSHO_RX_TXPACKET_SIZE);
			// bind packet: BC, TX ID, FF FF FF FF, RF table half, 00, 16 RF channels, TX type
			if (packet[0] == 0xBC && packet[9] <= 0x01 && packet[10] == 0x00
				&& (packet[27] == 0x05 || packet[27] == 0x07))	// FHSS is 5 and Syncro is 7
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
		A7105_WriteReg(A7105_0F_PLL_I, (packet_count++ & 1) ? 0x0D : 0x8C); // bind channels
		A7105_Strobe(A7105_RX);
		return 10000;

	case KYOSHO_RX_DATA:
		if (KYOSHO_RX_data_ready()) {
			A7105_ReadData(KYOSHO_RX_TXPACKET_SIZE);
			if (memcmp(&packet[1], rx_id, 4) == 0)
			{
				#if 0
					for(uint8_t i=0;i<KYOSHO_RX_TXPACKET_SIZE;i++)
						debug(" %02X",packet[i]);
					debugln("");
				#endif
				if (packet[0] == 0x58)
				{ // standard packet, send channels to TX
					if ((telemetry_link&0x7F) == 0)
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
					// The TX broadcasts the index of its next hop in the high bits of the last 2 channels,
					// which are always free since a channel value never exceeds 2140 (0x85C).
					// Following it keeps the RX locked whatever the TX packet period and jitter are.
					temp = ((packet[34] >> 4) | (packet[36] & 0xF0)) & (KYOSHO_RX_NUMFREQ - 1);
					if (KYOSHO_RX_hop_trust != 0xFF)
					{ // only trust that index once it has been seen tracking our own hopping
						if (temp == (uint16_t)((hopping_frequency_no + 1) & (KYOSHO_RX_NUMFREQ - 1)))
						{
							if (++KYOSHO_RX_hop_trust >= 20)
								KYOSHO_RX_hop_trust = 0xFF;
						}
						else
							KYOSHO_RX_hop_trust = 0;	// not a hop index on this TX, keep free running
					}
					if (KYOSHO_RX_hop_trust == 0xFF)
						KYOSHO_RX_next_ch = temp;		// exact resync on the TX
					else
						KYOSHO_RX_next_ch = (hopping_frequency_no + 1) & (KYOSHO_RX_NUMFREQ - 1);
					KYOSHO_RX_missed = 0;
				}
				rx_data_started = true;
				read_retry = 10;	// hop to the next channel on the next tick
				pps_counter++;
			}
		}

		// packets per second
		if (millis() - pps_timer >= 1000) {
			pps_timer = millis();
			debugln("%d pps, hop trust %d", pps_counter, KYOSHO_RX_hop_trust);
			RX_LQI = pps_counter / 2;
			pps_counter = 0;
		}

		// frequency hopping
		if (read_retry++ >= 10) {
			if (KYOSHO_RX_next_ch != 0xFF)
			{ // channel announced by the last packet received
				hopping_frequency_no = KYOSHO_RX_next_ch;
				KYOSHO_RX_next_ch = 0xFF;
			}
			else
				hopping_frequency_no = (hopping_frequency_no + 1) & (KYOSHO_RX_NUMFREQ - 1);
			A7105_WriteReg(A7105_0F_PLL_I, hopping_frequency[hopping_frequency_no]);
			A7105_Strobe(A7105_RX);
			if (rx_data_started && ++KYOSHO_RX_missed < KYOSHO_RX_NUMFREQ * 8)
				read_retry = 0;
			else
			{ // nothing for 8 full passes over the table, the free running hop clock has drifted away
				if (rx_data_started)
				{
					debugln("lost");
				}
				rx_data_started = false;
				KYOSHO_RX_missed = 0;
				read_retry = -127; // dwell on each channel until a packet is catched again
			}
		}
		return 385;
	}
	return 3852; // never reached
}

#endif
