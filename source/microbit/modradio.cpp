/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Contains some code from MouseJack (developed by BastilleResearch)
 * see: https://github.com/BastilleResearch/nrf-research-firmware/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

extern "C" {

#include <stdio.h>
#include <string.h>

#include "py/runtime0.h"
#include "py/runtime.h"
#include "microbitobj.h"

#define RADIO_DEFAULT_MODE          (0)
#define RADIO_SNIFF_MODE            (1)
#define RADIO_DEFAULT_MAX_PAYLOAD   (32)
#define RADIO_DEFAULT_QUEUE_LEN     (3)
#define RADIO_DEFAULT_CHANNEL       (7)
#define RADIO_DEFAULT_POWER_DBM     (0)
#define RADIO_DEFAULT_BASE0         (0x75626974) // "uBit"
#define RADIO_DEFAULT_PREFIX0       (0)
#define RADIO_DEFAULT_DATA_RATE     (RADIO_MODE_MODE_Nrf_1Mbit)

typedef struct _radio_state_t {
    uint8_t mode;           // radio mode (tx/rx or sniff)
    uint8_t max_payload;    // 1-251 inclusive
    uint8_t queue_len;      // 1-254 inclusive
    uint8_t channel;        // 0-100 inclusive
    int8_t power_dbm;       // one of: -30, -20, -16, -12, -8, -4, 0, 4
    uint32_t base0;         // for BASE0 register
    uint8_t prefix0;        // for PREFIX0 register (lower 8 bits only)
    uint8_t data_rate;      // one of: RADIO_MODE_MODE_Nrf_{250Kbit,1Mbit,2Mbit}
} radio_state_t;

static radio_state_t radio_state;
static uint8_t *buf_end = NULL;
static uint8_t *rx_buf = NULL;
static uint8_t esb_ready = 0;

/* CRC16-CCITT with 1-8 bits from a given byte */
uint16_t crc_update(uint16_t crc, uint8_t byte, uint8_t bits)
{
    crc = crc ^ (byte << 8);
    while(bits--)
        if((crc & 0x8000) == 0x8000) crc = (crc << 1) ^ 0x1021;
        else crc = crc << 1;
    crc = crc & 0xFFFF;
    return crc;
}

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_READY) {
        NRF_RADIO->EVENTS_READY = 0;
        NRF_RADIO->TASKS_START = 1;
    }

    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;

        if (radio_state.mode == RADIO_DEFAULT_MODE) {
            size_t max_len = NRF_RADIO->PCNF1 & 0xff;
            size_t len = rx_buf[0];
            if (len > max_len) {
                len = max_len;
                rx_buf[0] = len;
            }

            //printf("radio end pos=%d len=%d [%d %d %d %d]\r\n", rx_buf - MP_STATE_PORT(radio_buf), len, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

            // if the CRC was valid then accept the packet
            if (NRF_RADIO->CRCSTATUS == 1) {
                //printf("rssi: %d\r\n", -NRF_RADIO->RSSISAMPLE);

                // only move the rx_buf pointer if there is enough room for another full packet
                if (rx_buf + 1 + len + 1 + max_len <= buf_end) {
                    rx_buf += 1 + len;
                    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
                }
            }
        } else {
            int x, offset;
            uint8_t payload_length;
            uint16_t crc, crc_given;
            uint8_t payload[37];

            /* Copy sniffed packet. */
            memcpy(payload, rx_buf, 32);

            // In promiscuous mode without a defined address prefix, we attempt to
            // decode the payload as-is, and then shift it by one bit and try again
            // if the first attempt did not pass the CRC check. The purpose of this
            // is to minimize missed detections that happen if we were to use both
            // 0xAA and 0x55 as the nonzero promiscuous mode address bytes.
            for(offset = 0; offset < 2; offset++) {
                // Shift the payload right by one bit if this is the second pass
                if(offset == 1) {
                    for(x = 31; x >= 0; x--) {
                        if(x > 0) payload[x] = payload[x - 1] << 7 | payload[x] >> 1;
                        else payload[x] = payload[x] >> 1;
                    }
                }

                // Read the payload length
                payload_length = payload[5] >> 2;

                // Check for a valid payload length, which is less than the usual 32 bytes
                // because we need to account for the packet header, CRC, and part or all
                // of the address bytes.
                if(payload_length <= (32-9)) {
                    // Read the given CRC
                    crc_given = (payload[6 + payload_length] << 9) | ((payload[7 + payload_length]) << 1);
                    crc_given = (crc_given << 8) | (crc_given >> 8);
                    if(payload[8 + payload_length] & 0x80) crc_given |= 0x100;

                    // Calculate the CRC
                    crc = 0xFFFF;
                    for(x = 0; x < 6 + payload_length; x++) crc = crc_update(crc, payload[x], 8);
                    crc = crc_update(crc, payload[6 + payload_length] & 0x80, 1);
                    crc = (crc << 8) | (crc >> 8);

                    /* CRC match ? */
                    if(crc == crc_given)
                    {
                        /* Kinda lock we use to avoid conflicts. */
                        esb_ready = 0;

                        /* Write packet size to rx_buf. */
                        rx_buf[0] = 0xC0 | payload_length;

                        /* Write address to rx_buf. */
                        memcpy(&rx_buf[1], payload, 5);

                        for(x = 0; x < payload_length + 3; x++)
                            rx_buf[6+x] = ((payload[6 + x] << 1) & 0xFF) | (payload[7 + x] >> 7);

                        // only move the rx_buf pointer if there is enough room for another full packet
                        if (rx_buf + 2*radio_state.max_payload <= buf_end) {
                            rx_buf += radio_state.max_payload;
                            NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
                        }

                        /* Data updated, ready to be read. */
                        esb_ready = 1;
                    }
                }
            }
        }
        NRF_RADIO->TASKS_START = 1;
    }
}

static void ensure_enabled(void) {
    if (MP_STATE_PORT(radio_buf) == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "radio is not enabled"));
    }
}

static void radio_disable(void) {
    NVIC_DisableIRQ(RADIO_IRQn);
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    // free any old buffers
    if (MP_STATE_PORT(radio_buf) != NULL) {
        m_del(uint8_t, MP_STATE_PORT(radio_buf), buf_end - MP_STATE_PORT(radio_buf));
        MP_STATE_PORT(radio_buf) = NULL;
    }
}

static void radio_enable(void) {
    radio_disable();

    // Enable default mode
    radio_state.mode = RADIO_DEFAULT_MODE;

    // allocate tx and rx buffers
    size_t max_payload = radio_state.max_payload + 1; // an extra byte to store the length
    size_t queue_len = radio_state.queue_len + 1; // one extra for tx buffer
    MP_STATE_PORT(radio_buf) = m_new(uint8_t, max_payload * queue_len);
    buf_end = MP_STATE_PORT(radio_buf) + max_payload * queue_len;
    rx_buf = MP_STATE_PORT(radio_buf) + max_payload; // start is tx buffer

    // Enable the High Frequency clock on the processor. This is a pre-requisite for
    // the RADIO module. Without this clock, no communication is possible.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
    NRF_RADIO->TXPOWER = radio_state.power_dbm;

    // should be between 0 and 100 inclusive (actual physical freq is 2400MHz + this register)
    NRF_RADIO->FREQUENCY = radio_state.channel;

    // configure data rate
    NRF_RADIO->MODE = radio_state.data_rate;

    // The radio supports filtering packets at the hardware level based on an address.
    // We use a 5-byte address comprised of 4 bytes (set by BALEN=4 below) from the BASEx
    // register, plus 1 byte from PREFIXm.APn.
    // The (x,m,n) values are selected by the logical address.  We use logical address 0
    // which means using BASE0 with PREFIX0.AP0.
    NRF_RADIO->BASE0 = radio_state.base0;
    NRF_RADIO->PREFIX0 = radio_state.prefix0;
    NRF_RADIO->TXADDRESS = 0; // transmit on logical address 0
    NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

    // LFLEN=8 bits, S0LEN=0, S1LEN=0
    NRF_RADIO->PCNF0 = 0x00000008;
    // STATLEN=0, BALEN=4, ENDIAN=0 (little), WHITEEN=1
    NRF_RADIO->PCNF1 = 0x02040000 | radio_state.max_payload;

    // Enable automatic 16bit CRC generation and checking, and configure how the CRC is calculated.
    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two;
    NRF_RADIO->CRCINIT = 0xFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    // Set the start random value of the data whitening algorithm. This can be any non zero number.
    NRF_RADIO->DATAWHITEIV = 0x18;

    // set receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;

    // configure interrupts
    NRF_RADIO->INTENSET = 0x00000008;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

    // enable receiver
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
}

/* Enable sniff mode based on Goodspeed's hack */
static void radio_enable_sniff(void) {
    radio_disable();

    // Sniff mode on
    radio_state.mode = RADIO_SNIFF_MODE;

    // allocate tx and rx buffers
    size_t max_payload = radio_state.max_payload; // an extra byte to store the length
    size_t queue_len = radio_state.queue_len; // one extra for tx buffer
    MP_STATE_PORT(radio_buf) = m_new(uint8_t, max_payload * queue_len);
    buf_end = MP_STATE_PORT(radio_buf) + max_payload * queue_len;
    rx_buf = MP_STATE_PORT(radio_buf) + max_payload; // start is tx buffer

    // Enable the High Frequency clock on the processor. This is a pre-requisite for
    // the RADIO module. Without this clock, no communication is possible.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
    NRF_RADIO->TXPOWER = radio_state.power_dbm;

    // should be between 0 and 100 inclusive (actual physical freq is 2400MHz + this register)
    NRF_RADIO->FREQUENCY = radio_state.channel;

    // configure data rate
    NRF_RADIO->MODE = radio_state.data_rate;

    // The radio supports filtering packets at the hardware level based on an address.
    // We use a 5-byte address comprised of 4 bytes (set by BALEN=4 below) from the BASEx
    // register, plus 1 byte from PREFIXm.APn.
    // The (x,m,n) values are selected by the logical address.  We use logical address 0
    // which means using BASE0 with PREFIX0.AP0.
    NRF_RADIO->BASE0 = 0x00000000;
    NRF_RADIO->PREFIX0 = 0x55; // preamble

    // LFLEN=0 bits, S0LEN=0, S1LEN=0
    NRF_RADIO->PCNF0 = 0x00000000;
    // STATLEN=0, BALEN=0, ENDIAN=0 (little), WHITEEN=0
    NRF_RADIO->PCNF1 = 0x01012020;

    // Disable CRC
    NRF_RADIO->CRCCNF = 0x0;
    NRF_RADIO->CRCINIT = 0xFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    // set receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;

    // configure interrupts
    NRF_RADIO->INTENSET = 0x00000008;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    //NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
    NRF_RADIO->SHORTS = 0;

    // enable receiver
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
}

void radio_send(const void *buf, size_t len, const void *buf2, size_t len2) {
    ensure_enabled();

    // construct the packet
    // note: we must send from RAM
    size_t max_len = NRF_RADIO->PCNF1 & 0xff;
    if (len + len2 > max_len) {
        if (len > max_len) {
            len = max_len;
            len2 = 0;
        } else {
            len2 = max_len - len;
        }
    }
    MP_STATE_PORT(radio_buf)[0] = len + len2;
    memcpy(MP_STATE_PORT(radio_buf) + 1, buf, len);
    if (len2 != 0) {
        memcpy(MP_STATE_PORT(radio_buf) + 1 + len, buf2, len2);
    }

    // transmission will occur synchronously
    NVIC_DisableIRQ(RADIO_IRQn);

    // Turn off the transceiver.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);

    // Configure the radio to send the buffer provided.
    NRF_RADIO->PACKETPTR = (uint32_t)MP_STATE_PORT(radio_buf);

    // Turn on the transmitter, and wait for it to signal that it's ready to use.
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    // Start transmission and wait for end of packet.
    NRF_RADIO->TASKS_START = 1;
    NRF_RADIO->EVENTS_END = 0;
    while (NRF_RADIO->EVENTS_END == 0);

    // Return the radio to using the default receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;

    // Turn off the transmitter.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);

    // Start listening for the next packet
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
}

static mp_obj_t radio_receive(bool typed_packet) {
    ensure_enabled();

    // disable the radio irq while we receive the packet
    NVIC_DisableIRQ(RADIO_IRQn);

    // get the pointer to the next packet
    uint8_t *buf = MP_STATE_PORT(radio_buf) + (NRF_RADIO->PCNF1 & 0xff) + 1; // skip tx buf

    // return None if there are no packets waiting
    if (rx_buf == buf) {
        NVIC_EnableIRQ(RADIO_IRQn);
        return mp_const_none;
    }

    // convert the packet data into a Python object
    size_t len = buf[0];
    mp_obj_t ret;
    if (!typed_packet) {
        ret = mp_obj_new_bytes(buf + 1, len); // if it raises the radio irq remains disabled...
    } else if (len >= 3 && buf[1] == 1 && buf[2] == 0 && buf[3] == 1) {
        ret = mp_obj_new_str((char*)buf + 4, len - 3, false); // if it raises the radio irq remains disabled...
    } else {
        NVIC_EnableIRQ(RADIO_IRQn);
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "received packet is not a string"));
    }

    // copy the rest of the packets down and restart the radio
    memmove(buf, buf + 1 + len, rx_buf - (buf + 1 + len));
    rx_buf -= 1 + len;
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
    NVIC_EnableIRQ(RADIO_IRQn);

    return ret;
}

static mp_obj_t radio_sniff(void) {
    ensure_enabled();

    // disable the radio irq while we receive the packet
    NVIC_DisableIRQ(RADIO_IRQn);

    // get the pointer to the next packet
    uint8_t *buf = MP_STATE_PORT(radio_buf) + (NRF_RADIO->PCNF1 & 0xff) ; // skip tx buf

    // must wait (writing in progress ?)
    if (esb_ready == 0) {
        NVIC_EnableIRQ(RADIO_IRQn);
        return mp_const_none;
    }

    // return None if there are no packets waiting
    if (rx_buf == buf) {
        NVIC_EnableIRQ(RADIO_IRQn);
        return mp_const_none;
    }

    /* Wrong packet format */
    if ((buf[0] & 0xC0) != 0xC0) {
        NVIC_EnableIRQ(RADIO_IRQn);
        return mp_const_none;
    }


    size_t len = buf[0]&0x3F;
    if (len > (32-6))
        len = (32-6);
    mp_obj_t ret;
    ret = mp_obj_new_bytes(&buf[1], len + 5); // if it raises the radio irq remains disabled...

    // copy the rest of the packets down and restart the radio
    memmove(buf, buf + radio_state.max_payload, rx_buf - (buf + radio_state.max_payload));
    rx_buf -= radio_state.max_payload;
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
    NVIC_EnableIRQ(RADIO_IRQn);

    return ret;
}


/*****************************************************************************/
// MicroPython bindings and module

STATIC mp_obj_t mod_radio_reset(void) {
    radio_state.mode = RADIO_DEFAULT_MODE;
    radio_state.max_payload = RADIO_DEFAULT_MAX_PAYLOAD;
    radio_state.queue_len = RADIO_DEFAULT_QUEUE_LEN;
    radio_state.channel = RADIO_DEFAULT_CHANNEL;
    radio_state.power_dbm = RADIO_DEFAULT_POWER_DBM;
    radio_state.base0 = RADIO_DEFAULT_BASE0;
    radio_state.prefix0 = RADIO_DEFAULT_PREFIX0;
    radio_state.data_rate = RADIO_DEFAULT_DATA_RATE;
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_reset_obj, mod_radio_reset);

STATIC mp_obj_t mod_radio_config(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    (void)pos_args; // unused

    if (n_args != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_TypeError, "arguments must be keyword arguments"));
    }

    // make a copy of the radio state so we don't change anything if there are value errors
    radio_state_t new_state = radio_state;

    qstr arg_name = MP_QSTR_;
    for (size_t i = 0; i < kw_args->alloc; ++i) {
        if (MP_MAP_SLOT_IS_FILLED(kw_args, i)) {
            mp_int_t value = mp_obj_get_int_truncated(kw_args->table[i].value);
            arg_name = mp_obj_str_get_qstr(kw_args->table[i].key);
            switch (arg_name) {
                case MP_QSTR_length:
                    if (!(1 <= value && value <= 251)) {
                        goto value_error;
                    }
                    new_state.max_payload = value;
                    break;

                case MP_QSTR_queue:
                    if (!(1 <= value && value <= 254)) {
                        goto value_error;
                    }
                    new_state.queue_len = value;
                    break;

                case MP_QSTR_channel:
                    if (!(0 <= value && value <= 100)) {
                        goto value_error;
                    }
                    new_state.channel = value;
                    break;

                case MP_QSTR_power: {
                    if (!(0 <= value && value <= 7)) {
                        goto value_error;
                    }
                    static int8_t power_dbm_table[8] = {-30, -20, -16, -12, -8, -4, 0, 4};
                    new_state.power_dbm = power_dbm_table[value];
                    break;
                }

                case MP_QSTR_data_rate:
                    if (!(value == RADIO_MODE_MODE_Nrf_250Kbit
                        || value == RADIO_MODE_MODE_Nrf_1Mbit
                        || value == RADIO_MODE_MODE_Nrf_2Mbit)) {
                        goto value_error;
                    }
                    new_state.data_rate = value;
                    break;

                case MP_QSTR_address:
                    new_state.base0 = value;
                    break;

                case MP_QSTR_group:
                    if (!(0 <= value && value <= 255)) {
                        goto value_error;
                    }
                    new_state.prefix0 = value;
                    break;

                default:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "unknown argument '%q'", arg_name));
                    break;
            }
        }
    }

    // reconfigure the radio with the new state

    if (MP_STATE_PORT(radio_buf) == NULL) {
        // radio disabled, just copy state
        radio_state = new_state;
    } else {
        // radio eabled
        if (new_state.max_payload != radio_state.max_payload || new_state.queue_len != radio_state.queue_len) {
            // tx/rx buffer size changed which requires reallocating the buffers
            radio_disable();
            radio_state = new_state;
            radio_enable();
        } else {
            // only registers changed so make the changes go through efficiently

            // disable radio
            NVIC_DisableIRQ(RADIO_IRQn);
            NRF_RADIO->EVENTS_DISABLED = 0;
            NRF_RADIO->TASKS_DISABLE = 1;
            while (NRF_RADIO->EVENTS_DISABLED == 0);

            // change state
            radio_state = new_state;
            NRF_RADIO->TXPOWER = radio_state.power_dbm;
            NRF_RADIO->FREQUENCY = radio_state.channel;
            NRF_RADIO->MODE = radio_state.data_rate;

            /* Update BASE0 and PREFIX0 if required, except when sniffing. */
            if (radio_state.mode == RADIO_DEFAULT_MODE) {
                NRF_RADIO->BASE0 = radio_state.base0;
                NRF_RADIO->PREFIX0 = radio_state.prefix0;
            } else {
                /* Specific values used for sniffing. */
                NRF_RADIO->BASE0 = 0x00000000;
                NRF_RADIO->PREFIX0 = 0x55;
            }

            // need to set RXEN for FREQUENCY decision point
            NRF_RADIO->EVENTS_READY = 0;
            NRF_RADIO->TASKS_RXEN = 1;
            while (NRF_RADIO->EVENTS_READY == 0);

            // need to set START for BASE0 and PREFIX0 decision point
            NRF_RADIO->EVENTS_END = 0;
            NRF_RADIO->TASKS_START = 1;

            NVIC_ClearPendingIRQ(RADIO_IRQn);
            NVIC_EnableIRQ(RADIO_IRQn);
        }
    }

    return mp_const_none;

value_error:
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "value out of range for argument '%q'", arg_name));
}
MP_DEFINE_CONST_FUN_OBJ_KW(mod_radio_config_obj, 0, mod_radio_config);

STATIC mp_obj_t mod_radio_on(void) {
    radio_enable();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_on_obj, mod_radio_on);

STATIC mp_obj_t mod_radio_sniff_on(void) {
    radio_enable_sniff();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_sniff_on_obj, mod_radio_sniff_on);


STATIC mp_obj_t mod_radio_off(void) {
    radio_disable();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_off_obj, mod_radio_off);

STATIC mp_obj_t mod_radio_send_bytes(mp_obj_t buf_in) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);
    radio_send(bufinfo.buf, bufinfo.len, NULL, 0);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_send_bytes_obj, mod_radio_send_bytes);

STATIC mp_obj_t mod_radio_receive_bytes(void) {
    return radio_receive(false);
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_receive_bytes_obj, mod_radio_receive_bytes);

STATIC mp_obj_t mod_radio_send(mp_obj_t buf_in) {
    mp_uint_t len;
    const char *data = mp_obj_str_get_data(buf_in, &len);
    radio_send("\x01\x00\x01", 3, data, len);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_send_obj, mod_radio_send);

STATIC mp_obj_t mod_radio_receive(void) {
    return radio_receive(true);
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_receive_obj, mod_radio_receive);

STATIC mp_obj_t mod_radio_sniff(void) {
    return radio_sniff();
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_sniff_obj, mod_radio_sniff);


STATIC const mp_map_elem_t radio_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_radio) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___init__), (mp_obj_t)&mod_radio_reset_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_reset), (mp_obj_t)&mod_radio_reset_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_config), (mp_obj_t)&mod_radio_config_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_on), (mp_obj_t)&mod_radio_on_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sniff_on), (mp_obj_t)&mod_radio_sniff_on_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_off), (mp_obj_t)&mod_radio_off_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_send_bytes), (mp_obj_t)&mod_radio_send_bytes_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_receive_bytes), (mp_obj_t)&mod_radio_receive_bytes_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_send), (mp_obj_t)&mod_radio_send_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_receive), (mp_obj_t)&mod_radio_receive_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sniff), (mp_obj_t)&mod_radio_sniff_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_RATE_250KBIT), MP_OBJ_NEW_SMALL_INT(RADIO_MODE_MODE_Nrf_250Kbit) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_RATE_1MBIT), MP_OBJ_NEW_SMALL_INT(RADIO_MODE_MODE_Nrf_1Mbit) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_RATE_2MBIT), MP_OBJ_NEW_SMALL_INT(RADIO_MODE_MODE_Nrf_2Mbit) },
};

STATIC MP_DEFINE_CONST_DICT(radio_module_globals, radio_module_globals_table);

const mp_obj_module_t radio_module = {
    .base = { &mp_type_module },
    .name = MP_QSTR_radio,
    .globals = (mp_obj_dict_t*)&radio_module_globals,
};

}
