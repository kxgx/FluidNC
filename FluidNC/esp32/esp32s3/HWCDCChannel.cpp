// Copyright (c) 2023 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "HWCDCChannel.h"

#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT

#include "Machine/MachineConfig.h"  // config
#include "Serial.h"                 // allChannels
#include <HWCDC.h>

// There are two compiler flags that control how the Arduino framework predefines USB CDC serial class instances.
// If ARDUINO_USB_MODE is 1, the TinyUSB variant is preferred, and
//   If ARDUINO_USB_CDC_ON_BOOT it true
//       Serial is a USBCDC (tinyusb)
//       Serial0 is a HardwareSerial (UART)
// Else (ARDUINO_USB_MODE is 0 or undefined)), the hardware CDC is preferred, and
//   If ARDUINO_USB_CDC_ON_BOOT it true
//       Serial is a HWCDC (hardware)
//       Serial0 is a HardwareSerial (UART)
//   Else
//       Serial is a HardwareSerial (UART)
//       USBSerial is a HWCDC (hardware)


HWCDCChannel::HWCDCChannel(bool addCR) : Channel("usbcdc", addCR), _cdc(HWCDCSerial) {
    _lineedit = new Lineedit(this, _line, Channel::maxLine - 1);
}

void HWCDCChannel::init() {
    _cdc.setRxBufferSize(1040);  // 1K + change
    _cdc.begin(115200);
    delay_ms(300);  // Time for the host USB to reconnect
    allChannels.registration(this);
}

size_t HWCDCChannel::write(uint8_t c) {
    return _cdc.write(c);
}

size_t HWCDCChannel::write(const uint8_t* buffer, size_t length) {
    // Replace \n with \r\n
    if (_addCR) {
        size_t rem      = length;
        char   lastchar = '\0';
        size_t j        = 0;
        while (rem) {
            const int bufsize = 80;
            uint8_t   modbuf[bufsize];
            // bufsize-1 in case the last character is \n
            size_t k = 0;
            while (rem && k < (bufsize - 1)) {
                char c = buffer[j++];
                if (c == '\n' && lastchar != '\r') {
                    modbuf[k++] = '\r';
                }
                lastchar    = c;
                modbuf[k++] = c;
                --rem;
            }
            size_t actual = _cdc.write(modbuf, k);
            if (actual != k) {
                // ::printf("dropped %d\n", k - actual);
            }
        }
        return length;
    }
    size_t actual = _cdc.write(buffer, length);
    if (actual != length) {
        // ::printf("dropped %d\n", length - actual);
    }
    return actual;
}

int HWCDCChannel::available() {
    return _cdc.available();
}

int HWCDCChannel::peek() {
    return _cdc.peek();
}

int HWCDCChannel::rx_buffer_available() {
    return 64 - _cdc.available();
}

bool HWCDCChannel::realtimeOkay(char c) {
    return _lineedit->realtime(c);
}

bool HWCDCChannel::lineComplete(char* line, char c) {
    if (_lineedit->step(c)) {
        _linelen        = _lineedit->finish();
        _line[_linelen] = '\0';
        strcpy(line, _line);
        _linelen = 0;
        return true;
    }
    return false;
}

Error HWCDCChannel::pollLine(char* line) {
    if (_lineedit == nullptr) {
        return Error::NoData;
    }
    return Channel::pollLine(line);
}

int HWCDCChannel::read() {
    return _cdc.read();
}

void HWCDCChannel::flushRx() {
    Channel::flushRx();
}

extern Channel& Console;

size_t HWCDCChannel::timedReadBytes(char* buffer, size_t length, TickType_t timeout) {
    size_t remlen = length;

    // It is likely that _queue will be empty because timedReadBytes() is only
    // used in situations where the UART is not receiving GCode commands
    // and Grbl realtime characters.
    while (remlen && _queue.size()) {
        *buffer++ = _queue.front();
        _queue.pop();
        --remlen;
    }

    // The Arduino framework does not expose a timed read function
    // for USBCDC so we have to do the timeout the hard way
    while (remlen && timeout) {
        int thislen = _cdc.read(buffer, remlen);
        if (thislen < 0) {
            // Error
            return 0;
        }
        buffer += thislen;
        remlen -= thislen;
        if (remlen) {
            delay_ms(1);
            timeout -= 1;
        }
    }

    return length - remlen;
}

#endif
