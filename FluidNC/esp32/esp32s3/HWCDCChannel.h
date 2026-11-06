#pragma once

#include <sdkconfig.h>
#include <HWCDC.h>

#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT

#include "Channel.h"
#include "lineedit.h"

class HWCDCChannel : public Channel {
private:
    Lineedit* _lineedit;
    HWCDC&    _cdc;

public:
    HWCDCChannel(bool addCR = false);

    void init() override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buf, size_t len) override;

    int peek(void) override;
    int available(void) override;
    int read() override;

    int    rx_buffer_available() override;
    void   flushRx() override;
    size_t timedReadBytes(char* buffer, size_t length, TickType_t timeout);
    size_t timedReadBytes(uint8_t* buffer, size_t length, TickType_t timeout) { return timedReadBytes((char*)buffer, length, timeout); }
    bool   realtimeOkay(char c) override;
    bool   lineComplete(char* line, char c) override;
    Error  pollLine(char* line) override;
};

extern HWCDCChannel CDCChannel;

#endif
