// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Machine::CanBus -- a CAN (TWAI) bus for motor drivers that are commanded
    over CAN instead of by step/direction pulses.

    The bus owns the ESP32 TWAI controller and a small worker task.  Motor
    drivers register as CanBusClient objects; the worker calls their poll()
    method periodically, and that method runs the blocking request/response
    transactions.  Keeping all bus traffic in one task is what makes the
    protocol work: the PD42S1 protocol has no request IDs, so a response can
    only be matched to its request if no two transactions overlap.  A
    transaction holds CanBus::Lock for its whole duration.
*/

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "Configuration/Configurable.h"
#    include "Pin.h"

#    include <cstddef>
#    include <cstdint>
#    include <freertos/FreeRTOS.h>
#    include <freertos/semphr.h>
#    include <freertos/task.h>

namespace Machine {
    class CanBus;

    class CanBusClient {
    public:
        // Called from the CAN worker task.  A client may run blocking
        // transactions here; it must not hold a CanBus::Lock across calls.
        virtual void can_poll(uint32_t now_ms) = 0;

        virtual ~CanBusClient() = default;
    };

    class CanBus : public Configuration::Configurable {
    public:
        CanBus() = default;

        static constexpr size_t MAX_CLIENTS = 8;

        // Installs the TWAI driver and starts the worker task.  Safe to call
        // when no pins are configured; it logs and does nothing in that case.
        void init();

        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

        bool configured() const { return _tx.defined() && _rx.defined(); }
        bool ready() const { return _ready; }

        uint32_t command_id() const { return _command_id; }

        void add_client(CanBusClient* client);

        // Splits one logical protocol message into 8-byte CAN data frames and
        // transmits them back-to-back.  Returns false if any frame could not
        // be queued for transmission.
        bool send_message(const uint8_t* data, size_t len, uint32_t timeout_ms);

        // Waits for the next CAN data frame and copies its payload into buf.
        // Returns the payload length, or 0 when no frame arrived in time.
        size_t receive_chunk(uint8_t* buf, size_t max_len, uint32_t timeout_ms);

        // Sends the PD42S1 "read version" command to addresses 1..8 and logs
        // every device that answers.  Returns the number of responders.
        // Holds the transaction lock for the whole scan, so the motor worker
        // cannot interleave its own requests.
        int scan();

        // Releases the TWAI pins so diagnostics (like the transceiver test or
        // the pin sweep) can use them as plain GPIO, then restores the bus.
        void suspend();
        bool resume();

        // Serializes transactions.  The protocol has no request IDs, so a
        // transaction must hold this from its first transmitted frame until
        // its last received response frame has been assembled.
        class Lock {
        public:
            explicit Lock(CanBus& bus) : _bus(bus) { _bus.take(); }
            ~Lock() { _bus.release(); }
            Lock(const Lock&)            = delete;
            Lock& operator=(const Lock&) = delete;

        private:
            CanBus& _bus;
        };

        ~CanBus();

    private:
        void        take();
        void        release();
        void        check_state();
        bool        install_driver();
        void        start_worker();
        bool        test_transceiver();
        int         scan_address(uint8_t address, uint32_t timeout_ms);
        bool        transact(
            const uint8_t* request, size_t requestLen, uint8_t* response, size_t responseCapacity, size_t& responseLen, uint32_t timeoutMs);
        void        worker();
        static void worker_entry(void* arg);
        void        stop_worker();

        Pin      _tx;
        Pin      _rx;
        uint32_t _baud       = 500000;
        uint32_t _command_id = 0x1000;

        bool              _ready          = false;
        SemaphoreHandle_t _bus_mutex      = nullptr;
        SemaphoreHandle_t _list_mutex     = nullptr;
        TaskHandle_t      _task_handle    = nullptr;
        volatile bool     _stop_requested = false;
        volatile bool     _task_exited    = false;

        uint32_t      _last_state_check_ms  = 0;
        CanBusClient* _clients[MAX_CLIENTS] = { nullptr };
        size_t        _n_clients            = 0;
    };
}

#endif  // MAX_N_CAN
