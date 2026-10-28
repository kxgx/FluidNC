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

        // PD42S1 manual section 4.1 item 7: two complete instructions must be
        // at least this far apart so the driver can finish parsing the first
        // one.  The frames that make up a single instruction are still sent
        // back-to-back.
        static constexpr uint32_t MIN_INSTRUCTION_GAP_MS = 2;

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
        // be queued for transmission.  Waits for MIN_INSTRUCTION_GAP_MS since
        // the end of the previous message before it transmits the first frame.
        // Callers must hold the Lock so that the pace is tracked consistently.
        bool send_message(const uint8_t* data, size_t len, uint32_t timeout_ms);

        // Waits for the next CAN data frame and copies its payload into buf.
        // Returns the payload length, or 0 when no frame arrived in time.
        size_t receive_chunk(uint8_t* buf, size_t max_len, uint32_t timeout_ms);

        // Discards frames that are still queued in the TWAI receive or
        // transmit queue.  A transaction uses these to start from a clean
        // state and to remove a partial response after a failure.  Call them
        // only while holding the Lock (no transaction in flight), otherwise
        // they could discard another transaction's frames.
        void flush_receive();
        void flush_transmit();

        // Sends the PD42S1 "read version" command to addresses 1..8 and logs
        // every device that answers.  Returns the number of responders.
        // Holds the transaction lock for the whole scan, so the motor worker
        // cannot interleave its own requests.
        int scan();

        // Tries the same scan at every PD42S1 bit rate (125k, 250k, 500k and
        // 1M), then restores the configured bit rate and restarts the worker.
        // Useful when the drivers' CAN bit rate is not known.
        int scan_all_bauds();

        // Broadcasts the PD42S1 "command echo" setting (0x6F) to all drivers:
        // true = enable replies, false = disable replies.  Takes the
        // transaction lock internally; do not call it while already holding a
        // Lock.  Broadcast commands are executed but never answered.
        void set_driver_echo(bool enable);

        // Sends the same 0x6F = 0 command to one specific driver.  This is
        // more reliable than a broadcast when several drivers share the bus:
        // a driver that missed one broadcast would otherwise stay silent for
        // every later transaction.  Takes the lock internally.
        void enable_driver_echo(uint8_t address);

        // Sends 0x6F = 0 to every possible address (1..8).  Needed only when
        // a driver was left with command echo disabled by an earlier tool;
        // normal operation and scans leave 0x6F at its factory default.
        void enable_all_driver_echo();

        // Logs the TWAI state, error counters and restart count of the bus.
        // Diagnostic only; takes the transaction lock internally.
        void report_status();

        // Sends 0xFB (clear stall / brake / disabled state) to all drivers:
        // broadcast first, then every address 1..8.  Does not enable the
        // motors; the normal motor sync will re-enable them when needed.
        void clear_driver_faults();

        // Broadcasts the PD42S1 reset command (0x02) to recover a driver that
        // stopped answering, waits for it to boot, then re-enables echo.
        void restart_drivers();

        // Reads and displays each PD42S1's run status (0x2C), stall flag
        // (0x2D), enable state (0x2F) and in-position flag (0x30), marking
        // abnormal conditions (overload/stall/undervoltage).
        void report_driver_faults();

        // Reads and displays position (0x2A), position error (0x2B), speed
        // (0x29) and in-position flag (0x30) from every answering driver.
        void report_driver_positions();

        // Broadcasts the immediate-brake command (0xFC) to all drivers.
        void stop_all_drivers();

        // Sends one read command and prints the raw response bytes.  Useful
        // for exploring function codes that do not have a dedicated command.
        void read_raw(uint8_t address, uint8_t function);

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
        void        enable_echoes_at_startup();
        bool        send_echo_state(bool enable);          // caller must hold the Lock
        bool        send_echo_state_forced(bool enable);   // caller must hold the Lock
        bool        enable_driver_echo_locked(uint8_t address);  // caller must hold the Lock
        void        enable_all_driver_echo_locked();             // caller must hold the Lock
        int         scan_addresses();
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
        uint32_t      _last_state_log_ms    = 0;
        uint32_t      _bus_off_count        = 0;
        CanBusClient* _clients[MAX_CLIENTS] = { nullptr };
        size_t        _n_clients            = 0;

        // Last echo state we configured on the drivers; avoids re-sending the
        // broadcast (and flooding the log) when several motors re-sync.
        bool _echo_state = false;

        // Pacing between logical instructions.  _message_sent distinguishes
        // "no message yet" (so the first message after boot is not delayed)
        // from a timestamp that happens to be zero.
        uint32_t _last_message_end_ms = 0;
        bool     _message_sent        = false;
    };
}

#endif  // MAX_N_CAN
