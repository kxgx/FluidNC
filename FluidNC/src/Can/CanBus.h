// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Machine::CanBus -- a CAN (TWAI) bus for motor drivers that are commanded
    over CAN instead of by step/direction pulses.

    This class is a transport only.  It owns the ESP32 TWAI controller, a small
    worker task, the client list and the generic bus state.  Everything that is
    specific to a motor protocol -- framing, command echo handling, bit-rate
    negotiation, scanning, diagnostics and the maintenance commands -- lives in
    a CanDriver implementation selected with can.driver.

    Motor drivers register as CanBusClient objects; the worker calls their
    poll() method periodically, and that method runs the blocking
    request/response transactions.  Keeping all bus traffic in one task is what
    makes many vendor protocols work: without request IDs a response can only
    be matched to its request if no two transactions overlap.  A transaction
    holds CanBus::Lock for its whole duration.
*/

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "Configuration/Configurable.h"
#    include "CanDriver.h"
#    include "CanFrame.h"
#    include "Pin.h"

#    include <atomic>
#    include <cstddef>
#    include <cstdint>
#    include <string>
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

        // Slave address used to detect duplicate configured addresses; 0 for
        // clients that do not use an address-based protocol.
        virtual uint8_t bus_address() const { return 0; }

        // True once the client has rebuilt its cached position after a zero
        // or a recovery sync.  Used to make $CANZero synchronous.
        virtual bool is_synced() const { return false; }

        // The driver's encoder position was zeroed by a maintenance command.
        // The client must resynchronise its cached base position before it can
        // send another absolute target.
        virtual void on_position_zeroed() {}

        // A foreground diagnostic report just released the bus.  Clients can
        // defer their next background status poll so the worker does not
        // immediately stack a backlog of polls on top of the report tail.
        virtual void on_diagnostic_complete() {}

        virtual ~CanBusClient() = default;
    };

    // Generic TWAI transport.  It does not know any vendor protocol; the
    // selected CanDriver uses the public transport API below.
    class CanBus : public Configuration::Configurable {
    public:
        CanBus() = default;

        static constexpr size_t MAX_CLIENTS = 16;

        // Bus status as seen by a vendor driver's diagnostic commands.
        struct BusStatus {
            const char* state;
            uint32_t    tx_error;
            uint32_t    rx_error;
            uint32_t    bus_errors;
            uint32_t    msgs_to_tx;
            uint32_t    msgs_to_rx;
            uint32_t    bus_off_count;
        };

        // Installs the TWAI driver, starts the worker task and then lets the
        // selected CanDriver perform its own probe/initialisation sequence.
        // Safe to call when no pins are configured; it logs and does nothing in
        // that case.
        void init();

        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

        // Selected protocol driver (created through CanDriverRegistry).  The
        // command layer assumes this is never null after configuration
        // validation; on a bad can.driver value it falls back to a stub driver
        // so a diagnostic command cannot crash the firmware.
        CanDriver*  driver();
        const char* driver_name() const;

        bool     configured() const { return _tx.defined() && _rx.defined(); }
        bool     ready() const { return _ready; }
        uint32_t baud() const { return _baud; }
        const Pin& tx_pin() const { return _tx; }
        const Pin& rx_pin() const { return _rx; }

        // True when at least one CAN motor driver was configured.  The bus,
        // worker and all $CAN.../M15x commands are gated on this so a machine
        // without a CAN motor never starts the CAN subsystem.
        static void mark_driver_configured();
        static void unmark_driver_configured();
        static bool driver_configured();
        static int  configured_driver_count();

        // Addresses registered from the parsed motor configurations before the
        // bus starts.  Drivers use these rather than the client list because
        // CanBus::init() runs before Axes::init() adds the clients.
        static void   register_configured_address(uint8_t address);
        static void   unregister_configured_address(uint8_t address);
        static size_t configured_addresses(uint8_t* out, size_t max);

        // CAN background logging is off by default.  Explicit $CAN... report
        // commands still print their results; this gate only controls the
        // asynchronous/boot/worker messages.
        static bool logging_enabled();
        static void set_logging_enabled(bool enabled);
        static bool toggle_logging();

        // Generic bus control state.
        bool control_ready() const { return _control_ready; }
        void set_control_ready(bool ready) { _control_ready = ready; }

        // Machine-level motion gate.  CAN axes must not accept G-code, jog or
        // homing while the worker is absent, driver init/auto-config failed,
        // a stop is latched, or control has been inhibited.  Diagnostics are
        // deliberately not gated by this.
        bool all_clients_synced() const {
            for (size_t i = 0; i < _n_clients; ++i) {
                if (_clients[i] && !_clients[i]->is_synced()) {
                    return false;
                }
            }
            return true;
        }

        bool motion_ready() const {
            return _control_ready && _task_handle != nullptr && !_stop_latched.load(std::memory_order_acquire) && !_inhibited &&
                   all_clients_synced();
        }

        bool stop_latched() const { return _stop_latched.load(std::memory_order_acquire); }
        void latch_stop() { _stop_latched.store(true, std::memory_order_release); }
        void clear_stop_latch() { _stop_latched.store(false, std::memory_order_release); }

        // Stops all CAN motion control (worker stops polling clients) and
        // warns.  Used when the configuration contains duplicate slave
        // addresses; diagnostics remain available.
        void        inhibit_control(const char* reason);
        bool        inhibited() const { return _inhibited; }
        const char* inhibit_reason() const { return _inhibit_reason; }

        // A foreground diagnostic read is in progress.  The CAN worker should
        // avoid starting a new background status poll so the foreground
        // command does not wait for a poll transaction.
        bool diagnostic_active() const { return _diagnostic_active.load(std::memory_order_acquire); }
        void begin_diagnostic() { _diagnostic_active.store(true, std::memory_order_release); }
        void end_diagnostic();

        // Waits until the worker is not inside a client poll.  A driver uses
        // this before a latched safe stop so an in-flight poll cannot re-enable
        // a motor after the stop.
        void wait_for_worker_idle();

        // Client list operations.
        void add_client(CanBusClient* client);
        bool has_client(uint8_t address) const;
        bool client_synced(uint8_t address) const;

        // Unique slave addresses of configured clients, in client order.
        size_t client_addresses(uint8_t* out, size_t max) const;

        // A maintenance command changed this driver's position outside the
        // normal streaming path.  Ask the owning client to discard its cached
        // position base before it sends another absolute target.
        void invalidate_client_position(uint8_t address);
        void invalidate_all_client_positions();

        // Sends one raw CAN data frame with an explicit identifier and frame
        // format: extended=false selects an 11-bit standard identifier,
        // extended=true a 29-bit extended one.  This is the primitive the
        // logical-message helpers below -- and standard protocols such as
        // CANopen/CiA301, which use 11-bit frames -- are built on.  It sends
        // exactly one frame, so len may not exceed 8 bytes.  It does not apply
        // the inter-message gap; use send_message()/send_frames() when a
        // vendor protocol requires pacing between complete messages.
        bool send_frame(uint32_t id, bool extended, const uint8_t* data, size_t len, uint32_t timeout_ms);

        // Sends one logical protocol message in 8-byte CAN data frames.  The
        // caller supplies the 29-bit extended identifier; the frames are
        // transmitted back-to-back.  Returns false if any frame could not be
        // queued for transmission.  Waits for the configured inter-message gap
        // since the end of the previous message before it transmits the first
        // frame.  Callers must hold the Lock so that the pace is tracked
        // consistently.
        bool send_message(uint32_t can_id, const uint8_t* data, size_t len, uint32_t timeout_ms);

        // Sends a pre-built sequence of frames as one logical message.  This is
        // for protocols whose framing is owned by a protocol codec (for example
        // ZDT split frames): the codec fills CanFrame values, the transport only
        // applies the inter-message gap and transmits them in order.  Callers
        // must hold the Lock so the gap accounting is consistent.
        bool send_frames(const CanFrame* frames, size_t count, uint32_t timeout_ms);

        // Waits for the next CAN data frame, copies its payload into buf and
        // reports the identifier and frame format (extended=false is an 11-bit
        // standard frame).  Returns the payload length, or 0 when no frame
        // arrived in time.  RTR frames are discarded.  A protocol that shares
        // the bus with other nodes uses this to match a response against its
        // own expected COB-ID.
        size_t receive_frame(uint32_t& id, bool& extended, uint8_t* buf, size_t max_len, uint32_t timeout_ms);

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

        // TWAI error/state information for vendor diagnostic commands.
        bool        bus_status(BusStatus& out) const;
        uint32_t    tx_error_counter() const;
        uint32_t    bus_off_count() const { return _bus_off_count; }

        // Minimum gap between two logical messages.  A driver whose protocol
        // requires a pause between complete instructions sets this in
        // init_driver().  Zero disables the extra delay.
        void     set_min_message_gap_ms(uint32_t gap) { _min_message_gap_ms = gap; }
        uint32_t min_message_gap_ms() const { return _min_message_gap_ms; }

        // Releases the TWAI pins so diagnostics (like a vendor transceiver
        // test or the pin sweep) can use them as plain GPIO, then restores the
        // bus.
        bool suspend();
        bool resume();

        // Install/remove the TWAI controller.  A driver may call these from
        // init_driver() while probing several bit rates; the worker must be
        // suspended first (suspend() does that).
        bool install_transport();
        bool install_transport(uint32_t baud);
        void uninstall_transport();

        // Serializes transactions.  Vendor protocols often have no request
        // IDs, so a transaction must hold this from its first transmitted
        // frame until its last received response frame has been assembled.
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
        void take();
        void release();
        void check_state();
        void start_worker();
        bool stop_worker();
        void notify_diagnostic_complete();
        void worker();
        static void worker_entry(void* arg);

        const char* effective_driver_name() const;

        // Configured by can: (empty driver name selects the registry default).
        std::string _driverName;
        CanDriver*  _driver = nullptr;
        // Name the cached _driver was created for, so a configuration reload
        // with a different driver recreates it.
        std::string _resolvedDriverName;

        Pin      _tx;
        Pin      _rx;
        uint32_t _baud = 500000;

        static bool    _driver_configured;
        static int     _driver_count;
        static bool    _logging_enabled;
        static uint8_t _configuredAddresses[MAX_CLIENTS];
        static size_t  _configuredAddressCount;

        bool              _ready         = false;
        bool              _control_ready = false;
        bool              _initializing  = false;
        bool              _inhibited     = false;
        const char*       _inhibit_reason = nullptr;
        std::atomic<bool> _stop_latched{ false };
        std::atomic<bool> _diagnostic_active{ false };
        std::atomic<bool> _worker_busy{ false };
        SemaphoreHandle_t _bus_mutex      = nullptr;
        SemaphoreHandle_t _list_mutex     = nullptr;
        TaskHandle_t      _task_handle    = nullptr;
        volatile bool     _stop_requested = false;
        volatile bool     _task_exited    = false;

        uint32_t      _last_state_check_ms = 0;
        uint32_t      _last_state_log_ms   = 0;
        uint32_t      _bus_off_count       = 0;
        CanBusClient* _clients[MAX_CLIENTS] = { nullptr };
        size_t        _n_clients            = 0;

        // Pacing between logical messages.  _message_sent distinguishes "no
        // message yet" (so the first message after boot is not delayed) from a
        // timestamp that happens to be zero.
        uint32_t _last_message_end_ms = 0;
        bool     _message_sent        = false;
        uint32_t _min_message_gap_ms  = 0;
    };
}

// Gate the asynchronous/boot/worker CAN messages behind $CANLog.  Explicit
// $CANStatus/$CANScan/... report commands use log_* directly so they still
// print when background logging is off.
#define can_log_info(...)    do { if (Machine::CanBus::logging_enabled()) { log_info(__VA_ARGS__); } } while (0)
#define can_log_verbose(...) do { if (Machine::CanBus::logging_enabled()) { log_verbose(__VA_ARGS__); } } while (0)
// Errors and warnings are safety/diagnostic information and must never be
// hidden by the verbose CAN logging switch.
#define can_log_warn(...)  do { log_warn(__VA_ARGS__); } while (0)
#define can_log_error(...) do { log_error(__VA_ARGS__); } while (0)

#endif  // MAX_N_CAN

