// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "CanBus.h"

#    include "Logging.h"
#    include "Motors/Pd42s1Protocol.h"

#    include <Arduino.h>
#    include <driver/twai.h>
#    include <esp_timer.h>

#    include <cstdio>
#    include <cstring>

namespace Machine {

    bool CanBus::_driver_configured = false;
    int  CanBus::_driver_count      = 0;
    bool CanBus::_logging_enabled   = false;

    bool CanBus::logging_enabled() {
        return _logging_enabled;
    }

    void CanBus::set_logging_enabled(bool enabled) {
        _logging_enabled = enabled;
    }

    bool CanBus::toggle_logging() {
        _logging_enabled = !_logging_enabled;
        return _logging_enabled;
    }

    using MotorDrivers::Pd42s1Protocol;

    namespace {
        uint32_t now_ms() {
            return uint32_t(esp_timer_get_time() / 1000);
        }

        uint32_t twai_tx_error_counter() {
            twai_status_info_t status;
            return twai_get_status_info(&status) == ESP_OK ? status.tx_error_counter : 0;
        }

        const char* motor_status_name(uint8_t status) {
            switch (status) {
                case 0:
                    return "stopped";
                case 1:
                    return "done";
                case 2:
                    return "running";
                case 3:
                    return "OVERLOAD";
                case 4:
                    return "STALL";
                case 5:
                    return "UNDERVOLTAGE";
                default:
                    return "unknown";
            }
        }

        const char* twai_state_name(twai_state_t state) {
            switch (state) {
                case TWAI_STATE_STOPPED:
                    return "stopped";
                case TWAI_STATE_RUNNING:
                    return "running";
                case TWAI_STATE_BUS_OFF:
                    return "bus-off";
                case TWAI_STATE_RECOVERING:
                    return "recovering";
                default:
                    return "unknown";
            }
        }

        bool timing_for_baud(uint32_t baud, twai_timing_config_t& timing) {
            switch (baud) {
                case 125000: {
                    twai_timing_config_t value = TWAI_TIMING_CONFIG_125KBITS();
                    timing                     = value;
                    return true;
                }
                case 250000: {
                    twai_timing_config_t value = TWAI_TIMING_CONFIG_250KBITS();
                    timing                     = value;
                    return true;
                }
                case 500000: {
                    twai_timing_config_t value = TWAI_TIMING_CONFIG_500KBITS();
                    timing                     = value;
                    return true;
                }
                case 1000000: {
                    twai_timing_config_t value = TWAI_TIMING_CONFIG_1MBITS();
                    timing                     = value;
                    return true;
                }
                default:
                    return false;
            }
        }
    }

    void CanBus::group(Configuration::HandlerBase& handler) {
        // @config tx_pin
        // @default NO_PIN
        // @pin_attributes output
        // TWAI/CAN transmit pin.  Connect it to the D (driver input) pin of the
        // CAN transceiver -- for a SN65HVD230, pin 1.
        handler.item("tx_pin", _tx);

        // @config rx_pin
        // @default NO_PIN
        // @pin_attributes input
        // TWAI/CAN receive pin.  Connect it to the R (receiver output) pin of
        // the CAN transceiver -- for a SN65HVD230, pin 4.
        handler.item("rx_pin", _rx);

        // @config baud
        // @default 500000
        // @tuning per-machine
        // CAN bit rate in bits per second.  Must match the bit rate configured
        // in the motor drivers.  The PD42S1 driver supports 125000, 250000,
        // 500000 and 1000000; this bus supports the same four values.
        handler.item("baud", _baud);

        // @config command_id
        // @default 0x1000
        // @tuning per-machine
        // 29-bit extended CAN ID used for frames sent by the controller.  The
        // vendor's CAN examples use 0x1000, which is the PD42S1 default.  The
        // driver only accepts IDs in 0x1000..0x100F; validate() enforces that.
        // Only change this if the drivers were reprogrammed with command ID
        // 0x6C.
        handler.item("command_id", _command_id);
    }

    void CanBus::validate() {
        if (!configured()) {
            return;  // A CAN motor driver will report the missing bus.
        }
        Assert(_tx.defined() && _rx.defined(), "CAN bus needs both tx_pin and rx_pin");
        Assert(_tx != _rx, "CAN tx_pin and rx_pin must be different pins");
        Assert(_baud == 125000 || _baud == 250000 || _baud == 500000 || _baud == 1000000,
               "CAN baud must be 125000, 250000, 500000 or 1000000");
        Assert(_command_id <= 0x1FFFFFFF, "CAN command_id must fit in 29 bits");
        Assert((_command_id & 0xFFFFFFF0u) == 0x1000u, "CAN command_id must be in 0x1000..0x100F for PD42S1");
    }

    void CanBus::init() {
        if (!_driver_configured) {
            can_log_verbose("CAN: no PD42S1 motor configured; CAN disabled");
            return;
        }
        if (_ready) {
            return;
        }
        if (!configured()) {
            can_log_verbose("CAN: bus not configured");
            return;
        }

        _bus_mutex  = xSemaphoreCreateMutex();
        _list_mutex = xSemaphoreCreateMutex();
        if (!_bus_mutex || !_list_mutex) {
            can_log_error("CAN: cannot create mutexes");
            return;
        }

        if (!auto_configure_drivers()) {
            return;
        }
        _ready = true;
        start_worker();
        can_log_info("CAN: TX:" << _tx.name() << " RX:" << _rx.name() << " baud:" << _baud << " command_id:" << uint32_t(_command_id));
    }

    bool CanBus::install_driver() {
        twai_timing_config_t timing;
        if (!timing_for_baud(_baud, timing)) {
            can_log_error("CAN: unsupported baud " << _baud);
            return false;
        }

        auto tx_pin = _tx.getNative(Pin::Capabilities::Native | Pin::Capabilities::Output);
        auto rx_pin = _rx.getNative(Pin::Capabilities::Native | Pin::Capabilities::Input);
        Assert(tx_pin != INVALID_PINNUM, "CAN tx_pin must be a native GPIO");
        Assert(rx_pin != INVALID_PINNUM, "CAN rx_pin must be a native GPIO");

        twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NORMAL);
        general.rx_queue_len          = 32;
        general.tx_queue_len          = 8;

        twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        esp_err_t err = twai_driver_install(&general, &timing, &filter);
        if (err != ESP_OK) {
            can_log_error("CAN: driver install failed: " << esp_err_to_name(err));
            return false;
        }
        err = twai_start();
        if (err != ESP_OK) {
            can_log_error("CAN: start failed: " << esp_err_to_name(err));
            twai_driver_uninstall();
            return false;
        }
        return true;
    }

    bool CanBus::probe_address_quiet(uint8_t address, uint8_t& firmware, uint8_t& hardware, uint32_t timeout_ms, int attempts) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));
        for (int attempt = 0; attempt < attempts; ++attempt) {
            size_t responseLen = 0;
            if (transact(request, len, response, sizeof(response), responseLen, timeout_ms) &&
                Pd42s1Protocol::parse_version(response, responseLen, firmware, hardware)) {
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        return false;
    }

    void CanBus::set_echo_to_address_locked(uint8_t address, bool enable) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t echo = enable ? 0 : 1;
        size_t  len  = Pd42s1Protocol::build_frame(address, Pd42s1Protocol::FUNC_SET_ECHO, &echo, 1, request, sizeof(request));
        if (!len) {
            return;
        }
        send_message(request, len, 50);
        vTaskDelay(pdMS_TO_TICKS(2));
        flush_receive();
    }

    void CanBus::disable_all_driver_echo_locked() {
        // Broadcast first, then each address; the addressed calls also drain
        // the acknowledgment from drivers that were still echoing.
        send_echo_state_forced(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        for (uint8_t address = 1; address <= 8; ++address) {
            set_echo_to_address_locked(address, false);
        }
        flush_receive();
        _echo_state = false;
        can_log_info("CAN: command echo disabled on the drivers after auto-config");
    }

    bool CanBus::auto_configure_drivers() {
        static const uint32_t bauds[] = { 125000, 250000, 500000, 1000000 };
        const uint32_t        target  = _baud;

        can_log_info("CAN: auto-config: target " << target << " baud; probing all driver rates");
        int expected  = configured_driver_count();
        int discovered = 0;
        if (expected <= 0) {
            expected = 1;  // at least one client should be configured; use discovery as fallback
        }

        // Phase 1: discover which rate each driver uses.  Echo is enabled at
        // every rate first; a broadcast that is acknowledged proves that at
        // least one driver is listening at that rate.
        for (uint32_t rate : bauds) {
            _baud = rate;
            if (!install_driver()) {
                continue;
            }
            _ready = true;

            if (!send_echo_state_forced(true)) {
                twai_stop();
                twai_driver_uninstall();
                _ready = false;
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            for (uint8_t address = 1; address <= 8; ++address) {
                enable_driver_echo_locked(address);
            }
            flush_receive();

            int found = 0;
            for (uint8_t address = 1; address <= 8; ++address) {
                uint8_t firmware = 0;
                uint8_t hardware = 0;
                if (probe_address_quiet(address, firmware, hardware, 80, 3)) {
                    found++;
                    can_log_info("CAN: found address " << int(address) << " at " << rate << " baud (fw V" << int(firmware / 10) << "."
                                                   << int(firmware % 10) << " hw V" << int(hardware / 10) << "." << int(hardware % 10)
                                                   << ")");
                }
            }
            discovered += found;

            if (found && rate != target) {
                uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
                size_t  len = Pd42s1Protocol::build_set_can_baud_command(0, uint16_t(target / 1000), request, sizeof(request));
                if (len) {
                    can_log_info("CAN: broadcasting CAN baud " << target << " to " << found << " driver(s) currently at " << rate);
                    send_message(request, len, 100);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }

            twai_stop();
            twai_driver_uninstall();
            _ready = false;
        }

        // Phase 2: install the configured rate, persist the parameters and
        // verify that the expected number of drivers now answers there.
        _baud = target;
        if (!install_driver()) {
            return false;
        }
        _ready = true;

        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  len = Pd42s1Protocol::build_save_params_command(0, request, sizeof(request));
        if (len) {
            can_log_info("CAN: broadcasting parameter save (0x04) at " << target << " baud");
            send_message(request, len, 100);
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        send_echo_state_forced(true);
        vTaskDelay(pdMS_TO_TICKS(10));
        for (uint8_t address = 1; address <= 8; ++address) {
            enable_driver_echo_locked(address);
        }
        flush_receive();

        int confirmed = 0;
        for (uint8_t address = 1; address <= 8; ++address) {
            uint8_t firmware = 0;
            uint8_t hardware = 0;
            if (probe_address_quiet(address, firmware, hardware, 100, 3)) {
                confirmed++;
            }
        }
        can_log_info("CAN: auto-config: configured=" << configured_driver_count() << " discovered=" << discovered << " confirmed=" << confirmed
                                                << " at " << target << " baud");

        // Persist the setting explicitly per address and read it back (0x32
        // byte 8..9 contains the saved CAN bit rate).  A broadcast save is not
        // acknowledged, so this is what proves that the new rate survives a
        // power cycle.
        int savedOk = 0;
        for (uint8_t address = 1; address <= 8; ++address) {
            uint8_t  currentEcho = 0;
            uint8_t  currentMode = 0;
            uint16_t currentBaud = 0;
            if (!read_driver_config(address, currentBaud, currentEcho, currentMode)) {
                continue;
            }
            if (currentBaud == uint16_t(target / 1000)) {
                // Already correct: do not rewrite the driver's NVRAM on every
                // boot.  Echo/baud were confirmed above.
                savedOk++;
                can_log_info("CAN: address " << int(address) << " already saved CAN baud=" << currentBaud << " kbps; skip write");
                continue;
            }
            uint8_t request2[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response2[64];
            size_t  len2 = Pd42s1Protocol::build_set_can_baud_command(address, uint16_t(target / 1000), request2, sizeof(request2));
            for (int attempt = 0; attempt < 2; ++attempt) {
                size_t responseLen = 0;
                if (transact(request2, len2, response2, sizeof(response2), responseLen, 100)) {
                    break;
                }
            }
            len2 = Pd42s1Protocol::build_save_params_command(address, request2, sizeof(request2));
            bool saved = false;
            for (int attempt = 0; attempt < 2; ++attempt) {
                size_t responseLen = 0;
                if (transact(request2, len2, response2, sizeof(response2), responseLen, 100) &&
                    Pd42s1Protocol::parse_ack(response2, responseLen)) {
                    saved = true;
                    break;
                }
            }
            if (!saved) {
                can_log_warn("CAN: address " << int(address) << " did not acknowledge parameter save");
                continue;
            }
            uint16_t savedBaud = 0;
            if (read_driver_config(address, savedBaud, currentEcho, currentMode) && savedBaud == uint16_t(target / 1000)) {
                savedOk++;
                can_log_info("CAN: address " << int(address) << " saved CAN baud=" << savedBaud << " kbps");
            } else {
                can_log_warn("CAN: address " << int(address) << " saved CAN baud readback=" << savedBaud << " kbps (expected "
                                         << uint16_t(target / 1000) << ")");
            }
        }

        if (expected > 0 && (confirmed < expected || savedOk < confirmed)) {
            can_log_warn("CAN: " << (expected - confirmed) << " configured driver(s) did not answer at the configured baud; command echo is left ENABLED");
            // Do not disable echo when the system is incomplete: motor control
            // needs acknowledgments and a disabled driver would make it
            // impossible to recover without a power cycle.
        } else {
            // All configured drivers answered and saved the new rate: close
            // the echo setting as requested.  Motor sync re-enables it before
            // streaming.
            disable_all_driver_echo_locked();
        }
        return true;
    }

    bool CanBus::run_auto_config() {
        if (!driver_ok("auto-config")) {
            return false;
        }
        const uint32_t target = _baud;
        can_log_info("CAN: starting manual auto-config (target " << target << " baud)");
        suspend();
        bool ok = auto_configure_drivers();
        if (!ok) {
            _baud = target;
            if (!install_driver()) {
                can_log_error("CAN: auto-config failed and the bus could not be restored");
                return false;
            }
            _ready = true;
        }
        start_worker();
        can_log_info("CAN: auto-config finished");
        return ok;
    }

    void CanBus::start_worker() {
        _stop_requested = false;
        _task_exited    = false;
        _ready          = true;

        if (xTaskCreate(worker_entry, "can", 4096, this, 2, &_task_handle) != pdPASS) {
            can_log_error("CAN: cannot create worker task");
            _task_handle = nullptr;
            _ready       = false;
        }
    }

    void CanBus::suspend() {
        if (!_ready) {
            return;
        }
        stop_worker();
        twai_stop();
        twai_driver_uninstall();
        _ready = false;
        delay(5);
    }

    bool CanBus::resume() {
        if (_ready) {
            return true;
        }
        if (!install_driver()) {
            can_log_error("CAN: cannot restore the bus");
            return false;
        }
        start_worker();
        return true;
    }

    bool CanBus::test_transceiver() {
        // The transceiver has no status output that the controller could read,
        // but it does loop the bus state back on its R pin: driving D dominant
        // (TX low) must make R dominant (RX low), and driving D recessive (TX
        // high) must make R recessive (RX high).  This test temporarily gives
        // the TX/RX pins back to the GPIO controller, so the TWAI driver and
        // worker are suspended for the duration.
        if (!_ready) {
            log_info("CAN transceiver test: bus not ready");
            return false;
        }

        auto tx_pin = _tx.getNative(Pin::Capabilities::Native | Pin::Capabilities::Output);
        auto rx_pin = _rx.getNative(Pin::Capabilities::Native | Pin::Capabilities::Input);

        suspend();

        pinMode(rx_pin, INPUT_PULLUP);
        pinMode(tx_pin, OUTPUT);

        digitalWrite(tx_pin, 1);  // recessive
        delay(2);
        int rx_recessive = digitalRead(rx_pin);

        digitalWrite(tx_pin, 0);  // dominant
        delay(2);
        int rx_dominant = digitalRead(rx_pin);

        digitalWrite(tx_pin, 1);  // back to recessive
        delay(2);

        log_info("CAN transceiver test: D=recessive -> R=" << rx_recessive << ", D=dominant -> R=" << rx_dominant);
        bool present = (rx_recessive == 1 && rx_dominant == 0);
        if (present) {
            log_info("CAN transceiver: detected (D/R loopback OK)");
        } else {
            log_error("CAN transceiver: NOT detected or not responding");
        }

        pinMode(tx_pin, INPUT_PULLUP);
        pinMode(rx_pin, INPUT_PULLUP);
        delay(10);
        resume();
        return present;
    }

    void CanBus::add_client(CanBusClient* client) {
        if (!_list_mutex) {
            return;
        }
        uint8_t duplicate = 0;
        uint8_t address   = client->bus_address();
        xSemaphoreTake(_list_mutex, portMAX_DELAY);
        if (address) {
            for (size_t i = 0; i < _n_clients; ++i) {
                if (_clients[i] && _clients[i]->bus_address() == address) {
                    duplicate = address;
                    break;
                }
            }
        }
        if (!duplicate) {
            if (_n_clients < MAX_CLIENTS) {
                _clients[_n_clients++] = client;
            } else {
                can_log_error("CAN: too many clients");
            }
        }
        xSemaphoreGive(_list_mutex);

        if (duplicate) {
            can_log_error("CAN: duplicate PD42S1 address " << int(duplicate) << " configured; stopping all CAN control");
            inhibit_control("duplicate PD42S1 address");
        }
    }

    void CanBus::inhibit_control(const char* reason) {
        _inhibited      = true;
        _inhibit_reason = reason;
        can_log_error("CAN: control inhibited: " << reason);
        if (!_ready) {
            return;
        }
        // Stop the motors immediately: broadcast "disable" (0xFA data 1).
        Lock lock(*this);
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t data = 1;
        size_t  len  = Pd42s1Protocol::build_frame(0, Pd42s1Protocol::FUNC_SET_MOTOR_ENABLE, &data, 1, request, sizeof(request));
        if (len) {
            send_message(request, len, 100);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        flush_receive();
    }

    void CanBus::mark_driver_configured() {
        if (!_driver_configured) {
            _driver_configured = true;
        }
        _driver_count++;
    }

    void CanBus::unmark_driver_configured() {
        if (_driver_count > 0) {
            _driver_count--;
        }
    }

    int CanBus::configured_driver_count() {
        return _driver_count;
    }

    bool CanBus::driver_configured() {
        return _driver_configured;
    }

    bool CanBus::driver_ok(const char* operation) {
        if (!_driver_configured) {
            can_log_info("CAN " << operation << ": no PD42S1 motor configured; CAN disabled");
            return false;
        }
        return true;
    }

    void CanBus::take() {
        if (_bus_mutex) {
            xSemaphoreTake(_bus_mutex, portMAX_DELAY);
        }
    }

    void CanBus::release() {
        if (_bus_mutex) {
            xSemaphoreGive(_bus_mutex);
        }
    }

    bool CanBus::send_message(const uint8_t* data, size_t len, uint32_t timeout_ms) {
        if (!_ready) {
            return false;
        }

        // PD42S1 manual section 4.1 item 7: at least MIN_INSTRUCTION_GAP_MS
        // between two complete instructions.  One call here is one complete
        // instruction, so the wait belongs before the first frame; the frames
        // below are still queued back-to-back.  This runs in task context (the
        // CAN worker or the main task), never in an ISR.
        if (_message_sent) {
            uint32_t elapsed = now_ms() - _last_message_end_ms;
            if (elapsed < MIN_INSTRUCTION_GAP_MS) {
                vTaskDelay(pdMS_TO_TICKS(MIN_INSTRUCTION_GAP_MS - elapsed));
            }
        }

        bool   ok     = true;
        size_t offset = 0;
        while (offset < len) {
            size_t chunk = len - offset;
            if (chunk > TWAI_FRAME_MAX_DLC) {
                chunk = TWAI_FRAME_MAX_DLC;
            }

            twai_message_t message   = {};
            message.extd             = 1;
            message.identifier       = _command_id;
            message.data_length_code = chunk;
            memcpy(message.data, data + offset, chunk);

            esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(timeout_ms));
            if (err != ESP_OK) {
                ok = false;
                break;
            }
            offset += chunk;
        }

        _last_message_end_ms = now_ms();
        _message_sent        = true;
        return ok;
    }

    size_t CanBus::receive_chunk(uint8_t* buf, size_t max_len, uint32_t timeout_ms) {
        if (!_ready) {
            return 0;
        }
        twai_message_t message;
        esp_err_t      err = twai_receive(&message, pdMS_TO_TICKS(timeout_ms));
        if (err != ESP_OK) {
            return 0;
        }
        if (message.rtr) {
            return 0;
        }
        size_t len = message.data_length_code;
        if (len > max_len) {
            len = max_len;
        }
        memcpy(buf, message.data, len);
        return len;
    }

    void CanBus::flush_receive() {
        if (_ready) {
            twai_clear_receive_queue();
        }
    }

    void CanBus::flush_transmit() {
        if (_ready) {
            twai_clear_transmit_queue();
        }
    }

    bool CanBus::transact(
        const uint8_t* request, size_t requestLen, uint8_t* response, size_t responseCapacity, size_t& responseLen, uint32_t timeoutMs) {
        // The caller (scan) holds Lock, so no other transaction can be in
        // flight while the queues are flushed.
        responseLen = 0;
        if (!_ready || requestLen == 0) {
            return false;
        }

        flush_receive();
        if (!send_message(request, requestLen, timeoutMs)) {
            flush_receive();
            flush_transmit();
            return false;
        }

        const uint32_t deadline = now_ms() + timeoutMs;
        size_t         total    = 0;
        bool           complete = false;
        while (total < responseCapacity) {
            int32_t remaining = int32_t(deadline - now_ms());
            if (remaining <= 0) {
                break;
            }
            size_t received = receive_chunk(response + total, responseCapacity - total, uint32_t(remaining));
            if (received == 0) {
                break;
            }
            total += received;

            size_t expected = Pd42s1Protocol::expected_response_length(response, total);
            if (expected == SIZE_MAX) {
                break;
            }
            if (expected != 0 && total >= expected) {
                complete = (total == expected);
                break;
            }
        }

        // The framing must be intact, the answer must come from the slave we
        // addressed and carry the function code we asked for, and a 6-byte
        // frame with a 0xE1..0xE6 status is a command error, not a success.
        if (!complete || !Pd42s1Protocol::valid_response(response, total) || response[1] != request[1] || response[2] != request[2] ||
            (total == 6 && response[3] >= 0xE1 && response[3] <= 0xE6)) {
            flush_receive();
            flush_transmit();
            return false;
        }

        responseLen = total;
        return true;
    }

    bool CanBus::send_echo_state(bool enable) {
        if (_echo_state == enable) {
            return true;  // already configured, no need to retransmit or log
        }
        // Broadcast (address 0) "set command echo" (0x6F).  Broadcast
        // commands are executed but not answered, so this works even when the
        // drivers are currently set to not reply.  A present receiver ACKs
        // the frame at the CAN level regardless of its echo setting.
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t echo = enable ? 0 : 1;
        size_t  len  = Pd42s1Protocol::build_frame(0, Pd42s1Protocol::FUNC_SET_ECHO, &echo, 1, request, sizeof(request));
        bool    ok   = false;
        if (len) {
            can_log_info("CAN: " << (enable ? "enabling" : "disabling") << " command echo (broadcast 0x6F=" << int(echo) << ")");
            ok = send_message(request, len, 50);
            if (ok) {
                _echo_state = enable;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        flush_receive();
        return ok;
    }

    bool CanBus::send_echo_state_forced(bool enable) {
        // The scan must transmit the echo setting every time, even when the
        // cached state says it is already correct: a driver may have missed an
        // earlier broadcast (for example because another driver was not
        // powered at that moment).
        _echo_state = !enable;
        return send_echo_state(enable);
    }

    bool CanBus::enable_driver_echo_locked(uint8_t address) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t echo = 0;  // 0 = acknowledge commands
        size_t  len  = Pd42s1Protocol::build_frame(address, Pd42s1Protocol::FUNC_SET_ECHO, &echo, 1, request, sizeof(request));
        if (!len) {
            return false;
        }
        can_log_verbose("CAN: enabling command echo at address " << int(address));
        bool ok = send_message(request, len, 50);
        vTaskDelay(pdMS_TO_TICKS(2));  // let the driver apply the setting
        flush_receive();               // discard its ack if echo was already on
        return ok;
    }

    void CanBus::enable_driver_echo(uint8_t address) {
        if (!_ready) {
            return;
        }
        Lock lock(*this);
        enable_driver_echo_locked(address);
    }

    void CanBus::enable_all_driver_echo_locked() {
        // Broadcast first: every powered driver executes it without replying,
        // even if it is currently set to "no reply".  Then send the same
        // setting to each address individually, which also covers drivers
        // that filter broadcast frames.
        send_echo_state_forced(true);
        vTaskDelay(pdMS_TO_TICKS(5));
        for (uint8_t address = 1; address <= 8; ++address) {
            enable_driver_echo_locked(address);
        }
    }

    void CanBus::enable_all_driver_echo() {
        if (!driver_ok("echo")) return;
        if (!_ready) {
            can_log_info("CAN echo: bus not ready");
            return;
        }
        Lock lock(*this);
        enable_all_driver_echo_locked();
        _last_all_echo_ms = now_ms();
        can_log_info("CAN echo: sent 0x6F=0 to addresses 1..8");
    }

    void CanBus::ensure_driver_echo() {
        if (!_driver_configured || !_ready) {
            return;
        }
        const uint32_t now = now_ms();
        if (_last_all_echo_ms && now - _last_all_echo_ms < 3000) {
            return;
        }
        enable_all_driver_echo();
    }

    void CanBus::enable_echoes_at_startup() {
        // The drivers' 0x6F setting may have been left at "no reply" by an
        // earlier configuration tool.  The vendor example relies on the
        // factory default, so it never sends this; doing it once at boot
        // makes a silent driver recover automatically without adding 0x6F
        // traffic to the normal motor transactions.
        //
        // No worker exists yet, so the bus lock is not needed here.
        if (!send_echo_state_forced(true)) {
            // Nobody acknowledged the broadcast: no powered driver on the
            // bus, so skip the addressed attempts (they would only raise the
            // error counters).
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        for (uint8_t address = 1; address <= 8; ++address) {
            enable_driver_echo_locked(address);
        }
        flush_receive();
        can_log_info("CAN: enabled command echo on the drivers at startup");
    }

    void CanBus::set_driver_echo(bool enable) {
        if (!_ready) {
            return;
        }
        Lock lock(*this);
        send_echo_state(enable);
    }

    int CanBus::scan_address(uint8_t address, uint32_t timeout_ms) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  requestLen = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));

        // A marginal bus can lose a response occasionally, so give every
        // address a few tries before declaring it absent.
        constexpr int MAX_ATTEMPTS = 3;
        for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
            size_t  responseLen     = 0;
            uint8_t firmwareVersion = 0;
            uint8_t hardwareVersion = 0;
            if (transact(request, requestLen, response, sizeof(response), responseLen, timeout_ms) &&
                Pd42s1Protocol::parse_version(response, responseLen, firmwareVersion, hardwareVersion)) {
                log_info("  CAN device at address " << int(address) << " firmware V" << int(firmwareVersion / 10) << "."
                                                    << int(firmwareVersion % 10) << " hardware V" << int(hardwareVersion / 10) << "."
                                                    << int(hardwareVersion % 10)
                                                    << (attempt > 1 ? " (after retry)" : ""));
                return 1;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        return 0;
    }

    int CanBus::scan() {
        if (!driver_ok("scan")) return 0;
        if (!_ready) {
            log_info("CAN scan: bus not ready");
            return 0;
        }

        // Check the physical layer first: this also restarts the bus and the
        // worker when it is done.
        test_transceiver();
        if (!_ready) {
            log_info("CAN scan: bus not ready");
            return 0;
        }

        int found = 0;
        {
            Lock lock(*this);

            twai_status_info_t status;
            if (twai_get_status_info(&status) == ESP_OK) {
                log_info("CAN scan: state=" << twai_state_name(status.state) << " tx_err=" << status.tx_error_counter
                                            << " rx_err=" << status.rx_error_counter << " bus_errors=" << status.bus_error_count);
            }

            // Discard any frames that are still queued from earlier transactions.
            flush_receive();

            uint32_t tecBefore = twai_tx_error_counter();
            found              = scan_addresses();
            uint32_t tecAfter  = twai_tx_error_counter();
            can_log_info("  TWAI tx_err " << tecBefore << " -> " << tecAfter
                                      << (tecAfter == 0 ? " (frames were ACKed by a node)"
                                                        : " (no ACK - check driver power, wiring and bit rate)"));
        }

        return found;
    }

    int CanBus::scan_addresses() {
        int found = 0;
        for (uint8_t address = 1; address <= 8; ++address) {
            found += scan_address(address, 100);
        }
        if (!found) {
            can_log_info("  (no devices)");
        }
        return found;
    }

    int CanBus::scan_all_bauds() {
        if (!driver_ok("scan")) return 0;
        if (!_ready) {
            log_info("CAN scan: bus not ready");
            return 0;
        }

        // Transceiver detection works with GPIOs and is independent of the bit
        // rate, so run it once at the configured rate before changing anything.
        test_transceiver();
        if (!_ready) {
            log_info("CAN scan: bus not ready");
            return 0;
        }

        static const uint32_t bauds[]    = { 125000, 250000, 500000, 1000000 };
        const uint32_t        configured = _baud;
        int                   total      = 0;

        // Stop the worker and release the bus; no client transaction may run
        // while the driver is reinstalled at the different bit rates.
        suspend();

        {
            Lock lock(*this);
            for (uint32_t baud : bauds) {
                _baud = baud;
                if (!install_driver()) {
                    log_error("CAN scan: cannot install driver at " << baud << " baud");
                    continue;
                }
                _ready = true;
                log_info("CAN scan at " << baud << " baud:");
                flush_receive();
                uint32_t tecBefore = twai_tx_error_counter();
                // The multi-baud recovery scan explicitly enables command
                // echo on every address before reading; the plain $CANScan
                // stays faithful to the vendor example (no 0x6F commands).
                log_info("CAN scan: enabling echo on addresses 1..8");
                enable_all_driver_echo_locked();
                vTaskDelay(pdMS_TO_TICKS(5));
                total += scan_addresses();
                uint32_t tecAfter = twai_tx_error_counter();
                can_log_info("  TWAI tx_err " << tecBefore << " -> " << tecAfter
                                          << (tecAfter == 0 ? " (frames were ACKed by a node)"
                                                            : " (no ACK - check driver power, wiring and bit rate)"));
                twai_stop();
                twai_driver_uninstall();
                _ready = false;
            }
        }

        _baud = configured;
        if (!install_driver()) {
            can_log_error("CAN: cannot restore the bus after the baud scan");
            return total;
        }
        start_worker();
        log_info("CAN scan: restored " << _baud << " baud");
        return total;
    }

    void CanBus::check_state() {
        // The worker calls this with no lock held, but a transaction from the
        // main task may still be in progress during startup; recovering or
        // restarting the controller must not race with a queued TWAI transfer.
        Lock lock(*this);

        twai_status_info_t status;
        if (twai_get_status_info(&status) != ESP_OK) {
            return;
        }
        const uint32_t now = now_ms();
        if (status.state == TWAI_STATE_BUS_OFF) {
            _bus_off_count++;
            if (_last_state_log_ms == 0 || now - _last_state_log_ms >= 2000) {
                _last_state_log_ms = now;
                can_log_error("CAN: bus off (count:" << _bus_off_count << "), starting recovery");
            }
            twai_initiate_recovery();
        } else if (status.state == TWAI_STATE_STOPPED) {
            can_log_verbose("CAN: restarting bus after recovery");
            twai_start();
        } else if (status.tx_error_counter > 127 && (_last_state_log_ms == 0 || now - _last_state_log_ms >= 2000)) {
            _last_state_log_ms = now;
            can_log_warn("CAN: error-passive, tx_err=" << status.tx_error_counter << " rx_err=" << status.rx_error_counter);
        }
    }

    void CanBus::report_status() {
        if (!driver_ok("status")) return;
        if (!_ready) {
            log_info("CAN status: bus not ready");
            return;
        }
        Lock lock(*this);

        twai_status_info_t status;
        esp_err_t          err = twai_get_status_info(&status);
        if (err != ESP_OK) {
            log_error("CAN status: cannot read TWAI status: " << esp_err_to_name(err));
            return;
        }
        log_info("CAN status: state=" << twai_state_name(status.state) << " tx_err=" << status.tx_error_counter
                                      << " rx_err=" << status.rx_error_counter << " bus_errors=" << status.bus_error_count
                                      << " tx_queue=" << status.msgs_to_tx << " rx_queue=" << status.msgs_to_rx
                                      << " bus_off_count=" << _bus_off_count);
        log_info("CAN config: tx=" << _tx.name() << " rx=" << _rx.name() << " baud=" << _baud
                                   << " command_id=" << uint32_t(_command_id) << " echoes=" << (_echo_state ? "on" : "off"));
        if (_inhibited) {
            can_log_error("CAN control inhibited: " << (_inhibit_reason ? _inhibit_reason : "unknown reason"));
        }
    }

    void CanBus::clear_driver_faults() {
        if (!driver_ok("clear")) return;
        if (!_ready) {
            log_info("CAN clear: bus not ready");
            return;
        }
        Lock lock(*this);

        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];

        // 0xF9 releases the stall protection.  Unlike 0xFB it does not clear
        // the enable/brake state; in testing on firmware V1.6 a broadcast
        // 0xFB made all four drivers stop answering until they were power
        // cycled, so this command deliberately avoids 0xFB and broadcast.
        log_info("CAN: releasing stall protection (0xF9) on addresses 1..8");
        int acked = 0;
        for (uint8_t address = 1; address <= 8; ++address) {
            size_t len = Pd42s1Protocol::build_clear_stall_command(address, request, sizeof(request));
            for (int attempt = 0; attempt < 2; ++attempt) {
                size_t responseLen = 0;
                if (transact(request, len, response, sizeof(response), responseLen, 50) &&
                    Pd42s1Protocol::parse_ack(response, responseLen)) {
                    acked++;
                    break;
                }
            }
        }
        log_info("CAN: 0xF9 stall-release acknowledged by " << acked << " address(es)");
    }

    void CanBus::report_driver_faults() {
        if (!driver_ok("faults")) return;
        if (!_ready) {
            log_info("CAN faults: bus not ready");
            return;
        }
        Lock lock(*this);

        auto read_byte = [this](uint8_t address, uint8_t function, uint8_t& value) {
            uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
            size_t  len = Pd42s1Protocol::build_read_command(address, function, request, sizeof(request));
            for (int attempt = 0; attempt < 2; ++attempt) {
                size_t responseLen = 0;
                if (transact(request, len, response, sizeof(response), responseLen, 50) &&
                    Pd42s1Protocol::parse_byte_value(response, responseLen, function, value)) {
                    return true;
                }
            }
            return false;
        };

        int found    = 0;
        int abnormal = 0;
        for (uint8_t address = 1; address <= 8; ++address) {
            uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
            size_t  len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));

            uint8_t firmware = 0;
            uint8_t hardware = 0;
            bool    have     = false;
            for (int attempt = 0; attempt < 2 && !have; ++attempt) {
                size_t responseLen = 0;
                have = transact(request, len, response, sizeof(response), responseLen, 50) &&
                       Pd42s1Protocol::parse_version(response, responseLen, firmware, hardware);
            }
            if (!have) {
                continue;
            }
            found++;

            uint8_t status = 0xFF;
            uint8_t stall  = 0xFF;
            uint8_t enable = 0xFF;
            uint8_t inpos  = 0xFF;
            read_byte(address, Pd42s1Protocol::FUNC_READ_STATUS, status);
            read_byte(address, Pd42s1Protocol::FUNC_READ_STALL, stall);
            read_byte(address, Pd42s1Protocol::FUNC_READ_ENABLE, enable);
            read_byte(address, Pd42s1Protocol::FUNC_READ_IN_POSITION, inpos);

            log_info("  address " << int(address) << ": fw V" << int(firmware / 10) << "." << int(firmware % 10) << " hw V"
                                  << int(hardware / 10) << "." << int(hardware % 10) << " status=" << motor_status_name(status)
                                  << " stall=" << (stall == 0xFF ? "?" : (stall ? "YES" : "no"))
                                  << " enabled=" << (enable == 0xFF ? "?" : (enable ? "no" : "yes"))
                                  << " in-position=" << (inpos == 0xFF ? "?" : (inpos ? "yes" : "no")));
            if (status >= 3 || stall == 1) {
                abnormal++;
            }
        }

        if (!found) {
            log_info("CAN faults: (no devices)");
        } else if (abnormal) {
            log_warn("CAN faults: " << abnormal << " of " << found << " driver(s) abnormal");
        } else {
            log_info("CAN faults: no abnormal drivers");
        }
    }

    bool CanBus::read_driver_config(uint8_t address, uint16_t& can_baud_kbps, uint8_t& echo, uint8_t& mode) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[64];
        size_t  len = Pd42s1Protocol::build_read_command(address, 0x32, request, sizeof(request));
        for (int attempt = 0; attempt < 2; ++attempt) {
            flush_receive();
            if (!send_message(request, len, 150)) {
                continue;
            }
            // The 0x32 response is 50 bytes across several CAN frames.  Collect
            // it directly instead of relying on transact()'s length logic.
            size_t   total    = 0;
            uint32_t deadline = now_ms() + 300;
            while (total < sizeof(response)) {
                int32_t left = int32_t(deadline - now_ms());
                if (left <= 0) {
                    break;
                }
                size_t received = receive_chunk(response + total, sizeof(response) - total, left > 50 ? 50 : uint32_t(left));
                if (received == 0) {
                    break;
                }
                total += received;
            }
            if (total == 50 && response[0] == 0xC5 && response[49] == 0x5C &&
                Pd42s1Protocol::checksum(response, 48) == response[48] && response[2] == 0x32 &&
                response[3] == Pd42s1Protocol::STATUS_OK) {
                mode          = response[4];
                echo          = response[5];
                can_baud_kbps = uint16_t((uint16_t(response[10]) << 8) | response[11]);
                return true;
            }
        }
        return false;
    }

    void CanBus::report_driver_configs() {
        if (!driver_ok("config")) return;
        if (!_ready) {
            log_info("CAN config: bus not ready");
            return;
        }
        Lock lock(*this);
        int  found = 0;
        for (uint8_t address = 1; address <= 8; ++address) {
            uint16_t baud = 0;
            uint8_t  echo = 0;
            uint8_t  mode = 0;
            if (read_driver_config(address, baud, echo, mode)) {
                found++;
                log_info("  address " << int(address) << ": mode=" << int(mode) << " echo=" << int(echo) << " saved-CAN-baud=" << baud
                                      << " kbps");
            }
        }
        if (!found) {
            log_info("CAN config: (no devices)");
        }
    }

    void CanBus::report_driver_positions() {
        if (!driver_ok("positions")) return;
        if (!_ready) {
            log_info("CAN positions: bus not ready");
            return;
        }
        Lock lock(*this);

        int found = 0;
        for (uint8_t address = 1; address <= 8; ++address) {
            uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
            size_t  len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));

            uint8_t firmware = 0;
            uint8_t hardware = 0;
            bool    have     = false;
            for (int attempt = 0; attempt < 2 && !have; ++attempt) {
                size_t responseLen = 0;
                have = transact(request, len, response, sizeof(response), responseLen, 50) &&
                       Pd42s1Protocol::parse_version(response, responseLen, firmware, hardware);
            }
            if (!have) {
                continue;
            }
            found++;

            int32_t position = 0;
            int32_t error    = 0;
            int16_t speed    = 0;
            uint8_t inpos    = 0xFF;
            bool    hasPos   = false;
            bool    hasError = false;
            bool    hasSpeed = false;

            for (int attempt = 0; attempt < 2 && !hasPos; ++attempt) {
                size_t responseLen = 0;
                len                = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_POSITION, request, sizeof(request));
                hasPos = transact(request, len, response, sizeof(response), responseLen, 50) &&
                         Pd42s1Protocol::parse_int32_value(response, responseLen, Pd42s1Protocol::FUNC_READ_POSITION, position);
            }
            for (int attempt = 0; attempt < 2 && !hasError; ++attempt) {
                size_t responseLen = 0;
                len                = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_POS_ERROR, request, sizeof(request));
                hasError = transact(request, len, response, sizeof(response), responseLen, 50) &&
                           Pd42s1Protocol::parse_int32_value(response, responseLen, Pd42s1Protocol::FUNC_READ_POS_ERROR, error);
            }
            for (int attempt = 0; attempt < 2 && !hasSpeed; ++attempt) {
                size_t responseLen = 0;
                len                = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_SPEED, request, sizeof(request));
                hasSpeed = transact(request, len, response, sizeof(response), responseLen, 50) &&
                           Pd42s1Protocol::parse_int16_value(response, responseLen, Pd42s1Protocol::FUNC_READ_SPEED, speed);
            }
            for (int attempt = 0; attempt < 2 && inpos == 0xFF; ++attempt) {
                size_t responseLen = 0;
                len                = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_IN_POSITION, request, sizeof(request));
                if (transact(request, len, response, sizeof(response), responseLen, 50)) {
                    Pd42s1Protocol::parse_byte_value(response, responseLen, Pd42s1Protocol::FUNC_READ_IN_POSITION, inpos);
                }
            }

            char posText[16];
            char errText[16];
            char speedText[16];
            snprintf(posText, sizeof(posText), hasPos ? "%ld" : "?", long(position));
            snprintf(errText, sizeof(errText), hasError ? "%ld" : "?", long(error));
            snprintf(speedText, sizeof(speedText), hasSpeed ? "%d" : "?", int(speed));

            log_info("  address " << int(address) << ": fw V" << int(firmware / 10) << "." << int(firmware % 10) << " pos=" << posText
                                  << " error=" << errText << " speed=" << speedText << "RPM in-position="
                                  << (inpos == 0xFF ? "?" : (inpos ? "yes" : "no")));
        }
        if (!found) {
            log_info("CAN positions: (no devices)");
        }
    }

    void CanBus::stop_all_drivers() {
        if (!driver_ok("stop")) return;
        if (!_ready) {
            log_info("CAN stop: bus not ready");
            return;
        }
        Lock lock(*this);

        // Safe stop = disable all motor outputs.  The 0xFC immediate-brake
        // command is deliberately NOT used: on firmware V1.6 the brake state
        // can only be cleared with 0xFB, and a broadcast 0xFB leaves all
        // drivers silent until they are power cycled (bench-tested).
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t disable = 1;
        size_t  len     = Pd42s1Protocol::build_frame(0, Pd42s1Protocol::FUNC_SET_MOTOR_ENABLE, &disable, 1, request, sizeof(request));
        if (!len) {
            return;
        }
        can_log_warn("CAN: safe stop: broadcasting motor disable (0xFA=1); 0xFC brake is disabled for safety");
        send_message(request, len, 100);
        flush_receive();
    }

    void CanBus::read_raw(uint8_t address, uint8_t function) {
        if (!driver_ok("read")) return;
        if (!_ready) {
            log_info("CAN read: bus not ready");
            return;
        }
        Lock lock(*this);

        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[128];
        size_t  len = Pd42s1Protocol::build_read_command(address, function, request, sizeof(request));

        char funcText[8];
        snprintf(funcText, sizeof(funcText), "0x%02X", int(function));

        flush_receive();
        if (!send_message(request, len, 150)) {
            log_warn("CAN read: address " << int(address) << " function " << funcText << ": transmit failed");
            return;
        }

        // Deliberately not using transact(): this diagnostic should show the
        // raw bytes even when the response length is not modelled yet.
        size_t   total    = 0;
        uint32_t deadline = now_ms() + 300;
        while (total < sizeof(response)) {
            int32_t left = int32_t(deadline - now_ms());
            if (left <= 0) {
                break;
            }
            size_t received = receive_chunk(response + total, sizeof(response) - total, left > 50 ? 50 : uint32_t(left));
            if (received == 0) {
                break;
            }
            total += received;
        }
        flush_receive();

        if (!total) {
            log_warn("CAN read: address " << int(address) << " function " << funcText << ": no response");
            return;
        }
        char  text[3 * 128 + 1];
        char* out = text;
        for (size_t i = 0; i < total; ++i) {
            out += snprintf(out, size_t(text + sizeof(text) - out), "%02X ", response[i]);
        }
        log_info("CAN read: address " << int(address) << " function " << funcText << " response(" << int(total) << "): " << text);
    }

    void CanBus::restart_drivers() {
        if (!driver_ok("reset")) return;
        if (!_ready) {
            log_info("CAN reset: bus not ready");
            return;
        }
        Lock lock(*this);

        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  len = Pd42s1Protocol::build_frame(0, Pd42s1Protocol::FUNC_RESTART, nullptr, 0, request, sizeof(request));
        if (!len) {
            return;
        }
        can_log_info("CAN: broadcasting driver reset (0x02)");
        send_message(request, len, 100);
        flush_receive();

        // Give the driver firmware time to boot and re-initialise its CAN
        // controller before trying to talk to it again.
        vTaskDelay(pdMS_TO_TICKS(2500));
        flush_receive();

        send_echo_state_forced(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        for (uint8_t address = 1; address <= 8; ++address) {
            enable_driver_echo_locked(address);
        }
        flush_receive();
        can_log_info("CAN: driver reset done, command echo re-enabled; run $CANScan");
    }

    void CanBus::worker() {
        uint32_t now         = now_ms();
        _last_state_check_ms = now;

        while (!_stop_requested) {
            now = now_ms();

            CanBusClient* snapshot[MAX_CLIENTS];
            size_t        count = 0;
            xSemaphoreTake(_list_mutex, portMAX_DELAY);
            count = _n_clients;
            for (size_t i = 0; i < count; ++i) {
                snapshot[i] = _clients[i];
            }
            xSemaphoreGive(_list_mutex);

            if (!_inhibited) {
                for (size_t i = 0; i < count; ++i) {
                    if (snapshot[i]) {
                        snapshot[i]->can_poll(now);
                    }
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            if (now - _last_state_check_ms >= 500) {
                _last_state_check_ms = now;
                check_state();
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }

        _task_exited = true;
        vTaskDelete(nullptr);
    }

    void CanBus::worker_entry(void* arg) {
        static_cast<CanBus*>(arg)->worker();
    }

    void CanBus::stop_worker() {
        if (!_task_handle) {
            return;
        }
        _stop_requested = true;
        // A transaction is bounded by the protocol timeout (well under two
        // seconds), so the worker normally exits promptly.  If it does not,
        // leave it and its mutexes alone rather than deleting a mutex that the
        // task might still be holding.
        for (int i = 0; i < 400 && !_task_exited; ++i) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (!_task_exited) {
            can_log_error("CAN: worker task did not stop");
            _task_handle = nullptr;
            _bus_mutex   = nullptr;
            _list_mutex  = nullptr;
            return;
        }
        _task_handle = nullptr;
    }

    CanBus::~CanBus() {
        stop_worker();
        if (_ready) {
            twai_stop();
            twai_driver_uninstall();
            _ready = false;
        }
        if (_bus_mutex) {
            vSemaphoreDelete(_bus_mutex);
            _bus_mutex = nullptr;
        }
        if (_list_mutex) {
            vSemaphoreDelete(_list_mutex);
            _list_mutex = nullptr;
        }
    }
}

#endif  // MAX_N_CAN
