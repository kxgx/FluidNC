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

#    include <cstring>

namespace Machine {
    using MotorDrivers::Pd42s1Protocol;

    namespace {
        uint32_t now_ms() {
            return uint32_t(esp_timer_get_time() / 1000);
        }

        uint32_t twai_tx_error_counter() {
            twai_status_info_t status;
            return twai_get_status_info(&status) == ESP_OK ? status.tx_error_counter : 0;
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
        if (_ready) {
            return;
        }
        if (!configured()) {
            log_verbose("CAN: bus not configured");
            return;
        }

        _bus_mutex  = xSemaphoreCreateMutex();
        _list_mutex = xSemaphoreCreateMutex();
        if (!_bus_mutex || !_list_mutex) {
            log_error("CAN: cannot create mutexes");
            return;
        }

        if (!install_driver()) {
            return;
        }
        start_worker();
        log_info("CAN: TX:" << _tx.name() << " RX:" << _rx.name() << " baud:" << _baud << " command_id:" << uint32_t(_command_id));
    }

    bool CanBus::install_driver() {
        twai_timing_config_t timing;
        if (!timing_for_baud(_baud, timing)) {
            log_error("CAN: unsupported baud " << _baud);
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
            log_error("CAN: driver install failed: " << esp_err_to_name(err));
            return false;
        }
        err = twai_start();
        if (err != ESP_OK) {
            log_error("CAN: start failed: " << esp_err_to_name(err));
            twai_driver_uninstall();
            return false;
        }
        return true;
    }

    void CanBus::start_worker() {
        _stop_requested = false;
        _task_exited    = false;
        _ready          = true;

        if (xTaskCreate(worker_entry, "can", 4096, this, 2, &_task_handle) != pdPASS) {
            log_error("CAN: cannot create worker task");
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
            log_error("CAN: cannot restore the bus");
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
        xSemaphoreTake(_list_mutex, portMAX_DELAY);
        if (_n_clients < MAX_CLIENTS) {
            _clients[_n_clients++] = client;
        } else {
            log_error("CAN: too many clients");
        }
        xSemaphoreGive(_list_mutex);
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

    void CanBus::send_echo_state(bool enable) {
        if (_echo_state == enable) {
            return;  // already configured, no need to retransmit or log
        }
        // Broadcast (address 0) "set command echo" (0x6F).  Broadcast
        // commands are executed but not answered, so this works even when the
        // drivers are currently set to not reply.  A present receiver ACKs
        // the frame at the CAN level regardless of its echo setting.
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t echo = enable ? 0 : 1;
        size_t  len  = Pd42s1Protocol::build_frame(0, Pd42s1Protocol::FUNC_SET_ECHO, &echo, 1, request, sizeof(request));
        if (len) {
            log_info("CAN: " << (enable ? "enabling" : "disabling") << " command echo (broadcast 0x6F=" << int(echo) << ")");
            if (send_message(request, len, 50)) {
                _echo_state = enable;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        flush_receive();
    }

    void CanBus::send_echo_state_forced(bool enable) {
        // The scan must transmit the echo setting every time, even when the
        // cached state says it is already correct: a driver may have missed an
        // earlier broadcast (for example because another driver was not
        // powered at that moment).
        _echo_state = !enable;
        send_echo_state(enable);
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

            // The requirement for a read is: enable command echo first.
            send_echo_state_forced(true);
            // Give the drivers a moment to apply the setting before reading.
            vTaskDelay(pdMS_TO_TICKS(10));

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
            log_info("  TWAI tx_err " << tecBefore << " -> " << tecAfter
                                      << (tecAfter == tecBefore ? " (frames were ACKed by a node)"
                                                                 : " (no ACK - check driver power, wiring and bit rate)"));

            // ... and disable command echo again when the read is finished.
            send_echo_state_forced(false);
        }

        return found;
    }

    int CanBus::scan_addresses() {
        int found = 0;
        for (uint8_t address = 1; address <= 8; ++address) {
            found += scan_address(address, 100);
        }
        if (!found) {
            log_info("  (no devices)");
        }
        return found;
    }

    int CanBus::scan_all_bauds() {
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
                send_echo_state_forced(true);
                vTaskDelay(pdMS_TO_TICKS(10));  // let the drivers apply the echo setting
                total += scan_addresses();
                uint32_t tecAfter = twai_tx_error_counter();
                log_info("  TWAI tx_err " << tecBefore << " -> " << tecAfter
                                          << (tecAfter == tecBefore ? " (frames were ACKed by a node)"
                                                                     : " (no ACK - check driver power, wiring and bit rate)"));
                // Leave the drivers in the requested "no unsolicited replies"
                // state after every rate, including the last one.
                send_echo_state_forced(false);
                twai_stop();
                twai_driver_uninstall();
                _ready = false;
            }
        }

        _baud = configured;
        if (!install_driver()) {
            log_error("CAN: cannot restore the bus after the baud scan");
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
        if (status.state == TWAI_STATE_BUS_OFF) {
            log_error("CAN: bus off, starting recovery");
            twai_initiate_recovery();
        } else if (status.state == TWAI_STATE_STOPPED) {
            twai_start();
        }
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

            for (size_t i = 0; i < count; ++i) {
                if (snapshot[i]) {
                    snapshot[i]->can_poll(now);
                }
            }

            if (now - _last_state_check_ms >= 500) {
                _last_state_check_ms = now;
                check_state();
            }

            vTaskDelay(pdMS_TO_TICKS(2));
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
            log_error("CAN: worker task did not stop");
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
