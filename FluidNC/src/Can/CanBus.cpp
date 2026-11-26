// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "CanBus.h"
#    include "CanDriverRegistry.h"

#    include "Logging.h"

#    include <Arduino.h>
#    include <driver/twai.h>
#    include <esp_timer.h>

#    include <cstdio>
#    include <cstring>

namespace Machine {

    bool    CanBus::_driver_configured                   = false;
    int     CanBus::_driver_count                        = 0;
    bool    CanBus::_logging_enabled                     = false;
    uint8_t CanBus::_configuredAddresses[CanBus::MAX_CLIENTS] = {};
    size_t  CanBus::_configuredAddressCount              = 0;

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

    namespace {
        uint32_t now_ms() {
            return uint32_t(esp_timer_get_time() / 1000);
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

    const char* CanBus::effective_driver_name() const {
        if (!_driverName.empty()) {
            return _driverName.c_str();
        }
        return CanDriverRegistry::default_name();
    }

    CanDriver* CanBus::driver() {
        if (_driver && _resolvedDriverName == _driverName) {
            return _driver;
        }
        delete _driver;
        _driver = nullptr;

        const char* name = effective_driver_name();
        if (name) {
            _driver = CanDriverRegistry::create(name, this);
        }
        if (!_driver) {
            // Keep the command layer's non-null invariant.  validate() turns
            // an unknown name or a failed factory into a configuration error
            // before use; the stub owns no hardware and never touches the bus.
            can_log_error("CAN: cannot create can.driver '" << (name ? name : "(none)") << "'");
            _driver = CanDriverRegistry::create("null_can", this);
        }
        _resolvedDriverName = _driverName;
        return _driver;
    }

    const char* CanBus::driver_name() const {
        if (_driver) {
            return _driver->driver_name();
        }
        const char* name = effective_driver_name();
        return name ? name : "none";
    }

    void CanBus::group(Configuration::HandlerBase& handler) {
        // @config driver
        // @default "" (the default driver registered by the firmware)
        // CAN motor protocol.  Every vendor protocol is a CanDriver
        // implementation registered in CanDriverRegistry, so adding a vendor
        // does not require changes in GCode.cpp or ProcessSettings.cpp.
        handler.item("driver", _driverName);

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
        // in the motor drivers.  This transport supports 125000, 250000,
        // 500000 and 1000000.
        handler.item("baud", _baud);
    }

    void CanBus::validate() {
        const char* name = effective_driver_name();
        if (!name) {
            log_error("CAN: no CAN driver is registered");
            Assert(false, "no CAN driver registered");
        }
        if (!CanDriverRegistry::known(name)) {
            log_error("Unknown can.driver '" << name << "'; supported: " << CanDriverRegistry::names());
            Assert(false, "unknown can.driver");
        }
        CanDriver* selected = driver();
        if (!selected || std::strcmp(selected->driver_name(), name) != 0) {
            log_error("can.driver factory failed for '" << name << "'");
            Assert(false, "can.driver factory failed");
        }
        if (!selected->uses_transport()) {
            return;
        }
        if (!configured()) {
            return;  // A CAN motor driver will report the missing bus.
        }
        Assert(_tx.defined() && _rx.defined(), "CAN bus needs both tx_pin and rx_pin");
        Assert(_tx != _rx, "CAN tx_pin and rx_pin must be different pins");
        Assert(_baud == 125000 || _baud == 250000 || _baud == 500000 || _baud == 1000000,
               "CAN baud must be 125000, 250000, 500000 or 1000000");
    }

    void CanBus::init() {
        CanDriver* selected = driver();
        if (!selected) {
            can_log_error("CAN: no can.driver selected");
            return;
        }
        if (!selected->uses_transport()) {
            can_log_info("CAN: driver '" << selected->driver_name() << "' selected; TWAI transport not used");
            return;
        }
        if (!_driver_configured) {
            can_log_verbose("CAN: no CAN motor configured; CAN disabled");
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

        // Uniform startup order for every vendor driver: create the registry
        // driver, install the shared TWAI transport, start the worker and then
        // let the driver run its own protocol initialisation.
        if (!install_transport()) {
            return;
        }
        // The first worker start is part of the driver's initialisation (some
        // drivers suspend/resume the transport while probing).  After this
        // point a worker may only be restarted when control_ready is true.
        _initializing = true;
        start_worker();
        if (!selected->init_driver()) {
            can_log_error("CAN: driver '" << selected->driver_name() << "' init failed");
            set_control_ready(false);
            stop_worker();
            _initializing = false;
            return;
        }
        _initializing = false;
        if (!_task_handle) {
            // The driver may have suspended/resumed the transport; a worker
            // that did not come back up means control is not available even
            // though the driver initialised.
            can_log_error("CAN: driver '" << selected->driver_name() << "' initialised but the bus worker is not running");
            set_control_ready(false);
            return;
        }
        set_control_ready(true);
        can_log_info("CAN: TX:" << _tx.name() << " RX:" << _rx.name() << " baud:" << _baud << " driver:" << selected->driver_name());
    }

    bool CanBus::install_transport() {
        return install_transport(_baud);
    }

    bool CanBus::install_transport(uint32_t baud) {
        if (_ready) {
            uninstall_transport();
        }

        twai_timing_config_t timing;
        if (!timing_for_baud(baud, timing)) {
            can_log_error("CAN: unsupported baud " << baud);
            return false;
        }

        auto tx_pin = _tx.getNative(Pin::Capabilities::Native | Pin::Capabilities::Output);
        auto rx_pin = _rx.getNative(Pin::Capabilities::Native | Pin::Capabilities::Input);
        if (tx_pin == INVALID_PINNUM || rx_pin == INVALID_PINNUM) {
            can_log_error("CAN: tx_pin and rx_pin must be native GPIOs");
            return false;
        }

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

        _baud  = baud;
        _ready = true;
        return true;
    }

    void CanBus::uninstall_transport() {
        if (!_ready) {
            return;
        }
        twai_stop();
        twai_driver_uninstall();
        _ready = false;
        // A new transport starts with an empty history; do not insert an
        // artificial gap before the first message after reinstall.
        _message_sent = false;
    }

    bool CanBus::suspend() {
        if (!_ready) {
            return true;
        }
        if (!stop_worker()) {
            can_log_error("CAN: suspend aborted; worker did not stop");
            return false;
        }
        uninstall_transport();
        delay(5);
        return true;
    }

    bool CanBus::resume() {
        if (!_ready && !install_transport()) {
            can_log_error("CAN: cannot restore the bus");
            return false;
        }
        if (!_task_handle) {
            start_worker();
        }
        return _ready && _task_handle != nullptr;
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
            can_log_error("CAN: duplicate CAN address " << int(duplicate) << " configured; stopping all CAN control");
            inhibit_control("duplicate CAN address");
        }
    }

    bool CanBus::has_client(uint8_t address) const {
        for (size_t i = 0; i < _n_clients; ++i) {
            if (_clients[i] && _clients[i]->bus_address() == address) {
                return true;
            }
        }
        return false;
    }

    bool CanBus::client_synced(uint8_t address) const {
        for (size_t i = 0; i < _n_clients; ++i) {
            CanBusClient* client = _clients[i];
            if (client && client->bus_address() == address && client->is_synced()) {
                return true;
            }
        }
        return false;
    }

    size_t CanBus::client_addresses(uint8_t* out, size_t max) const {
        size_t count = 0;
        for (size_t i = 0; i < _n_clients && count < max; ++i) {
            CanBusClient* client = _clients[i];
            if (!client) {
                continue;
            }
            uint8_t address = client->bus_address();
            if (!address) {
                continue;
            }
            bool duplicate = false;
            for (size_t j = 0; j < count; ++j) {
                if (out[j] == address) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                out[count++] = address;
            }
        }
        return count;
    }

    void CanBus::invalidate_client_position(uint8_t address) {
        for (size_t i = 0; i < _n_clients; ++i) {
            CanBusClient* client = _clients[i];
            if (client && client->bus_address() == address) {
                client->on_position_zeroed();
            }
        }
    }

    void CanBus::invalidate_all_client_positions() {
        for (size_t i = 0; i < _n_clients; ++i) {
            if (_clients[i]) {
                _clients[i]->on_position_zeroed();
            }
        }
    }

    void CanBus::notify_diagnostic_complete() {
        for (size_t i = 0; i < _n_clients; ++i) {
            if (_clients[i]) {
                _clients[i]->on_diagnostic_complete();
            }
        }
    }

    void CanBus::end_diagnostic() {
        _diagnostic_active.store(false, std::memory_order_release);
        notify_diagnostic_complete();
    }

    void CanBus::wait_for_worker_idle() {
        while (_worker_busy && !_task_exited) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    void CanBus::inhibit_control(const char* reason) {
        _inhibited      = true;
        _inhibit_reason = reason;
        can_log_error("CAN: control inhibited: " << reason);
        if (!_ready) {
            return;
        }
        // Use the driver's latched, readback-verified stop path.  It waits for
        // an in-flight worker poll so it cannot re-enable a motor after the
        // inhibit was raised.
        CanDriver* selected = driver();
        if (selected) {
            selected->stop_all_drivers();
        }
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

    void CanBus::register_configured_address(uint8_t address) {
        if (address < 1 || address > MAX_CLIENTS) {
            return;
        }
        for (size_t i = 0; i < _configuredAddressCount; ++i) {
            if (_configuredAddresses[i] == address) {
                return;
            }
        }
        if (_configuredAddressCount < MAX_CLIENTS) {
            _configuredAddresses[_configuredAddressCount++] = address;
        }
    }

    void CanBus::unregister_configured_address(uint8_t address) {
        for (size_t i = 0; i < _configuredAddressCount; ++i) {
            if (_configuredAddresses[i] == address) {
                for (size_t j = i + 1; j < _configuredAddressCount; ++j) {
                    _configuredAddresses[j - 1] = _configuredAddresses[j];
                }
                --_configuredAddressCount;
                return;
            }
        }
    }

    size_t CanBus::configured_addresses(uint8_t* out, size_t max) {
        size_t count = _configuredAddressCount < max ? _configuredAddressCount : max;
        for (size_t i = 0; i < count; ++i) {
            out[i] = _configuredAddresses[i];
        }
        return count;
    }

    bool CanBus::driver_configured() {
        return _driver_configured;
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

    bool CanBus::send_frame(uint32_t id, bool extended, const uint8_t* data, size_t len, uint32_t timeout_ms) {
        if (!_ready) {
            return false;
        }
        // CAN data frames carry at most 8 bytes.  A longer request would be
        // silently truncated by the controller, so reject it instead.
        if (len > TWAI_FRAME_MAX_DLC) {
            can_log_error("CAN: frame with " << int(len) << " bytes exceeds the 8-byte CAN limit");
            return false;
        }
        if (len && !data) {
            can_log_error("CAN: frame data is null");
            return false;
        }
        // Keep standard and extended identifiers in their legal ranges.  A
        // misconfigured COB-ID must not wrap into another node's identifier.
        const uint32_t maxId = extended ? 0x1FFFFFFFu : 0x7FFu;
        if (id > maxId) {
            can_log_error("CAN: identifier " << id << (extended ? " is not a 29-bit extended id" : " is not an 11-bit standard id"));
            return false;
        }

        twai_message_t message   = {};
        message.extd             = extended ? 1 : 0;
        message.identifier       = id;
        message.data_length_code = len;
        if (len) {
            memcpy(message.data, data, len);
        }
        return twai_transmit(&message, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
    }

    bool CanBus::send_message(uint32_t can_id, const uint8_t* data, size_t len, uint32_t timeout_ms) {
        if (!_ready) {
            return false;
        }

        if (_min_message_gap_ms && _message_sent) {
            uint32_t elapsed = now_ms() - _last_message_end_ms;
            if (elapsed < _min_message_gap_ms) {
                vTaskDelay(pdMS_TO_TICKS(_min_message_gap_ms - elapsed));
            }
        }

        bool   ok     = true;
        size_t offset = 0;
        while (offset < len) {
            size_t chunk = len - offset;
            if (chunk > TWAI_FRAME_MAX_DLC) {
                chunk = TWAI_FRAME_MAX_DLC;
            }
            // Vendor protocols built on this helper use 29-bit extended
            // identifiers, so the logical-message path keeps that format even
            // though send_frame() itself supports standard frames too.
            if (!send_frame(can_id, true, data + offset, chunk, timeout_ms)) {
                ok = false;
                break;
            }
            offset += chunk;
        }

        _last_message_end_ms = now_ms();
        _message_sent        = true;
        return ok;
    }

    size_t CanBus::receive_frame(uint32_t& id, bool& extended, uint8_t* buf, size_t max_len, uint32_t timeout_ms) {
        id       = 0;
        extended = false;
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
        if (len && buf) {
            memcpy(buf, message.data, len);
        }
        id       = message.identifier;
        extended = message.extd != 0;
        return len;
    }

    size_t CanBus::receive_chunk(uint8_t* buf, size_t max_len, uint32_t timeout_ms) {
        uint32_t id       = 0;
        bool     extended = false;
        return receive_frame(id, extended, buf, max_len, timeout_ms);
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

    bool CanBus::bus_status(BusStatus& out) const {
        twai_status_info_t status;
        if (twai_get_status_info(&status) != ESP_OK) {
            return false;
        }
        out.state         = twai_state_name(status.state);
        out.tx_error      = status.tx_error_counter;
        out.rx_error      = status.rx_error_counter;
        out.bus_errors    = status.bus_error_count;
        out.msgs_to_tx    = status.msgs_to_tx;
        out.msgs_to_rx    = status.msgs_to_rx;
        out.bus_off_count = _bus_off_count;
        return true;
    }

    uint32_t CanBus::tx_error_counter() const {
        twai_status_info_t status;
        return twai_get_status_info(&status) == ESP_OK ? status.tx_error_counter : 0;
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

    void CanBus::start_worker() {
        if (_task_handle) {
            can_log_error("CAN: worker already running; refusing duplicate task");
            return;
        }
        if (!_ready || !_bus_mutex) {
            can_log_error("CAN: refusing to start worker; transport is not ready");
            return;
        }
        if (!_initializing && !_control_ready) {
            can_log_error("CAN: refusing to start worker; control is not ready (driver init/auto-config failed)");
            return;
        }
        _stop_requested = false;
        _task_exited    = false;

        // Keep the CAN target/stop worker on the motion core instead of letting
        // the scheduler place it on core0 where WiFi/lwIP/AsyncTCP preempt it.
        if (xTaskCreatePinnedToCore(worker_entry, "can", 4096, this, 2, &_task_handle, 1) != pdPASS) {
            can_log_error("CAN: cannot create worker task");
            _task_handle = nullptr;
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

            if (!_inhibited && !_stop_latched) {
                _worker_busy = true;
                for (size_t i = 0; i < count; ++i) {
                    if (snapshot[i]) {
                        snapshot[i]->can_poll(now);
                    }
                }
                _worker_busy = false;
            } else {
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            if (now - _last_state_check_ms >= 20) {
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

    bool CanBus::stop_worker() {
        if (!_task_handle) {
            return true;
        }
        _stop_requested = true;
        // Wait until the worker has actually exited.  Some sync/tuning paths
        // can take several seconds on a marginal bus; returning early and
        // deleting the object would leave a live task with a dangling this.
        // A hang here is safer than a use-after-free during config reload.
        uint32_t lastLog = now_ms();
        while (!_task_exited) {
            vTaskDelay(pdMS_TO_TICKS(10));
            uint32_t now = now_ms();
            if (now - lastLog >= 2000) {
                lastLog = now;
                can_log_warn("CAN: waiting for worker task to exit");
            }
        }
        _task_handle = nullptr;
        return true;
    }

    CanBus::~CanBus() {
        if (!stop_worker()) {
            // Do not delete mutexes or uninstall TWAI while the worker may
            // still be executing against this object.
            return;
        }
        uninstall_transport();
        delete _driver;
        _driver = nullptr;
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

