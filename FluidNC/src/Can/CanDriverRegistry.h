// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "CanDriver.h"

#    include <cstring>
#    include <string>
#    include <vector>

namespace Machine {
    class CanBus;

    // Factory signature for a vendor driver: the driver receives the generic
    // bus transport and owns all protocol-specific logic.
    using CanDriverFactory = CanDriver* (*)(CanBus* bus);

    class CanDriverRegistry {
    public:
        // isDefault marks the driver selected when can.driver is omitted.  The
        // first registered default wins.
        static void add(const char* name, CanDriverFactory factory, bool isDefault = false) {
            for (const auto& entry : entries()) {
                if (std::strcmp(entry.name, name) == 0) {
                    return;
                }
            }
            entries().push_back(Entry { name, factory, isDefault });
        }

        static bool known(const char* name) {
            for (const auto& entry : entries()) {
                if (std::strcmp(entry.name, name) == 0) {
                    return true;
                }
            }
            return false;
        }

        static CanDriver* create(const char* name, CanBus* bus) {
            for (const auto& entry : entries()) {
                if (std::strcmp(entry.name, name) == 0) {
                    return entry.factory(bus);
                }
            }
            return nullptr;
        }

        // Name used when can.driver is empty.  Returns nullptr when no driver
        // is registered at all.
        static const char* default_name() {
            for (const auto& entry : entries()) {
                if (entry.isDefault) {
                    return entry.name;
                }
            }
            return entries().empty() ? nullptr : entries().front().name;
        }

        static std::string names() {
            std::string result;
            for (const auto& entry : entries()) {
                if (!result.empty()) {
                    result += ", ";
                }
                result += entry.name;
            }
            return result;
        }

    private:
        struct Entry {
            const char*      name;
            CanDriverFactory factory;
            bool             isDefault;
        };

        static std::vector<Entry>& entries() {
            static std::vector<Entry> list;
            return list;
        }
    };

    struct CanDriverRegistration {
        CanDriverRegistration(const char* name, CanDriverFactory factory, bool isDefault = false) {
            CanDriverRegistry::add(name, factory, isDefault);
        }
    };
}

#endif
