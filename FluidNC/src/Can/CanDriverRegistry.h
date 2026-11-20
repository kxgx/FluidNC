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

    // Transitional factory signature: pd42s1 currently reuses the CanBus
    // object itself, while a standalone vendor driver can ignore the argument.
    using CanDriverFactory = CanDriver* (*)(CanBus* bus);

    class CanDriverRegistry {
    public:
        static void add(const char* name, CanDriverFactory factory) {
            for (const auto& entry : entries()) {
                if (std::strcmp(entry.name, name) == 0) {
                    return;
                }
            }
            entries().push_back(Entry { name, factory });
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
        };

        static std::vector<Entry>& entries() {
            static std::vector<Entry> list;
            return list;
        }
    };

    struct CanDriverRegistration {
        CanDriverRegistration(const char* name, CanDriverFactory factory) { CanDriverRegistry::add(name, factory); }
    };
}

#endif
