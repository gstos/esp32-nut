#ifndef UPS_TIMER_LOGIC_H
#define UPS_TIMER_LOGIC_H

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <vector>
#include "HIDUsages.h"
#include "BeeperLogic.h"

// The two HID PDC countdowns behind the NUT shutdown commands (issue #23)
enum class UPSTimer : uint8_t {
    Shutdown, // DelayBeforeShutdown: the outlets switch off when it reaches 0
    Startup   // DelayBeforeStartup: the outlets switch back on when it reaches 0
};

class UPSTimerLogic {
public:
    // -1 stops a countdown (usbhid-ups shutdown.stop)
    static constexpr int32_t CANCEL = -1;
    // usbhid-ups DEFAULT_OFFDELAY / DEFAULT_ONDELAY
    static constexpr int32_t DEFAULT_SHUTDOWN_S = 20;
    static constexpr int32_t DEFAULT_STARTUP_S = 30;
    // 0 means "switch off now": a bug or a typo must never get there
    static constexpr int32_t MIN_DELAY_S = 10;
    // usbhid-ups waits this long between the startup and the shutdown write
    static constexpr uint32_t STEP_GAP_MS = 125;

    // usbhid-ups / apc-hid.c lookup order. APCGeneralCollection paths are left out:
    // on a Smart-UPS the APC vendor delays are the stored configuration, not countdowns.
    static const char* const* candidatePaths(UPSTimer timer) {
        static const char* const shutdown_paths[] = {
            "UPS.PowerSummary.DelayBeforeShutdown",
            "UPS.Output.DelayBeforeShutdown",
            nullptr,
        };
        static const char* const startup_paths[] = {
            "UPS.PowerSummary.DelayBeforeStartup",
            "UPS.Output.DelayBeforeStartup",
            nullptr,
        };
        return timer == UPSTimer::Shutdown ? shutdown_paths : startup_paths;
    }

    /**
     * The field a timer command writes, or nullptr when the device has none that is
     * safe to write. A candidate is used only when it is a countdown (it accepts -1,
     * which a stored setting does not), sits alone in a FEATURE report at bit 0 (so the
     * write carries nothing but this value) and is 16 to 32 bits wide.
     */
    static const HIDUsageDef* resolve(const std::vector<HIDUsageDef>& usages, UPSTimer timer) {
        for (const char* const* p = candidatePaths(timer); *p; p++) {
            for (const auto& u : usages) {
                if (strcmp(u.path, *p) != 0) continue;
                if (isWritableTimer(usages, u)) return &u;
            }
        }
        return nullptr;
    }

    static bool isWritableTimer(const std::vector<HIDUsageDef>& usages, const HIDUsageDef& def) {
        if (def.report_type != 0x03) return false; // the parser files OUTPUT fields as FEATURE too
        if (def.logical_min != CANCEL) return false;
        if (def.bit_offset != 0 || def.bit_size < 16 || def.bit_size > 32 || def.bit_size % 8 != 0) return false;
        return !BeeperLogic::reportHasOtherFields(usages, def);
    }

    // -1, or a delay of at least MIN_DELAY_S that the field can hold
    static bool valueAllowed(const HIDUsageDef& def, int32_t seconds) {
        if (seconds == CANCEL) return true;
        return seconds >= MIN_DELAY_S && seconds <= def.logical_max;
    }

    // SET_REPORT payload: report ID (if any), then the value little endian in two's
    // complement. Returns the length, 0 when the value is not allowed or does not fit.
    static size_t encode(const HIDUsageDef& def, int32_t seconds, uint8_t* buffer, size_t capacity) {
        if (!buffer || !valueAllowed(def, seconds)) return 0;
        size_t value_bytes = def.bit_size / 8;
        size_t len = (def.report_id != 0 ? 1 : 0) + value_bytes;
        if (len > capacity) return 0;
        size_t i = 0;
        if (def.report_id != 0) buffer[i++] = def.report_id;
        uint32_t raw = (uint32_t)seconds;
        for (size_t b = 0; b < value_bytes; b++) buffer[i++] = (uint8_t)(raw >> (8 * b));
        return len;
    }

    struct Step {
        UPSTimer timer;
        int32_t seconds;
    };

    // Whole seconds only: "-1" or digits. No signs, units or exponents.
    static bool parseSeconds(const String& text, int32_t& seconds) {
        if (text == "-1") { seconds = CANCEL; return true; }
        if (text.length() == 0 || text.length() > 6) return false;
        int32_t v = 0;
        for (size_t i = 0; i < text.length(); i++) {
            char c = text[i];
            if (c < '0' || c > '9') return false;
            v = v * 10 + (c - '0');
        }
        seconds = v;
        return true;
    }

    /**
     * The timer writes behind a NUT command, in order, as usbhid-ups sends them.
     * An optional value replaces the shutdown delay (the startup delay for
     * load.on.delay). Returns false with a NUT error name; nothing may be written then.
     */
    static bool plan(const String& command, const String* value, std::vector<Step>& steps, const char*& error) {
        steps.clear();
        error = nullptr;
        bool takes_value = command == "load.off.delay" || command == "load.on.delay" ||
                           command == "shutdown.return" || command == "shutdown.stayoff";
        if (!takes_value && command != "shutdown.stop") {
            error = "CMD-NOT-SUPPORTED";
            return false;
        }
        int32_t delay = command == "load.on.delay" ? DEFAULT_STARTUP_S : DEFAULT_SHUTDOWN_S;
        if (value) {
            if (!takes_value || !parseSeconds(*value, delay)) {
                error = "INVALID-ARGUMENT";
                return false;
            }
            // Only load.on.delay may cancel through a value; a cancelled shutdown is shutdown.stop
            bool cancel_ok = command == "load.on.delay" && delay == CANCEL;
            if (!cancel_ok && delay < MIN_DELAY_S) {
                error = "INVALID-ARGUMENT";
                return false;
            }
        }

        if (command == "load.off.delay") {
            steps.push_back({UPSTimer::Shutdown, delay});
        } else if (command == "load.on.delay") {
            steps.push_back({UPSTimer::Startup, delay});
        } else if (command == "shutdown.stop") {
            steps.push_back({UPSTimer::Shutdown, CANCEL});
        } else if (command == "shutdown.return") {
            // The startup countdown runs alongside the shutdown one: it must end later
            int32_t startup = delay + (DEFAULT_STARTUP_S - DEFAULT_SHUTDOWN_S);
            steps.push_back({UPSTimer::Startup, startup});
            steps.push_back({UPSTimer::Shutdown, delay});
        } else { // shutdown.stayoff
            steps.push_back({UPSTimer::Startup, CANCEL});
            steps.push_back({UPSTimer::Shutdown, delay});
        }
        return true;
    }

    // The timers a command needs, for LIST CMD
    static bool needsShutdown(const String& command) { return command != "load.on.delay"; }
    static bool needsStartup(const String& command) {
        return command == "load.on.delay" || command == "shutdown.return" || command == "shutdown.stayoff";
    }

    static const char* const* commands() {
        static const char* const names[] = {
            "load.off.delay", "load.on.delay", "shutdown.return", "shutdown.stayoff", "shutdown.stop", nullptr,
        };
        return names;
    }

    static const char* dataKey(UPSTimer timer) {
        return timer == UPSTimer::Shutdown ? "ups.timer.shutdown" : "ups.timer.start";
    }
};

#endif // UPS_TIMER_LOGIC_H
