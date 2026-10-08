#include <Arduino.h>
#include <unity.h>
#include <fstream>
#include <sstream>
#include <ArduinoJson.h>
#include "UPSTimerLogic.h"
#include "HIDParser.h"

void setUp(void) {}
void tearDown(void) {}

static HIDUsageDef makeTimer(uint32_t usage, uint8_t id, uint16_t size, int32_t lmin, int32_t lmax, const char* path) {
    HIDUsageDef d;
    d.usage = usage;
    d.report_id = id;
    d.report_type = 3;
    d.bit_offset = 0;
    d.bit_size = size;
    d.logical_min = lmin;
    d.logical_max = lmax;
    d.found = true;
    strncpy(d.path, path, sizeof(d.path) - 1);
    return d;
}

static bool loadDescriptor(const char* file, HIDParser& parser) {
    std::ifstream f(file);
    if (!f.is_open()) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    JsonDocument doc;
    if (deserializeJson(doc, ss.str())) return false;
    std::vector<uint8_t> raw;
    for (JsonVariant v : doc["report_descriptor_hex"].as<JsonArray>()) {
        raw.push_back((uint8_t)strtol(v.as<std::string>().c_str(), nullptr, 16));
    }
    return parser.parseReportDescriptor(raw.data(), raw.size());
}

static const char* SMARTUPS_3000 = "test/fixtures/apc/apc_smartups_3000_vid051d_pid0002_issue23.json";

// --- encode ---

void test_encode_16bit_cancel_is_ffff() {
    HIDUsageDef d = makeTimer(0x00840057, 0x12, 16, -1, 32767, "UPS.PowerSummary.DelayBeforeShutdown");
    uint8_t buf[8] = {0xAA, 0xAA, 0xAA, 0xAA};
    TEST_ASSERT_EQUAL(3, UPSTimerLogic::encode(d, -1, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, buf[3]); // nothing past the report
}

void test_encode_16bit_little_endian() {
    HIDUsageDef d = makeTimer(0x00840057, 0x12, 16, -1, 32767, "UPS.PowerSummary.DelayBeforeShutdown");
    uint8_t buf[8] = {0};
    TEST_ASSERT_EQUAL(3, UPSTimerLogic::encode(d, 20, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_HEX8(0x14, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[2]);
    TEST_ASSERT_EQUAL(3, UPSTimerLogic::encode(d, 300, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_HEX8(0x2C, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[2]);
}

void test_encode_24bit_and_no_report_id() {
    HIDUsageDef d = makeTimer(0x00840055, 0, 24, -1, 1295640, "UPS.Output.DelayBeforeReboot");
    uint8_t buf[8] = {0};
    TEST_ASSERT_EQUAL(3, UPSTimerLogic::encode(d, 70000, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_HEX8(0x70, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x11, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[2]);
}

void test_encode_refuses_instant_and_out_of_range() {
    HIDUsageDef d = makeTimer(0x00840057, 0x12, 16, -1, 32767, "UPS.PowerSummary.DelayBeforeShutdown");
    uint8_t buf[8] = {0};
    // 0 is "switch off now"
    TEST_ASSERT_EQUAL(0, UPSTimerLogic::encode(d, 0, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(0, UPSTimerLogic::encode(d, 9, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(0, UPSTimerLogic::encode(d, -2, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(0, UPSTimerLogic::encode(d, 32768, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(3, UPSTimerLogic::encode(d, 10, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(3, UPSTimerLogic::encode(d, 32767, buf, sizeof(buf)));
    // Too small a buffer
    TEST_ASSERT_EQUAL(0, UPSTimerLogic::encode(d, 20, buf, 2));
}

// --- resolve ---

void test_resolve_prefers_power_summary_like_usbhid_ups() {
    std::vector<HIDUsageDef> u;
    u.push_back(makeTimer(0x00840057, 0x27, 16, -1, 32767, "UPS.Output.DelayBeforeShutdown"));
    u.push_back(makeTimer(0x00840057, 0x12, 16, -1, 32767, "UPS.PowerSummary.DelayBeforeShutdown"));
    const HIDUsageDef* d = UPSTimerLogic::resolve(u, UPSTimer::Shutdown);
    TEST_ASSERT_NOT_NULL(d);
    TEST_ASSERT_EQUAL_HEX8(0x12, d->report_id);
}

void test_resolve_skips_stored_settings() {
    // A delay that cannot be -1 is a configuration value, not a countdown
    std::vector<HIDUsageDef> u;
    u.push_back(makeTimer(0x00840057, 0x12, 16, 0, 630, "UPS.PowerSummary.DelayBeforeShutdown"));
    TEST_ASSERT_NULL(UPSTimerLogic::resolve(u, UPSTimer::Shutdown));
    u.push_back(makeTimer(0x00840057, 0x27, 16, -1, 32767, "UPS.Output.DelayBeforeShutdown"));
    const HIDUsageDef* d = UPSTimerLogic::resolve(u, UPSTimer::Shutdown);
    TEST_ASSERT_NOT_NULL(d);
    TEST_ASSERT_EQUAL_HEX8(0x27, d->report_id);
}

void test_resolve_skips_shared_reports() {
    // Another field in the same report would go out as zero
    std::vector<HIDUsageDef> u;
    u.push_back(makeTimer(0x00840057, 0x12, 16, -1, 32767, "UPS.PowerSummary.DelayBeforeShutdown"));
    HIDUsageDef other = makeTimer(0x00840056, 0x12, 16, -1, 32767, "UPS.PowerSummary.DelayBeforeStartup");
    other.bit_offset = 16;
    u.push_back(other);
    TEST_ASSERT_NULL(UPSTimerLogic::resolve(u, UPSTimer::Shutdown));
    TEST_ASSERT_NULL(UPSTimerLogic::resolve(u, UPSTimer::Startup));
}

void test_resolve_skips_input_and_odd_sizes() {
    std::vector<HIDUsageDef> u;
    HIDUsageDef in = makeTimer(0x00840057, 0x12, 16, -1, 32767, "UPS.PowerSummary.DelayBeforeShutdown");
    in.report_type = 1;
    u.push_back(in);
    u.push_back(makeTimer(0x00840057, 0x13, 8, -1, 127, "UPS.PowerSummary.DelayBeforeShutdown"));
    TEST_ASSERT_NULL(UPSTimerLogic::resolve(u, UPSTimer::Shutdown));
}

void test_resolve_ignores_apc_vendor_delays() {
    std::vector<HIDUsageDef> u;
    u.push_back(makeTimer(0xFF86007D, 0x30, 16, -1, 32767, "UPS.APCGeneralCollection.APCDelayBeforeShutdown"));
    TEST_ASSERT_NULL(UPSTimerLogic::resolve(u, UPSTimer::Shutdown));
}

void test_resolve_smartups_3000_descriptor() {
    HIDParser parser;
    TEST_ASSERT_TRUE_MESSAGE(loadDescriptor(SMARTUPS_3000, parser), "fixture");
    const auto& u = parser.getUsages();

    // What usbhid-ups would pick on this unit: shutdown in PowerSummary, startup in Output
    const HIDUsageDef* off = UPSTimerLogic::resolve(u, UPSTimer::Shutdown);
    TEST_ASSERT_NOT_NULL(off);
    TEST_ASSERT_EQUAL_STRING("UPS.PowerSummary.DelayBeforeShutdown", off->path);
    TEST_ASSERT_EQUAL_HEX8(0x12, off->report_id);
    TEST_ASSERT_EQUAL(16, off->bit_size);
    TEST_ASSERT_EQUAL(-1, off->logical_min);
    TEST_ASSERT_EQUAL(32767, off->logical_max);

    const HIDUsageDef* on = UPSTimerLogic::resolve(u, UPSTimer::Startup);
    TEST_ASSERT_NOT_NULL(on);
    TEST_ASSERT_EQUAL_STRING("UPS.Output.DelayBeforeStartup", on->path);
    TEST_ASSERT_EQUAL_HEX8(0x28, on->report_id);

    // Never one of the stored APC settings (reports 0x25/0x26)
    TEST_ASSERT_NOT_EQUAL(0x25, on->report_id);
    TEST_ASSERT_NOT_EQUAL(0x26, off->report_id);
}

void test_resolve_never_picks_a_setting_on_any_apc_fixture() {
    static const char* fixtures[] = {
        "test/fixtures/apc/apc_backups_bx1500g_vid051d_pid0002_issue55.json",
        "test/fixtures/apc/apc_backups_bx750mi_vid051d_pid0002_issue60.json",
        "test/fixtures/apc/apc_backups_cs500_vid051d_pid0002_issue18.json",
        "test/fixtures/apc/apc_backups_cs750rs_vid051d_pid0002_issue48.json",
        "test/fixtures/apc/apc_backups_es700g_vid051d_pid0002_issue71.json",
        "test/fixtures/apc/apc_backups_rs900mi_vid051d_pid0002_issue18.json",
        "test/fixtures/apc/apc_backups_xs700u_vid051d_pid0002_issue20.json",
        "test/fixtures/apc/apc_smartups_750_vid051d_pid0002_issue13.json",
        "test/fixtures/apc/apc_smartups_750_vid051d_pid0003_issue22.json",
        SMARTUPS_3000,
    };
    for (const char* f : fixtures) {
        HIDParser parser;
        TEST_ASSERT_TRUE_MESSAGE(loadDescriptor(f, parser), f);
        for (UPSTimer t : {UPSTimer::Shutdown, UPSTimer::Startup}) {
            const HIDUsageDef* d = UPSTimerLogic::resolve(parser.getUsages(), t);
            if (!d) continue;
            TEST_ASSERT_EQUAL_MESSAGE(-1, d->logical_min, f);
            TEST_ASSERT_EQUAL_MESSAGE(3, d->report_type, f);
            TEST_ASSERT_FALSE_MESSAGE(BeeperLogic::reportHasOtherFields(parser.getUsages(), *d), f);
        }
    }
}

// --- parseSeconds / plan ---

void test_parse_seconds_strict() {
    int32_t v = 0;
    TEST_ASSERT_TRUE(UPSTimerLogic::parseSeconds("20", v));
    TEST_ASSERT_EQUAL(20, v);
    TEST_ASSERT_TRUE(UPSTimerLogic::parseSeconds("-1", v));
    TEST_ASSERT_EQUAL(-1, v);
    TEST_ASSERT_FALSE(UPSTimerLogic::parseSeconds("", v));
    TEST_ASSERT_FALSE(UPSTimerLogic::parseSeconds("20s", v));
    TEST_ASSERT_FALSE(UPSTimerLogic::parseSeconds("+20", v));
    TEST_ASSERT_FALSE(UPSTimerLogic::parseSeconds("-2", v));
    TEST_ASSERT_FALSE(UPSTimerLogic::parseSeconds("1e3", v));
    TEST_ASSERT_FALSE(UPSTimerLogic::parseSeconds("2.5", v));
    TEST_ASSERT_FALSE(UPSTimerLogic::parseSeconds("9999999", v));
}

static void assertStep(const UPSTimerLogic::Step& s, UPSTimer t, int32_t sec) {
    TEST_ASSERT_TRUE(s.timer == t);
    TEST_ASSERT_EQUAL(sec, s.seconds);
}

void test_plan_defaults_match_usbhid_ups() {
    std::vector<UPSTimerLogic::Step> st;
    const char* err = nullptr;

    TEST_ASSERT_TRUE(UPSTimerLogic::plan("load.off.delay", nullptr, st, err));
    TEST_ASSERT_EQUAL(1, st.size());
    assertStep(st[0], UPSTimer::Shutdown, 20);

    TEST_ASSERT_TRUE(UPSTimerLogic::plan("load.on.delay", nullptr, st, err));
    TEST_ASSERT_EQUAL(1, st.size());
    assertStep(st[0], UPSTimer::Startup, 30);

    TEST_ASSERT_TRUE(UPSTimerLogic::plan("shutdown.stop", nullptr, st, err));
    TEST_ASSERT_EQUAL(1, st.size());
    assertStep(st[0], UPSTimer::Shutdown, -1);
}

void test_plan_shutdown_return_writes_startup_first_and_later() {
    std::vector<UPSTimerLogic::Step> st;
    const char* err = nullptr;
    TEST_ASSERT_TRUE(UPSTimerLogic::plan("shutdown.return", nullptr, st, err));
    TEST_ASSERT_EQUAL(2, st.size());
    assertStep(st[0], UPSTimer::Startup, 30);
    assertStep(st[1], UPSTimer::Shutdown, 20);

    String v = "120";
    TEST_ASSERT_TRUE(UPSTimerLogic::plan("shutdown.return", &v, st, err));
    assertStep(st[0], UPSTimer::Startup, 130);
    assertStep(st[1], UPSTimer::Shutdown, 120);
    TEST_ASSERT_TRUE(st[0].seconds > st[1].seconds);
}

void test_plan_shutdown_stayoff_cancels_startup() {
    std::vector<UPSTimerLogic::Step> st;
    const char* err = nullptr;
    TEST_ASSERT_TRUE(UPSTimerLogic::plan("shutdown.stayoff", nullptr, st, err));
    TEST_ASSERT_EQUAL(2, st.size());
    assertStep(st[0], UPSTimer::Startup, -1);
    assertStep(st[1], UPSTimer::Shutdown, 20);
}

void test_plan_refuses_short_or_bad_values() {
    std::vector<UPSTimerLogic::Step> st;
    const char* err = nullptr;
    const char* bad[] = {"0", "5", "9", "-1", "abc", "20s", ""};
    for (const char* b : bad) {
        String v = b;
        TEST_ASSERT_FALSE_MESSAGE(UPSTimerLogic::plan("load.off.delay", &v, st, err), b);
        TEST_ASSERT_EQUAL_STRING("INVALID-ARGUMENT", err);
        TEST_ASSERT_EQUAL(0, st.size());
        TEST_ASSERT_FALSE_MESSAGE(UPSTimerLogic::plan("shutdown.return", &v, st, err), b);
        TEST_ASSERT_EQUAL(0, st.size());
    }
    // shutdown.stop takes no value
    String v = "30";
    TEST_ASSERT_FALSE(UPSTimerLogic::plan("shutdown.stop", &v, st, err));
    TEST_ASSERT_EQUAL_STRING("INVALID-ARGUMENT", err);
    // load.on.delay may be cancelled with -1
    String c = "-1";
    TEST_ASSERT_TRUE(UPSTimerLogic::plan("load.on.delay", &c, st, err));
    assertStep(st[0], UPSTimer::Startup, -1);
}

void test_plan_unknown_command() {
    std::vector<UPSTimerLogic::Step> st;
    const char* err = nullptr;
    TEST_ASSERT_FALSE(UPSTimerLogic::plan("shutdown.reboot", nullptr, st, err));
    TEST_ASSERT_EQUAL_STRING("CMD-NOT-SUPPORTED", err);
    TEST_ASSERT_FALSE(UPSTimerLogic::plan("load.off", nullptr, st, err));
    TEST_ASSERT_EQUAL_STRING("CMD-NOT-SUPPORTED", err);
}

void test_every_planned_value_encodes() {
    // Whatever plan() produces must pass encode() on the real Smart-UPS fields
    HIDParser parser;
    TEST_ASSERT_TRUE(loadDescriptor(SMARTUPS_3000, parser));
    std::vector<UPSTimerLogic::Step> st;
    const char* err = nullptr;
    for (const char* const* c = UPSTimerLogic::commands(); *c; c++) {
        TEST_ASSERT_TRUE_MESSAGE(UPSTimerLogic::plan(*c, nullptr, st, err), *c);
        for (const auto& s : st) {
            const HIDUsageDef* d = UPSTimerLogic::resolve(parser.getUsages(), s.timer);
            TEST_ASSERT_NOT_NULL(d);
            uint8_t buf[8];
            TEST_ASSERT_EQUAL_MESSAGE(3, UPSTimerLogic::encode(*d, s.seconds, buf, sizeof(buf)), *c);
        }
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_16bit_cancel_is_ffff);
    RUN_TEST(test_encode_16bit_little_endian);
    RUN_TEST(test_encode_24bit_and_no_report_id);
    RUN_TEST(test_encode_refuses_instant_and_out_of_range);
    RUN_TEST(test_resolve_prefers_power_summary_like_usbhid_ups);
    RUN_TEST(test_resolve_skips_stored_settings);
    RUN_TEST(test_resolve_skips_shared_reports);
    RUN_TEST(test_resolve_skips_input_and_odd_sizes);
    RUN_TEST(test_resolve_ignores_apc_vendor_delays);
    RUN_TEST(test_resolve_smartups_3000_descriptor);
    RUN_TEST(test_resolve_never_picks_a_setting_on_any_apc_fixture);
    RUN_TEST(test_parse_seconds_strict);
    RUN_TEST(test_plan_defaults_match_usbhid_ups);
    RUN_TEST(test_plan_shutdown_return_writes_startup_first_and_later);
    RUN_TEST(test_plan_shutdown_stayoff_cancels_startup);
    RUN_TEST(test_plan_refuses_short_or_bad_values);
    RUN_TEST(test_plan_unknown_command);
    RUN_TEST(test_every_planned_value_encodes);
    return UNITY_END();
}
