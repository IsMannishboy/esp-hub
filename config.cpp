#include "config.h"
#include "ports.h"
#include <ArduinoJson.h>
#include <set>
#include "driver/gpio.h"
#include "soc/soc_caps.h"

std::vector<std::string> config_errors;
std::vector<std::string> config_warnings;

// ---------------------------------------------------------------- chip limits

#if CONFIG_IDF_TARGET_ESP32
static const char *CHIP = "ESP32";
static bool pin_is_flash(int p) { return p >= 6 && p <= 11; }          // SPI flash
static bool pin_is_usb(int p)   { (void)p; return false; }
static bool pin_is_strap(int p) { return p == 0 || p == 2 || p == 5 || p == 12 || p == 15; }
#elif CONFIG_IDF_TARGET_ESP32C3
static const char *CHIP = "ESP32-C3";
static bool pin_is_flash(int p) { return p >= 12 && p <= 17; }         // SPI flash
static bool pin_is_usb(int p)   { return p == 18 || p == 19; }         // USB D-/D+
static bool pin_is_strap(int p) { return p == 2 || p == 8 || p == 9; }
#elif CONFIG_IDF_TARGET_ESP32S3
static const char *CHIP = "ESP32-S3";
static bool pin_is_flash(int p) { return p >= 26 && p <= 32; }         // SPI flash / PSRAM
static bool pin_is_usb(int p)   { return p == 19 || p == 20; }         // USB D-/D+
static bool pin_is_strap(int p) { return p == 0 || p == 3 || p == 45 || p == 46; }
#else
#error "unknown chip: add its pin rules to config.cpp"
#endif

bool config_log = true;              // "log": UART0 prints the log
int config_led_pin = -1;             // "led_pin": status LED, -1 = none

// UART0 is the log / flashing port unless "log": false
static bool UART0_FREE = false;

static int uarts_available() { return SOC_UART_NUM - (UART0_FREE ? 0 : 1); }

// ---------------------------------------------------------------- helpers

struct PortCfg {
    std::string name;
    std::string mode;                // FD | HD | SBUS
    int uart = -1;
    int rx_pin = -1;                 // FD
    int tx_pin = -1;                 // FD, SBUS
    int pin = -1;                    // HD
    uint32_t baud = 0;
    uint32_t cfg = SERIAL_8N1;
    bool inv = false;
    std::vector<uint8_t> rx;         // types it takes in
    std::vector<uint8_t> tx;         // types it is subscribed to
};

static void err(const std::string &s)  { config_errors.push_back(s); }
static void warn(const std::string &s) { config_warnings.push_back(s); }

static std::string hex(uint8_t t)
{
    char b[8];
    snprintf(b, sizeof b, "0x%02X", t);
    return b;
}

static bool parse_type(JsonVariantConst v, uint8_t &out)
{
    if (v.is<int>()) {
        int x = v.as<int>();
        if (x < 0 || x > 255) return false;
        out = (uint8_t)x;
        return true;
    }
    if (!v.is<const char*>()) return false;
    std::string s = v.as<const char*>();
    if (s == "RC")       { out = TYPE_RC;       return true; }
    if (s == "WS")       { out = TYPE_WS;       return true; }
    if (s == "WS_REPLY") { out = TYPE_WS_REPLY; return true; }
    if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        char *end;
        long x = strtol(s.c_str() + 2, &end, 16);
        if (*end == 0 && x >= 0 && x <= 255) { out = (uint8_t)x; return true; }
    }
    return false;
}

static bool parse_serial_cfg(const std::string &s, uint32_t &out)
{
    if (s == "8N1") { out = SERIAL_8N1; return true; }
    if (s == "8N2") { out = SERIAL_8N2; return true; }
    if (s == "8E1") { out = SERIAL_8E1; return true; }
    if (s == "8E2") { out = SERIAL_8E2; return true; }
    if (s == "8O1") { out = SERIAL_8O1; return true; }
    if (s == "8O2") { out = SERIAL_8O2; return true; }
    return false;
}

static void parse_types(const std::string &who, const char *key, JsonVariantConst v,
                        std::vector<uint8_t> &out)
{
    if (v.isNull()) return;
    if (!v.is<JsonArrayConst>()) {
        err(who + ": \"" + key + "\" must be an array, e.g. [\"RC\", \"WS\"]");
        return;
    }
    for (JsonVariantConst t : v.as<JsonArrayConst>()) {
        uint8_t type;
        if (!parse_type(t, type)) {
            err(who + ": unknown packet type in \"" + key + "\" (use RC, WS, WS_REPLY or 0x..)");
            continue;
        }
        out.push_back(type);
    }
}

// pin: must exist, not flash; output pins must be able to drive
static void check_pin(const std::string &who, const char *key, int p, bool output,
                      std::set<int> &used)
{
    if (p < 0) return;
    std::string w = who + ": " + key + "=" + std::to_string(p);
    if (p >= 64 || !GPIO_IS_VALID_GPIO(p)) { err(w + " does not exist on " + CHIP); return; }
    if (pin_is_flash(p))  { err(w + " is a SPI flash pin on " + CHIP); return; }
    if (output && !GPIO_IS_VALID_OUTPUT_GPIO(p)) { err(w + " is input-only on " + CHIP); return; }
    if (pin_is_usb(p))    warn(w + " is a USB pin on " + CHIP + " (USB stops working)");
    if (pin_is_strap(p))  warn(w + " is a strapping pin on " + CHIP + " (may break boot)");
    if (!used.insert(p).second) err(w + " is already used by another port");
}

// ---------------------------------------------------------------- parse + check

static void parse_port(JsonObjectConst o, int idx, PortCfg &c)
{
    c.name = o["name"].is<const char*>() ? o["name"].as<const char*>()
                                          : "port" + std::to_string(idx);
    const std::string &who = c.name;

    if (!o["mode"].is<const char*>()) { err(who + ": \"mode\" is missing (FD, HD or SBUS)"); return; }
    c.mode = o["mode"].as<const char*>();
    if (c.mode != "FD" && c.mode != "HD" && c.mode != "SBUS") {
        err(who + ": mode \"" + c.mode + "\" is unknown (FD, HD or SBUS)");
        return;
    }
    const bool sbus = (c.mode == "SBUS");

    if (!o["uart"].is<int>()) err(who + ": \"uart\" is missing");
    else c.uart = o["uart"].as<int>();

    // SBUS has fixed line settings: 100000 8E2 inverted, only fill them in when left out
    c.baud = o["baud"].is<uint32_t>() ? o["baud"].as<uint32_t>() : (sbus ? 100000 : 0);
    if (c.baud < 1200 || c.baud > 5000000) err(who + ": \"baud\" is missing or out of range");
    if (sbus && c.baud != 100000) err(who + ": SBUS runs at 100000 baud, not " + std::to_string(c.baud));

    std::string cfg = o["cfg"].is<const char*>() ? o["cfg"].as<const char*>() : (sbus ? "8E2" : "8N1");
    if (!parse_serial_cfg(cfg, c.cfg)) err(who + ": cfg \"" + cfg + "\" is unknown (8N1, 8E2, ...)");
    if (sbus && cfg != "8E2") err(who + ": SBUS needs cfg 8E2");

    c.inv = o["inv"].is<bool>() ? o["inv"].as<bool>() : sbus;

    c.rx_pin = o["rx_pin"] | -1;
    c.tx_pin = o["tx_pin"] | -1;
    c.pin    = o["pin"]    | -1;

    parse_types(who, "rx", o["rx"], c.rx);
    parse_types(who, "tx", o["tx"], c.tx);

    if (c.mode == "FD") {
        if (c.pin >= 0) err(who + ": FD uses rx_pin / tx_pin, not pin");
        if (c.rx_pin < 0 && c.tx_pin < 0) err(who + ": FD needs rx_pin and/or tx_pin");
        if (!c.rx.empty() && c.rx_pin < 0) err(who + ": has \"rx\" types but no rx_pin");
        if (!c.tx.empty() && c.tx_pin < 0) err(who + ": has \"tx\" types but no tx_pin");
        if (c.rx_pin >= 0 && c.rx.empty()) warn(who + ": rx_pin set but \"rx\" is empty: input ignored");
    } else if (c.mode == "HD") {
        if (c.pin < 0) err(who + ": HD needs \"pin\"");
        if (c.rx_pin >= 0 || c.tx_pin >= 0) err(who + ": HD uses one \"pin\", not rx_pin / tx_pin");
    } else {  // SBUS
        if (c.tx_pin < 0) err(who + ": SBUS needs tx_pin");
        if (c.rx_pin >= 0 || c.pin >= 0) err(who + ": SBUS is output only (tx_pin)");
        if (!c.rx.empty()) err(who + ": SBUS is output only, \"rx\" must be empty");
        for (uint8_t t : c.tx)
            if (t != TYPE_RC) err(who + ": SBUS can only carry RC, not " + hex(t));
        if (c.tx.empty()) c.tx.push_back(TYPE_RC);
    }
}

// returns whether led_pin itself is fine (so it can still blink on a bad config)
static bool check_all(std::vector<PortCfg> &cfgs, int led_pin)
{
    // number of ports vs. this chip
    const int avail = uarts_available();
    if ((int)cfgs.size() > avail) {
        err(std::to_string(cfgs.size()) + " ports in the config, but " + CHIP + " has only " +
            std::to_string(avail) + " free UART(s) (" + std::to_string(SOC_UART_NUM) + " total" +
            (UART0_FREE ? "" : ", UART0 is the log port: set \"log\": false to use it") + ")");
    }

    std::set<std::string> names;
    std::set<int> uarts, pins;
    for (PortCfg &c : cfgs) {
        if (c.mode.empty()) continue;            // already failed in parse_port
        const std::string &who = c.name;
        if (!names.insert(c.name).second) err(who + ": name is used twice");

        if (c.uart >= 0) {
            if (c.uart >= SOC_UART_NUM)
                err(who + ": uart " + std::to_string(c.uart) + " does not exist, " + CHIP +
                    " has UART0.." + std::to_string(SOC_UART_NUM - 1));
            else if (c.uart == 0 && !UART0_FREE)
                err(who + ": UART0 is the log port (set \"log\": false to use it)");
            else if (!uarts.insert(c.uart).second)
                err(who + ": uart " + std::to_string(c.uart) + " is already used by another port");
        }

        check_pin(who, "rx_pin", c.rx_pin, false, pins);
        check_pin(who, "tx_pin", c.tx_pin, true, pins);
        check_pin(who, "pin", c.pin, true, pins);
    }
    const size_t errors_before = config_errors.size();
    check_pin("config", "led_pin", led_pin, true, pins);
    const bool led_ok = config_errors.size() == errors_before;

    // a type somebody sends out but nobody takes in (or the other way round) is dead weight
    std::set<uint8_t> in, out;
    for (PortCfg &c : cfgs) {
        in.insert(c.rx.begin(), c.rx.end());
        out.insert(c.tx.begin(), c.tx.end());
    }
    for (uint8_t t : out)
        if (!in.count(t)) warn("type " + hex(t) + ": ports subscribed to it, but no port takes it in");
    for (uint8_t t : in)
        if (!out.count(t)) warn("type " + hex(t) + ": taken in, but no port is subscribed to it");
    return led_ok;
}

// ---------------------------------------------------------------- build

bool load_config(const char *json)
{
    config_errors.clear();
    config_warnings.clear();

    DynamicJsonDocument doc(8192);
    DeserializationError e = deserializeJson(doc, json);
    if (e) {
        err(std::string("config is not valid JSON: ") + e.c_str());
        return false;
    }
    // optional "chip": the config is written for this chip only
    if (doc["chip"].is<const char*>()) {
        auto norm = [](std::string s) {
            std::string r;
            for (char ch : s) if (ch != '-' && ch != '_') r += (char)toupper((unsigned char)ch);
            return r;
        };
        if (norm(doc["chip"].as<const char*>()) != norm(CHIP)) {
            err(std::string("config is for \"") + doc["chip"].as<const char*>() +
                "\", but the firmware is built for " + CHIP);
            return false;
        }
    }

    // board-wide settings
    if (!doc["log"].isNull() && !doc["log"].is<bool>()) err("\"log\" must be true or false");
    config_log = doc["log"] | true;
    UART0_FREE = !config_log;
    if (!doc["led_pin"].isNull() && !doc["led_pin"].is<int>()) err("\"led_pin\" must be a number");
    int led_pin = doc["led_pin"] | -1;

    JsonArrayConst arr = doc["ports"];
    if (arr.isNull() || arr.size() == 0) {
        err("config has no \"ports\" array");
        return false;
    }

    std::vector<PortCfg> cfgs(arr.size());
    int i = 0;
    for (JsonVariantConst v : arr) {
        if (!v.is<JsonObjectConst>()) err("ports[" + std::to_string(i) + "] is not an object");
        else parse_port(v.as<JsonObjectConst>(), i, cfgs[i]);
        i++;
    }
    config_led_pin = check_all(cfgs, led_pin) ? led_pin : -1;
    if (!config_errors.empty()) {
        return false;                // nothing is created on a bad config
    }

    for (PortCfg &c : cfgs) {
        Port *p;
        int8_t rx = -1, tx = -1;
        if (c.mode == "HD")        { p = new HD(c.name, c.uart);   rx = c.pin; }
        else if (c.mode == "SBUS") { p = new Sbus(c.name, c.uart); tx = c.tx_pin; }
        else                       { p = new Port(c.name, c.uart); rx = c.rx_pin; tx = c.tx_pin; }

        p->begin(c.baud, c.cfg, rx, tx, c.inv);

        for (uint8_t t : c.rx) p->accept[t] = true;
        if (rx >= 0 && !c.rx.empty()) registry.push_back(p);   // has an input: poll it
        for (uint8_t t : c.tx) subscribe(t, p);                // has an output: route to it
    }
    return true;
}
