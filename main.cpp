#include <Arduino.h>
#include "config.h"
#include "ports.h"

// conf.json is built into the firmware (board_build.embed_txtfiles in platformio.ini)
extern const char conf_json[] asm("_binary_conf_json_start");

#define LOGF(...) do { if (config_log) Serial.printf(__VA_ARGS__); } while (0)

static bool config_ok = false;

void setup()
{
    // read conf.json, check it, create + start the ports, fill routes and registry
    config_ok = load_config(conf_json);

    // "log": true -> UART0 is the log (the config check made sure no port took it)
    if (config_log) {
        Serial.begin(115200);
        delay(200);
    }
    if (config_led_pin >= 0) {
        pinMode(config_led_pin, OUTPUT);
    }

    for (const std::string &w : config_warnings) LOGF("config warning: %s\n", w.c_str());
    for (const std::string &e : config_errors)   LOGF("config error: %s\n", e.c_str());

    if (config_ok) {
        LOGF("config ok: %u input port(s), %u packet type(s) routed\n",
             (unsigned)registry.size(), (unsigned)routes.size());
    } else {
        LOGF("config rejected: no ports started\n");
    }
}

void loop()
{
    if (!config_ok) {
        // bad config: fast blink, nothing runs
        if (config_led_pin >= 0) {
            digitalWrite(config_led_pin, (millis() / 100) & 1);
        }
        delay(10);
        return;
    }

    for (Port *p : registry) {
        p->input_handler();
    }
}
