#pragma once
#include <string>
#include <vector>

// Errors / warnings of the last load_config() call (also printed to the log when it is on)
extern std::vector<std::string> config_errors;
extern std::vector<std::string> config_warnings;

// Board-wide settings from the config (valid after load_config(), defaults before)
extern bool config_log;              // "log": print the log on UART0 (default true)
extern int config_led_pin;           // "led_pin": status LED, -1 = none

// Parse the JSON config, check it against this chip, create the ports, start them and fill
// routes + registry. On any error nothing is created and false is returned.
bool load_config(const char *json);
