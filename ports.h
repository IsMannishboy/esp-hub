#pragma once
#include <Arduino.h>
#include <bitset>
#include <string>
#include <unordered_map>
#include <vector>

// CRSF frame types used in the config ("RC", "WS", "WS_REPLY")
#define TYPE_RC        0x16
#define TYPE_WS        0x54
#define TYPE_WS_REPLY  0x55

class Port;

// packet type -> ports that want it (ports with an output: FD with TX, HD, SBUS)
extern std::unordered_map<uint8_t, std::vector<Port*>> routes;
// ports with an input: polled in loop()
extern std::vector<Port*> registry;

void subscribe(uint8_t type, Port *p);
uint8_t crc8(const uint8_t *data, uint8_t len);

// Full-duplex CRSF port: RX and TX on separate pins (either may be absent).
class Port {
public:
    Port(const std::string &name, uint8_t uart);
    virtual ~Port() {}

    virtual void begin(uint32_t baud, uint32_t cfg, int8_t rx_pin, int8_t tx_pin, bool inv);
    virtual void input_handler();
    virtual void output_handler(const uint8_t *data, int n);

    void drain();
    void clear_params();

    std::string name;
    uint8_t uart_num;
    HardwareSerial *Serial;
    std::bitset<256> accept;         // frame types this port takes in (config "rx")

protected:
    void send();
    void finish_frame();
    void complete_packet();

    uint8_t buff[256];
    uint8_t counter = 0;
    int len = 0;
    bool type_is_parsed = false;
    bool addr_is_parsed = false;
    bool len_is_parsed = false;
    bool ready = false;
    bool valid = false;
    bool is_not_completed = false;
    bool frame_done = false;         // a whole valid frame arrived in this input_handler() call
};

// Single-wire half-duplex port. The pin is an input; for sending it is switched to the UART
// TX for one frame (RX detached meanwhile, so no echo). Works with inverted and normal lines.
// Frames for it are queued and sent one per reply slot: right after a whole frame from the
// other side, while the line is quiet.
class HD : public Port {
public:
    HD(const std::string &name, uint8_t uart) : Port(name, uart) {}

    void begin(uint32_t baud, uint32_t cfg, int8_t rx_pin, int8_t tx_pin, bool inv) override;
    void input_handler() override;
    void output_handler(const uint8_t *data, int n) override;

private:
    void transmit(const uint8_t *data, int n);

    static const int TXQ = 6;
    uint8_t txq[TXQ][64];
    uint8_t txq_len[TXQ] = {0};
    int txq_head = 0;
    int txq_count = 0;
    int8_t pin = -1;
    bool inv = false;
};

// SBUS output: CRSF RC frames (0x16) -> 25-byte SBUS frames. Output only.
class Sbus : public Port {
public:
    Sbus(const std::string &name, uint8_t uart) : Port(name, uart) {}
    void input_handler() override {}
    void output_handler(const uint8_t *data, int n) override;
};
