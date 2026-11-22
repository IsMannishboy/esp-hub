#include "ports.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "soc/uart_periph.h"

std::unordered_map<uint8_t, std::vector<Port*>> routes;
std::vector<Port*> registry;

void subscribe(uint8_t type, Port *p)
{
    routes[type].push_back(p);
}

// CRSF CRC8: poly 0xD5, init 0x00, over [type .. last payload byte]
uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------- Port (full duplex)

Port::Port(const std::string &name, uint8_t uart) : name(name), uart_num(uart)
{
    Serial = new HardwareSerial(uart);
    clear_params();
}

void Port::begin(uint32_t baud, uint32_t cfg, int8_t rx_pin, int8_t tx_pin, bool inv)
{
    Serial->begin(baud, cfg, rx_pin, tx_pin, inv);
    drain();
}

void Port::drain()
{
    while (Serial->available()) {
        Serial->read();
    }
}

void Port::clear_params()
{
    counter = 0;
    len = 0;
    addr_is_parsed = false;
    len_is_parsed = false;
    type_is_parsed = false;
    ready = false;
    valid = false;
    is_not_completed = false;
    memset(buff, 0, sizeof(buff));
}

// send the current frame to every subscriber of its type, except ourselves
void Port::send()
{
    auto it = routes.find(buff[2]);
    if (it == routes.end()) {
        return;
    }
    for (Port *p : it->second) {
        if (p == this) {
            continue;
        }
        p->output_handler(buff, len);
    }
}

// frame fully received: check CRC, route it, get ready for the next one
void Port::finish_frame()
{
    ready = true;
    valid = crc8(buff + 2, len - 3) == buff[len - 1];
    if (valid) {
        send();
        frame_done = true;
    }
    clear_params();                  // valid or not: start over with the next address
}

void Port::complete_packet()
{
    while (counter < len && Serial->available()) {
        buff[counter] = Serial->read();
        counter++;
    }
    is_not_completed = (counter < len);
}

void Port::input_handler()
{
    frame_done = false;

    // finish a frame that was cut off in the previous call
    if (is_not_completed) {
        complete_packet();
        if (is_not_completed) {
            return;
        }
        finish_frame();
    }

    while (Serial->available()) {
        uint8_t byte = Serial->read();

        // addr
        if (counter == 0) {
            if (byte == 0xC8 || byte == 0xEE || byte == 0xEA) {
                buff[counter] = byte;
                counter++;
                addr_is_parsed = true;
            }
            continue;                // not an address: skip, keep searching
        }

        // len = type + payload + crc
        if (counter == 1) {
            if (byte < 2 || byte > 62) {
                clear_params();      // garbage: search the next address
            } else {
                len = byte + 2;      // whole frame
                buff[counter] = byte;
                counter++;
                len_is_parsed = true;
            }
            continue;
        }

        // type: only the types this port takes in
        if (counter == 2) {
            if (!accept[byte]) {
                clear_params();
                continue;
            }
            buff[counter] = byte;
            counter++;
            type_is_parsed = true;
            continue;                // len >= 4, so the frame always goes on
        }

        // payload + crc
        if (addr_is_parsed && len_is_parsed && type_is_parsed) {
            buff[counter] = byte;
            counter++;
            complete_packet();
            if (is_not_completed) {
                return;              // rest of the frame comes in the next call
            }
            finish_frame();
        }
    }
}

void Port::output_handler(const uint8_t *data, int n)
{
    Serial->write(data, n);
}

// ---------------------------------------------------------------- HD (single wire)

void HD::begin(uint32_t baud, uint32_t cfg, int8_t rx_pin, int8_t tx_pin, bool inv)
{
    (void)tx_pin;
    this->pin = rx_pin;
    this->inv = inv;
    Serial->begin(baud, cfg, pin, -1, inv);  // RX only; TX is attached per frame
    // idle level on the shared line: high for a normal UART, low when inverted
    gpio_set_pull_mode((gpio_num_t)pin, inv ? GPIO_PULLDOWN_ONLY : GPIO_PULLUP_ONLY);
    drain();
}

void HD::transmit(const uint8_t *data, int n)
{
    const uint32_t rx_sig = UART_PERIPH_SIGNAL(uart_num, SOC_UART_RX_PIN_IDX);
    const uint32_t tx_sig = UART_PERIPH_SIGNAL(uart_num, SOC_UART_TX_PIN_IDX);

    // 1. detach RX (feed it the idle level) so we don't hear ourselves
    esp_rom_gpio_connect_in_signal(inv ? GPIO_MATRIX_CONST_ZERO_INPUT : GPIO_MATRIX_CONST_ONE_INPUT,
                                   rx_sig, false);
    // 2. pin -> UART TX, idle level first so there is no false start bit
    gpio_set_level((gpio_num_t)pin, inv ? 0 : 1);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(pin, tx_sig, false, false);

    Serial->write(data, n);
    Serial->flush();                 // wait until the last stop bit is out

    // 3. back to input, RX attached again
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)pin, inv ? GPIO_PULLDOWN_ONLY : GPIO_PULLUP_ONLY);
    esp_rom_gpio_connect_in_signal(pin, rx_sig, false);
}

void HD::output_handler(const uint8_t *data, int n)
{
    if (n > 64) {
        return;
    }
    if (txq_count == TXQ) {          // nobody drains the queue: drop the oldest
        txq_head = (txq_head + 1) % TXQ;
        txq_count--;
    }
    int slot = (txq_head + txq_count) % TXQ;
    memcpy(txq[slot], data, n);
    txq_len[slot] = n;
    txq_count++;
}

void HD::input_handler()
{
    Port::input_handler();

    // reply slot: a whole frame just ended and nothing follows it yet
    if (frame_done && counter == 0 && Serial->available() == 0 && txq_count > 0) {
        transmit(txq[txq_head], txq_len[txq_head]);
        txq_head = (txq_head + 1) % TXQ;
        txq_count--;
    }
}

// ---------------------------------------------------------------- SBUS (output only)

// The CRSF RC payload is already 16 channels x 11 bits LSB-first, exactly the SBUS packing,
// and the value range (172..1811) is the same, so the 22 bytes are copied as they are.
void Sbus::output_handler(const uint8_t *data, int n)
{
    if (n != 26 || data[2] != TYPE_RC) {
        return;
    }
    uint8_t f[25];
    f[0] = 0x0F;                     // header
    memcpy(f + 1, data + 3, 22);     // channels
    f[23] = 0x00;                    // flags: ch17/18 off, no frame lost, no failsafe
    f[24] = 0x00;                    // footer
    if (Serial->availableForWrite() < (int)sizeof f) {
        return;                      // line still busy: skip, the next RC frame comes soon
    }
    Serial->write(f, sizeof f);
}
