#include "cocis_app.h"

static const char *TAG = "cocis";

#define RC_COMMAND_REG       0x01
#define RC_COM_IRQ_REG       0x04
#define RC_ERROR_REG         0x06
#define RC_STATUS2_REG       0x08
#define RC_FIFO_DATA_REG     0x09
#define RC_FIFO_LEVEL_REG    0x0A
#define RC_BIT_FRAMING_REG   0x0D
#define RC_COLL_REG          0x0E
#define RC_MODE_REG          0x11
#define RC_TX_CONTROL_REG    0x14
#define RC_TX_ASK_REG        0x15
#define RC_T_MODE_REG        0x2A
#define RC_T_PRESCALER_REG   0x2B
#define RC_T_RELOAD_REG_H    0x2C
#define RC_T_RELOAD_REG_L    0x2D
#define RC_VERSION_REG       0x37

static spi_device_handle_t rfid_readers[2];

/* ---- low-level SPI helpers ---- */

static uint8_t rc_read(int reader, uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = (uint8_t)(((reg << 1) & 0x7E) | 0x80);
    tx[1] = 0;
    rx[0] = 0;
    rx[1] = 0;

    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    spi_device_transmit(rfid_readers[reader], &transaction);
    return rx[1];
}

/* Burst-read FIFO in a single SPI transaction */
static void rc_read_fifo(int reader, uint8_t *data, int count)
{
    if (count <= 0 || count > 64) {
        return;
    }
    uint8_t tx[65] = { 0 };
    uint8_t rx[65] = { 0 };
    const uint8_t address = (uint8_t)(((RC_FIFO_DATA_REG << 1) & 0x7E) | 0x80);

    tx[0] = address;
    for (int i = 1; i < count; i++) {
        tx[i] = address;
    }

    spi_transaction_t transaction = {
        .length = (count + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    spi_device_transmit(rfid_readers[reader], &transaction);
    memcpy(data, &rx[1], count);
}

static void rc_write(int reader, uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = (uint8_t)((reg << 1) & 0x7E);
    tx[1] = value;

    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = tx
    };
    spi_device_transmit(rfid_readers[reader], &transaction);
}

static void rc_set_bits(int reader, uint8_t reg, uint8_t mask)
{
    uint8_t value = rc_read(reader, reg);
    rc_write(reader, reg, value | mask);
}

static void rc_clear_bits(int reader, uint8_t reg, uint8_t mask)
{
    uint8_t value = rc_read(reader, reg);
    rc_write(reader, reg, value & (uint8_t)~mask);
}

/* ---- RC522 init ---- */
static void rc_init(int reader)
{
    rc_write(reader, RC_COMMAND_REG, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));

    rc_write(reader, RC_T_MODE_REG, 0x80);
    rc_write(reader, RC_T_PRESCALER_REG, 0xA9);
    rc_write(reader, RC_T_RELOAD_REG_H, 0x03);
    rc_write(reader, RC_T_RELOAD_REG_L, 0xE8);
    rc_write(reader, RC_TX_ASK_REG, 0x40);
    rc_write(reader, RC_MODE_REG, 0x3D);

    if (!(rc_read(reader, RC_TX_CONTROL_REG) & 0x03)) {
        rc_set_bits(reader, RC_TX_CONTROL_REG, 0x03);
    }
}

/* ---- transceive with deadline-based wait ---- */
static int rc_transceive(int reader, const uint8_t *send, int send_len,
                         uint8_t *receive, int *receive_len, uint8_t tx_bits)
{
    rc_write(reader, RC_COMMAND_REG, 0x00);
    rc_write(reader, RC_COM_IRQ_REG, 0x7F);
    rc_write(reader, RC_FIFO_LEVEL_REG, 0x80);

    for (int i = 0; i < send_len; i++) {
        rc_write(reader, RC_FIFO_DATA_REG, send[i]);
    }

    rc_write(reader, RC_BIT_FRAMING_REG, tx_bits);
    rc_write(reader, RC_COMMAND_REG, 0x0C);
    rc_set_bits(reader, RC_BIT_FRAMING_REG, 0x80);

    uint8_t irq = 0;
    int64_t deadline = esp_timer_get_time() + 40000;

    while (1) {
        irq = rc_read(reader, RC_COM_IRQ_REG);
        if (irq & 0x31) {
            break;
        }
        if (esp_timer_get_time() >= deadline) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    rc_clear_bits(reader, RC_BIT_FRAMING_REG, 0x80);

    if (!(irq & 0x31)) {
        return -1;
    }
    if (rc_read(reader, RC_ERROR_REG) & 0x13) {
        return -2;
    }
    if (irq & 0x01) {
        return -3;
    }

    int count = rc_read(reader, RC_FIFO_LEVEL_REG);
    if (count > *receive_len) {
        count = *receive_len;
    }
    rc_read_fifo(reader, receive, count);
    *receive_len = count;
    return 0;
}

/* ---- read UID ---- */
static bool rc_read_uid(int reader, char *uid)
{
    uint8_t buffer[8];
    int buffer_len = sizeof(buffer);

    rc_clear_bits(reader, RC_STATUS2_REG, 0x08);
    rc_clear_bits(reader, RC_COLL_REG, 0x80);

    const uint8_t request = 0x26;
    if (rc_transceive(reader, &request, 1, buffer, &buffer_len, 0x07) != 0 ||
        buffer_len != 2) {
        return false;
    }

    const uint8_t anti_collision[2] = { 0x93, 0x20 };
    buffer_len = sizeof(buffer);
    if (rc_transceive(reader, anti_collision, 2, buffer, &buffer_len, 0x00) != 0 ||
        buffer_len != 5) {
        return false;
    }

    if ((buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3]) != buffer[4]) {
        return false;
    }

    snprintf(uid, UID_LEN, "%02X:%02X:%02X:%02X",
             buffer[0], buffer[1], buffer[2], buffer[3]);
    return true;
}

/* ---- setup ---- */
void rc522_setup(void)
{
    gpio_config_t reset_pin = {
        .pin_bit_mask = 1ULL << PIN_RC522_RST,
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&reset_pin);

    gpio_set_level(PIN_RC522_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RC522_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    const int chip_select_pins[2] = { PIN_RC522_SS_IN, PIN_RC522_SS_OUT };

    for (int reader = 0; reader < 2; reader++) {
        spi_device_interface_config_t device_config = {
            .clock_speed_hz = 4 * 1000 * 1000,
            .mode = 0,
            .spics_io_num = chip_select_pins[reader],
            .queue_size = 2
        };
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device_config,
                                           &rfid_readers[reader]));

        rc_init(reader);
        uint8_t version = rc_read(reader, RC_VERSION_REG);
        rfid_reader_present[reader] = (version != 0x00 && version != 0xFF);

        ESP_LOGI(TAG, "RC522 %s: version 0x%02X (%s)",
                 reader == 0 ? "ENTRY" : "EXIT",
                 version,
                 rfid_reader_present[reader] ? "present" : "not detected");
    }
}

/* Public wrapper */
bool rfid_read_uid(int reader, char *uid)
{
    return rc_read_uid(reader, uid);
}