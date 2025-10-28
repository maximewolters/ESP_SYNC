#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

// ========================= Config =========================
static const char *TAG = "one_way_sync";

#define UARTX UART_NUM_1
#define UART_BAUD 115200
#define UART_RX_BUF 2048
#define UART_TX_BUF 2048

#define UART_TX_PIN 9
#define UART_RX_PIN 10

#define STROBE_GPIO 6

// ========================= Framing ========================
// | 'TSYN'(4) | type(1) | len(2 LE) | seq(1) | payload(len) |
#pragma pack(push, 1)
typedef struct
{
    uint8_t magic[4];
    uint8_t type;
    uint16_t len; // payload length
    uint8_t seq;  // per-cycle sequence id
} pkt_hdr_t;
#pragma pack(pop)

enum
{
    // zero-length headers used ONLY to generate/identify SOF captures
    PKT_SYNC_HDR = 10, // M→S marker (t1 on M TX, t2 on S RX)
    PKT_RESP_HDR = 11, // S→M marker (t3 on S TX, t4 on M RX)

    // data packets
    PKT_SYNC_REQ = 20,   // M→S payload: int64_t t1
    PKT_SYNC_RESP = 21,  // S→M payload: { int64_t t2, int64_t t3 }
    PKT_SET_OFFSET = 22, // M→S payload: int64_t offset (optional convenience)
};

static inline void make_hdr(pkt_hdr_t *h, uint8_t type, uint16_t len, uint8_t seq)
{
    h->magic[0] = 'T';
    h->magic[1] = 'S';
    h->magic[2] = 'Y';
    h->magic[3] = 'N';
    h->type = type;
    h->len = len;
    h->seq = seq;
}
static inline bool hdr_ok(const pkt_hdr_t *h)
{
    return h->magic[0] == 'T' && h->magic[1] == 'S' && h->magic[2] == 'Y' && h->magic[3] == 'N';
}
// interframe gap between marker header and data

#define UART_BITS_PER_BYTE 10
#define HDR_BYTES (sizeof(pkt_hdr_t))
#define INTERFRAME_GAP_US ((int)((HDR_BYTES * UART_BITS_PER_BYTE * 1000000UL) / UART_BAUD) + 300)

// ========================= UART helpers ========================

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UARTX, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UARTX, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UARTX, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static esp_err_t uart_send(uart_port_t u, uint8_t type, uint8_t seq, const void *payload, uint16_t len)
{
    pkt_hdr_t h;
    make_hdr(&h, type, len, seq);
    int w = uart_write_bytes(u, (const char *)&h, sizeof(h));
    if (w != (int)sizeof(h))
        return ESP_FAIL;
    if (len && payload)
    {
        int w2 = uart_write_bytes(u, (const char *)payload, len);
        if (w2 != (int)len)
            return ESP_FAIL;
    }
    return ESP_OK;
}

static int uart_recv(uart_port_t u, uint8_t *payload, size_t pay_cap, pkt_hdr_t *out_hdr, int timeout_ms)
{
    // read header fully
    size_t got = 0; // bytes we've got so far
    pkt_hdr_t h;    // temporary header storage
    int64_t dl = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (got < sizeof(h))
    {
        int to = (int)((dl - esp_timer_get_time()) / 1000); // compute time left
        if (to <= 0)
            return -1;
        int n = uart_read_bytes(u, ((uint8_t *)&h) + got, sizeof(h) - got, pdMS_TO_TICKS(to));
        if (n > 0) // partial read, add to got
            got += n;
        else if (n < 0)
            return -1;
    }
    if (!hdr_ok(&h)) // check header magic
        return -2;
    if (h.len > pay_cap) // refuse payload larger than buffer
        return -3;
    // read payload
    got = 0;
    while (got < h.len) // same as header read loop but for the payload
    {
        int to = (int)((dl - esp_timer_get_time()) / 1000);
        if (to <= 0)
            return -4;
        int n = uart_read_bytes(u, payload + got, h.len - got, pdMS_TO_TICKS(to));
        if (n > 0)
            got += n;
        else if (n < 0)
            return -4;
    }
    if (out_hdr)
        *out_hdr = h;
    return h.type;
}

// ================= HW capture & mapping ===================
// We capture start-bit falling edges on UART TX and RX pins.
// A 10 kHz strobe toggles STROBE_GPIO so the capture timer latches ticks.
// We keep the last two (tick, time) bases and bracket each event.

static mcpwm_cap_timer_handle_t cap_timer;
static mcpwm_cap_channel_handle_t cap_tx_ch, cap_rx_ch, cap_strobe_ch;
static double ticks_to_us = 0.0;

typedef struct // struct to hold tick and time epoch in us, used for bases
{
    uint32_t tick;
    int64_t t_us;
} base_t;

static base_t base0, base1;                                  // last two bases we've captured on the strobe
static portMUX_TYPE base_mux = portMUX_INITIALIZER_UNLOCKED; // mutex for accessing bases

static volatile uint32_t rx_tick, tx_tick; // last captured ticks for rx and tx edges
static volatile bool rx_seen, tx_seen;     // flags indicating if edges have been seen
static volatile bool rx_armed, tx_armed;   // flags indicating if edges are armed for capture

// takes the captured strobe edge and puts the ticks and time epoch in us on the the new base, puts the old base to base1,
static bool IRAM_ATTR on_cap_strobe(mcpwm_cap_channel_handle_t ch, const mcpwm_capture_event_data_t *e, void *u)
{
    (void)ch;
    (void)u;
    base_t b0; // temporary base storage
    b0.tick = e->cap_value;
    b0.t_us = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&base_mux); // enter critical section
    base1 = base0;
    base0 = b0;
    portEXIT_CRITICAL_ISR(&base_mux); // exit critical section
    return true;
}

// takes the captured rx edge and puts the ticks in rx_tick if rx is armed
static bool IRAM_ATTR on_cap_rx(mcpwm_cap_channel_handle_t ch, const mcpwm_capture_event_data_t *e, void *u)
{
    (void)ch;
    (void)u;
    if (!rx_armed)
        return true;
    rx_tick = e->cap_value;
    rx_seen = true;
    rx_armed = false;
    return true;
}

// tales the captured tx edge and puts the ticks in tx_tick if tx is armed
static bool IRAM_ATTR on_cap_tx(mcpwm_cap_channel_handle_t ch, const mcpwm_capture_event_data_t *e, void *u)
{
    (void)ch;
    (void)u;
    if (!tx_armed)
        return true;
    tx_tick = e->cap_value;
    tx_seen = true;
    tx_armed = false;
    return true;
}

// toggles the level of the strobe gpiio pin
static void strobe_cb(void *arg)
{
    (void)arg;
    static bool lvl = false;
    lvl = !lvl;
    gpio_set_level(STROBE_GPIO, lvl);
}

static void capture_init_start(void) // all the init stuff of the mcpwm capture channels etc
{
    // Strobe pin
    ESP_ERROR_CHECK(gpio_config(&(gpio_config_t){
        .pin_bit_mask = 1ULL << STROBE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    }));
    gpio_set_level(STROBE_GPIO, 0);

    // Capture timer @ 80 MHz
    mcpwm_capture_timer_config_t tcfg = {.clk_src = MCPWM_CAPTURE_CLK_SRC_APB, .resolution_hz = 80000000};
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&tcfg, &cap_timer));
    ticks_to_us = 1000000.0 / (double)tcfg.resolution_hz; // 0.0125 µs per tick

    // Channels
    mcpwm_capture_channel_config_t c_rx = {.gpio_num = UART_RX_PIN, .prescale = 1, .flags = {.pos_edge = 0, .neg_edge = 1, .pull_up = 1}};
    mcpwm_capture_channel_config_t c_tx = {.gpio_num = UART_TX_PIN, .prescale = 1, .flags = {.pos_edge = 0, .neg_edge = 1, .pull_up = 1}};
    mcpwm_capture_channel_config_t c_st = {.gpio_num = STROBE_GPIO, .prescale = 1, .flags = {.pos_edge = 1, .neg_edge = 0, .pull_up = 0}};
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &c_rx, &cap_rx_ch));
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &c_tx, &cap_tx_ch));
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &c_st, &cap_strobe_ch));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_rx_ch, &(mcpwm_capture_event_callbacks_t){.on_cap = on_cap_rx}, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_tx_ch, &(mcpwm_capture_event_callbacks_t){.on_cap = on_cap_tx}, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_strobe_ch, &(mcpwm_capture_event_callbacks_t){.on_cap = on_cap_strobe}, NULL));

    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_rx_ch));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_tx_ch));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_strobe_ch));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    // 10 kHz mapping strobe via esp_timer (C function callback)
    const esp_timer_create_args_t ta = {
        .callback = &strobe_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "map_strobe"};
    static esp_timer_handle_t strobe_tmr;
    ESP_ERROR_CHECK(esp_timer_create(&ta, &strobe_tmr));
    ESP_ERROR_CHECK(esp_timer_start_periodic(strobe_tmr, 100)); // 100 µs = 10 kHz

    // init bases
    base_t z;
    z.tick = 0;
    z.t_us = esp_timer_get_time();
    portENTER_CRITICAL(&base_mux);
    base0 = z;
    base1 = z;
    portEXIT_CRITICAL(&base_mux);
}

static inline void arm_rx(void) // arming of rx edge capture
{
    rx_seen = false;
    rx_armed = true;
}
static inline void arm_tx(void) // arming of tx edge capture
{
    tx_seen = false;
    tx_armed = true;
}

// Project a captured tick to esp_timer epoch using the nearer base (two-point bracket), this does all the reasoning for pairing tick to us time
static inline int64_t project_tick(uint32_t tick)
{
    base_t b0, b1;
    portENTER_CRITICAL(&base_mux); // enter critical section, create a copy so that we don't hold the lock too long
    b0 = base0;
    b1 = base1;
    portEXIT_CRITICAL(&base_mux);
    int32_t d0 = (int32_t)(tick - b0.tick);                                   // difference to base0
    int32_t d1 = (int32_t)(tick - b1.tick);                                   // difference to base1
    base_t b = ((d0 >= 0 && (d0 <= d1 || d1 < 0)) ? b0 : b1);                 // choose the nearer base
    int32_t dt = (int32_t)(tick - b.tick);                                    // delta ticks to chosen base
    double dus = (double)dt * ticks_to_us;                                    // convert delta ticks to delta us
    int64_t res = (int64_t)(b.t_us + (dus >= 0 ? (dus + 0.5) : (dus - 0.5))); // round to µs
    return res;
}

static inline bool consume_rx_edge(int64_t *t) // consume rx edge if seen
{
    if (!rx_seen)
        return false;
    *t = project_tick(rx_tick);
    rx_seen = false;
    return true;
}
static inline bool consume_tx_edge(int64_t *t) // consume tx edge if seen
{
    if (!tx_seen)
        return false;
    *t = project_tick(tx_tick);
    tx_seen = false;
    return true;
}

// ================= Sequence-indexed edge store =================
typedef struct
{
    bool have;                                        // whether we have a stored edge
    int64_t t_us;                                     // stored time in us
} seq_edge_t;                                         // struct to hold stored edge info
static seq_edge_t edges[256];                         // array to hold edges for sequences 0-255
static inline void store_edge(uint8_t seq, int64_t t) // store edge for a given sequence
{
    edges[seq].have = true;
    edges[seq].t_us = t;
}
static inline bool take_edge(uint8_t seq, int64_t *t) // take edge for a given sequence
{
    if (!edges[seq].have)
        return false;
    *t = edges[seq].t_us;
    edges[seq].have = false;
    return true;
}

static volatile int64_t g_offset_us = 0;
static inline int64_t synced_time_us(void) { return esp_timer_get_time() - g_offset_us; }

// ========================== MASTAH ==============================

static void master_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Role: MASTER. TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);
    uint8_t seq = 0;
    uint8_t buf[32];
    // Master task implementation goes here
    while (1)
    {
        // Send sync header, capture t1 on TX
        arm_tx();
        ESP_ERROR_CHECK(uart_send(UARTX, PKT_SYNC_HDR, seq, NULL, 0));
        uart_wait_tx_done(UARTX, pdMS_TO_TICKS(2));
        esp_rom_delay_us(INTERFRAME_GAP_US);

        int64_t t1 = 0;

        for (int i = 0; i < 4 && !consume_tx_edge(&t1); ++i)
        {
            vTaskDelay(1);
        }
        if (!t1)
        {
            ESP_LOGW(TAG, "no t1 edge");
            goto next;
        }

        // 2) DATA: send SYNC_REQ{t1}
        ESP_ERROR_CHECK(uart_send(UARTX, PKT_SYNC_REQ, seq, &t1, sizeof(t1)));

    next:
        seq++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ========================== SLAVE ==============================
#if CONFIG_SYNC_ROLE_SLAVE
static void slave_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Role: SLAVE. TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);
    uint8_t buf[32];
    bool drop_next = true;

    while (1)
    {
        arm_rx();
        pkt_hdr_t h;
        int typ = uart_recv(UARTX, buf, sizeof(buf), &h, 5000);
        if (typ != PKT_SYNC_HDR)
        {
            ESP_LOGW(TAG, "unexpected type %d (want SYNC_HDR)", typ);
            drop_next = true;
            continue;
        }

        // consume t2 (RX SOF) now
        int64_t t2 = 0;
        if (!consume_rx_edge(&t2))
        {
            vTaskDelay(1);
            if (!consume_rx_edge(&t2))
            {
                ESP_LOGW(TAG, "no t2 edge");
                drop_next = true;
                continue;
            }
        }

        uint8_t seq = h.seq;
        // Read SYNC_REQ{t1} with same seq
        typ = uart_recv(UARTX, buf, sizeof(buf), &h, 50);
        if (typ != PKT_SYNC_REQ || h.len != 8 || h.seq != seq)
        {
            ESP_LOGW(TAG, "bad SYNC_REQ");
            drop_next = true;
            continue;
        }

        int64_t t1;
        memcpy(&t1, buf, 8);
        // Calculate offset based on t1 and t2
        int64_t offset = t2 - t1;
        ESP_LOGI(TAG, "Calculated offset: %lld us", (long long)offset);

        // compute synced time based on offset
        int64_t synced_time = esp_timer_get_time() - offset;
        ESP_LOGI(TAG, "Synced time: %lld us", (long long)synced_time);
    }
}
#endif // SLAVE

// ====================== app_main ===========================

void app_main(void)
{
    ESP_LOGI(TAG, "App starting...");
    capture_init_start();
    uart_init();
    ESP_ERROR_CHECK(uart_set_pin(UARTX, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    vTaskDelay(pdMS_TO_TICKS(50));

#if CONFIG_SYNC_ROLE_MASTER
    xTaskCreatePinnedToCore(master_task, "master", 4096, NULL, 5, NULL, 0);
#elif CONFIG_SYNC_ROLE_SLAVE
    xTaskCreatePinnedToCore(slave_task, "slave", 4096, NULL, 5, NULL, 1);
#else
#warning "No role set; defaulting to MASTER"
    xTaskCreatePinnedToCore(master_task, "master", 4096, NULL, 5, NULL, 0);
#endif
}
