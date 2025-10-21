
// UART_Sync_Simplified_main.c (C-only, IDF-compatible)
// Clean, minimal UART time sync using HW SOF capture + sequence-matched pairing.
// - Distinguishes marker vs data with a per-cycle seq (0..255)
// - Arms capture ONLY for marker headers and disarms on first edge
// - Inter-frame gap computed from actual on-wire header time (+ margin)
// - 10 kHz mapping strobe + two-base bracketing for low-jitter tick→epoch mapping
// - Drops first result after (re)arm; small sanity filter on delay outliers
//
// Build one device as MASTER and another as SLAVE via menuconfig:
//   CONFIG_SYNC_ROLE_MASTER=y  (master)
//   CONFIG_SYNC_ROLE_SLAVE=y   (slave)
//
// Pins/baud are easy to change in the defines below.

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
static const char *TAG = "SYNC";

// Primary link (UART1)
#define UARTX UART_NUM_1
#define UART_BAUD 115200
#define UART_RX_BUF 2048
#define UART_TX_BUF 2048

// GPIOs (adjust to your board)
#define UART_TX_PIN 9
#define UART_RX_PIN 10

// Mapping strobe GPIO (any free pin; we toggle it at 10 kHz)
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

// Inter-frame gap between marker header and its data
#define UART_BITS_PER_BYTE 10
#define HDR_BYTES (sizeof(pkt_hdr_t))
#define INTERFRAME_GAP_US ((int)((HDR_BYTES * UART_BITS_PER_BYTE * 1000000UL) / UART_BAUD) + 300)

// ===================== UART helpers =======================
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
    size_t got = 0;
    pkt_hdr_t h;
    int64_t dl = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (got < sizeof(h))
    {
        int to = (int)((dl - esp_timer_get_time()) / 1000);
        if (to <= 0)
            return -1;
        int n = uart_read_bytes(u, ((uint8_t *)&h) + got, sizeof(h) - got, pdMS_TO_TICKS(to));
        if (n > 0)
            got += n;
        else if (n < 0)
            return -1;
    }
    if (!hdr_ok(&h))
        return -2;
    if (h.len > pay_cap)
        return -3;
    // read payload
    got = 0;
    while (got < h.len)
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
static mcpwm_cap_channel_handle_t cap_rx_ch, cap_tx_ch, cap_strobe_ch;
static double tick_to_us = 0.0; // e.g., 12.5 ns → 0.0125 µs

typedef struct
{
    uint32_t tick;
    int64_t t_us;
} base_t;
static volatile base_t base0, base1; // last two bases
static portMUX_TYPE base_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t rx_tick, tx_tick; // latched edge ticks
static volatile bool rx_seen, tx_seen;
static volatile bool rx_armed, tx_armed;

static bool IRAM_ATTR on_cap_strobe(mcpwm_cap_channel_handle_t ch, const mcpwm_capture_event_data_t *e, void *u)
{
    (void)ch;
    (void)u;
    base_t b0;
    b0.tick = e->cap_value;
    b0.t_us = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&base_mux);
    base1 = base0;
    base0 = b0;
    portEXIT_CRITICAL_ISR(&base_mux);
    return true;
}
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

static void strobe_cb(void *arg)
{
    (void)arg;
    static bool lvl = false;
    lvl = !lvl;
    gpio_set_level(STROBE_GPIO, lvl);
}

static void capture_init_start(void)
{
    // Strobe pin
    gpio_config_t io = {0};
    io.pin_bit_mask = 1ULL << STROBE_GPIO;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = 0;
    io.pull_down_en = 0;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(STROBE_GPIO, 0);

    // Capture timer @ 80 MHz
    mcpwm_capture_timer_config_t tcfg = {.clk_src = MCPWM_CAPTURE_CLK_SRC_APB, .resolution_hz = 80000000};
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&tcfg, &cap_timer));
    tick_to_us = 1000000.0 / (double)tcfg.resolution_hz; // 0.0125 µs per tick

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

static inline void arm_rx(void)
{
    rx_seen = false;
    rx_armed = true;
}
static inline void arm_tx(void)
{
    tx_seen = false;
    tx_armed = true;
}

// Project a captured tick to esp_timer epoch using the nearer base (two-point bracket)
static inline int64_t project_tick(uint32_t tick)
{
    base_t b0, b1;
    portENTER_CRITICAL(&base_mux);
    b0 = base0;
    b1 = base1;
    portEXIT_CRITICAL(&base_mux);
    int32_t d0 = (int32_t)(tick - b0.tick);
    int32_t d1 = (int32_t)(tick - b1.tick);
    base_t b = ((d0 >= 0 && (d0 <= d1 || d1 < 0)) ? b0 : b1);
    int32_t dt = (int32_t)(tick - b.tick);
    double dus = (double)dt * tick_to_us;
    int64_t res = (int64_t)(b.t_us + (dus >= 0 ? (dus + 0.5) : (dus - 0.5))); // round to µs
    return res;
}

static inline bool consume_rx_edge(int64_t *t)
{
    if (!rx_seen)
        return false;
    *t = project_tick(rx_tick);
    rx_seen = false;
    return true;
}
static inline bool consume_tx_edge(int64_t *t)
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
    bool have;
    int64_t t_us;
} seq_edge_t;
static seq_edge_t edges[256];
static inline void store_edge(uint8_t seq, int64_t t)
{
    edges[seq].have = true;
    edges[seq].t_us = t;
}
static inline bool take_edge(uint8_t seq, int64_t *t)
{
    if (!edges[seq].have)
        return false;
    *t = edges[seq].t_us;
    edges[seq].have = false;
    return true;
}

// =================== Slave offset (optional) =================
static volatile int64_t g_offset_us = 0;
static inline int64_t synced_time_us(void) { return esp_timer_get_time() - g_offset_us; }

// ========================= Master ==========================
#if CONFIG_SYNC_ROLE_MASTER
static void master_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Role: MASTER. TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);
    uint8_t seq = 0;
    uint8_t buf[32];
    bool drop_next = true; // drop first result after arm

    while (1)
    {
        // 1) MARKER: send SYNC_HDR(seq) → captures t1 on TX; we use HW t4 later
        arm_tx();
        ESP_ERROR_CHECK(uart_send(UARTX, PKT_SYNC_HDR, seq, NULL, 0));
        uart_wait_tx_done(UARTX, pdMS_TO_TICKS(2));
        esp_rom_delay_us(INTERFRAME_GAP_US);
        int64_t t1 = 0;
        for (int i = 0; i < 4 && !consume_tx_edge(&t1); ++i)
            vTaskDelay(1);
        if (!t1)
        {
            ESP_LOGW(TAG, "no t1 edge");
            goto next;
        }

        // 2) DATA: send SYNC_REQ{t1}
        ESP_ERROR_CHECK(uart_send(UARTX, PKT_SYNC_REQ, seq, &t1, sizeof(t1)));

        // 3) Expect RESP_HDR(seq) → t4 on RX
        arm_rx();
        pkt_hdr_t h;
        int typ = uart_recv(UARTX, buf, sizeof(buf), &h, 1000);
        if (typ != PKT_RESP_HDR)
        {
            ESP_LOGW(TAG, "unexpected hdr %d", typ);
            goto next;
        }
        if (h.seq != seq)
        {
            ESP_LOGW(TAG, "resp hdr seq mismatch (%u!=%u)", h.seq, seq);
            goto next;
        }
        int64_t t4 = 0;
        if (!consume_rx_edge(&t4))
        {
            vTaskDelay(1);
            if (!consume_rx_edge(&t4))
            {
                ESP_LOGW(TAG, "no t4 edge");
                goto next;
            }
        }
        store_edge(seq, t4); // store for pairing with RESP data

        // 4) Read RESP data with same seq → {t2,t3}
        typ = uart_recv(UARTX, (uint8_t *)buf, sizeof(buf), &h, 1000);
        if (typ != PKT_SYNC_RESP || h.len != 16 || h.seq != seq)
        {
            ESP_LOGW(TAG, "bad resp data");
            goto next;
        }
        int64_t t2, t3;
        memcpy(&t2, buf, 8);
        memcpy(&t3, buf + 8, 8);
        {
            int64_t t4_match = 0;
            if (!take_edge(seq, &t4_match))
            {
                ESP_LOGW(TAG, "missing t4 for seq %u", seq);
                goto next;
            }
            int64_t offset = ((t2 - t1) + (t3 - t4_match)) / 2;
            int64_t delay = ((t4_match - t1) - (t3 - t2)) / 2;

            // simple outlier guard
            static int64_t med = 0;
            static bool have = false;
            if (drop_next)
            {
                drop_next = false;
                goto report;
            }
            if (have)
            {
                if (llabs(delay - med) > 50000)
                {
                    ESP_LOGW(TAG, "outlier delay=%lld (drop)", (long long)delay);
                    goto next;
                }
                med = (med * 3 + delay) / 4;
            }
            else
            {
                med = delay;
                have = true;
            }

        report:
            ESP_LOGI(TAG, "RESP: t1=%lld  t2=%lld  t3=%lld  t4=%lld  => delay=%lld us  offset=%lld us",
                     (long long)t1, (long long)t2, (long long)t3, (long long)t4_match, (long long)delay, (long long)offset);
            // optional convenience
            ESP_ERROR_CHECK(uart_send(UARTX, PKT_SET_OFFSET, seq, &offset, sizeof(offset)));
        }
    next:
        seq++;
        vTaskDelay(pdMS_TO_TICKS(200)); // ~5 Hz
    }
}
#endif

// ========================= Slave ===========================
#if CONFIG_SYNC_ROLE_SLAVE
static void slave_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Role: SLAVE. TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);
    uint8_t buf[32];
    bool drop_next = true;

    while (1)
    {
        // We can receive: SET_OFFSET anytime, or SYNC_HDR(seq) to start a cycle
        arm_rx();
        pkt_hdr_t h;
        int typ = uart_recv(UARTX, buf, sizeof(buf), &h, 5000);
        if (typ == PKT_SET_OFFSET && h.len == 8)
        {
            int64_t off;
            memcpy(&off, buf, 8);
            g_offset_us = off;
            ESP_LOGI(TAG, "OFFSET set=%lld", (long long)off);
            drop_next = true;
            continue;
        }
        if (typ != PKT_SYNC_HDR)
        {
            ESP_LOGW(TAG, "unexpected type %d (want SYNC_HDR)", typ);
            drop_next = true;
            continue;
        }
        uint8_t seq = h.seq;

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

        // Send RESP_HDR(seq) → t3 on TX
        arm_tx();
        ESP_ERROR_CHECK(uart_send(UARTX, PKT_RESP_HDR, seq, NULL, 0));
        uart_wait_tx_done(UARTX, pdMS_TO_TICKS(2));
        esp_rom_delay_us(INTERFRAME_GAP_US);
        int64_t t3 = 0;
        for (int i = 0; i < 4 && !consume_tx_edge(&t3); ++i)
            vTaskDelay(1);
        if (!t3)
        {
            ESP_LOGW(TAG, "no t3 edge");
            drop_next = true;
            continue;
        }

        // Send RESP data {t2,t3}
        int64_t payload[2] = {t2, t3};
        ESP_ERROR_CHECK(uart_send(UARTX, PKT_SYNC_RESP, seq, payload, sizeof(payload)));

        // Drain an immediate SET_OFFSET so it won't bleed into next cycle
        int t2t = uart_recv(UARTX, buf, sizeof(buf), &h, 30);
        if (t2t == PKT_SET_OFFSET && h.len == 8)
        {
            int64_t off;
            memcpy(&off, buf, 8);
            g_offset_us = off;
            ESP_LOGI(TAG, "OFFSET set=%lld", (long long)off);
            drop_next = true;
        }
        else if (t2t > 0 && t2t != PKT_SET_OFFSET)
        {
            ESP_LOGW(TAG, "trailing type %d ignored", t2t);
        }
    }
}
#endif

// ============================ app_main =====================
void app_main(void)
{
    ESP_LOGI(TAG, "App start");
    capture_init_start();
    uart_init();
    // re-bind pins in case MCPWM capture claimed them
    ESP_ERROR_CHECK(uart_set_pin(UARTX, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    vTaskDelay(pdMS_TO_TICKS(50)); // let bases accrue

#if CONFIG_SYNC_ROLE_MASTER
    xTaskCreatePinnedToCore(master_task, "master", 4096, NULL, 5, NULL, 0);
#elif CONFIG_SYNC_ROLE_SLAVE
    xTaskCreatePinnedToCore(slave_task, "slave", 4096, NULL, 5, NULL, 1);
#else
#warning "No role set; defaulting to MASTER"
    xTaskCreatePinnedToCore(master_task, "master", 4096, NULL, 5, NULL, 0);
#endif
}

/*
THIS VERSION ONLY HARDWARE STAMPS ONE TIME (t2) ON THE SLAVE USING MCPWM CAPTURE
// main.c — UART two-way time sync (MASTER unchanged, SLAVE uses HW t2 on RX)
// Build one board as MASTER (CONFIG_SYNC_ROLE_MASTER=y) and the other as SLAVE.
//
// No wiring changes required.
// - MASTER: identical to your simple UART-only version.
// - SLAVE: MCPWM Capture timestamps the falling edge on RX as t2_exact.
//          A 1 kHz local strobe keeps a fresh mapping from capture ticks to esp_timer µs.
// this version only HW maps one uart RX pin, for t2.

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "sdkconfig.h"
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#if !CONFIG_SYNC_ROLE_MASTER
#include "driver/mcpwm_cap.h"
#endif

// UART pins & params
#define UARTX_1 UART_NUM_1
#define UARTX_2 UART_NUM_2
#define TX_PIN_1 9
#define RX_PIN_1 10
#define TX_PIN_2 4
#define RX_PIN_2 5
#define UART_BAUD 115200
#define RX_BUF_SIZE 2048

static const char *TAG = "SYNC";

// -------------------- Simple binary framing ----------------------------
enum
{
    PKT_SYNC_REQ = 1,   // payload: int64_t t1
    PKT_SYNC_RESP = 2,  // payload: int64_t t2, int64_t t3
    PKT_SET_OFFSET = 3, // payload: int64_t offset
};
// Packet header that precedes the payload, if receiver doesn't read full TSYN, it discards the packet. Type is for the type of packet and len is the length of the payload.
typedef struct __attribute__((packed))
{
    char magic[4]; // 'T','S','Y','N'
    uint8_t type;
    uint8_t len; // payload bytes
} pkt_hdr_t;

static inline void make_hdr(pkt_hdr_t *h, uint8_t type, uint8_t len)
{
    h->magic[0] = 'T';
    h->magic[1] = 'S';
    h->magic[2] = 'Y';
    h->magic[3] = 'N';
    h->type = type;
    h->len = len;
}

// UART init
static void uart_init_common(void)
{
    uart_config_t cfg_1 = {
        .baud_rate = UART_BAUD, // can be increased to 921600 for faster comms
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UARTX_1, RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UARTX_1, &cfg_1));
    ESP_ERROR_CHECK(uart_set_pin(UARTX_1, TX_PIN_1, RX_PIN_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_config_t cfg_2 = {
        .baud_rate = UART_BAUD, // can be increased to 921600 for faster comms
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UARTX_2, RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UARTX_2, &cfg_2));
    ESP_ERROR_CHECK(uart_set_pin(UARTX_2, TX_PIN_2, RX_PIN_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Read exactly N bytes, and blocks until timer is up or all bytes are read. Returns bytes read.
static int uart_read_exact(uint8_t *dst, int n, int timeout_ms, uart_port_t uart_num)
{
    int bsr = 0;                       // bsr -> bytes successfully read
    int64_t t0 = esp_timer_get_time(); // read start time
    while (bsr < n)                    // continues until all bytes are read or timeout occurs
    {
        int ms_left = timeout_ms - (int)((esp_timer_get_time() - t0) / 1000); // Calculate remaining time
        if (ms_left <= 0)                                                     // if timer is up exit loop
            break;
        int chunk = uart_read_bytes(uart_num, dst + bsr, n - bsr,
                                    pdMS_TO_TICKS(ms_left < 10 ? 10 : ms_left));
        if (chunk > 0) // if bytes were read, increment the count
            bsr += chunk;
    }
    return bsr; // return total bytes read
}

// Send header + payload (blocking)
static esp_err_t uart_send_pkt(uint8_t type, const void *payload, uint8_t len)
{
    pkt_hdr_t h;
    make_hdr(&h, type, len);                                         // build packet header
    int w1 = uart_write_bytes(UARTX_1, (const char *)&h, sizeof(h)); // write packer header
    int w2 = 0;
    if (len) // if there is a payload, write it
        w2 = uart_write_bytes(UARTX_1, (const char *)payload, len);
    return (w1 == sizeof(h) && (len == 0 || w2 == len)) ? ESP_OK : ESP_FAIL; // return ESP_OK if all bytes were written, ESP_FAIL otherwise
}

// Receive one packet (blocking). Returns type, fills payload into buf (up to buf_sz).
static int uart_recv_pkt(uint8_t *buf, int buf_sz, int timeout_ms, uart_port_t uart_num)
{
    pkt_hdr_t h; // if all bytes of the header are not read, or if the magic bytes are wrong, or if the payload is too big, or if the payload is not fully read, return -1
    if (uart_read_exact((uint8_t *)&h, sizeof(h), timeout_ms, uart_num) != sizeof(h))
        return -1;
    if (h.magic[0] != 'T' || h.magic[1] != 'S' || h.magic[2] != 'Y' || h.magic[3] != 'N')
        return -1;
    if (h.len > buf_sz)
        return -1;
    if (h.len) // if there is a payload, read it
    {
        if (uart_read_exact(buf, h.len, timeout_ms, uart_num) != h.len)
            return -1;
    }
    return h.type;
}

// ========================== SLAVE (HW t2) ==============================
#if !CONFIG_SYNC_ROLE_MASTER

// ---- Mapping strobe GPIO and capture GPIO ----
#define ARM_GPIO 16            // random free GPIO for strobe output
#define CAP_SYNC_GPIO RX_PIN_1 // capture the falling edge on UART RX

// MCPWM capture handles & timing
static mcpwm_cap_timer_handle_t s_cap_timer;
static mcpwm_cap_channel_handle_t s_cap_arm_ch;
static mcpwm_cap_channel_handle_t s_cap_rx_ch;

static uint32_t s_cap_res_hz = 0; // queried (80 MHz on S3)
static double s_tick_to_us = 0.0; // 1e6 / res_hz

// Mapping pair (updated by strobe): capture ticks ↔ esp_timer µs
static _Atomic uint32_t s_cap_base_ticks = 0;
static _Atomic int64_t s_T_base_us = 0;

// Latest RX falling edge ticks (from ISR)
static _Atomic uint32_t s_cap_edge_ticks = 0;
static _Atomic bool s_edge_seen = false;

// Offset storage (from master)
static _Atomic int64_t g_offset_us = 0;

// --- Capture callbacks ---
static bool IRAM_ATTR on_cap_arm(mcpwm_cap_channel_handle_t ch,
                                 const mcpwm_capture_event_data_t *e, void *arg)
{ // we capture the base ticks on the ARM strobe rising edge, and then we store the cap_value into s_cap_base_ticks
    atomic_store(&s_cap_base_ticks, e->cap_value);
    return true;
}

static bool IRAM_ATTR on_cap_rx(mcpwm_cap_channel_handle_t ch,
                                const mcpwm_capture_event_data_t *e, void *arg)
{
    // we capture the falling edge on rx, and then we store the cap_value into s_cap_edge_ticks
    atomic_store(&s_cap_edge_ticks, e->cap_value);
    atomic_store(&s_edge_seen, true);
    return true;
}

// --- Capture + channels init ---
static void cap_init_slave(void)
{
    // Set up the strobe GPIO as output (low)
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << ARM_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(ARM_GPIO, 0);

    // Create & start capture timer; resolution is fixed to 80 MHz on S3
    mcpwm_capture_timer_config_t tcfg = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .resolution_hz = 0, // ignored, we'll query actual
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&tcfg, &s_cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(s_cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(s_cap_timer));

    // Query actual resolution
    uint32_t res_hz = 0;
    ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(s_cap_timer, &res_hz)); // capture resolution of the timer
    s_cap_res_hz = res_hz;
    s_tick_to_us = 1e6 / (double)res_hz; // convert tick to microseconds
    ESP_LOGI(TAG, "Capture res: %" PRIu32 " Hz (tick=%.2f ns)",
             (uint32_t)s_cap_res_hz, 1e9 / (double)s_cap_res_hz);

    // Channel A: ARM strobe rising edge
    mcpwm_capture_channel_config_t a = {
        .gpio_num = ARM_GPIO,
        .prescale = 1,
        .flags = {.pos_edge = 1, .neg_edge = 0, .pull_up = 1},
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(s_cap_timer, &a, &s_cap_arm_ch));
    mcpwm_capture_event_callbacks_t cba = {.on_cap = on_cap_arm};
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(s_cap_arm_ch, &cba, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(s_cap_arm_ch));

    // Channel B: UART RX falling edge
    mcpwm_capture_channel_config_t b = {
        .gpio_num = CAP_SYNC_GPIO,
        .prescale = 1,
        .flags = {.pos_edge = 0, .neg_edge = 1, .pull_up = 1},
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(s_cap_timer, &b, &s_cap_rx_ch));
    mcpwm_capture_event_callbacks_t cbb = {.on_cap = on_cap_rx};
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(s_cap_rx_ch, &cbb, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(s_cap_rx_ch));
}

// --- 1 kHz mapping strobe (keeps cap_base_ticks ↔ T_base_us fresh) ---
static esp_timer_handle_t s_map_timer;

static void IRAM_ATTR mapping_strobe_cb(void *arg)
{
    // Brief rising pulse; read esp_timer right after the edge.
    gpio_set_level(ARM_GPIO, 1);
    int64_t t_us = esp_timer_get_time(); // T_base_us
    __asm__ __volatile__("nop;nop;nop;nop");
    gpio_set_level(ARM_GPIO, 0);

    atomic_store(&s_T_base_us, t_us);
}

static void mapping_strobe_start(void)
{
    const esp_timer_create_args_t args = {
        .callback = &mapping_strobe_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "map_strobe"};
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_map_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_map_timer, 1000)); // 1 ms
}

// --- Convert RX edge ticks to esp_timer epoch (µs since boot) ---
static int64_t t2_exact_epoch_us(void)
{
    uint32_t base = atomic_load(&s_cap_base_ticks);
    int64_t T0 = atomic_load(&s_T_base_us);
    uint32_t edge = atomic_load(&s_cap_edge_ticks);

    int32_t dticks = (int32_t)(edge - base); // wrap-safe delta
    double delta_us = (double)dticks * s_tick_to_us;
    return T0 + (int64_t)(delta_us + 0.5); // round to nearest µs
}

static inline int64_t synced_time_us(void)
{
    return esp_timer_get_time() - atomic_load(&g_offset_us);
}

// Slave main loop: respond to requests, accept offset
static void slave_loop(void)
{
    ESP_LOGI(TAG, "Running as SLAVE. TX=%d RX=%d", TX_PIN_1, RX_PIN_1);

    // init capture + mapping strobe
    cap_init_slave();
    mapping_strobe_start();
    // >>> RE-ATTACH UART PINS AFTER MCPWM CLAIMS THE GPIO <<<
    ESP_ERROR_CHECK(uart_set_pin(UARTX_1, TX_PIN_1, RX_PIN_1,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uint8_t buf[32];
    while (true)
    {
        int type = uart_recv_pkt(buf, sizeof(buf), 5000, UARTX_1); // wait up to 5 s
        if (type == PKT_SYNC_REQ && sizeof(int64_t) <= sizeof(buf))
        {
            int64_t t1;
            memcpy(&t1, buf, sizeof(int64_t));
            (void)t1;

            // t2 = hardware-captured RX falling-edge time in esp_timer µs
            if (!atomic_load(&s_edge_seen))
            {
                // If edge hasn't latched yet this cycle, you can wait a short time or proceed;
                // typically it's already set by the time we read.
                // vTaskDelay(1); // optional
            }
            int64_t t2 = t2_exact_epoch_us();

            // prepare response (t2 exact, t3 just before send)
            int64_t payload[2];
            payload[0] = t2;
            payload[1] = esp_timer_get_time(); // t3 (software; ok)

            uart_send_pkt(PKT_SYNC_RESP, payload, sizeof(payload));
            ESP_LOGI(TAG, "RESP: t2_exact=%lld  t3=%lld", (long long)payload[0], (long long)payload[1]);

            atomic_store(&s_edge_seen, false); // ready for next exchange
        }
        else if (type == PKT_SET_OFFSET && sizeof(int64_t) <= sizeof(buf))
        {
            int64_t off;
            memcpy(&off, buf, sizeof(int64_t));
            atomic_store(&g_offset_us, off);
            int64_t raw = esp_timer_get_time();
            ESP_LOGI(TAG, "OFFSET set: %lld us  (raw=%lld, synced=%lld)",
                     (long long)off, (long long)raw, (long long)(raw - off));
        }

        // optional slow heartbeat
        static int hb = 0;
        if ((hb++ % 100) == 0)
        {
            ESP_LOGI(TAG, "heartbeat: raw=%lld  synced=%lld  offset=%lld",
                     (long long)esp_timer_get_time(),
                     (long long)synced_time_us(),
                     (long long)atomic_load(&g_offset_us));
        }
    }
}
#endif // !MASTER

// =========================== MASTER (unchanged) ========================
#if CONFIG_SYNC_ROLE_MASTER
static void master_loop(void)
{
    ESP_LOGI(TAG, "Running as MASTER. TX=%d RX=%d", TX_PIN_1, RX_PIN_1);

    uint8_t buf[32];

    while (true)
    {
        // 1) send SYNC_REQ{t1}
        uart_flush_input(UARTX_1);
        int64_t t1 = esp_timer_get_time();
        uart_send_pkt(PKT_SYNC_REQ, &t1, sizeof(t1));

        // 2) receive SYNC_RESP{t2, t3}; stamp t4 as soon as we get it
        int type = uart_recv_pkt(buf, sizeof(buf), 1000, UARTX_1); // 1 s timeout
        int64_t t4 = esp_timer_get_time();

        if (type == PKT_SYNC_RESP)
        {
            int64_t t2, t3;
            memcpy(&t2, buf, sizeof(int64_t));
            memcpy(&t3, buf + sizeof(int64_t), sizeof(int64_t));

            // 3) compute offset & path delay (NTP-style)
            int64_t offset = ((t2 - t1) + (t3 - t4)) / 2;
            int64_t delay = ((t4 - t1) - (t3 - t2)) / 2;

            ESP_LOGI(TAG, "t1=%lld t2=%lld t3=%lld t4=%lld  => delay≈%lld us, offset≈%lld us",
                     (long long)t1, (long long)t2, (long long)t3, (long long)t4,
                     (long long)delay, (long long)offset);

            // 4) send SET_OFFSET{offset} so the slave can expose synced time
            uart_send_pkt(PKT_SET_OFFSET, &offset, sizeof(offset));
        }
        else
        {
            ESP_LOGW(TAG, "Timeout or bad packet; retrying...");
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // demo: 5 Hz
    }
}
#endif // MASTER

// ----------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "App starting...");
    uart_init_common();

#if CONFIG_SYNC_ROLE_MASTER
    master_loop();
#else
    slave_loop();
#endif
}
*/

/*proper explanation strobe:
3) Why do we need the strobe at all?

Because we have two different clocks on the slave:

Capture timer (80 MHz, gives you edge in ticks via hardware latching)

esp_timer (~µs since boot, what we use in the protocol)

To put the RX edge into the esp_timer epoch, we need a recent relationship between these two clocks. The strobe gives us that relationship, refreshed frequently, with these benefits:

A) A hardware + software coincident pair

At the strobe’s rising edge:

hardware latches the capture timer (exactly on the edge),

software samples esp_timer_get_time() right then.

That yields a clean pair (cap_base_ticks, T_base_us) at almost the same instant (the few CPU cycles difference are tiny and very repeatable).

B) No need to read “live” capture count

The MCPWM capture API latches values on edges; there’s no standard “read current counter” call exposed. The strobe creates regular edges so we can obtain capture ticks precisely when we want (every 1 ms).

C) Keeps the mapping fresh (limits drift & wrap worries)

The capture and esp_timer clocks are not phase-locked and can drift a little. Refreshing the mapping every 1 ms means:

any drift between the two clocks over that 1 ms is negligible (e.g., 50 ppm → 0.05 µs per ms),

dticks stays small (≤ ~80 000 ticks for 1 ms), so 32-bit wrap math stays simple and robust.

D) Avoids ISR timing jitter contamination

You might wonder: “why not just read esp_timer_get_time() inside the RX-edge ISR and pair it with cap_edge_ticks?” You could — but then your critical timestamp (t2) would include ISR entry/exit and function call latency/jitter. With the strobe, any minor software skew happens at the strobe, not at the sync moment, and is very consistent (happens on a relaxed 1 ms cadence). The edge time itself remains a pure hardware measurement; we only add a deterministic projection.

E) Works independent of traffic

Even if no packets arrive for a while, the mapping stays current. When the next edge finally arrives, you still have a mapping within ≤1 ms, so projection error remains tiny.

Tiny timeline to visualize
time →
...   S_k        E               S_{k+1}    ...
      |----1 ms----|----(≤1 ms)----|

S_k:  strobe rises  → cap_base_ticks := CAP(S_k)   (ISR)
                       T_base_us     := esp_timer_get_time()

E:    RX falls      → cap_edge_ticks := CAP(E)     (ISR)

Compute t2:
dticks   = (int32_t)(cap_edge_ticks - cap_base_ticks)   // wrap-safe
delta_us = dticks * 0.0125                              // S3 tick → µs
t2_exact = T_base_us + delta_us                         // esp_timer epoch


Because E is within ~1 ms of S_k, the projection is very accurate.

Could we do it without a strobe?

If you had a way to read the capture timer count synchronously (API call) and you called that exactly when you call esp_timer_get_time(), you could create the mapping on demand (no strobe). The current capture API doesn’t expose a direct “get counter” read.

If you called esp_timer_get_time() inside the RX ISR, you’d pair edge tick with esp_timer at (almost) the same time. That can work, but it moves software jitter into your most critical sample. The 1 ms strobe keeps jitter off the sync event and makes behavior more repeatable.

So the strobe is a practical, robust way to get a regular, near-simultaneous (capture ticks ↔ esp_timer µs) pair with the tools IDF gives us, and it keeps your t2 both clean (hardware edge timing) and expressed in the right epoch for the master’s math.
*/

/* FIRST TIME SYNC PROTOCOL WORKING VERSION

main.c — UART-only two-way time sync (simple, no extra wiring)
// Build as MASTER or SLAVE via CONFIG_SYNC_ROLE_MASTER in sdkconfig.

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

// ---------------- UART pins & params (match your board) ----------------
#define UARTX UART_NUM_1
#define TX_PIN 17
#define RX_PIN_1 18
#define UART_BAUD 115200
#define RX_BUF_SIZE 2048

static const char *TAG = "SYNC";

// -------------------- Simple binary framing ----------------------------
// | magic 'TSYN'(4) | type(1) | len(1) | payload(len) |
// Types:
enum
{
    PKT_SYNC_REQ = 1,   // payload: int64_t t1
    PKT_SYNC_RESP = 2,  // payload: int64_t t2, int64_t t3
    PKT_SET_OFFSET = 3, // payload: int64_t offset
};

typedef struct __attribute__((packed))
{
    char magic[4]; // 'T','S','Y','N'
    uint8_t type;
    uint8_t len; // payload bytes
} pkt_hdr_t;

static inline void make_hdr(pkt_hdr_t *h, uint8_t type, uint8_t len)
{
    h->magic[0] = 'T';
    h->magic[1] = 'S';
    h->magic[2] = 'Y';
    h->magic[3] = 'N';
    h->type = type;
    h->len = len;
}

// --------------- UART init (blocking read/write) -----------------------
static void uart_init_common(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UARTX, RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UARTX, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UARTX, TX_PIN, RX_PIN_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Read exactly N bytes (blocking with overall timeout in ms). Returns bytes read.
static int uart_read_exact(uint8_t *dst, int n, int timeout_ms)
{
    int got = 0;
    int chunk;
    int64_t t0 = esp_timer_get_time();
    while (got < n)
    {
        int ms_left = timeout_ms - (int)((esp_timer_get_time() - t0) / 1000);
        if (ms_left <= 0)
            break;
        chunk = uart_read_bytes(UARTX, dst + got, n - got, pdMS_TO_TICKS(ms_left < 10 ? 10 : ms_left));
        if (chunk > 0)
            got += chunk;
    }
    return got;
}

// Send header + payload (blocking)
static esp_err_t uart_send_pkt(uint8_t type, const void *payload, uint8_t len)
{
    pkt_hdr_t h;
    make_hdr(&h, type, len);
    int w1 = uart_write_bytes(UARTX, (const char *)&h, sizeof(h));
    int w2 = 0;
    if (len)
        w2 = uart_write_bytes(UARTX, (const char *)payload, len);
    return (w1 == sizeof(h) && (len == 0 || w2 == len)) ? ESP_OK : ESP_FAIL;
}

// Receive one packet (blocking). Returns type, fills payload into buf (up to buf_sz).
static int uart_recv_pkt(uint8_t *buf, int buf_sz, int timeout_ms)
{
    pkt_hdr_t h;
    if (uart_read_exact((uint8_t *)&h, sizeof(h), timeout_ms) != sizeof(h))
        return -1;
    if (h.magic[0] != 'T' || h.magic[1] != 'S' || h.magic[2] != 'Y' || h.magic[3] != 'N')
        return -1;
    if (h.len > buf_sz)
        return -1;
    if (h.len)
    {
        if (uart_read_exact(buf, h.len, timeout_ms) != h.len)
            return -1;
    }
    return h.type;
}

// ---------------- SLAVE state -----------------------------------------
#if !CONFIG_SYNC_ROLE_MASTER
static _Atomic int64_t g_offset_us = 0;

static inline int64_t synced_time_us(void)
{
    return esp_timer_get_time() - atomic_load(&g_offset_us);
}

// Slave main loop: respond to requests, accept offset
static void slave_loop(void)
{
    ESP_LOGI(TAG, "Running as SLAVE. TX=%d RX=%d", TX_PIN, RX_PIN_1);

    uint8_t buf[32];
    while (true)
    {
        int type = uart_recv_pkt(buf, sizeof(buf), 5000); // wait up to 5 s
        if (type == PKT_SYNC_REQ && sizeof(int64_t) <= sizeof(buf))
        {
            int64_t t1;
            memcpy(&t1, buf, sizeof(int64_t));
            (void)t1;
            // t2 = receive time
            int64_t t2 = esp_timer_get_time();
            // prepare response (t2, then t3 taken right before send)
            int64_t payload[2];
            payload[0] = t2;
            payload[1] = esp_timer_get_time(); // t3 (immediately before send)
            uart_send_pkt(PKT_SYNC_RESP, payload, sizeof(payload));
            ESP_LOGI(TAG, "RESP: t2=%lld t3=%lld", (long long)payload[0], (long long)payload[1]);
        }
        else if (type == PKT_SET_OFFSET && sizeof(int64_t) <= sizeof(buf))
        {
            int64_t off;
            memcpy(&off, buf, sizeof(int64_t));
            atomic_store(&g_offset_us, off);
            ESP_LOGI(TAG, "OFFSET set: %lld us", (long long)off);
        }
        // optional: heartbeat print
        int64_t raw = esp_timer_get_time();
        int64_t syn = synced_time_us();
        static int cnt = 0;
        if ((cnt++ % 100) == 0)
        {
            ESP_LOGI(TAG, "raw=%lld us  synced=%lld us  offset=%lld",
                     (long long)raw, (long long)syn, (long long)atomic_load(&g_offset_us));
        }
    }
}
#endif

// ---------------- MASTER state ----------------------------------------
#if CONFIG_SYNC_ROLE_MASTER
static void master_loop(void)
{
    ESP_LOGI(TAG, "Running as MASTER. TX=%d RX=%d", TX_PIN, RX_PIN_1);

    uint8_t buf[32];

    while (true)
    {
        // 1) send SYNC_REQ{t1}
        int64_t t1 = esp_timer_get_time();
        uart_send_pkt(PKT_SYNC_REQ, &t1, sizeof(t1));

        // 2) receive SYNC_RESP{t2, t3}; stamp t4 as soon as we get it
        int type = uart_recv_pkt(buf, sizeof(buf), 2000); // 2 s timeout
        int64_t t4 = esp_timer_get_time();

        if (type == PKT_SYNC_RESP)
        {
            int64_t t2, t3;
            memcpy(&t2, buf, sizeof(int64_t));
            memcpy(&t3, buf + sizeof(int64_t), sizeof(int64_t));

            // 3) compute offset & path delay (NTP-style)
            // offset: how much *slave* clock is ahead of *master* clock
            int64_t offset = ((t2 - t1) + (t3 - t4)) / 2;
            int64_t delay = ((t4 - t1) - (t3 - t2)) / 2;

            ESP_LOGI(TAG, "t1=%lld t2=%lld t3=%lld t4=%lld  => delay≈%lld us, offset≈%lld us",
                     (long long)t1, (long long)t2, (long long)t3, (long long)t4,
                     (long long)delay, (long long)offset);

            // 4) send SET_OFFSET{offset} so the slave can expose synced time
            uart_send_pkt(PKT_SET_OFFSET, &offset, sizeof(offset));
        }
        else
        {
            ESP_LOGW(TAG, "Timeout or bad packet; retrying...");
        }

        // demo rate: one sync every 1 s
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

// ----------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "App starting...");
    uart_init_common();

#if CONFIG_SYNC_ROLE_MASTER
    master_loop();
#else
    slave_loop();
#endif
}
*/

/* CODE FOR TESTING MASTER SLAVE COMMS, NO TIME SYNC YET!
// main.c
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "hal/uart_hal.h"
#include "hal/uart_ll.h"

#define UARTX UART_NUM_1
#define TX_PIN 17
#define RX_PIN_1 18
#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024

QueueHandle_t uart_queue = NULL;

// --- Forward declarations ---
static void uart_init_common(void);
static inline void send_sync_break(void);
#if !CONFIG_SYNC_ROLE_MASTER
static void uart_event_task(void *arg);
#endif

// --- UART init ---
static void uart_init_common(void)
{
    printf("Initializing UART%d on TX=%d RX=%d ...\n", UARTX, TX_PIN, RX_PIN_1);

    uart_config_t cfg = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };

#if CONFIG_SYNC_ROLE_MASTER
    ESP_ERROR_CHECK(uart_driver_install(UARTX, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0));
#else
    ESP_ERROR_CHECK(uart_driver_install(UARTX, RX_BUF_SIZE, TX_BUF_SIZE, 10, &uart_queue, 0));
#endif

    ESP_ERROR_CHECK(uart_param_config(UARTX, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UARTX, TX_PIN, RX_PIN_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    printf("UART%d initialized.\n", UARTX);
}

// --- Master: send BREAK ---
static inline void send_sync_break(void)
{
    printf("[MASTER] Sending BREAK...\n");
    uart_ll_tx_break(UART_LL_GET_HW(UARTX), 20);
    uart_wait_tx_done(UARTX, pdMS_TO_TICKS(10));
    printf("[MASTER] BREAK done.\n");
}

// --- Slave: listen for BREAKs ---
#if !CONFIG_SYNC_ROLE_MASTER
static void uart_event_task(void *arg)
{
    printf("[SLAVE] UART event task started.\n");
    uart_event_t ev;

    for (;;)
    {
        if (xQueueReceive(uart_queue, &ev, portMAX_DELAY))
        {
            switch (ev.type)
            {
            case UART_BREAK:
            {

                printf("[SLAVE] SYNC BREAK detected at");
                int64_t t_us = esp_timer_get_time();
                printf(" %lld us\n", (long long)t_us);
                break;
            }
            case UART_DATA:
            {
                printf("[SLAVE] UART_DATA event (%d bytes)\n", ev.size);
                uint8_t buf[256];
                int n = uart_read_bytes(UARTX, buf, ev.size, 10 / portTICK_PERIOD_MS);
                buf[n < sizeof(buf) ? n : sizeof(buf) - 1] = '\0';
                printf("[SLAVE] Data: %s\n", buf);
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                printf("[SLAVE] UART overflow or buffer full! Flushing.\n");
                uart_flush_input(UARTX);
                xQueueReset(uart_queue);
                break;
            default:
                printf("[SLAVE] Other UART event: %d\n", ev.type);
                break;
            }
        }
    }
}
#endif

// --- app_main ---
void app_main(void)
{
    printf("App starting...\n");
    uart_init_common();

#if CONFIG_SYNC_ROLE_MASTER
    printf("Running as MASTER.\n");
    while (true)
    {
        //  Write data to UART.
        int64_t t_us = esp_timer_get_time();
        send_sync_break();
        uart_write_bytes(UARTX, (const char *)&t_us, sizeof(t_us));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
#else
    printf("Running as SLAVE.\n");
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
#endif
}
*/
