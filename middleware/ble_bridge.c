/**
 * ble_bridge.c — BLE 指令桥接模块
 *
 * 协议:
 *   !AT<cmd>          → 转发 AT 指令给 JDY-16，等待回复
 *   !HEX <bytes>      → 解析 HEX 发送到 JDY-16
 *   !ENC <dur> <int> [save] → 定时读取编码器
 *
 * 硬件: UART_1 (PB6=TX, PB7=RX) 连接 JDY-16
 */

#include "ti_msp_dl_config.h"
#include "ble_bridge.h"
#include <string.h>
#include <stdlib.h>

/* ==========================================================================
 * 常量
 * ========================================================================== */

#define RX_BUF_SIZE         256
#define LINE_BUF_SIZE       128
#define RESP_BUF_SIZE       256
#define RESP_TIMEOUT_TICK   50      /* 500ms */
#define RESP_TAIL_TICK      5       /* 50ms 尾等 */

/* ==========================================================================
 * 环形缓冲区 (ISR 写入, poll 读取)
 * ========================================================================== */

static volatile uint8_t  rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

/* ==========================================================================
 * 行缓冲 + 回复缓冲
 * ========================================================================== */

static uint8_t  line_buf[LINE_BUF_SIZE];
static uint16_t line_len;

static uint8_t  resp_buf[RESP_BUF_SIZE];
static uint16_t resp_len;

/* ==========================================================================
 * 状态机
 * ========================================================================== */

typedef enum {
    STATE_IDLE,
    STATE_AWAITING_RESP,
} bridge_state_t;

static bridge_state_t g_state;
static uint32_t       g_resp_start_tick;
static uint8_t        g_resp_got_nl;

extern volatile uint32_t g_tick_10ms;

/* ==========================================================================
 * 编码器任务
 * ========================================================================== */

typedef struct {
    uint8_t  active;
    uint16_t interval_tick;
    uint16_t duration_tick;
    uint8_t  save;
    int32_t  buf_a[256];
    int32_t  buf_b[256];
    uint16_t buf_count;
} enc_task_t;

static enc_task_t g_enc;
static uint32_t   g_enc_start_tick;
static uint32_t   g_enc_last_tick;

extern int Get_Encoder_countA;
extern int Get_Encoder_countB;

int32_t ble_bridge_get_encoder_a(void) { return (int32_t)Get_Encoder_countA; }
int32_t ble_bridge_get_encoder_b(void) { return (int32_t)Get_Encoder_countB; }

/* ==========================================================================
 * 环形缓冲区
 * ========================================================================== */

static uint8_t ring_read(uint8_t *byte)
{
    if (rx_head == rx_tail) return 0;
    *byte = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}

/* ==========================================================================
 * UART_1 发送
 * ========================================================================== */

static void uart1_putc(uint8_t ch)
{
    while (DL_UART_isBusy(UART_1_INST) == true);
    DL_UART_Main_transmitData(UART_1_INST, ch);
}

static void uart1_send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) uart1_putc(buf[i]);
}

static void uart1_send_str(const char *s)
{
    while (*s) uart1_putc((uint8_t)*s++);
}

static void uart1_send_at_cmd(const char *at_cmd)
{
    uart1_send_str(at_cmd);
    uart1_putc('\r');
    uart1_putc('\n');
}

static void uart1_send_int(int32_t val)
{
    char buf[12];
    int i = 0;
    uint8_t neg = 0;
    if (val < 0) { neg = 1; val = -val; }
    do {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    } while (val > 0 && i < 11);
    if (neg) buf[i++] = '-';
    while (i > 0) uart1_putc((uint8_t)buf[--i]);
}

/* ==========================================================================
 * HEX 发送
 * ========================================================================== */

static uint8_t hex_digit(uint8_t ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return 0xFF;
}

static void uart1_send_hex(const char *hex_str)
{
    uint8_t high = 0, idx = 0;
    while (*hex_str) {
        uint8_t ch = *hex_str;
        if (ch == ' ' || ch == ',' || ch == '\r' || ch == '\n') { hex_str++; continue; }
        if (ch == '0' && (*(hex_str + 1) == 'x' || *(hex_str + 1) == 'X')) { hex_str += 2; continue; }
        uint8_t val = hex_digit(ch);
        if (val == 0xFF) { hex_str++; continue; }
        if (idx == 0) { high = val; idx = 1; }
        else { uart1_putc((high << 4) | val); idx = 0; }
        hex_str++;
    }
}

/* ==========================================================================
 * 命令处理
 * ========================================================================== */

static void handle_at_command(const char *cmd)
{
    uart1_send_at_cmd(cmd + 1);
    g_state          = STATE_AWAITING_RESP;
    resp_len         = 0;
    g_resp_start_tick = g_tick_10ms;
    g_resp_got_nl    = 0;
}

static void handle_hex_command(const char *cmd)
{
    const char *hex_str = cmd + 4;
    while (*hex_str == ' ') hex_str++;
    uart1_send_hex(hex_str);
    uart1_putc('\r');
    uart1_putc('\n');
}

/* !ENC <duration_ms> <interval_ms> [save]
 * duration_ms: 持续时长 (ms), 0 = 单次
 * interval_ms: 采样间隔 (ms), 最小 10ms
 * save: 可选 "1" 或 "save" → 结束时一次性发送全部数据 */
static void handle_enc_command(const char *cmd)
{
    int dur_ms = 0, int_ms = 100, save = 0;

    /* 跳过 "!ENC " */
    const char *p = cmd + 4;
    while (*p == ' ') p++;

    /* 解析参数 */
    dur_ms = atoi(p);
    while (*p >= '0' && *p <= '9') p++;
    while (*p == ' ') p++;
    int_ms = atoi(p);
    while (*p >= '0' && *p <= '9') p++;
    while (*p == ' ') p++;
    if (*p == '1' || *p == 's' || *p == 'S') save = 1;

    /* 参数限幅 */
    if (int_ms < 10) int_ms = 10;
    if (dur_ms <= 0) dur_ms = int_ms;   /* 至少采样一次 */
    if (dur_ms > 30000) dur_ms = 30000; /* 最大 30 秒 */

    g_enc.active        = 1;
    g_enc.interval_tick = (uint16_t)(int_ms / 10);
    g_enc.duration_tick = (uint16_t)(dur_ms / 10);
    g_enc.save          = (uint8_t)save;
    g_enc.buf_count     = 0;
    g_enc_start_tick    = g_tick_10ms;
    g_enc_last_tick     = g_tick_10ms;

    uart1_send_str("OK ENC:");
    uart1_send_int(dur_ms);
    uart1_send_str("ms int:");
    uart1_send_int(int_ms);
    uart1_send_str("ms");
    if (save) uart1_send_str(" [SAVE]");
    uart1_send_str("\r\n");
}

/* ==========================================================================
 * 编码器任务轮询 (在 ble_bridge_poll 中调用)
 * ========================================================================== */

static void enc_task_poll(void)
{
    if (!g_enc.active) return;

    uint32_t now   = g_tick_10ms;
    uint32_t elapsed = now - g_enc_start_tick;

    /* 到间隔 → 发送数据 */
    if ((now - g_enc_last_tick) >= g_enc.interval_tick) {
        g_enc_last_tick = now;

        if (g_enc.save) {
            if (g_enc.buf_count < 256) {
                g_enc.buf_a[g_enc.buf_count] = Get_Encoder_countA;
                g_enc.buf_b[g_enc.buf_count] = Get_Encoder_countB;
                g_enc.buf_count++;
            }
        } else {
            uart1_send_str("ENC:");
            uart1_send_int(Get_Encoder_countA);
            uart1_putc(',');
            uart1_send_int(Get_Encoder_countB);
            uart1_send_str("\r\n");
        }
    }

    /* 到期 → 结束 */
    if (elapsed >= g_enc.duration_tick) {
        if (g_enc.save && g_enc.buf_count > 0) {
            uint16_t i;
            for (i = 0; i < g_enc.buf_count; i++) {
                uart1_send_str("ENC:");
                uart1_send_int(g_enc.buf_a[i]);
                uart1_putc(',');
                uart1_send_int(g_enc.buf_b[i]);
                uart1_send_str("\r\n");
            }
        }
        uart1_send_str("ENC_DONE\r\n");
        g_enc.active = 0;
    }
}

/* ==========================================================================
 * 命令分发
 * ========================================================================== */

static void dispatch_command(void)
{
    /* !AT */
    if (line_len >= 3 && line_buf[0] == '!' && line_buf[1] == 'A' && line_buf[2] == 'T') {
        handle_at_command((const char *)line_buf);
        return;
    }
    /* !HEX */
    if (line_len >= 5 && line_buf[0] == '!' && line_buf[1] == 'H' &&
        line_buf[2] == 'E' && line_buf[3] == 'X') {
        handle_hex_command((const char *)line_buf);
        return;
    }
    /* !ENC */
    if (line_len >= 5 && line_buf[0] == '!' && line_buf[1] == 'E' &&
        line_buf[2] == 'N' && line_buf[3] == 'C') {
        handle_enc_command((const char *)line_buf);
        return;
    }
    /* 未知命令 */
    uart1_send_str("?CMD\r\n");
}

/* ==========================================================================
 * UART_1 中断
 * ========================================================================== */

void UART1_IRQHandler(void)
{
    uint32_t status = DL_UART_Main_getPendingInterrupt(UART_1_INST);
    if (status == DL_UART_IIDX_RX) {
        uint8_t  byte = DL_UART_Main_receiveData(UART_1_INST);
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) {
            rx_ring[rx_head] = byte;
            rx_head = next;
        }
    }
}

/* ==========================================================================
 * 公开 API
 * ========================================================================== */

void ble_bridge_init(void)
{
    rx_head = 0;
    rx_tail = 0;
    line_len = 0;
    resp_len = 0;
    g_state  = STATE_IDLE;

    /* 波特率 9600 */
    DL_UART_Main_setOversampling(UART_1_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST, 260, 27);

    /* FIFO */
    DL_UART_Main_enableFIFOs(UART_1_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_1_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(UART_1_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

    /* 中断 */
    DL_UART_Main_enableInterrupt(UART_1_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_SetPriority(UART_1_INST_INT_IRQN, 1);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

void ble_bridge_poll(void)
{
    uint8_t byte;

    /* ── 编码器任务 (非阻塞) ── */
    enc_task_poll();

    /* ── IDLE: 累积命令 ── */
    if (g_state == STATE_IDLE) {
        while (ring_read(&byte)) {
            if (line_len < LINE_BUF_SIZE - 1) {
                line_buf[line_len++] = byte;
            }
            if (byte == '\n' || byte == '\r') {
                line_buf[line_len] = '\0';
                if (line_len > 1) dispatch_command();
                line_len = 0;
            }
        }
    }

    /* ── AWAITING_RESP: 收集 JDY-16 回复 ── */
    if (g_state == STATE_AWAITING_RESP) {
        while (ring_read(&byte)) {
            if (resp_len < RESP_BUF_SIZE - 1) {
                resp_buf[resp_len++] = byte;
            }
            if (byte == '\n') g_resp_got_nl = 1;
        }
        uint32_t elapsed = g_tick_10ms - g_resp_start_tick;
        uint8_t timed_out = 0;
        if (g_resp_got_nl && elapsed >= RESP_TAIL_TICK && resp_len > 0) timed_out = 1;
        else if (!g_resp_got_nl && elapsed >= RESP_TIMEOUT_TICK) timed_out = 1;

        if (timed_out) {
            if (resp_len > 0) {
                resp_buf[resp_len] = '\0';
                uart1_send(resp_buf, resp_len);
            }
            g_state = STATE_IDLE;
        }
    }
}
