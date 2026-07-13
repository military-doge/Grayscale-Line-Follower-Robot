#include "jy62.h"

/* State machine states */
#define STATE_WAIT_HEADER  0
#define STATE_WAIT_TYPE    1
#define STATE_READ_DATA    2
#define STATE_CHECKSUM     3

/* Frame type bytes */
#define FRAME_ACCEL  0x51
#define FRAME_GYRO   0x52
#define FRAME_ANGLE  0x53
#define FRAME_HEADER 0x55

/* Scale factors */
#define GYRO_SCALE   2000.0f
#define ANGLE_SCALE  180.0f
#define SCALE_DIV    32768.0f

static volatile uint8_t  g_state       = STATE_WAIT_HEADER;
static volatile uint8_t  g_data_idx    = 0;
static volatile uint8_t  g_frame_type  = 0;
static volatile uint8_t  g_buf[8];
static volatile uint8_t  g_checksum;

static volatile float    g_wz  = 0.0f;
static volatile float    g_yaw = 0.0f;
static volatile uint8_t  g_data_ok = 0;

void JY62_Init(void)
{
    g_state      = STATE_WAIT_HEADER;
    g_data_idx   = 0;
    g_frame_type = 0;
    g_checksum   = 0;
    g_wz         = 0.0f;
    g_yaw        = 0.0f;
    g_data_ok    = 0;
}

float JY62_Get_AngularVelocityZ(void)
{
    return g_wz;
}

float JY62_Get_Yaw(void)
{
    return g_yaw;
}

uint8_t JY62_Is_Data_Ready(void)
{
    return g_data_ok;
}

void JY62_UART_RX_ISR(uint8_t byte)
{
    switch (g_state)
    {
    case STATE_WAIT_HEADER:
        if (byte == FRAME_HEADER)
        {
            g_checksum = byte;
            g_state = STATE_WAIT_TYPE;
        }
        break;

    case STATE_WAIT_TYPE:
        g_frame_type = byte;
        g_checksum += byte;
        if (byte == FRAME_ACCEL || byte == FRAME_GYRO || byte == FRAME_ANGLE)
        {
            g_data_idx = 0;
            g_state = STATE_READ_DATA;
        }
        else
        {
            g_state = STATE_WAIT_HEADER;
        }
        break;

    case STATE_READ_DATA:
        g_buf[g_data_idx] = byte;
        g_checksum += byte;
        g_data_idx++;
        if (g_data_idx >= 8)
        {
            g_state = STATE_CHECKSUM;
        }
        break;

    case STATE_CHECKSUM:
        /* Verify checksum: sum of first 10 bytes & 0xFF == byte */
        if ((g_checksum & 0xFF) == byte)
        {
            int16_t raw;

            switch (g_frame_type)
            {
            case FRAME_GYRO:
                raw = ((int16_t)g_buf[5] << 8) | g_buf[4];
                g_wz = (float)raw / SCALE_DIV * GYRO_SCALE;
                g_data_ok = 1;
                break;

            case FRAME_ANGLE:
                /* g_buf[4]=YawL, g_buf[5]=YawH */
                raw = ((int16_t)g_buf[5] << 8) | g_buf[4];
                g_yaw = (float)raw / SCALE_DIV * ANGLE_SCALE;
                break;

            case FRAME_ACCEL:
                /* Acceleration frame — ignored */
                break;
            }
        }
        /* Checksum fail or processed: back to waiting for next frame */
        g_state = STATE_WAIT_HEADER;
        break;
    }
}
