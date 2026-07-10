# OLED Display Module

**Version:** 1.0-dev

## 概述

本项目在 MSPM0G3507 平台上实现了 **SSD1306 OLED (128x64)** 显示屏驱动，采用 **app-bsp-middleware-core** 四层架构，基于软件 SPI（GPIO 位操作）与 OLED 通信。

## 接线说明

| OLED 引脚 | MSPM0G3507 引脚 | SysConfig 实例 |
|:---------:|:---------------:|:--------------:|
| VCC       | 3.3V            | —              |
| GND       | GND             | —              |
| SCL (时钟) | PA28            | OLED_SCL      |
| SDA (数据) | PA31            | OLED_SDA      |
| DC  (命令/数据) | PB15        | OLED_DC       |
| RST (复位) | PB14            | OLED_RST      |

**其他系统接口：**

| 功能     | MSPM0G3507 引脚 |
|:--------:|:---------------:|
| UART_TX  | PA10            |
| UART_RX  | PA11            |
| LED      | PB9             |

## 目录结构

```
├── app/main.c            # 应用层 — OLED 显示 demo
├── bsp/                  # 板级支持包
│   ├── board.h / board.c # SysTick 延时 + UART printf 重定向
│   ├── oled.h / oled.c   # SSD1306 OLED 驱动（软件 SPI）
│   ├── oledfont.h        # ASCII 字库 (6x12 / 8x16) + 中文 (16x16)
│   └── led.h / led.c     # LED 控制 (PB9)
├── core/
│   ├── empty.syscfg      # SysConfig GPIO 引脚配置
│   └── ti_msp_dl_config.* # SysConfig 自动生成
└── middleware/            # 中间件层（预留）
```

## 四层架构

| 层 | 说明 | 本模块内容 |
|:--|:-----|:----------|
| **app** | 应用层 — 业务逻辑 | `main.c` 初始化 OLED 并在主循环中显示字符串与递增计数器 |
| **bsp** | 板级支持包 — 硬件抽象 | OLED/ LED 驱动、SysTick 延时、UART 重定向 |
| **middleware** | 中间件层 — 协议栈 | 预留，OLED 无需中间件 |
| **core** | 核心层 — MCU 启动、系统时钟 | SysConfig 生成代码、启动文件 |

## OLED API 参考

```c
void OLED_Init(void);                                         // 初始化 OLED（硬件复位 + 寄存器配置序列）
void OLED_Clear(void);                                        // 清屏
void OLED_Display_On(void);                                   // 开启显示
void OLED_Display_Off(void);                                  // 关闭显示
void OLED_Refresh_Gram(void);                                 // 将 GRAM 缓冲刷新到屏幕
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);         // 画点 (t=1 亮, t=0 灭)
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr,
                   uint8_t size, uint8_t mode);               // 显示字符 (size=12 或 16)
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p); // 显示字符串
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num,
                     uint8_t len, uint8_t size);              // 显示数字
```

## 显示缓冲区

OLED 使用全局帧缓冲 `OLED_GRAM[128][8]`（128 列 x 8 页，每页 8 像素 = 64 行）。所有绘制函数操作此缓冲区，调用 `OLED_Refresh_Gram()` 刷写到屏幕。

## 依赖

- **MCU:** Texas Instruments MSPM0G3507 (Cortex-M0+)
- **SDK:** MSPM0 SDK 2.01.00.03
- **SysConfig:** 1.20.0
- **IDE:** Keil uVision (ARM Compiler V6.19+)
- **OLED:** SSD1306 兼容 128x64 OLED 显示屏

## 编译

1. 确保 `sdk_config.ini` 中 SDK / SysConfig 路径正确
2. 双击 `apply_sdk_paths.bat` 同步路径到 Keil 工程
3. 在 Keil 中打开 `Grayscale-Line-Follower-Robot.uvprojx`
4. Rebuild (F7 下拉 → Rebuild all target files)

## 运行效果

启动后 UART 输出 `OLED Init OK!`，OLED 屏幕显示 "HELLO WHEELTEC" 和递增计数器。板载 LED (PB9) 以约 1 Hz 频率闪烁。
