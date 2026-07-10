# Grayscale Line Follower Robot

**Version:** 1.2-dev

## 概述

本项目为基于 TI MSPM0G3507 的灰度循线机器人控制系统，采用 **app-bsp-middleware-core 四层架构**。

当前已实现 SSD1306 OLED 显示、TB6612 双电机驱动、MG513 编码器采集、增量式 PI 速度闭环控制。

## 接线说明

### OLED 显示屏

| OLED 引脚 | MSPM0G3507 引脚 | SysConfig 实例 |
|:---------:|:---------------:|:--------------:|
| VCC       | 3.3V            | —              |
| GND       | GND             | —              |
| SCL (时钟) | PA28            | OLED_SCL      |
| SDA (数据) | PA31            | OLED_SDA      |
| DC  (命令/数据) | PB15        | OLED_DC       |
| RST (复位) | PB14            | OLED_RST      |

### TB6612 电机驱动

| TB6612 引脚 | MSPM0G3507 引脚 | 说明 |
|:---:|:---:|:---|
| PWMA | PB2 | 电机 A PWM（TIMA1 CCP0） |
| PWMB | PB3 | 电机 B PWM（TIMA1 CCP1） |
| AIN1 | PA14 | 电机 A 方向控制 1 |
| AIN2 | PA13 | 电机 A 方向控制 2 |
| BIN1 | PA16 | 电机 B 方向控制 1 |
| BIN2 | PA17 | 电机 B 方向控制 2 |
| STBY | 3.3V/5V | 使能（拉高） |

### MG513 编码器

| 编码器引脚 | MSPM0G3507 引脚 | 说明 |
|:---:|:---:|:---|
| E1A | PA25 | 电机 A 编码器 A 相 |
| E1B | PA26 | 电机 A 编码器 B 相 |
| E2A | PB20 | 电机 B 编码器 A 相 |
| E2B | PB24 | 电机 B 编码器 B 相 |

### 其他外设

| 功能 | MSPM0G3507 引脚 |
|:---:|:---:|
| UART TX | PA10 |
| UART RX | PA11 |
| 按键 (KEY) | PA18 |
| LED | PB9 |

## 目录结构

```
├── app/                 # 应用层（主逻辑 + PID 控制）
│   ├── main.c           # 主程序：OLED 初始化 + 主循环 + 中断处理
│   ├── control.h        # PID 控制器接口（Motor_parameter、PID_Parameter 结构体）
│   └── control.c        # PID 控制器实现（速度转换、低通滤波、增量式 PI）
├── bsp/                 # 板级支持包
│   ├── board.h/c        # 类型定义、SysTick 延时、printf 重定向
│   ├── led.h/c          # LED 控制（亮/灭/翻转/闪烁）
│   ├── key.h/c          # 按键扫描（单击/双击/长按）
│   ├── motor.h/c        # TB6612 驱动（Set_PWM / limit_PWM）
│   ├── encoder.h/c      # 编码器读取（Get_Encoder / 正交解码）
│   ├── oled.h/c         # SSD1306 OLED 驱动（软件 SPI）
│   └── oledfont.h       # ASCII 字库 (6x12 / 8x16) + 中文 (16x16)
├── middleware/           # 中间件层（待填充）
│   └── .gitkeep
├── core/                 # 核心层
│   ├── empty.syscfg      # SysConfig 配置文件
│   ├── ti_msp_dl_config.h/c  # SysConfig 生成代码（gitignored）
│   ├── startup_mspm0g350x_uvision.s  # 启动文件
│   └── mspm0g3507.sct    # 散列文件
├── tools/keil/           # SysConfig 工具链集成
├── reference/            # 参考代码
│   ├── WHEELTEC_C07A_Bluetooth/
│   ├── WHEELTEC_C07A_OLED/
│   └── WHEELTEC_C07A_TB6612/
├── sdk_config.ini        # TI SDK / SysConfig 路径配置
├── apply_sdk_paths.bat   # 同步 SDK 路径到 .uvprojx
└── Grayscale-Line-Follower-Robot.uvprojx  # Keil 项目
```

## 模块说明

### app/main.c

主程序逻辑：
- `main()`：SYSCFG_DL_init() → OLED_Init() → 显示启动画面 → user_init() → user_main()
- `user_init()`：初始化 PID 参数（从 V_PID 结构体加载）、启动 PWM 计数器、使能编码器和定时器中断、设置目标速度为 0
- `user_main()`：主循环，每 500ms 更新 OLED 显示（左右电机速度 + 启停状态）
- `TIMER_0_INST_IRQHandler()`：10ms 定时器中断 — LED 闪烁、按键扫描、编码器速度转换、增量式 PI 控制、设置 PWM
- `GROUP1_IRQHandler()`：调用 `Get_Encoder()` 读取编码器

- **Flag_Stop**：全局启停标志，`1` = 停止，`0` = 运行（单击 KEY 切换）

### app/control — 速度计算 + PID 控制

- `Get_Velocity_From_Encoder(Encoder1, Encoder2)` — 编码器原始计数值转换为滤波后的速度（m/s），含一阶低通滤波
- `Incremental_PI_Left(Encoder, Target)` — 左电机增量式 PI 控制器，含死区
- `Incremental_PI_Right(Encoder, Target)` — 右电机增量式 PI 控制器，含死区
- `PWM_Limit(IN, max, min)` — PWM 输出限幅

**编码器 → 速度转换公式：**
`speed (m/s) = encoder_count × Frequency × Perimeter / (ENCODER_LINES × MULTIPLY_FACTOR × GEAR_RATIO)`

**增量式 PI 公式：**
`pwm += Kp × [e(k) - e(k-1)] + Ki × e(k)`

| 参数 | 变量名 | 默认值 | 说明 |
|:---:|:---:|:---:|:---|
| Kp | `Velocity_KP` | 400 | 速度环比例系数 |
| Ki | `Velocity_KI` | 300 | 速度环积分系数 |
| 死区 | `PI_DEADBAND` | 0.005 m/s | 偏差小于该值停止 PI 累加 |
| 滤波系数 | `SPEED_FILTER_ALPHA` | 0.4 | 一阶低通滤波系数 |
| PWM 限幅 | `PWM_MAX` | ±7800 | PWM 输出上限 |

### bsp/motor — TB6612 驱动

- `Set_PWM(pwmA, pwmB)` — 设置双电机 PWM 和方向：正值 → 正转（AIN1/BIN1 拉高），负值 → 反转
- `limit_PWM(value, low, high)` — PWM 限幅（整型版本）

### bsp/encoder — 编码器读取

- `Get_Encoder()` — GPIO 中断读取两路编码器，2 倍频正交解码，更新 `Get_Encoder_countA/B`
- `Get_Encoder_countA/B` — 全局编码器脉冲累加变量（10ms 定时器中断中清零）
- `GROUP1_IRQHandler()` — GPIO 中断入口，调用 `Get_Encoder()`

### bsp/oled — OLED 显示

- `OLED_Init()` — 初始化 OLED（硬件复位 + 寄存器配置序列）
- `OLED_Clear()` — 清屏
- `OLED_Refresh_Gram()` — 将 GRAM 缓冲刷新到屏幕
- `OLED_ShowString(x, y, p)` — 显示字符串
- `OLED_ShowNumber(x, y, num, len, size)` — 显示数字

### bsp/led — LED 控制

- `LED_ON/OFF/Toggle()` — 控制 PB9 引脚
- `LED_Flash(period_ms)` — 周期性闪烁（在 10ms 中断中调用，period=100 → 1Hz）

### bsp/key — 按键扫描

- `Key()` — 状态机扫描 PA18 按键，支持单击、双击、长按
- 单击：切换 `Flag_Stop` 启停，启动时设置目标速度 `MotorA/B.Target_Encoder = 1.0 m/s`，停止时置 0

## 使用方法

1. 上电后，OLED 显示启动画面，电机默认停止（`Flag_Stop = 1`，`Target_Encoder = 0`）
2. **单击 KEY 按键**：电机开始 PID 速度闭环运行，目标速度 1.0 m/s
3. **再次单击 KEY**：电机停止（目标速度置 0）
4. OLED 每 500ms 刷新显示左右电机速度和启停状态

## 依赖

- **MCU:** Texas Instruments MSPM0G3507 (Cortex-M0+)
- **SDK:** MSPM0 SDK 2.01.00.03
- **SysConfig:** 1.20.0
- **IDE:** Keil µVision (ARM Compiler V6.19)
- **OLED:** SSD1306 兼容 128x64 OLED 显示屏

## 编译

1. 安装 TI MSPM0 SDK 和 SysConfig
2. 修改 `sdk_config.ini` 中的实际路径
3. 双击 `apply_sdk_paths.bat` 同步路径到 `.uvprojx`
4. 在 Keil 中打开 SysConfig 文件（`core/empty.syscfg`）生成 `ti_msp_dl_config.h/c`
5. 编译下载
