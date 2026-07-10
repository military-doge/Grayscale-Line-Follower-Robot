# Grayscale Line Follower Robot

**Version:** 0.0.0

## 项目说明

本项目为一个 **空壳工程**，基于 TI MSPM0G3507 平台，采用 **app-bsp-middleware-core 四层架构**。

当前仅包含 Keil 项目配置、SDK 路径管理脚本以及最小可编译的 empty 示例代码（`main.c` 中仅有 `SYSCFG_DL_init()` + `while(1)` 循环）。

## 目录结构

```
├── app/                 # 应用层（当前仅 main.c）
├── bsp/                 # 板级支持包（待填充）
├── middleware/          # 中间件层（待填充）
├── core/                # 核心层（启动文件、SysConfig 生成代码、散列文件）
├── tools/keil/          # SysConfig 工具链集成
├── reference/           # 参考代码
│   ├── WHEELTEC_C07A_Bluetooth/   # 蓝牙通信参考
│   ├── WHEELTEC_C07A_OLED/        # OLED 显示参考
│   └── WHEELTEC_C07A_TB6612/      # 电机驱动 TB6612 参考
├── sdk_config.ini       # Windows TI SDK / SysConfig 路径配置
├── apply_sdk_paths.bat  # 根据 sdk_config.ini 更新 .uvprojx 路径
└── Grayscale-Line-Follower-Robot.uvprojx  # Keil 项目文件
```

## 依赖

- **MCU:** Texas Instruments MSPM0G3507 (Cortex-M0+)
- **SDK:** MSPM0 SDK 2.01.00.03
- **SysConfig:** 1.20.0
- **IDE:** Keil µVision (ARM Compiler V6.22)

## 移植到新机器

1. 安装 TI SDK 和 SysConfig
2. 修改 `sdk_config.ini` 中的实际路径
3. 双击 `apply_sdk_paths.bat` 同步路径到 `.uvprojx`
4. 打开 Keil 编译

## 后续计划

### 1.0 dev — 实现 OLED 显示功能

当前版本为 0.0.0 空壳工程，接下来将在独立分支上开发 OLED 显示功能。

**操作方式：**

在 VSCode 左下角点击分支名称 → "创建新分支" → 输入 `1.0-dev`

开发完成后，在 VSCode 源码管理面板提交代码，然后点击分支名称 → 选择 `master` → "合并分支..." → 选 `1.0-dev`
