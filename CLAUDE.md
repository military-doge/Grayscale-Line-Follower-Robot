# MSPM0 灰度小车项目

## 项目概述
基于 TI MSPM0 的灰度循迹小车固件项目，使用 Keil + SysConfig + DriverLib。

## 四层架构 (1.3-dev+)

```
core/       L1 HAL — SysConfig 生成、启动文件、中断向量
bsp/        L2 BSP — 纯硬件驱动，仅依赖 core，不依赖业务逻辑
middleware/ L3 算法 — 协议解析、编码器换算、PID/PD 控制器、航向锁定
app/        L4 应用 — 主循环、ISR 编排、OLED UI、模式切换
```

### 层间依赖规则
- L2 BSP → L1 Core  ✅ (ti_msp_dl_config.h)
- L3 Middleware → L2 BSP / L1 Core ✅
- L4 App → L3 Middleware / L2 BSP ✅
- L2 BSP → L3/L4 ❌ (BSP 不依赖上层)

### 解耦要点
- `dma_rx` 使用回调模式，不直接依赖 `jy62`
- `key` 返回事件枚举，不直接操作 app 全局变量
- `board.h` 暂不处理（待后续清理）

## MSPM0 Skill
本项目使用 MSPM0-CCS skill 进行 SysConfig 配置、DriverLib 编程和硬件调试。
Skill 位置：`.claude/skills/mspm0-ccs/SKILL.md`

## 硬件参数
- 轮径：48mm
- 轴距：130mm
- 减速比：20:1
- 开发板：天盟星 MSPM0G3507（LCKFB）

## 注意事项
- `.syscfg` 是外设配置的唯一真源，不要手动编辑 `ti_msp_dl_config.c/h`
- 使用 `python scripts/check_syscfg.py <project-dir>` 进行静态检查
- 使用 `python scripts/detect_probe.py` 检测调试器
