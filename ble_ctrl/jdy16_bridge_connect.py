#!/usr/bin/env python3
"""
JDY-16 蓝牙桥接命令行工具
========================
通过 BLE 连接 JDY-16 透传模块，由 MCU 固件 (ble_bridge.c) 中继 AT 指令给 JDY-16。

协议: PC 发送 !AT<cmd> → MCU 接收 → 转发 AT<cmd> 给 JDY-16 → 收集回复 → 发回 PC

依赖: pip install bleak
运行: python jdy16_bridge_connect.py

命令:
  !AT              - 测试连接，返回 OK
  !AT+VERSION      - 查询固件版本
  !AT+NAME[xxx]    - 查询/设置设备名称
  !AT+BAUD[1-9]    - 查询/设置波特率
  !AT+PIN[xxxx]    - 查询/设置配对密码
  !AT+ROLE[0/1]    - 查询/设置角色 (0=从机, 1=主机)
  !AT+ADDR         - 查询 MAC 地址
  !AT+RESET        - 软复位
  !HEX xx xx xx    - 发送原始十六进制字节

内置命令:
  .scan            - 重新扫描 BLE 设备
  .info            - 显示当前连接信息
  .disconnect      - 断开连接
  .reconnect       - 重新连接
  .help            - 显示帮助
  .quit            - 退出程序
"""

import asyncio
import sys
import platform
import atexit
from typing import Optional

try:
    from bleak import BleakScanner, BleakClient
    from bleak.exc import BleakError
except ImportError:
    print("错误: 未安装 bleak 库，请运行: pip install bleak")
    sys.exit(1)

# JDY-16 BLE UUID
JDY16_CHAR_FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"

# 全局状态
g_client: Optional[BleakClient] = None
g_device_name: str = ""
g_device_addr: str = ""
g_running: bool = True
g_notify_queue: asyncio.Queue = asyncio.Queue()


# ══════════════════════════════════════════════════════════════════════════════
# 辅助函数
# ══════════════════════════════════════════════════════════════════════════════

def is_wsl() -> bool:
    try:
        with open("/proc/version", "r") as f:
            return "microsoft" in f.read().lower()
    except (FileNotFoundError, PermissionError):
        return False


def on_notify(sender_handle, data: bytearray):
    try:
        text = data.decode("utf-8", errors="replace")
        g_notify_queue.put_nowait(text)
    except Exception:
        g_notify_queue.put_nowait(repr(data))


def on_disconnect(client):
    global g_running
    print("\n[!] 蓝牙连接已断开")
    g_running = False


# ══════════════════════════════════════════════════════════════════════════════
# 设备扫描
# ══════════════════════════════════════════════════════════════════════════════

async def scan_devices(timeout: float = 8.0) -> list:
    print(f"正在扫描 BLE 设备 ({timeout}秒)...")
    print("提示: 请确保 JDY-16 模块已上电且未与其它设备连接\n")

    devices = []
    jdy_devices = []

    def detection_callback(device, advertisement_data):
        if device.address not in [d.address for d in devices]:
            devices.append(device)

    try:
        scanner = BleakScanner(detection_callback)
        await scanner.start()
        await asyncio.sleep(timeout)
        await scanner.stop()
    except BleakError as e:
        reason = str(e).lower()
        if "powered" in reason or "turn on" in reason:
            print("\n[X] 蓝牙未开启，请在 Windows 设置中打开蓝牙后重试。")
            print("    快捷操作: Win+A → 点击蓝牙图标开启")
        elif "not available" in reason or "not supported" in reason:
            print("\n[X] 此设备没有蓝牙适配器，或蓝牙驱动未正确安装。")
            print("    请检查设备管理器中是否有蓝牙适配器。")
        else:
            print(f"\n[X] 蓝牙错误: {e}")
        return []

    # 扫描完成后筛选 JDY 设备
    device_list = []
    for d in devices:
        name = d.name or "(未知)"
        rssi = getattr(d, 'rssi', 0) or 0
        name_lower = name.lower()
        if any(kw in name_lower for kw in ("jdy", "mlt", "ble")):
            jdy_devices.append((d, name, rssi))
        device_list.append((d, name, rssi))

    if jdy_devices:
        print(f"\n[√] 扫描完成，发现 {len(jdy_devices)} 个 JDY 兼容设备:")
        for d, name, rssi in jdy_devices:
            print(f"  [*] {name:<20} MAC: {d.address}")
        return jdy_devices

    print(f"\n扫描完成，共发现 {len(device_list)} 个 BLE 设备:")
    for i, (d, name, rssi) in enumerate(device_list):
        print(f"  [{i}] {name:<25} MAC: {d.address}")
    return device_list


async def select_device(devices: list):
    if not devices:
        print("\n未发现任何 BLE 设备！")
        print("请检查:")
        print("  1. JDY-16 模块是否已上电")
        print("  2. 模块是否处于广播状态 (LED 快闪)")
        print("  3. 蓝牙适配器是否正常工作")
        return None, None

    if len(devices) == 1:
        d, name, rssi = devices[0]
        print(f"\n自动选择唯一设备: {name} [{d.address}]")
        return d.address, name

    print(f"\n发现 {len(devices)} 个可选设备:")
    for i, (d, name, rssi) in enumerate(devices):
        print(f"  [{i}] {name:<25} RSSI: {rssi:>4} dBm  MAC: {d.address}")

    loop = asyncio.get_event_loop()
    while True:
        try:
            choice = (await loop.run_in_executor(
                None, lambda: input(f"\n请选择设备 [0-{len(devices) - 1}] (q 退出): ").strip()
            ))
            if choice.lower() == "q":
                return None, None
            idx = int(choice)
            if 0 <= idx < len(devices):
                d, name, rssi = devices[idx]
                return d.address, name
            print(f"输入无效，请输入 0-{len(devices) - 1}")
        except ValueError:
            print(f"输入无效，请输入数字或 'q'")
        except (EOFError, KeyboardInterrupt):
            return None, None


# ══════════════════════════════════════════════════════════════════════════════
# 连接管理
# ══════════════════════════════════════════════════════════════════════════════

async def connect_device(address: str) -> Optional[BleakClient]:
    global g_client

    print(f"\n正在连接 {address} ...")
    client = BleakClient(address, disconnected_callback=on_disconnect)

    try:
        await client.connect(timeout=15.0)
        if client.is_connected:
            print(f"[√] 连接成功! (MCU 桥接模式)")

            # 发现服务和特征值
            target_char = JDY16_CHAR_FFE1
            found = False
            for service in client.services:
                for char in service.characteristics:
                    if char.uuid.lower().endswith("ffe1"):
                        target_char = char.uuid
                        found = True
                        break
                if found:
                    break

            if found:
                print(f"[√] 通信特征值: {target_char}")
            else:
                print("[!] 未找到 FFE1 特征值，尝试第一个可写可通知的特征值")
                for service in client.services:
                    for char in service.characteristics:
                        if "write" in char.properties and "notify" in char.properties:
                            target_char = char.uuid
                            found = True
                            break
                    if found:
                        break

            if not found:
                print("[X] 无法找到可用的通信特征值")
                await client.disconnect()
                return None

            # 启用通知
            try:
                await client.start_notify(target_char, on_notify)
                print(f"[√] 已启用通知监听")
            except Exception as e:
                print(f"[!] 启用通知失败: {e}")

            g_client = client
            return client

    except asyncio.TimeoutError:
        print("[X] 连接超时")
        g_client = None
        return None
    except Exception as e:
        print(f"[X] 连接失败: {e}")
        g_client = None
        return None

    return None


async def disconnect_device():
    global g_client
    if g_client and g_client.is_connected:
        try:
            await g_client.disconnect()
        except Exception:
            pass
        g_client = None
        print("[√] 已断开连接")


# ══════════════════════════════════════════════════════════════════════════════
# 命令发送 (MCU 桥接协议: ! 前缀)
# ══════════════════════════════════════════════════════════════════════════════

async def send_bridge_command(cmd: str) -> None:
    """发送 MCU 桥接命令 (自动加 ! 前缀)"""
    if g_client is None or not g_client.is_connected:
        print("[!] 未连接到设备")
        return

    data = (cmd + "\r\n").encode("utf-8")
    try:
        await g_client.write_gatt_char(JDY16_CHAR_FFE1, data, response=False)
    except Exception as e:
        print(f"[X] 发送失败: {e}")


async def send_raw_hex(hex_str: str) -> None:
    """发送十六进制原始数据"""
    if g_client is None or not g_client.is_connected:
        print("[!] 未连接到设备")
        return

    try:
        hex_str = hex_str.replace(" ", "").replace("0x", "").replace(",", "")
        data = bytes.fromhex(hex_str)
        await g_client.write_gatt_char(JDY16_CHAR_FFE1, data, response=False)
        print(f"已发送 {len(data)} 字节: {' '.join(f'{b:02X}' for b in data)}")
    except ValueError as e:
        print(f"[X] HEX 格式错误: {e}")
    except Exception as e:
        print(f"[X] 发送失败: {e}")


# ══════════════════════════════════════════════════════════════════════════════
# 通知打印
# ══════════════════════════════════════════════════════════════════════════════

async def notification_printer():
    """后台任务：持续打印接收到的通知数据"""
    while True:
        data = await g_notify_queue.get()
        text = data.strip()
        if text:
            sys.stdout.write(f"\r← {text}\n> ")
            sys.stdout.flush()


# ══════════════════════════════════════════════════════════════════════════════
# 交互主循环
# ══════════════════════════════════════════════════════════════════════════════

def print_help():
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║              JDY-16 桥接命令行帮助 (MCU Relay)                        ║
╠══════════════════════════════════════════════════════════════════════╣
║  内置命令:                                                           ║
║    .scan        重新扫描 BLE 设备                                    ║
║    .info        显示当前连接信息                                      ║
║    .disconnect  断开连接                                             ║
║    .reconnect   重新连接                                             ║
║    .help        显示此帮助                                           ║
║    .quit        退出程序                                             ║
╠══════════════════════════════════════════════════════════════════════╣
║  MCU 桥接指令集:                                                     ║
║    !AT<cmd>      转发 AT 指令给 JDY-16 (如 !AT+VERSION)              ║
║    !HEX xx xx    发送原始 HEX 字节                                    ║
║    !ENC <dur> <int> [save]  编码器定时采样                            ║
║         dur=持续时长(ms)  int=采样间隔(ms)  save=1 累积保存           ║
║         例: !ENC 5000 100    每100ms读一次，持续5秒                    ║
║         例: !ENC 3000 50 1   每50ms读一次，结束一次性发送全部数据     ║
╚══════════════════════════════════════════════════════════════════════╝
""")


def print_info():
    print(f"""
╔══════════════════════════════════════════════╗
║  连接信息 (MCU 桥接模式)                       ║
╠══════════════════════════════════════════════╣
║  设备名称:  {g_device_name:<32} ║
║  MAC 地址:  {g_device_addr:<32} ║
║  连接状态:  {'已连接' if (g_client and g_client.is_connected) else '未连接':<32} ║
║  协议模式:  MCU Relay (!AT / !HEX)           ║
╚══════════════════════════════════════════════╝
""")


async def interactive_loop():
    global g_running, g_client, g_device_name, g_device_addr

    printer_task = asyncio.create_task(notification_printer())

    print("\n" + "=" * 60)
    print("JDY-16 BLE 桥接命令行交互模式 (MCU Relay)")
    print("协议: !AT<cmd> 通过 MCU 转发给 JDY-16")
    print("输入 .help 查看帮助，输入 .quit 退出")
    print("=" * 60 + "\n")

    loop = asyncio.get_event_loop()

    while g_running:
        try:
            if g_client is None or not g_client.is_connected:
                print("[!] 设备已断开，输入 .scan 重新扫描或 .quit 退出")

            cmd = await loop.run_in_executor(
                None, lambda: input("> ").strip()
            )

            if not cmd:
                continue

            # ── 内置命令 ──
            if cmd.startswith("."):
                parts = cmd.split(maxsplit=1)
                dot_cmd = parts[0].lower()
                arg = parts[1] if len(parts) > 1 else ""

                if dot_cmd in (".quit", ".exit"):
                    print("正在退出...")
                    g_running = False
                    break

                elif dot_cmd == ".help":
                    print_help()

                elif dot_cmd == ".info":
                    print_info()

                elif dot_cmd == ".scan":
                    print("正在扫描设备...")
                    devices = await scan_devices()
                    addr, name = await select_device(devices)
                    if addr:
                        await disconnect_device()
                        g_device_name = name
                        g_device_addr = addr
                        await connect_device(addr)
                    else:
                        print("已取消扫描")

                elif dot_cmd == ".disconnect":
                    await disconnect_device()

                elif dot_cmd == ".reconnect":
                    if g_device_addr:
                        await disconnect_device()
                        await connect_device(g_device_addr)
                    else:
                        print("[!] 没有可用的设备地址，请先使用 .scan")

                else:
                    print(f"未知命令: {dot_cmd}，输入 .help 查看帮助")

            # ── 桥接协议命令 ──
            else:
                if g_client is None or not g_client.is_connected:
                    print("[!] 未连接，请先使用 .scan 连接设备")
                    continue

                upper_cmd = cmd.upper()

                # 已经带 ! 前缀 → 直接发送
                if cmd.startswith("!"):
                    print(f"发送: {cmd}")
                    await send_bridge_command(cmd)

                # AT 开头 → 自动补 ! 前缀
                elif upper_cmd.startswith("AT"):
                    bridge_cmd = "!" + cmd
                    print(f"[?] 自动补全为: {bridge_cmd}")
                    await send_bridge_command(bridge_cmd)

                # 其他 → 加 ! 前缀
                else:
                    bridge_cmd = "!" + cmd
                    print(f"发送: {bridge_cmd}")
                    await send_bridge_command(bridge_cmd)

        except (EOFError, KeyboardInterrupt):
            print("\n正在退出...")
            g_running = False
            break
        except Exception as e:
            print(f"[X] 错误: {e}")

    printer_task.cancel()
    try:
        await printer_task
    except asyncio.CancelledError:
        pass


# ══════════════════════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════════════════════

async def main():
    global g_device_name, g_device_addr

    print("=" * 60)
    print("  JDY-16 BLE Bridge Tool v1.0 (MCU Relay)")
    print("  平台:", platform.system(), platform.release())
    print("=" * 60)

    if is_wsl():
        print("\n[!] 检测到 WSL 环境，BLE 蓝牙功能可能不可用。")
        print("    建议在 Windows 原生 Python 环境中运行本脚本。")
        resp = input("\n是否继续尝试? [y/N]: ").strip().lower()
        if resp != "y":
            return

    devices = await scan_devices()
    addr, name = await select_device(devices)

    if addr is None:
        print("未选择设备，退出。")
        return

    g_device_name = name
    g_device_addr = addr

    client = await connect_device(addr)
    if client is None or not client.is_connected:
        print("无法连接到设备，退出。")
        return

    try:
        await interactive_loop()
    finally:
        await disconnect_device()
        print("程序已退出。")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n用户中断，退出。")
    except BleakError as e:
        reason = str(e).lower()
        if "powered" in reason or "turn on" in reason:
            print("\n[X] 蓝牙未开启，请在 Windows 设置中打开蓝牙后重试。")
            print("    快捷操作: Win+A → 点击蓝牙图标开启")
        elif "not available" in reason or "not supported" in reason:
            print("\n[X] 此设备没有蓝牙适配器，或蓝牙驱动未正确安装。")
        else:
            print(f"\n程序错误: {e}")
    except Exception as e:
        print(f"\n程序异常: {e}")
