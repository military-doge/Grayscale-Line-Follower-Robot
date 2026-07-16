#!/usr/bin/env python3
"""
JDY-16 蓝牙模块直接连接脚本
===========================
通过 BLE 直连 JDY-16 模块，提供命令行交互界面，支持 AT 指令及透传数据收发。

依赖: pip install bleak
运行: python jdy16_direct_connect.py

JDY-16 常用 AT 指令集:
  AT               - 测试连接，返回 OK
  AT+RESET         - 软复位
  AT+BAUD[param]   - 查询/设置波特率 (1=1200,2=2400,3=4800,4=9600,5=19200,6=38400,7=57600,8=115200,9=230400)
  AT+NAME[param]   - 查询/设置设备名称
  AT+PIN[param]    - 查询/设置配对密码 (默认 1234)
  AT+ROLE[param]   - 查询/设置角色 (0=从机, 1=主机)
  AT+ADDR          - 查询 MAC 地址
  AT+VERSION       - 查询固件版本
  AT+BAUD          - 查询当前波特率
  AT+DISC           - 断开当前连接
  AT+SLEEP[param]  - 设置睡眠模式 (0=关闭, 1=开启)
  AT+ADV[param]    - 设置广播 (0=关闭, 1=开启)

特殊命令:
  .scan            - 重新扫描设备
  .devices         - 列出已发现设备
  .info            - 显示连接信息
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

# ── Bleak 导入 ──────────────────────────────────────────────────────────────
try:
    from bleak import BleakScanner, BleakClient
    from bleak.exc import BleakError
except ImportError:
    print("错误: 未安装 bleak 库，请运行: pip install bleak")
    sys.exit(1)

# ── JDY-16 BLE UUID 定义 ────────────────────────────────────────────────────
JDY16_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
JDY16_CHAR_FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"  # 透传数据通道
JDY16_CHAR_FFE2 = "0000ffe2-0000-1000-8000-00805f9b34fb"  # AT 命令通道
JDY16_CHAR_FFE3 = "0000ffe3-0000-1000-8000-00805f9b34fb"  # 配置通道

# 默认 TX/RX 特征值 (可运行时通过 .ffe1/.ffe2/.ffe3 切换)
g_tx_char_uuid: str = JDY16_CHAR_FFE1

# ── 全局状态 ────────────────────────────────────────────────────────────────
g_client: Optional[BleakClient] = None
g_device_name: str = ""
g_device_addr: str = ""
g_running: bool = True
g_notify_queue: asyncio.Queue = asyncio.Queue()


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                            辅助函数                                         ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def is_wsl() -> bool:
    """检测是否在 WSL 环境中运行"""
    try:
        with open("/proc/version", "r") as f:
            return "microsoft" in f.read().lower()
    except (FileNotFoundError, PermissionError):
        return False


def on_notify(sender_handle, data: bytearray):
    """BLE 通知回调：将接收数据放入队列"""
    try:
        text = data.decode("utf-8", errors="replace")
        g_notify_queue.put_nowait(text)
    except Exception:
        g_notify_queue.put_nowait(repr(data))


def on_disconnect(client):
    """断开连接回调"""
    global g_running
    print("\n[!] 蓝牙连接已断开")
    g_running = False


def format_bytes(data: bytes) -> str:
    """将字节数据格式化为可读的十六进制 + ASCII"""
    hex_part = " ".join(f"{b:02X}" for b in data)
    ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
    return f"HEX: {hex_part}\nASCII: {ascii_part}"


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                          设备扫描                                           ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

async def scan_devices(timeout: float = 8.0) -> list:
    """扫描 BLE 设备，筛选 JDY-16/JDY 相关设备，返回列表"""
    print(f"正在扫描 BLE 设备 ({timeout}秒)...")
    print("提示: 请确保 JDY-16 模块已上电且未与其它设备连接\n")

    devices = []
    jdy_devices = []

    def detection_callback(device, advertisement_data):
        if device.address not in [d.address for d in devices]:
            devices.append(device)
            name = device.name or advertisement_data.local_name or "(未知)"
            rssi = advertisement_data.rssi

            # 筛选 JDY 相关设备
            name_lower = name.lower()
            addr_lower = device.address.lower()
            if any(kw in name_lower or kw in addr_lower for kw in ("jdy", "mlt", "bt", "ble")):
                jdy_devices.append((device, name, rssi))
                print(f"  [*] 发现疑似 JDY 设备: {name:<20} RSSI: {rssi:>4} dBm  MAC: {device.address}")

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

    # 如果没有筛选到 JDY 设备，列出所有设备供手动选择
    if not jdy_devices:
        print(f"\n未自动识别到 JDY 设备，共发现 {len(devices)} 个 BLE 设备:")
        for i, d in enumerate(devices):
            name = d.name or "(未知)"
            print(f"  [{i}] {name:<25} MAC: {d.address}")
        return [(d, d.name or "(未知)", 0) for d in devices]

    return jdy_devices


async def select_device(devices: list):
    """让用户选择一个设备，返回 (address, name)"""
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

    while True:
        try:
            choice = input(f"\n请选择设备 [0-{len(devices) - 1}] (q 退出): ").strip()
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


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                          连接管理                                           ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

async def connect_device(address: str) -> Optional[BleakClient]:
    """连接到指定 BLE 设备，返回 BleakClient"""
    global g_client

    print(f"\n正在连接 {address} ...")
    client = BleakClient(address, disconnected_callback=on_disconnect)

    try:
        await client.connect(timeout=15.0)
        if client.is_connected:
            print(f"[√] 连接成功!")

            # 发现服务和特征值
            print("\n服务和特征值:")
            for service in client.services:
                print(f"  Service: {service.uuid}")
                for char in service.characteristics:
                    props = char.properties
                    prop_strs = []
                    if "read" in props:
                        prop_strs.append("R")
                    if "write" in props:
                        prop_strs.append("W")
                    if "write-without-response" in props:
                        prop_strs.append("W!")
                    if "notify" in props:
                        prop_strs.append("N")
                    if "indicate" in props:
                        prop_strs.append("I")
                    print(f"    Char: {char.uuid}  [{','.join(prop_strs)}]")

            # 根据发现的 FFE1 特征值确认路径
            target_char = None
            for service in client.services:
                for char in service.characteristics:
                    cuuid = char.uuid.lower()
                    # JDY-16 默认使用 FFE1
                    if cuuid.endswith("ffe1") or cuuid == JDY16_CHAR_FFE1.lower():
                        target_char = char.uuid
                        break
                if target_char:
                    break

            if target_char:
                print(f"\n[√] 找到通信特征值: {target_char}")
            else:
                print(f"\n[!] 未找到标准 JDY-16 通信特征值 FFE1")
                print("    尝试列出所有可写+可通知的特征值供手动尝试...")
                for service in client.services:
                    for char in service.characteristics:
                        if "write" in char.properties and "notify" in char.properties:
                            print(f"    候选: {char.uuid} (Service: {service.uuid})")
                            if target_char is None:
                                target_char = char.uuid
                            break
                    if target_char:
                        break

            if target_char is None:
                print("[X] 无法找到可用的通信特征值，连接可能无法工作")

            # 启用通知 — 同时监听 FFE1 和 FFE2
            notify_chars = [target_char] if target_char else []
            for service in client.services:
                for char in service.characteristics:
                    cuuid = char.uuid.lower()
                    if cuuid.endswith("ffe2") and char.uuid not in notify_chars:
                        notify_chars.append(char.uuid)
                    if cuuid.endswith("ffe3") and char.uuid not in notify_chars:
                        notify_chars.append(char.uuid)

            for nc in notify_chars:
                try:
                    await client.start_notify(nc, on_notify)
                    print(f"[√] 已启用通知监听 (特征值: {nc})")
                except Exception as e:
                    print(f"[!] 启用通知 {nc} 失败: {e}")

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
    """断开当前连接"""
    global g_client
    if g_client and g_client.is_connected:
        try:
            await g_client.disconnect()
        except Exception:
            pass
        g_client = None
        print("[√] 已断开连接")


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                          命令发送                                           ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

async def send_command(cmd: str) -> None:
    """通过 BLE 发送 AT 命令 (追加 \r\n)"""
    if g_client is None or not g_client.is_connected:
        print("[!] 未连接到设备")
        return

    data = (cmd + "\r\n").encode("utf-8")
    try:
        await g_client.write_gatt_char(g_tx_char_uuid, data, response=False)
    except Exception as e:
        print(f"[X] 发送失败: {e}")


async def send_raw(data: bytes) -> None:
    """发送原始字节（不追加任何后缀）"""
    if g_client is None or not g_client.is_connected:
        print("[!] 未连接到设备")
        return
    try:
        await g_client.write_gatt_char(g_tx_char_uuid, data, response=False)
    except Exception as e:
        print(f"[X] 发送失败: {e}")
        await g_client.write_gatt_char(char_uuid, data, response=False)
    except Exception as e:
        print(f"[X] 发送失败: {e}")


async def send_raw_hex(hex_str: str) -> None:
    """发送十六进制原始数据 (如 "41 54 0D 0A")"""
    if g_client is None or not g_client.is_connected:
        print("[!] 未连接到设备")
        return

    try:
        hex_str = hex_str.replace(" ", "").replace("0x", "").replace(",", "")
        data = bytes.fromhex(hex_str)
        await g_client.write_gatt_char(g_tx_char_uuid, data, response=False)
        print(f"已发送 {len(data)} 字节: {format_bytes(data)}")
    except ValueError as e:
        print(f"[X] HEX 格式错误: {e}")
    except Exception as e:
        print(f"[X] 发送失败: {e}")


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                          接收显示                                           ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

async def read_response(timeout: float = 2.0) -> Optional[str]:
    """非阻塞读取响应队列"""
    try:
        return await asyncio.wait_for(g_notify_queue.get(), timeout=timeout)
    except asyncio.TimeoutError:
        return None


async def notification_printer():
    """后台任务：持续打印接收到的通知数据"""
    while True:
        data = await g_notify_queue.get()
        text = data.strip()
        if text:
            # 清除当前输入行，打印接收内容，再恢复输入提示
            sys.stdout.write(f"\r← {text}\n> ")
            sys.stdout.flush()


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                          交互主循环                                         ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def print_help():
    """打印帮助信息"""
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║                    JDY-16 命令行交互帮助                              ║
╠══════════════════════════════════════════════════════════════════════╣
║  内置命令:                                                           ║
║    .scan        重新扫描 BLE 设备                                    ║
║    .ffe1        切换到 FFE1 通道发送 (透传数据)                       ║
║    .ffe2        切换到 FFE2 通道发送 (AT 命令)                        ║
║    .ffe3        切换到 FFE3 通道发送                                  ║
║    .info        显示当前连接信息和通道                                 ║
║    .disconnect  断开连接                                             ║
║    .reconnect   重新连接                                             ║
║    .hex <HEX>   发送十六进制原始数据 (如 .hex 41 54 0D 0A)            ║
║    .help        显示此帮助                                           ║
║    .quit        退出程序                                             ║
╠══════════════════════════════════════════════════════════════════════╣
║  AT 指令 (直接输入即可):                                             ║
║    AT           测试连接，应返回 OK                                   ║
║    AT+RESET     软复位模块                                           ║
║    AT+BAUD      查询当前波特率                                        ║
║    AT+BAUD4     设置波特率为 9600                                     ║
║    AT+NAME      查询设备名称                                          ║
║    AT+NAMEJDY16 设置设备名称为 JDY16                                  ║
║    AT+PIN       查询配对密码                                          ║
║    AT+PIN0000   设置配对密码为 0000                                   ║
║    AT+ROLE      查询角色 (0=从机, 1=主机)                             ║
║    AT+ADDR      查询 MAC 地址                                         ║
║    AT+VERSION   查询固件版本                                          ║
║    AT+DISC      断开模块当前的 BLE 连接                               ║
║    AT+SLEEP     查询睡眠模式                                           ║
║    AT+ADV       查询广播状态                                           ║
╚══════════════════════════════════════════════════════════════════════╝
""")

def print_info():
    """打印当前连接信息"""
    print(f"""
╔══════════════════════════════════════════════╗
║  连接信息                                     ║
╠══════════════════════════════════════════════╣
║  设备名称:  {g_device_name:<32} ║
║  MAC 地址:  {g_device_addr:<32} ║
║  连接状态:  {'已连接' if (g_client and g_client.is_connected) else '未连接':<32} ║
║  TX 通道:   {g_tx_char_uuid:<32} ║
╚══════════════════════════════════════════════╝
""")


async def interactive_loop():
    """交互式命令行主循环"""
    global g_running, g_client, g_device_name, g_device_addr, g_tx_char_uuid

    # 启动通知打印任务
    printer_task = asyncio.create_task(notification_printer())

    print("\n" + "=" * 60)
    print("JDY-16 BLE 命令行交互模式")
    print("输入 AT 命令与模块通信，输入 .help 查看帮助，输入 .quit 退出")
    print("=" * 60 + "\n")

    while g_running:
        try:
            # 检查连接状态
            if g_client is None or not g_client.is_connected:
                print("[!] 设备已断开，输入 .scan 重新扫描或 .quit 退出")

            cmd = await asyncio.get_event_loop().run_in_executor(
                None, lambda: input("> ").strip()
            )

            if not cmd:
                continue

            # ── 内置命令处理 ──
            if cmd.startswith("."):
                parts = cmd.split(maxsplit=1)
                dot_cmd = parts[0].lower()
                arg = parts[1] if len(parts) > 1 else ""

                if dot_cmd == ".quit" or dot_cmd == ".exit":
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

                elif dot_cmd == ".devices":
                    print("重新扫描以发现当前可用设备: 使用 .scan")

                elif dot_cmd == ".disconnect":
                    await disconnect_device()

                elif dot_cmd == ".reconnect":
                    if g_device_addr:
                        await disconnect_device()
                        await connect_device(g_device_addr)
                    else:
                        print("[!] 没有可用的设备地址，请先使用 .scan")

                elif dot_cmd == ".hex":
                    if arg:
                        await send_raw_hex(arg)
                    else:
                        print("用法: .hex <HEX>  例如: .hex 41 54 0D 0A")

                elif dot_cmd == ".ffe1":
                    g_tx_char_uuid = JDY16_CHAR_FFE1
                    print(f"[√] TX 通道已切换到: FFE1 (透传数据)")

                elif dot_cmd == ".ffe2":
                    g_tx_char_uuid = JDY16_CHAR_FFE2
                    print(f"[√] TX 通道已切换到: FFE2 (AT 命令)")

                elif dot_cmd == ".ffe3":
                    g_tx_char_uuid = JDY16_CHAR_FFE3
                    print(f"[√] TX 通道已切换到: FFE3")

                else:
                    print(f"未知命令: {dot_cmd}，输入 .help 查看帮助")

            # ── AT 命令处理 ──
            else:
                if g_client is None or not g_client.is_connected:
                    print("[!] 未连接，无法发送 AT 命令。请先使用 .scan 连接设备")
                    continue

                upper_cmd = cmd.upper()

                # +++ 特殊透传退出指令 —— 不加 AT 前缀，不加 \r\n，前后各等 1s
                if cmd == "+++":
                    print("发送: +++ (进入 AT 命令模式)")
                    await asyncio.sleep(1)
                    await send_raw(b"+++")
                    await asyncio.sleep(1)
                elif upper_cmd.startswith("AT"):
                    at_cmd = cmd
                    print(f"发送: {at_cmd}")
                    await send_command(at_cmd)
                else:
                    # 自动补充 AT 前缀
                    at_cmd = "AT+" + cmd
                    print(f"[?] 命令 '{cmd}' 缺少 AT 前缀，自动补全为 '{at_cmd}'")
                    print(f"发送: {at_cmd}")
                    await send_command(at_cmd)

        except (EOFError, KeyboardInterrupt):
            print("\n正在退出...")
            g_running = False
            break
        except Exception as e:
            print(f"[X] 错误: {e}")

    # 清理
    printer_task.cancel()
    try:
        await printer_task
    except asyncio.CancelledError:
        pass


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                            主入口                                           ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

async def main():
    global g_device_name, g_device_addr

    print("=" * 60)
    print("  JDY-16 BLE Direct Connection Tool v1.0")
    print("  平台:", platform.system(), platform.release())
    print("=" * 60)

    # WSL 警告
    if is_wsl():
        print("\n[!] 检测到 WSL 环境，BLE 蓝牙功能可能不可用。")
        print("    建议在 Windows 原生 Python 环境中运行本脚本。")
        print("    或使用 USB 蓝牙适配器并通过 usbipd 映射到 WSL。")
        resp = input("\n是否继续尝试? [y/N]: ").strip().lower()
        if resp != "y":
            return

    # 扫描设备
    devices = await scan_devices()
    addr, name = await select_device(devices)

    if addr is None:
        print("未选择设备，退出。")
        return

    g_device_name = name
    g_device_addr = addr

    # 连接
    client = await connect_device(addr)
    if client is None or not client.is_connected:
        print("无法连接到设备，退出。")
        return

    # 进入交互
    try:
        await interactive_loop()
    finally:
        await disconnect_device()
        print("程序已退出。")


def cleanup():
    """退出时的清理工作"""
    pass


atexit.register(cleanup)


if __name__ == "__main__":
    # 信号处理 (Linux)
    if platform.system() != "Windows":
        import signal as _signal
        try:
            loop = asyncio.get_event_loop()
            for sig in (_signal.SIGINT, _signal.SIGTERM):
                loop.add_signal_handler(sig, lambda: None)
        except (NotImplementedError, RuntimeError):
            pass

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
