import argparse
import importlib
import os
import statistics
import subprocess
import time

os.environ['CTR_TARGET'] = 'Hardware'


def measure_loop_jitter(target_period: float, samples: int) -> tuple[list[float], float]:
    latencies = []
    last_time = time.monotonic()

    for _ in range(samples):
        while time.monotonic() - last_time < target_period:
            time.sleep(0.0001)

        current_time = time.monotonic()
        actual_step = current_time - last_time
        last_time = current_time
        latencies.append(abs(actual_step - target_period) * 1000)

    return latencies, target_period * 1000


def print_nuc_summary(ping_host: str | None, ping_count: int, target_period: float, samples: int):
    print("==================================================")
    print("            TidyBot 2 NUC 效能檢測探針            ")
    print("==================================================")

    if ping_host:
        print(f"\n[1/3] 測試 WiFi / 網路延遲：ping {ping_host} ({ping_count} 次)...")
        ping_cmd = ['ping', '-c', str(ping_count), '-W', '1', ping_host]
        result = subprocess.run(ping_cmd, capture_output=True, text=True)
        if result.stdout.strip():
            print(result.stdout.rstrip())
        if result.returncode != 0 and result.stderr.strip():
            print(result.stderr.rstrip())
        if result.returncode != 0:
            print("  ⚠️  ping 測試失敗，請確認 WiFi 連線與目標主機是否可達。")
    else:
        print("\n[1/3] 已略過 WiFi / 網路延遲測試（未提供 --ping-host）")

    print("\n[2/3] 讀取 NUC CPU / 負載資訊...")
    cpu_count = os.cpu_count() or 1
    load1, load5, load15 = os.getloadavg()
    print(f"  CPU 核心數: {cpu_count}")
    print(f"  1/5/15 分鐘 load average: {load1:.2f}, {load5:.2f}, {load15:.2f}")
    print(f"  目前流程 ID: {os.getpid()}")

    print(f"\n[3/3] 進行 {target_period * 1000:.1f}ms 控制迴圈抖動測試 ({samples} 次)...")
    latencies, target_period_ms = measure_loop_jitter(target_period, samples)
    max_error = max(latencies)
    avg_error = statistics.fmean(latencies)
    p95_error = statistics.quantiles(latencies, n=20)[18] if len(latencies) >= 20 else max_error

    print(f"  目標週期: {target_period_ms:.3f} ms")
    print(f"  ⏱️  平均時間誤差 (Avg Jitter): {avg_error:.3f} ms")
    print(f"  📈 95% 抖動 (P95 Jitter): {p95_error:.3f} ms")
    print(f"  🔥 最大延遲抖動 (Max Jitter): {max_error:.3f} ms")

    if max_error > 2.0:
        print("  ⚠️  警告：迴圈時間抖動偏大，請檢查背景工作、CPU 排程與電源模式。")
    else:
        print("  🟢 看起來 NUC 的控制迴圈表現正常。")
    print("==================================================")


def check_can_motors():
    from constants import CAN_BUS_NAME

    try:
        hardware = importlib.import_module('phoenix6.hardware')
    except ModuleNotFoundError:
        print('缺少 phoenix6 套件，無法執行 CAN / 馬達檢查。')
        print('如果你只想測 NUC，請改用 --nuc-only。')
        return

    print("==================================================")
    print("       TidyBot 2 硬體效能與通訊即時性檢測探針       ")
    print("==================================================")

    print("\n[1/2] 正在進行強效阻斷通訊監聽 (等待四輪全部就緒)...")

    caster_ids = [1, 3, 5, 7]
    all_ready = False

    for attempt in range(1, 21):
        try:
            motors = [hardware.TalonFX(i, CAN_BUS_NAME) for i in caster_ids]
            time.sleep(0.3)

            voltages = []
            for motor in motors:
                v_sig = motor.get_supply_voltage()
                v_sig.refresh()
                voltages.append(float(v_sig.value))

            if all(v > 11.0 for v in voltages):
                print(f"\n🎉 🎉 🎉 【通訊完全打通！】在第 {attempt * 0.5:.1f} 秒時四輪全部安全連線：")
                for idx, v in enumerate(voltages, start=1):
                    print(f"  ⚡ Caster {idx} (Motor ID: {2 * idx - 1}): 🟢 連線正常 | 實體電壓: {v:.2f} V")
                all_ready = True
                break

            del motors
        except Exception:
            pass

        if attempt % 4 == 0:
            print(f"  -> 正在等待 Phoenix 背景網路與驅動完全就緒... ({attempt * 0.5:.1f}s)")
        time.sleep(0.5)

    if not all_ready:
        print("\n  ❌ 警告：逾時 10 秒仍無法完全讀取四輪電壓！")
        print("  請確認：1. 電池開關已開啟 2. 馬達是否閃爍橘燈 3. 外層 candump can0 有資料。")

    print("\n[2/2] 正在進行 4ms 控制迴圈即時性壓力測試 (測試 200 次)...")
    latencies, _ = measure_loop_jitter(0.004, 200)
    max_error = max(latencies)
    avg_error = statistics.fmean(latencies)

    print(f"  ⏱️  平均時間誤差 (Avg Jitter): {avg_error:.3f} ms")
    print(f"  🔥 最大延遲抖動 (Max Jitter): {max_error:.3f} ms")

    if max_error > 2.0:
        print("  ⚠️  警告：偵測到迴圈時間抖動較大，請確認是否有其他高負載程式在搶 CPU。")
    else:
        print("  🟢 優良：全新 NUC 效能強勁，時序控制極度精準，適合跑高頻率 RL 策略！")
    print("==================================================")


def parse_args():
    parser = argparse.ArgumentParser(description='TidyBot 2 performance checker')
    parser.add_argument('--nuc-only', action='store_true', help='只測 NUC，不檢查 CAN / 馬達')
    parser.add_argument('--ping-host', help='要測試 WiFi / 網路延遲的主機位址，例如路由器或遠端主機')
    parser.add_argument('--ping-count', type=int, default=10, help='ping 次數，預設 10')
    parser.add_argument('--loop-period', type=float, default=0.004, help='控制迴圈目標週期（秒），預設 0.004')
    parser.add_argument('--loop-samples', type=int, default=200, help='控制迴圈測試次數，預設 200')
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.nuc_only:
        print_nuc_summary(args.ping_host, args.ping_count, args.loop_period, args.loop_samples)
    else:
        check_can_motors()