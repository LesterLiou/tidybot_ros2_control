import os
os.environ['CTR_TARGET'] = 'Hardware'

import time
import numpy as np
from phoenix6 import hardware
from constants import CAN_BUS_NAME

def check_system():
    print("==================================================")
    print("       TidyBot 2 硬體效能與通訊即時性檢測探針       ")
    print("==================================================")
    
    print("\n[1/2] 正在進行強效阻斷通訊監聽 (等待四輪全部就緒)...")
    
    caster_ids = [1, 3, 5, 7]
    all_ready = False
    
    # 最多等待 20 次 (每次 0.5 秒，共 10 秒)
    for attempt in range(1, 21):
        # 每次都重新建立四顆馬達的 handle，確保避開首發死鎖
        try:
            motors = [hardware.TalonFX(i, CAN_BUS_NAME) for i in caster_ids]
            time.sleep(0.3)  # 給硬體微小的讀取時間
            
            voltages = []
            for m in motors:
                v_sig = m.get_supply_voltage()
                v_sig.refresh()
                voltages.append(float(v_sig.value))
            
            # 檢查是否「所有馬達」的電壓都大於 11.0V (代表通訊完全打通，且電池有電)
            if all(v > 11.0 for v in voltages):
                print(f"\n🎉 🎉 🎉 【通訊完全打通！】在第 {attempt*0.5:.1f} 秒時四輪全部安全連線：")
                for idx, v in enumerate(voltages, start=1):
                    print(f"  ⚡ Caster {idx} (Motor ID: {2*idx-1}): 🟢 連線正常 | 實體電壓: {v:.2f} V")
                all_ready = True
                break
            
            # 如果還沒全部就緒，清理掉舊物件，等下一輪重構
            del motors
        except Exception:
            pass
        
        if attempt % 4 == 0:
            print(f"  -> 正在等待 Phoenix 背景網路與驅動完全就緒... ({attempt*0.5:.1f}s)")
        time.sleep(0.5)
        
    if not all_ready:
        print("\n  ❌ 警告：逾時 10 秒仍無法完全讀取四輪電壓！")
        print("  請確認：1. 電池開關已開啟 2. 馬達是否閃爍橘燈 3. 外層 candump can0 有資料。")

    # 2. 測試新 NUC 的定時控制迴圈即時性
    print("\n[2/2] 正在進行 4ms 控制迴圈即時性壓力測試 (測試 200 次)...")
    TARGET_PERIOD = 0.004  
    latencies = []
    
    last_time = time.time()
    for _ in range(200):
        while time.time() - last_time < TARGET_PERIOD:
            time.sleep(0.0001)
        
        current_time = time.time()
        actual_step = current_time - last_time
        last_time = current_time
        
        error_ms = abs(actual_step - TARGET_PERIOD) * 1000
        latencies.append(error_ms)
        
    max_error = np.max(latencies)
    avg_error = np.mean(latencies)
    
    print(f"  ⏱️  平均時間誤差 (Avg Jitter): {avg_error:.3f} ms")
    print(f"  🔥 最大延遲抖動 (Max Jitter): {max_error:.3f} ms")
    
    if max_error > 2.0:
        print("  ⚠️  警告：偵測到迴圈時間抖動較大，請確認是否有其他高負載程式在搶 CPU。")
    else:
        print("  🟢 優良：全新 NUC 效能強勁，時序控制極度精準，適合跑高頻率 RL 策略！")
    print("==================================================")

if __name__ == "__main__":
    check_system()