# NUC / CAN 效能檢測說明

這份說明對應 [check_performance.py](check_performance.py)。目前腳本有兩種模式：

1. NUC-only 模式：只測 NUC，不碰 CAN / 馬達。
2. CAN / 馬達模式：嘗試讀四顆 caster 馬達的供電電壓，並接著做 4ms 迴圈抖動測試。

## 你要先知道的事

- 這份資料夾現在有 [Dockerfile](Dockerfile) 與 [docker-compose.yml](docker-compose.yml)，
  建立 Ubuntu 24.04 / ROS 2 Jazzy 控制環境；完整說明見 [DOCKER.md](DOCKER.md)。
- 如果你只是想單獨測 NUC，不需要接 CAN、馬達電源或電池。

## 1. NUC-only 測試

主機上原生執行的基本跑法：

```bash
cd /home/hrc/jerry/tidybot_ros2_control
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 check_performance.py --nuc-only
```

使用目前的 Jazzy 容器：

```bash
cd /home/hrc/jerry/tidybot_ros2_control
docker compose run --rm tidybot python3 \
  src/tidybot_ros2_control/check_performance.py --nuc-only
```

這會做三件事：

- 讀取 CPU 核心數與 load average
- 測試 4ms 控制迴圈抖動
- 預設不做 ping 測試

如果你想順便測 WiFi / 網路延遲，可以加上 `--ping-host`：

```bash
cd /home/hrc/jerry/tidybot_ros2_control
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 check_performance.py --nuc-only --ping-host 192.168.1.1
```

`--ping-host` 可以填：

- 路由器 IP，例如 `192.168.1.1`
- 另一台同網段主機
- 你想測的遠端設備 IP

如果想調整測試細節，也可以加：

- `--ping-count 20`：改 ping 次數
- `--loop-period 0.004`：改迴圈目標週期，預設是 4ms
- `--loop-samples 200`：改抖動採樣次數

例如：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 check_performance.py --nuc-only --ping-host 192.168.1.1 --ping-count 20 --loop-samples 500
```

## 2. CAN / 馬達模式

如果你有接 CAN、馬達電源和電池，直接執行：

```bash
cd /home/hrc/jerry/tidybot_ros2_control
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 check_performance.py
```

這個模式會：

- 嘗試讀四顆 caster 馬達的 supply voltage
- 確認 CAN 通訊是否正常
- 再做 4ms 迴圈抖動測試

如果 `phoenix6` 沒裝，腳本會提示你改用 `--nuc-only`。

## 3. 結果怎麼看

NUC-only 模式主要看這幾個輸出：

- `load average`：長時間偏高通常表示背景工作很多
- `Avg Jitter`：平均抖動，越小越好
- `P95 Jitter`：大部分時候的抖動上限
- `Max Jitter`：最差一次的抖動

如果 `Max Jitter` 明顯偏大，常見原因是：

- 背景程式在吃 CPU
- 電源模式不是 performance
- 有其他重負載任務同時跑

## 4. Docker 現況

Compose 會從本 repo 的 Dockerfile 建立 `tidybot_ros2_control:jazzy-noble`。
它不使用 `privileged`、host PID/IPC 或完整 `/dev` 掛載；SocketCAN 使用 host network，
並只加入 CAN 傳輸與 250 Hz 排程需要的 capabilities。建置、硬體可見性檢查與啟動方式
請依 [DOCKER.md](DOCKER.md)。
