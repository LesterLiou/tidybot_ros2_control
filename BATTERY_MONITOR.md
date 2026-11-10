# Emirai 電池畫面監控

這個服務使用另一顆一般 UVC USB 相機（例如 JinPei）拍攝手機上的 Emirai
藍牙電池畫面。它會定時用 Tesseract OCR 讀取 `0–100%` 的數字、將可信讀值寫入
SQLite，並提供網頁趨勢圖。它會自動排除名稱含 `RealSense` 的相機，不會占用 D415。

> 這是從手機畫面取得的監看資料，不是馬達控制器的安全電壓訊號。請勿拿它取代
> `base_controller.py` 的實體低電壓保護或當作充電安全聯鎖。

## 啟動

先插入 JinPei 相機，再重新建置並啟動獨立服務：

```bash
cd /home/hrc/jerry/tidybot_ros2_control
docker compose build
docker compose up -d battery-monitor
docker compose logs -f battery-monitor
```

看到 `Battery camera connected` 後，用 NUC 本機瀏覽器開啟：

```text
http://localhost:8080
```

同一個可信任區域網路中的電腦或手機可開啟
`http://<NUC-IP>:8080`。這個輕量 UI 沒有登入驗證，不應直接暴露到網際網路。

如果相機尚未插入，服務不會退出，而是每三秒重新尋找。預設 `camera: auto`
會選第一個能開啟且不是 RealSense 的 `/dev/v4l/by-id/...-video-index0` 裝置。

## 第一次校正

1. 固定相機與手機，避免之後相對位置改變。
2. 讓手機保持充電或關閉自動休眠，並把亮度固定在不會過曝的位置。
3. 在網頁相機畫面用滑鼠或手指，只框住電量數字；盡量不要包含 `%` 符號或其他數字。
4. 設定監測間隔（預設 60 秒）並按「儲存設定」。
5. 按「立即辨識」，確認裁切預覽、OCR 數字與手機畫面一致。

OCR 會比較灰階、Otsu、反相與自適應二值化四種結果。只有合法的百分比且多個
版本一致，或單一結果高於最低信心時才寫入歷史；失敗畫面不會變成一筆錯誤電量。

## 資料與 ROS 2 topic

設定及資料存放在 Docker 的 `tidybot_ros_home` volume：

```text
/home/tidybot/.ros/tidybot/battery_monitor/config.json
/home/tidybot/.ros/tidybot/battery_monitor/battery.sqlite3
```

相機畫面只保留在記憶體供即時預覽與 OCR，不會定時保存照片。歷史可從網頁匯出
CSV，也不會出現在 Git commit 或 GitHub push 中。

每個成功讀值會發布：

- `/tidybot/battery` (`sensor_msgs/msg/BatteryState`)，`percentage` 為 `0.0–1.0`。
- `/tidybot/battery_percentage` (`std_msgs/msg/Float32`)，數值為 `0–100`。

可在另一個已 source ROS 2 Jazzy 的終端查看：

```bash
ros2 topic echo /tidybot/battery_percentage
```

停止及再次啟動：

```bash
docker compose stop battery-monitor
docker compose up -d battery-monitor
```

設定檔的預設值在 `config/battery_monitor.yaml`。網頁校正值會持久化並優先於 YAML
的取樣間隔與門檻；要重新框選，直接在畫面拖曳新的區域再儲存即可。
