# D415 / ArUco 視覺對接（ROS 2 Humble）

第一階段：將車放在充電樁前方、相機可見標記的位置，收到 ROS 指令後對準，
低速進入示教位置並停止。`REACHED` 表示視覺估計到達目標，尚未確認機械卡合或充電。
目前沒有接觸感測器，受阻只回報「疑似受阻」，由手把接管；不會盲目往兩側搜尋。

## 安裝與環境檢查

在**實際執行 ROS 的主機或容器內**執行。Humble 使用系統 Python 3.10：

```bash
source /opt/ros/humble/setup.bash
cd /home/hrc/jerry/tidybot_ros2_control
/usr/bin/python3 check_docking_environment.py
bash scripts/install_docking_deps.sh
/usr/bin/python3 check_docking_environment.py --probe-camera
rs-enumerate-devices
```

安裝腳本需要 sudo 密碼。ROS 驅動會透過 apt 安裝其 SDK 依賴，不要另外用 pip
覆蓋系統 OpenCV 或 NumPy。`pyrealsense2` 僅供額外的 USB 檢查，ROS 驅動本身不依賴這個 Python 模組。
在容器中還需要 USB 裝置存取；找不到 `/dev/video*`、libusb/udev 初始化失敗，
通常要檢查目前執行環境是否看得到主機裝置。

目前環境的 ROS 軟體依賴已檢查通過，包括 `realsense2_camera` 4.58.4、
`realsense2_description` 和 ROS 版 SDK 2.58.4；OpenCV 為 4.5.4（含 ArUco）。
另有系統版 SDK 2.56.5，ROS 驅動會使用自己的 SDK 依賴。
重新連接後已列舉到 D415，韌體 5.17.0.9，目前連線模式是 USB 2.1（480 Mbps）。
此模式提供 640×480@30、1280×720@15，但沒有 1280×720@30；
請依下面的 `color_profile` 參數指定支援的模式。尚未進行底盤移動測試。

若安裝結尾出現 `nvidia-dkms-580-open` 在 `6.8.2-rt11` 建置失敗，
那是現有 NVIDIA 套件的設定錯誤；請分別檢查 RealSense 套件是否顯示 `ii` 和環境檢查是否通過。
安裝腳本在對接依賴都已裝好時會略過 apt；若依賴仍有缺漏但 dpkg 有未完成的套件，
會先列出問題並退出，避免混淆不同的安裝錯誤。

建置與載入（使用 `/usr/bin/colcon`，避免 Conda Python）：

```bash
cd /home/hrc/jerry
source /opt/ros/humble/setup.bash
/usr/bin/colcon build --symlink-install --base-paths tidybot_ros2_control \
  --packages-select tidybot_ros2_control
source install/setup.bash
```

## 標記與相機設定

現成標記：`markers/aruco_4x4_50_id0_120mm.pdf`。以 **100%／實際大小**列印，
量測外側黑色正方形應為 **120 mm**，不包含白邊。固定在不反光、平整且不會移動的板面。
需要不同尺寸時可重新產生，並同步調整 `marker_size`：

```bash
ros2 run tidybot_ros2_control generate_docking_marker \
  --dictionary DICT_4X4_50 --id 0 --size 0.12 --output /tmp/dock_marker.pdf
```

編輯原始碼內的 `config/docking.yaml`，特別是：

- `camera_xyz`：**camera_link 原點相對 base_link** 的實際位置（公尺）。
  目前依提供的安裝位置設為 `[0.225, 0.0, 0.40]`：前方 22.5 cm、左右置中、離地 40 cm。
  此高度以 `base_link` 原點位於地面平面為前提；若使用其他高度的車身原點，須改填相對高度。
- `camera_rpy`：相機機身相對車身的 roll/pitch/yaw（弧度）。車身 X 向前、Y 向左、Z 向上。
  相機光學座標的轉換與 RGB 感測器偏移由 RealSense 驅動的 TF 提供。
  已確認鏡頭水平朝正前方，目前設為 `[0.0, 0.0, 0.0]`。
- `camera_mount_calibrated`：程式預設 `false` 允許辨識，禁止示教與開始。
  本機已依提供的安裝量測與朝向確認設為 `true`；相機安裝改變時，須重新量測、確認並示教。
- `publish_camera_tf`：若已有 URDF 提供 `base_link -> camera_link`，設為 `false`，避免重複 TF。
  此時仍要讓 `camera_xyz` / `camera_rpy` 與實際安裝一致，供目標設定檢查使用。
- `dictionary`、`marker_id`、`marker_size`：必須與實體標記一致。尺寸是黑色正方形邊長，單位公尺。
- `tolerance_x/y/yaw`：初始容許誤差分別為 10 mm、5 mm、約 1.43 度，需以卡榫實測修正。
  若視覺誤差大於卡榫容許範圍，要先改善安裝與估計，不要僅靠放寬容差判定成功。
- `max_vertical_error/far_max_vertical_error`：接近卡榫時維持 4 cm 的高度一致性檢查；
  距離增加時逐步放寬，最遠處最多 8 cm，以容納遠距離 ArUco 透視估計的高度雜訊。
  這只影響不可由底盤修正的高度檢查，不放寬最後前後、左右或角度到位條件。
- `max_forward/lateral/yaw_rate`：自動對接上限目前為 0.04 m/s、0.04 m/s、0.12 rad/s。
  實測底盤收到 0.015 m/s 的前進命令時，8 秒只前進約 3 mm；後續測試也顯示命令降至
  約 0.020–0.021 m/s 後容易停止。精細接近因此改用 0.025–0.028 m/s，直線插入固定使用 0.025 m/s。
  橫移超出容許誤差時至少輸出 0.025 m/s，以避開實機在 0.015 m/s 時沒有動作的區域。
  `FAR_HEADING` 原地轉正至少使用 0.10 rad/s；精細角度修正仍使用比例控制，不套用這個下限。
  若更改上限，同步檢查 mux 的 `docking_max_velocity`；mux 會再次限速。

相機離地約 40 cm 時，可先將標記中心放在接近鏡頭高度的位置，讓整段對接過程都能看到
完整黑框與白邊。標記可以偏離卡榫中心，但必須和卡榫保持固定關係。
相機或標記的安裝位置改變後要重新示教；程式會檢查已記錄的標記與安裝設定是否一致。
使用外部 TF 時，外部 TF 的物理改動也必須重新示教。

## 先用預覽模式確認影像

```bash
ros2 launch tidybot_ros2_control tidybot_docking.launch.py
```

預設 `dry_run:=true`、`start_base:=false`：不啟動底盤，最終速度與啟用訊號送到
`/docking/preview/cmd_vel` 和 `/docking/preview/tidybot/enable`。
RealSense 只開 RGB，預設使用本機 D415 在 USB 2 下已驗證的 640×480@30；
第一版不依賴近距離深度。若只測影像，可略過手把：

```bash
ros2 launch tidybot_ros2_control tidybot_docking.launch.py \
  color_profile:=640x480x30 start_gamepad:=false
```

確認接上 USB 3 且相機列出支援模式後，可指定 `color_profile:=1280x720x30`。

啟動後會自動開啟影像視窗，直接顯示 `/docking/debug_image`：有辨識到標記時會畫出框線。
不需要另外開終端機或手動選擇 topic；關閉視窗只會關閉顯示，對接節點會繼續執行。
若要直接看原始 RGB，啟動時加上 `image_topic:=/camera/camera/color/image_raw`。
在 SSH 或沒有桌面的環境執行時，加上 `show_image:=false`。

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /docking/marker_pose
ros2 topic echo /docking/status
```

安裝腳本包含影像工具 `ros-humble-rqt-image-view`。
同時留意 TF、CameraInfo 或標記品質的警告。辨識節點只發布當前有效觀測，
不會以舊位置假裝成新觀測。

若已有相機驅動，使用 `start_camera:=false`；若只測影像，可使用 `start_gamepad:=false`。
第一階段 mux 預設要求新鮮的手把訊號，缺少手把時會拒絕自動移動。
日後整合無人巡檢時再調整 `require_gamepad` 與整機停止機制。

## 自動檢查、首次建站與換環境

一般啟動會自動載入相機安裝設定，啟動相機、ArUco 辨識、影像視窗與控制節點，
並讀取 `~/.ros/tidybot/dock_target.yaml`。終端機會顯示目前檢查結果：

| 狀態 | 意義 |
|---|---|
| `WAITING_FOR_DETECTOR` | 等待辨識節點的設定 |
| `NEEDS_CAMERA_SETUP` | 相機安裝設定尚未確認 |
| `WAITING_FOR_MARKER` | 尚無新鮮標記姿態，檢查影像、標記可見性、CameraInfo 與 TF |
| `NEEDS_TEACH` | 尚無目標，或目標與相機／標記設定不相符 |
| `POSE_MISMATCH` | 與示教目標相比，高度或傾斜差異過大；訊息列出實測值與限制 |
| `WAITING_FOR_CONTROL` | 等待控制權管理節點 |
| `READY` | 安裝設定、標記與已儲存目標就緒，等待 `/docking/start` |

`READY` 不會觸發移動；回站指令仍會檢查距離、角度、手把與停止互鎖。
待命時也會檢查高度／傾斜，超標時顯示 `POSE_MISMATCH`，恢復正常觀測後可回到 `READY`，仍須明確下回站命令。
訊息中的 `height` 是由目前與目標姿態換算的相對高度，不是單純將兩個標記的 Z 值相減。
預設限制仍為高度 0.04 m、傾斜 0.15 rad（約 8.59 度）。視覺姿態估計波動也可能讓它超標；
第一次測試先保留既有示教目標，將車定位在目標前 20–30 cm，保持相機與標記的固定關係再檢查。
相機參數來自已確認的安裝設定與驅動提供的 CameraInfo，啟動時不會重新推算未知的安裝位置。
RealSense 子 launch 使用獨立的參數範圍，避免把 `dry_run` 等對接選項誤報為不支援的相機參數。

首次沒有停靠目標時，先將車放在希望記錄的停止點，第一次選卡榫入口前，
保持車、相機及標記不動，確認完整標記可見，再執行：

```bash
ros2 launch tidybot_ros2_control tidybot_docking.launch.py \
  teach_on_start:=true start_gamepad:=false
```

此模式維持預設 `dry_run:=true start_base:=false`。收到有效標記觀測後，會自動收集穩定姿態並存檔；
看到 `TAUGHT` 才代表完成，不會接著開始對接。若標記遺失、姿態不穩或存檔失敗，會停止本次示教並顯示 `FAILED`。
`/docking/cancel` 也可取消尚未開始收集的啟動示教。既有目標檔（包含無效檔案）不會被此選項覆寫；
要替換目標，使用下面的 `/docking/teach` 明確重新示教。

之後只需一般啟動，便會沿用已儲存目標：

```bash
ros2 launch tidybot_ros2_control tidybot_docking.launch.py
```

同一台車相機固定，且充電樁、卡榫及標記作為整組搬到其他環境時，標記與停靠位置的相對幾何不變，
可沿用目標，無須因換房間重新示教。若相機安裝、標記尺寸／ID、標記相對卡榫的位置或希望的停止位置改變，
則需要更新設定並重新示教。此流程處理充電樁前方的相對對接；全場導航到新環境中的充電站位置仍屬後續整合。

`target_right_offset` 是示教目標之外的固定機構補償。目前設為 `0.005` m，代表以機器人
朝向充電樁的視角，將整條停靠中心線向機器人右側平移 5 mm。它不是持續送出橫移速度；
控制器仍會先對準補償後的目標，再直線插入。若日後在機構正確位置重新示教，應將此值重設為 `0.0`，
避免示教位移與設定補償重複計算。

## 手動定位、示教、自動對接

新的 docking launch 已包含手把、控制權管理與可選的底盤。
**先關閉舊的 `tidybot_gamepad.launch.py` 與其他 `/cmd_vel` 發布者**，讓控制權只經過 mux。

完成預覽檢查後，啟用底盤：

```bash
ros2 launch tidybot_ros2_control tidybot_docking.launch.py \
  dry_run:=false start_base:=true
```

1. 依原本方式按 Y、按住 LB+RB，以手把移到目標位置。
   第一次目標先設在卡榫入口前，測量重複定位誤差後再往內調。
2. 放開 LB+RB，讓車停穩。示教期間保持車、相機、標記不動。
3. 執行示教，觀察狀態變成 `TAUGHT`：

```bash
ros2 service call /docking/teach std_srvs/srv/Trigger '{}'
ros2 topic echo /docking/status
```

示教會收集約 1 秒的新鮮姿態，檢查數量與穩定性後，原子寫入
`~/.ros/tidybot/dock_target.yaml`。`teach` 服務回覆成功表示開始收集，
必須等 `TAUGHT` 才表示儲存成功。若晃動、標記遺失或存檔失敗，狀態會是 `FAILED`。

4. 手動將車退回標記仍可見的位置，建議先從約 20–30 cm 的短距離開始。
5. 放開 LB+RB，執行開始：

```bash
ros2 service call /docking/start std_srvs/srv/Trigger '{}'
```

也可以不開另一個 terminal：先按 Y 解鎖，確認 LB、RB 都已放開，再按一下 **A**。
手把節點會呼叫同一個 `/docking/start` 服務，terminal 會顯示接受或拒絕原因；長按 A
不會重複啟動，必須放開後再按才會送出下一次請求。

`start` 回覆表示已接受啟動請求，實際控制權取得與最終結果請看 `/docking/status`。
相機觀測不新鮮、目標設定不一致、位置在車後方、起始距離／角度太大，或 mux 不可用時會拒絕。
目前不會自動倒退；超過目標時停止，改由手把重新定位。

控制分為三段：

- 距離大於 10 cm：`FAR_APPROACH` 允許前進、橫移與轉向同時進行；左右超過 8 cm
  時進入 `FAR_ALIGN` 並暫停前進。接近粗對準上限時會逐漸降低前進速度。
  若起始朝向超過約 11.5 度，先以 `FAR_HEADING` 原地轉到約 2.86 度內，再以
  `FAR_ALIGN` 修正轉正後的實際左右誤差，最後才開始前進。
- 距離 5–10 cm：前進速度降到 0.025–0.028 m/s。左右超過 15 mm 或朝向超過約
  2.86 度時進入 `FINE_ALIGN`，對準後以 `FINE_APPROACH` 繼續前進。
- 最後 5 cm：先在 `FINAL_ALIGN` 將左右壓到 5 mm、朝向壓到約 1.43 度；接著
  `INSERTING` 以 0.025 m/s 直線前進，鎖住橫移與旋轉，避免進入卡榫後左右擺動。
  插入期間若左右超過 25 mm 或朝向超過約 5.73 度，立即失敗並要求手動倒退。
- 原示教點穩定 0.6 秒後，進入 `EXTRA_INSERT`：以 ArUco 姿態記住該點，保持橫移與
  旋轉鎖定，再以 0.025 m/s 向前量測 5 mm。到達後在 `EXTRA_SETTLING` 停穩才回報
  `REACHED`；若 2 秒內未完成或對準偏移過大，會停止並回報失敗。

全部誤差進入容許範圍後先輸出零速，穩定約 0.6 秒，再執行設定的 5 mm 額外插入；
額外位置也穩定約 0.6 秒後才回報 `REACHED`。
遇到標記逾時、姿態跳動、接近逾時或持續沒有視覺進展，會停止並回報 `FAILED`。
若 ArUco 姿態短暫中斷，超過 0.15 秒便先進入 `WAITING_FOR_POSE` 並輸出零速；
0.75 秒內恢復觀測會從原階段繼續，持續超過 0.75 秒才回報失敗並釋放控制權。

無進展的等待時間為 8 秒（`progress_timeout`），整次接近以 120 秒為上限。
先前的 45 秒上限不足以讓車從允許的最遠 1.5 m 起點，以最高 0.03 m/s 完成接近；
延長總時間不會取消 8 秒無進展、標記逾時或姿態跳動等停止保護。
各階段只檢查當下有輸出控制的軸；接近階段也計入前後改善，避免前方距離較大時
忽略小幅橫移。任一適用軸比上次記錄的最佳值改善超過 4 mm，或轉角改善超過
0.016 rad（約 0.92°），就更新進展計時；較小的改善可累積，階段來回切換本身不會重設計時。
失敗訊息會附上階段、前後／左右／朝向誤差與無進展秒數，供實機判讀。
最後到位容許誤差維持前後 1 cm、左右 5 mm、朝向約 1.43°；
延長等待不代表能確認卡榫已接合，仍需實機量測與到位回饋。

啟動時 log 會列出載入的控制器路徑、`progress_timeout`、分段距離、精細對準門檻與速度上限。
修改程式或 YAML 後，須結束舊 launch 並重新啟動，已執行的 Python 程序不會自動重新載入。
對接期間每秒輸出一行 `CONTROL`，包含前後／左右／朝向誤差、送出的速度、
控制權與觀測距今秒數；`forward_blocked_by=lateral` 或 `heading` 表示左右或朝向超出門檻，
因此前進速度被設為零。這是控制命令，實際有沒有移動仍需比對 `/odom` 或實機量測。
`/docking/status` 同時提供 `command`（m/s、m/s、rad/s）、`control_granted`、
`forward_blocked_by`，原有 `error` 的角度單位仍為 rad。

隨時取消：

```bash
ros2 service call /docking/cancel std_srvs/srv/Trigger '{}'
```

手把接管與停止：

- 已按 Y 解鎖後，按住 LB+RB 會取消自動對接並切到手動，包括搖桿維持零的情況。
- 放開 LB+RB 後按 A，會送出新的自動對接請求；若 Back 尚未以 Y 解鎖則不會啟動。
- 放開 LB/RB，車停下；自動對接不會自行恢復，必須重新呼叫 `start`。
- Back 會停止並鎖定；再次按 Y 解除鎖定。Y 本身不會重新開始對接。
- 手把斷線或訊號逾時會停止並鎖定；重新連線後仍要按 Y。
- 自動控制心跳或速度訊息逾時，mux 撤銷控制權；恢復訊息後也不會自行續跑。

## 使用手把錄製 rosbag

Xbox **Menu** 鍵（中央右側、三條線圖示，預設 `buttons[11]`）用來切換錄製：

- 按一次：開始錄製，terminal 顯示 `ROSBAG RECORDING STARTED` 與資料夾路徑。
- 再按一次：送出安全停止，寫完 metadata 後顯示 `ROSBAG SAVED`。
- 關閉 launch 時若仍在錄製，手把節點也會先要求 rosbag 正常結束。

檔案預設存放在 `~/jerry/tidybot_ros2_control/rosbags/docking_日期_時間/`。為控制容量，帶框畫面以
10 FPS、JPEG quality 85 發布至 `/docking/debug_image/compressed`，bag 再使用 zstd 檔案壓縮。
同一份 bag 也會記錄停靠狀態、ArUco 姿態、速度命令、里程計、Joy 與 TF。剩餘空間低於
2 GB 時不會開始錄製；錄製期間低於門檻則會自動安全停止。

查看最近的錄影：

```bash
ls -1dt ~/jerry/tidybot_ros2_control/rosbags/docking_* | head
ros2 bag info ~/jerry/tidybot_ros2_control/rosbags/docking_日期_時間
ros2 bag play ~/jerry/tidybot_ros2_control/rosbags/docking_日期_時間
```

播放時可在另一個 terminal 顯示帶框影像：

```bash
rqt_image_view /docking/debug_image/compressed
```

若 Menu 沒反應，先用 `ros2 topic echo /joy` 確認按鍵是否讓 `buttons[11]` 從 0 變成 1；
不同手把可在 `config/gamepad.yaml` 修改 `button_rosbag`。

## 無 SSH 時用手把關閉 launch

Xbox／Guide 鍵（手把中央圓形 Xbox 標誌，預設 `buttons[12]`）會先發布停止並鎖定底盤，
再讓 gamepad node 正常退出。`tidybot_docking.launch.py` 和 `tidybot_gamepad.launch.py` 都會在
此節點退出後關閉同一個 launch 內的其餘程序，因此不需要保持 SSH 連線。若正在錄製 rosbag，
關閉流程會先要求錄影安全結束並寫入 metadata；terminal 會依序出現
`GAMEPAD SHUTDOWN REQUESTED`、`ROSBAG SAVED`（若有錄影）以及 launch shutdown 訊息。

若 Xbox／Guide 鍵沒反應，使用 `ros2 topic echo /joy` 檢查 `buttons[12]`；不同手把可修改
`config/gamepad.yaml` 的 `button_shutdown`。Back/View 仍只是緊急停止與鎖定，不會關閉 launch。

## 架構與 topic

```text
D415 RGB + CameraInfo + TF -> aruco_detector -> /docking/marker_pose
                                                      |
                                               docking_controller
                                                      |
                                               /docking/cmd_vel
                                                      |
Joy -> gamepad_teleop -> /teleop/cmd_vel ----------> control_mux
                                                      |
                                               /cmd_vel + enable
                                                      |
                                                  base_node
```

控制器與視覺節點不匯入馬達程式。原本單獨的 gamepad launch 仍可使用。
`/tidybot/control_status` 回報目前由誰控制，`/docking/status` 回報對接狀態與誤差。
`/docking/set_control` 是控制器與 mux 的內部握手服務，操作時使用 `start/cancel` 即可。

## 驗證與後續

```bash
cd /home/hrc/jerry/tidybot_ros2_control
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pytest -q tests
```

測試只使用合成影像與模擬姿態，不建立底盤或 CAN 連線。ROS 測試強制使用 localhost、
domain 179，且所有測試 topic/service 重新映射到獨立前綴。
涵蓋座標換算、閉迴路收斂、示教穩定性、設定變更、標記遺失、到位等待、
手把接管、Back、手把斷線、控制器心跳中斷，以及相機影像／CameraInfo／TF 的整合。

目前驗證：colcon 建置成功，49 項測試通過。涵蓋大角度先轉正再橫移、
遠距離同步前進與修正、
近距離精細對準、低速直線插入、插入期間偏差過大立即停止、慢速橫移／轉向仍能持續修正、
慢啟動寬限、靜止 8 秒仍會停止、階段反覆切換不會續命，以及停滯失敗後釋放控制權的回歸測試。
狀態回報另驗證了 3 cm 側偏在遠距離仍會前進、進入近距離後因相同側偏暫停前進，
以及取消後的零速與控制權釋放。
三個新節點先前也已透過 launch 以獨立程序
完成預覽模式的啟動與正常關閉（相機、手把與底盤均未啟動）。
其餘測試涵蓋啟動就緒檢查、目標重載、首次啟動示教、取消待處理示教、不穩定觀測失敗後不自動重試，
以及既有目標與設定不符或無效時不覆寫。另以實際安裝的 launch 與模擬姿態驗證
`teach_on_start:=true` 到 `TAUGHT`、重新啟動到 `READY`，預覽速度保持零、啟用訊號保持關閉。
RealSense 參數範圍已用不啟動程序的 launch 檢查驗證，不再把外層對接選項誤報為相機參數。
高度／傾斜拒絕訊息與待命檢查另有回歸測試；重播 100 筆實機姿態，確認門檻未變、
仍拒絕相同的 29 筆觀測。此批觀測的目標前向誤差中位數約 1.28 m，適合先縮短測試起始距離再檢查姿態穩定性。
自動影像視窗另以虛擬顯示驗證啟動、自動訂閱 `/docking/debug_image` 與正常關閉，
並發布 8 張合成影像；確認 `show_image:=false` 會省略視窗節點。

另已用實體 D415 驗證 USB 2.1 下的 `color_profile:=640x480x30`：
收到 205 張 RGB 影像，接收平均約 27 fps；CameraInfo、
`base_link -> camera_color_optical_frame` TF 與 `/docking/debug_image` 皆有效，
相機及所有新節點正常關閉，預覽速度保持零且沒有啟用底盤。
這次未偵測到指定的 ID 0；仍需將列印後的實體標記放入鏡頭視野，驗證標記姿態。

下一階段需要實機量測定位誤差與卡榫容許範圍，再加入到位開關或充電回饋。
Humble 到 Jazzy 的移植仍要檢查套件版本、相機 topic/TF 與實機行為；
目前使用一般 ROS 訊息與 service，不將底盤控制綁到特定 Nav2 版本。
