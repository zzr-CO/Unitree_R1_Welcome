# R1 展厅控制使用说明

> 面向日常使用和现场测试。本文只写怎么操作、怎么启动、怎么排查。

## 1. 当前功能

R1 展厅程序可以通过遥控器完成：

- 播放 4 段迎宾话术。
- 执行对应手臂动作。
- 在第 2 个场景中让右手灵巧手比耶。
- 切换原始音色和东北话音色。
- 开启或停止 4 个场景循环播放。

主程序文件：

```text
R1本体/r1_show_control.cpp
```

R1 上的服务名：

```text
r1-show-control.service
```

日志文件：

```text
/var/log/r1_show_control.log
```

## 2. 遥控器操作

| 按键 | 效果 | 动作 |
|---|---|---|
| `← + A` | 播放话术 1 | 奥特曼光线 |
| `← + B` | 播放话术 2 | 脸部挥手 + 右手比耶 |
| `← + X` | 播放话术 3 | 手放胸口鞠躬 |
| `← + Y` | 播放话术 4 | 高举挥手 |
| `↓ + A` | 开启/停止 4 场景循环 | 依次播放 4 个场景 |
| `SELECT` | 切换音色 | 原始音色 / 东北话音色 |

注意：

- 遥控器长时间不用可能休眠，按任意键唤醒后再操作。
- `↓ + A` 是循环开关，按一次开始，再按一次停止。
- `← + B` 需要灵巧手服务正常运行，否则只会执行话术和手臂动作，右手不会比耶。

## 3. 启动和停止

查看迎宾服务：

```bash
systemctl status r1-show-control.service --no-pager
```

启动迎宾服务：

```bash
sudo systemctl start r1-show-control.service
```

停止迎宾服务：

```bash
sudo systemctl stop r1-show-control.service
```

重启迎宾服务：

```bash
sudo systemctl restart r1-show-control.service
```

设置开机自启动：

```bash
sudo systemctl enable r1-show-control.service
```

取消开机自启动：

```bash
sudo systemctl disable r1-show-control.service
```

## 4. 灵巧手服务

如果要使用 `← + B` 的右手比耶，需要确认灵巧手服务正在运行。

查看灵巧手服务：

```bash
systemctl status brainco_hand.service --no-pager
```

启动灵巧手服务：

```bash
sudo systemctl start brainco_hand.service
```

停止灵巧手服务：

```bash
sudo systemctl stop brainco_hand.service
```

重启灵巧手服务：

```bash
sudo systemctl restart brainco_hand.service
```

手动启动灵巧手服务：

```bash
cd ~/brainco_hand_service/bin
sudo ./brainco_hand_server --network_interface eth10
```

当前已知设备状态：

| 手 | 串口 | ID | 状态 |
|---|---|---:|---|
| 左手 | `/dev/ttyCH343USB0` | 100 | 已调通 |
| 右手 | `/dev/ttyCH343USB1` | 127 | 已调通 |
| 空口 | `/dev/ttyCH343USB2` | - | 未发现设备 |

## 5. 更新迎宾程序

### 5.1 推荐方式：使用部署脚本

把下面两个文件上传到 R1：

```text
/home/unitree/unitree_sdk2-1.0/r1_show_control.cpp
/home/unitree/unitree_sdk2-1.0/deploy_r1.sh
```

执行：

```bash
cd /home/unitree/unitree_sdk2-1.0
chmod +x deploy_r1.sh
bash ./deploy_r1.sh
```

部署完成后检查：

```bash
systemctl status r1-show-control.service --no-pager
tail -n 30 /var/log/r1_show_control.log
```

### 5.2 手动编译方式

进入目录：

```bash
cd /home/unitree/unitree_sdk2-1.0
```

编译：

```bash
g++ -std=c++17 r1_show_control.cpp \
    -I./include -I./thirdparty/include/ddscxx \
    -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
    -lunitree_sdk2 -lddscxx -lddsc -lpthread \
    -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
    -o r1_show_control
```

临时运行测试：

```bash
./r1_show_control eth10
```

让服务使用新版本：

```bash
sudo cp r1_show_control /usr/local/bin/r1_show_control
sudo chmod +x /usr/local/bin/r1_show_control
sudo systemctl restart r1-show-control.service
```

记住这个区别：

| 文件 | 用途 |
|---|---|
| `/home/unitree/unitree_sdk2-1.0/r1_show_control` | 手动编译出来的测试版 |
| `/usr/local/bin/r1_show_control` | 服务实际运行的版本 |

## 6. 日志查看

实时查看主日志：

```bash
tail -f /var/log/r1_show_control.log
```

查看最近 50 行：

```bash
tail -n 50 /var/log/r1_show_control.log
```

查看服务日志：

```bash
journalctl -u r1-show-control.service -f
```

查看所有 R1 相关服务：

```bash
systemctl list-units --type=service --all 'r1*'
systemctl list-unit-files 'r1*'
```

查看相关进程：

```bash
ps aux | grep '[r]1'
ps aux | grep '[b]rainco_hand'
```

## 7. 常见问题

| 现象 | 处理方法 |
|---|---|
| 遥控器无反应 | 先按任意键唤醒遥控器，再重新按组合键 |
| 服务没运行 | `sudo systemctl restart r1-show-control.service` |
| 改了代码但没生效 | 确认已经复制到 `/usr/local/bin/r1_show_control` 并重启服务 |
| 没有声音 | 检查音频文件是否在 `/home/unitree/voice_pack/audio_show/` |
| 手臂不动 | 确认机器人站稳，重启迎宾服务后再试 |
| `← + B` 不比耶 | 检查 `brainco_hand.service` 是否运行 |
| 灵巧手抖动 | 检查是否同时运行了多个测试程序 |
| 日志文件还在 | 日志文件残留不代表服务还在运行 |

灵巧手抖动时，可以先查进程：

```bash
ps aux | grep '[t]est_right'
ps aux | grep '[t]est_left'
ps aux | grep '[r]1_show_control'
```

停止右手测试程序：

```bash
pkill -f test_right_gesture_lab
```

停止左手测试程序：

```bash
pkill -f test_left_gesture_lab
```

## 8. 灵巧手独立测试

测试前确认灵巧手服务已经运行：

```bash
systemctl status brainco_hand.service --no-pager
```

右手测试：

```bash
./test_right_gesture_lab eth10
```

左手测试：

```bash
./test_left_gesture_lab eth10
```

常用输入：

| 输入 | 手势 |
|---|---|
| `0` | 复位张开 |
| `1` | 半握拳 |
| `2` | 剪刀手/比耶 |
| `3` | 大拇指点赞 |
| `4` | 竖起食指 |
| `5` | 电话手势/六 |
| `6` | 轻松自然手 |
| `8` | OK 手势 |
| `9` | 三指展示 |
| `a` | 四指展示 |
| `c` | 抓取杯子 |

如果要重新编译右手测试程序：

```bash
g++ -std=c++17 test_right_gesture_lab.cpp \
    -I/usr/local/include \
    -I/usr/local/include/ddscxx \
    -lunitree_sdk2 -lddscxx -lddsc -lpthread \
    -o test_right_gesture_lab
```

如果要重新编译左手测试程序：

```bash
g++ -std=c++17 test_left_gesture_lab.cpp \
    -I/usr/local/include \
    -I/usr/local/include/ddscxx \
    -lunitree_sdk2 -lddscxx -lddsc -lpthread \
    -o test_left_gesture_lab
```

## 9. 头部测试程序

头部测试程序只用于单独测试，不属于日常迎宾流程。

测试文件：

```text
test_head_motion.cpp
```

测试前先停止迎宾服务：

```bash
sudo systemctl stop r1-show-control.service
```

编译：

```bash
g++ -std=c++17 test_head_motion.cpp \
    -I/usr/local/include \
    -I/usr/local/include/ddscxx \
    -lunitree_sdk2 -lddscxx -lddsc -lpthread \
    -o test_head_motion
```

普通测试：

```bash
./test_head_motion eth10
```

不要使用 `--release-motion`。当前测试程序已禁用这个参数，因为释放运动模式可能导致机器人身体不稳定。

程序内输入：

| 输入 | 功能 |
|---|---|
| `0` | 头部回正 |
| `1` | 轻微点头 |
| `2` | 轻微摇头 |
| `p` | 打印当前头部位置 |
| `q` 或 `Ctrl+C` | 回正并退出 |

测试完成后恢复迎宾服务：

```bash
sudo systemctl restart r1-show-control.service
```

## 10. 文件清单

| 文件 | 说明 |
|---|---|
| `R1本体/r1_show_control.cpp` | 展厅迎宾主程序 |
| `R1本体/deploy_r1.sh` | 一键部署脚本 |
| `R1本体/cleanup_r1.sh` | 清理部署残留 |
| `R1本体/test_head_motion.cpp` | 头部测试程序 |
| `R1本体/R1手臂动作完整列表.md` | 手臂动作 ID 记录 |
| `灵巧手/test_right_gesture_lab.cpp` | 右手测试程序 |
| `灵巧手/test_left_gesture_lab.cpp` | 左手测试程序 |

## 11. 日常最常用命令

```bash
# 看迎宾服务
systemctl status r1-show-control.service --no-pager

# 重启迎宾服务
sudo systemctl restart r1-show-control.service

# 看迎宾日志
tail -f /var/log/r1_show_control.log

# 看灵巧手服务
systemctl status brainco_hand.service --no-pager

# 重启灵巧手服务
sudo systemctl restart brainco_hand.service

# 停止迎宾服务后测试头部
sudo systemctl stop r1-show-control.service
./test_head_motion eth10
```
