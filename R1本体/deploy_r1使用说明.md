# deploy_r1.sh 脚本使用说明

> **版本**：1.0 | **日期**：2026-05-12 | **文件**：`deploy_r1.sh`

---

## 1. 脚本功能

`deploy_r1.sh` 是一个智能部署脚本，用于简化 R1 展厅接待程序的编译、部署和开机自启动设置。

### 核心功能

| 功能 | 说明 |
|------|------|
| **首次部署** | 自动创建 systemd 服务 + 启用开机自启动 |
| **更新程序** | 编译 + 部署 + 重启服务（让新代码生效） |
| **智能检测** | 自动检测脚本所在目录，无需手动配置路径 |
| **错误检查** | 编译失败时自动停止，避免部署错误程序 |

---

## 2. 文件位置

### 机器人端

```
/home/unitree/unitree_sdk2-1.0/deploy_r1.sh
```

> **推荐**：把脚本和 `r1_show_control.cpp` 放在同一目录，方便管理。

---

## 3. 配置说明

脚本开头有配置区（第17-45行），**现在只需要修改一个地方**：

```bash
# 只需要修改这一行：源文件名（相对于脚本所在目录）
SOURCE_FILE="r1_show_control.cpp"
```

### 自动化推导逻辑

脚本会根据 `SOURCE_FILE` **自动推导** 所有其他路径：

| 变量 | 推导规则 | 示例（SOURCE_FILE="r1_chat.cpp"） |
|------|----------|------------------------------|
| `OUTPUT_FILE` | 去掉 `.cpp` 后缀 | `r1_chat` |
| `DEPLOY_PATH` | `/usr/local/bin/${OUTPUT_FILE}` | `/usr/local/bin/r1_chat` |
| `SERVICE_NAME` | `${OUTPUT_FILE}` 中 `_` 替换成 `-` + `.service` | `r1-chat.service` |
| `LOG_FILE` | `/var/log/${OUTPUT_FILE}.log` | `/var/log/r1_chat.log` |
| `LOGROTATE_CONF` | `/etc/logrotate.d/${OUTPUT_FILE}` 中 `_` 替换成 `-` | `/etc/logrotate.d/r1-chat` |

### 常见修改场景

| 场景 | 修改内容 |
|------|----------|
| **换新的 cpp 文件** | 第24行：`SOURCE_FILE="新文件名.cpp"` |
| **网口名不是 eth10** | 第45行：`NETWORK_IF="eth0"` |
| **SDK 目录不在脚本所在目录** | 第34行：手动设置 `SDK_DIR="/path/to/sdk"` |

> **提示**：修改 `SOURCE_FILE` 后，所有相关路径（部署路径、服务名、日志文件）都会自动更新，无需手动逐个修改。

---

## 4. 首次使用（设置开机自启动）

### 第1步：上传文件到机器人

在 MobaXterm 里：

1. 上传 `D:\unitree\r1_show_control.cpp` → `/home/unitree/unitree_sdk2-1.0/`
2. 上传 `D:\unitree\deploy_r1.sh` → `/home/unitree/unitree_sdk2-1.0/`

### 第2步：赋予执行权限

```bash
chmod +x /home/unitree/unitree_sdk2-1.0/deploy_r1.sh
```

### 第3步：运行脚本

```bash
cd /home/unitree/unitree_sdk2-1.0/
./deploy_r1.sh
```

### 第4步：查看输出

脚本会显示7个步骤的进度：

```
[1/7] 检查源文件...
  [OK] 找到源文件: /home/unitree/unitree_sdk2-1.0/r1_show_control.cpp

[2/7] 创建日志文件...
  [OK] 日志文件已创建: /var/log/r1_show_control.log

[3/7] 开始编译...
  [OK] 编译成功: r1_show_control

[4/7] 部署程序...
  [OK] 已复制到: /usr/local/bin/r1_show_control

[5/7] 检查 systemd 服务...
  [INFO] 首次部署，创建 systemd 服务...
  [OK] 服务文件已创建: /etc/systemd/system/r1-show.service
  [INFO] 启用开机自启动...
  [OK] 开机自启动已启用

[6/7] 配置日志轮转...
  [OK] logrotate 配置已创建: /etc/logrotate.d/r1-show
  [INFO] 日志文件将保留最近7天，自动压缩旧日志

[7/7] 重启服务...
  [OK] 服务已重启
```

### 第5步：查看程序日志

脚本运行完成后，会自动显示最近的程序日志：

```
最近程序日志（最后20行）：
[2026-05-12 11:00:15] [INFO] ===== 程序启动 =====
[2026-05-12 11:00:15] [INFO] 启动时间: 2026-05-12 11:00:15
[2026-05-12 11:00:15] [INFO] DDS 网卡: eth10
[2026-05-12 11:00:15] [INFO] 语音就绪
[2026-05-12 11:00:15] [INFO] 手臂就绪
[2026-05-12 11:00:15] [INFO] LED 红灯（空闲）
[2026-05-12 11:00:15] [INFO] 遥控器订阅已启动，等待连接...
```

### 第6步：验证开机自启动

```bash
# 查看服务是否启用
sudo systemctl is-enabled r1-show
# 应该输出：enabled

# 重启机器人测试
sudo reboot
# 等待重启完成，SSH重新登录
sudo systemctl status r1-show
# 应该显示：Active: active (running)
```

---

## 5. 脚本智能行为


| 场景 | 脚本行为 |
|------|----------|
| **首次运行** | 创建systemd服务 + 启用自启动 + 启动服务 |
| **以后运行** | 检测到服务已存在，跳过创建步骤 + 重新编译部署 + 重启服务 |

---

## 6. 常见问题

### Q1: 编译失败怎么办？

**现象**：

```
[3/7] 开始编译...
  [ERROR] 编译失败！请检查代码
```

**解决方法**：

1. 检查代码语法错误
2. 确认 SDK 目录正确（脚本和cpp在同一目录）
3. 查看完整编译错误信息

### Q2: 如何修改网口名？

**问题**：机器人网口名不是 `eth10`，是 `eth0`

**解决**：编辑脚本第37行：

```bash
nano /home/unitree/unitree_sdk2-1.0/deploy_r1.sh
```

把 `NETWORK_IF="eth10"` 改成 `NETWORK_IF="eth0"`，保存退出。

重新运行脚本：

```bash
./deploy_r1.sh
```

### Q3: 如何查看程序日志？

**方法1：查看程序日志文件（推荐）**

```bash
# 查看实时日志
tail -f /var/log/r1_show_control.log

# 查看最后50行
tail -n 50 /var/log/r1_show_control.log

# 搜索错误信息
grep -i "error\|warn" /var/log/r1_show_control.log
```

**方法2：查看 systemd 日志**

```bash
# 查看实时日志
sudo journalctl -u r1-show -f

# 查看最近50行
sudo journalctl -u r1-show -n 50 --no-pager
```

### Q4: 如何判断程序是正常运行还是崩溃了？

**查看日志最后一行的时间**：

```bash
tail -n 5 /var/log/r1_show_control.log
```

- **日志时间在最近1分钟内** → 程序正常运行
- **日志时间是很久以前** → 程序可能崩溃，重启服务：`sudo systemctl restart r1-show`

### Q5: 如何禁用开机自启动？

```bash
sudo systemctl disable r1-show
```

### Q6: 如何重新启用开机自启动？

```bash
sudo systemctl enable r1-show
```

### Q7: 脚本运行后，机器人断电重启，程序会自动运行吗？

**是的！** ✅

只要看到以下输出，就说明开机自启动已启用：

```
[OK] 开机自启动已启用
```

断电重启后，程序会自动运行，直接用遥控器控制即可，无需网线。

### Q8: 日志文件太大怎么办？

**无需担心！** ✅

脚本已自动配置 `logrotate`：
- 日志文件保留最近7天
- 自动压缩旧日志
- 配置文件：`/etc/logrotate.d/r1-show`

手动触发日志轮转：
```bash
sudo logrotate /etc/logrotate.d/r1-show
```

---
