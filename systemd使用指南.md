# systemd 使用指南

## 📚 什么是 systemd？

**systemd** 是 Linux 系统的**初始化系统和服务管理器**，负责：
- 系统启动时启动服务
- 管理后台服务（启动、停止、重启）
- 查看服务状态和日志
- 服务崩溃时自动重启

**为什么需要 systemd？**
- ✅ 开机自动启动程序（不需要手动运行）
- ✅ 程序崩溃自动重启
- ✅ 统一管理所有后台服务
- ✅ 集中查看日志

---

## 🔧 systemd 核心概念

### **1. Unit（单元）**
systemd 管理的基本对象，分为多种类型：
- **service**：系统服务（最常用）
- **target**：一组服务的集合（类似运行级别）
- **timer**：定时器（类似 cron）
- **socket**：网络套接字

### **2. Service（服务）**
一个后台运行的程序，比如：
- `ssh.service`：SSH 服务
- `nginx.service`：Nginx Web 服务器
- `r1-show.service`：你的 R1 机器人控制程序

### **3. Service File（服务文件）**
定义服务如何启动、重启、依赖关系的配置文件，位于：
```
/etc/systemd/system/r1-show.service
```

---

## 📝 服务文件结构

### **示例：r1-show.service**
```ini
[Unit]
Description=Unitree R1 Showroom Control
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash -c '/usr/local/bin/r1_show_control eth10 >> /var/log/r1_show_control.log 2>&1'
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

### **各部分说明**

#### **[Unit] 部分**
| 指令 | 说明 | 示例 |
|------|------|------|
| `Description` | 服务描述 | `Description=Unitree R1 Showroom Control` |
| `After` | 在哪个服务启动后再启动 | `After=network-online.target` |
| `Wants` | 弱依赖（可选） | `Wants=network-online.target` |
| `Requires` | 强依赖（必须） | `Requires=nginx.service` |

#### **[Service] 部分**
| 指令 | 说明 | 示例 |
|------|------|------|
| `Type` | 服务类型 | `Type=simple` |
| `ExecStart` | 启动命令 | `ExecStart=/usr/local/bin/app` |
| `ExecStop` | 停止命令 | `ExecStop=/bin/kill -TERM $MAINPID` |
| `Restart` | 重启策略 | `Restart=always` |
| `RestartSec` | 重启延迟 | `RestartSec=5` |
| `User` | 运行用户 | `User=unitree` |
| `WorkingDirectory` | 工作目录 | `WorkingDirectory=/home/unitree` |

**Restart 策略：**
- `no`：不重启（默认）
- `always`：总是重启
- `on-failure`：只有非正常退出才重启
- `on-abnormal`：只有信号杀死或超时才重启

#### **[Install] 部分**
| 指令 | 说明 | 示例 |
|------|------|------|
| `WantedBy` | 启用时链接到哪个 target | `WantedBy=multi-user.target` |

**常用 target：**
- `multi-user.target`：多用户命令行模式（常用）
- `graphical.target`：图形界面模式

---

## 🛠️ 常用命令

### **1. 服务管理**

#### **启动服务**
```bash
sudo systemctl start r1-show.service
```

#### **停止服务**
```bash
sudo systemctl stop r1-show.service
```

#### **重启服务**
```bash
sudo systemctl restart r1-show.service
```

#### **重新加载配置（不重启）**
```bash
sudo systemctl reload r1-show.service
```

#### **启用开机自启动**
```bash
sudo systemctl enable r1-show.service
```

#### **禁用开机自启动**
```bash
sudo systemctl disable r1-show.service
```

#### **查看服务状态**
```bash
sudo systemctl status r1-show.service
```

**输出示例：**
```
● r1-show.service - Unitree R1 Showroom Control
   Loaded: loaded (/etc/systemd/system/r1-show.service; enabled)
   Active: active (running) since Tue 2026-05-12 13:00:00 CST; 5min ago
 Main PID: 12345 (r1_show_contro)
    Tasks: 10
   Memory: 50.0M
   CGroup: /system.slice/r1-show.service
           └─12345 /usr/local/bin/r1_show_control eth10
```

**状态说明：**
- `active (running)`：正在运行
- `active (exited)`：成功执行并退出
- `inactive (dead)`：未运行
- `failed`：启动失败

---

### **2. 查看日志**

#### **查看服务日志（实时）**
```bash
sudo journalctl -u r1-show.service -f
```
- `-u`：指定服务
- `-f`：实时跟踪（类似 `tail -f`）

#### **查看最近 50 行日志**
```bash
sudo journalctl -u r1-show.service -n 50
```

#### **查看今天以来的日志**
```bash
sudo journalctl -u r1-show.service --since today
```

#### **查看指定时间段的日志**
```bash
sudo journalctl -u r1-show.service --since "2026-05-12 10:00:00" --until "2026-05-12 12:00:00"
```

#### **查看错误日志**
```bash
sudo journalctl -u r1-show.service --priority=err
```

**优先级：**
- `0` / `emerg`：紧急
- `1` / `alert`：警报
- `2` / `crit`：严重
- `3` / `err`：错误
- `4` / `warning`：警告
- `5` / `notice`：注意
- `6` / `info`：信息
- `7` / `debug`：调试

---

### **3. 系统级命令**

#### **重新加载 systemd 配置**
```bash
sudo systemctl daemon-reload
```
**什么时候用？**
- 修改了服务文件（`.service`）
- 创建了新的服务文件

#### **列出所有正在运行的服务**
```bash
sudo systemctl list-units --type=service --state=running
```

#### **列出所有服务（包括未运行的）**
```bash
sudo systemctl list-units --type=service --all
```

#### **检查服务是否启用**
```bash
sudo systemctl is-enabled r1-show.service
```
**输出：**
- `enabled`：已启用开机自启动
- `disabled`：未启用
- `static`：不能单独启用（被其他服务依赖）

---

### **4. 关机/重启**

#### **重启系统**
```bash
sudo systemctl reboot
```

#### **关机**
```bash
sudo systemctl poweroff
```

#### **休眠**
```bash
sudo systemctl hibernate
```

---

## 🐛 常见问题排查

### **问题1：服务启动失败**
```bash
# 1. 查看服务状态
sudo systemctl status r1-show.service

# 2. 查看详细日志
sudo journalctl -u r1-show.service -n 50

# 3. 检查服务文件语法
sudo systemd-analyze verify /etc/systemd/system/r1-show.service
```

### **问题2：程序路径错误**
**错误示例：**
```
ExecStart=/usr/local/bin/r1_show_control
```
**排查：**
```bash
# 检查文件是否存在
ls -l /usr/local/bin/r1_show_control

# 检查是否有执行权限
chmod +x /usr/local/bin/r1_show_control
```

### **问题3：服务启动太快，网络还没准备好**
**解决方法：**
```ini
[Unit]
After=network-online.target
Wants=network-online.target
```

### **问题4：查看依赖关系**
```bash
sudo systemctl list-dependencies r1-show.service
```

---

## 📋 实用案例

### **案例1：创建自定义服务**

#### **步骤1：创建服务文件**
```bash
sudo nano /etc/systemd/system/my-app.service
```

#### **步骤2：写入配置**
```ini
[Unit]
Description=My Custom Application
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/local/bin/my-app
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

#### **步骤3：重新加载配置**
```bash
sudo systemctl daemon-reload
```

#### **步骤4：启用并启动服务**
```bash
sudo systemctl enable my-app.service
sudo systemctl start my-app.service
```

#### **步骤5：查看状态**
```bash
sudo systemctl status my-app.service
```

---

### **案例2：延迟启动服务**

如果服务需要在其他服务启动后 30 秒再启动：

```ini
[Service]
ExecStartPre=/bin/sleep 30
ExecStart=/usr/local/bin/my-app
```

---

### **案例3：服务依赖多个条件**

```ini
[Unit]
Description=My App
After=network-online.target mysql.service redis.service
Requires=mysql.service redis.service
```

---

## 🔍 高级技巧

### **1. 查看服务启动时间**
```bash
sudo systemd-analyze blame | head -20
```

### **2. 查看启动耗时**
```bash
sudo systemd-analyze time
```

### **3. 调试服务启动**
```bash
sudo systemctl start r1-show.service
sudo journalctl -u r1-show.service -f
```

### **4. 临时禁用服务**
```bash
sudo systemctl mask r1-show.service
```
**恢复：**
```bash
sudo systemctl unmask r1-show.service
```

### **5. 设置环境变量**
```ini
[Service]
Environment="PATH=/usr/local/bin:/usr/bin"
Environment="MY_VAR=my_value"
```

或者从文件读取：
```ini
[Service]
EnvironmentFile=/etc/my-app/env.conf
```

---

## 📊 日志文件位置

| 日志类型 | 路径 | 说明 |
|---------|------|------|
| **systemd 日志** | `/var/log/journal/` | 二进制格式，用 `journalctl` 查看 |
| **系统日志** | `/var/log/syslog` 或 `/var/log/messages` | 系统级日志 |
| **服务自定义日志** | 由 `ExecStart` 重定向决定 | 例如 `/var/log/r1_show_control.log` |

---

## 🎯 快速参考表

### **服务管理**
| 命令 | 说明 |
|------|------|
| `systemctl start <服务>` | 启动服务 |
| `systemctl stop <服务>` | 停止服务 |
| `systemctl restart <服务>` | 重启服务 |
| `systemctl reload <服务>` | 重新加载配置 |
| `systemctl status <服务>` | 查看状态 |
| `systemctl enable <服务>` | 启用开机自启动 |
| `systemctl disable <服务>` | 禁用开机自启动 |

### **日志查看**
| 命令 | 说明 |
|------|------|
| `journalctl -u <服务>` | 查看服务日志 |
| `journalctl -u <服务> -f` | 实时查看日志 |
| `journalctl -u <服务> -n 50` | 查看最近50行 |
| `journalctl --since today` | 查看今天日志 |

---

## ✅ 总结

**systemd 的核心价值：**
- ✅ 自动化管理（开机启动、崩溃重启）
- ✅ 统一接口（所有服务用同一套命令）
- ✅ 强大日志（journalctl 集中管理）
- ✅ 依赖管理（控制启动顺序）

**记住这5个命令就能应付90%的场景：**
```bash
sudo systemctl status <服务>    # 查看状态
sudo systemctl restart <服务>   # 重启服务
sudo journalctl -u <服务> -f   # 查看实时日志
sudo systemctl enable <服务>    # 启用开机启动
sudo systemctl daemon-reload    # 重新加载配置
```

---

**文档版本：** 1.0  
**更新日期：** 2026-05-12  
**适用系统：** Ubuntu 16.04+, Debian 8+, CentOS 7+
