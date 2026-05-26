# R1 项目 systemd 与 sudo 使用指南

这份文档只围绕当前 R1 迎宾项目常用操作展开。重点是看懂 `sudo`、`systemctl`、`journalctl`，以及怎么查看、启动、停止、排查 `r1-show-control.service`。

## 一、先分清几个概念

### 1. sudo 是什么

`sudo` 的意思是“用管理员权限执行这一条命令”。

普通用户 `unitree` 可以查看很多信息，但不能随便修改系统服务、删除系统目录里的文件、重启系统。遇到这些操作时，就需要在命令前面加 `sudo`。

常见规律：

| 操作 | 是否通常需要 `sudo` | 示例 |
|---|---:|---|
| 查看服务状态 | 不一定 | `systemctl status r1-show-control.service` |
| 查看服务列表 | 不需要 | `systemctl list-units --type=service --all` |
| 启动/停止/重启服务 | 需要 | `sudo systemctl restart r1-show-control.service` |
| 设置开机自启 | 需要 | `sudo systemctl enable r1-show-control.service` |
| 修改 `/etc/systemd/system/` | 需要 | `sudo nano /etc/systemd/system/r1-show-control.service` |
| 删除 `/var/log/` 日志 | 通常需要 | `sudo rm -f /var/log/r1_show_control.log` |
| 重启机器人 | 需要 | `sudo reboot` |

简单记法：

```text
看信息：大多不用 sudo
改系统：基本都要 sudo
```

### 2. systemd 是什么

`systemd` 是 Linux 系统里的服务管理器。机器人开机后，哪些后台程序要自动启动、程序挂了要不要重启，都是它管。

在本项目里，迎宾程序就是通过 systemd 做成后台服务运行的。

### 3. systemctl 是什么

`systemctl` 是你操作 systemd 的命令。

比如：

```bash
systemctl status r1-show-control.service
sudo systemctl restart r1-show-control.service
sudo systemctl enable r1-show-control.service
```

可以理解为：

```text
systemd：后台管理系统
systemctl：你用来指挥 systemd 的命令
```

### 4. journalctl 是什么

`journalctl` 用来查看 systemd 收集到的服务日志。

如果服务启动失败、反复重启、没有输出，可以先用：

```bash
journalctl -u r1-show-control.service -n 50
```

实时跟踪日志：

```bash
journalctl -u r1-show-control.service -f
```

如果提示权限不够，就加 `sudo`：

```bash
sudo journalctl -u r1-show-control.service -f
```

## 二、当前 R1 项目里常见对象

### 1. 当前主服务

当前迎宾主程序服务名：

```text
r1-show-control.service
```

常见服务文件路径：

```text
/etc/systemd/system/r1-show-control.service
```

常见可执行文件路径：

```text
/usr/local/bin/r1_show_control
```

常见日志文件：

```text
/var/log/r1_show_control.log
```

### 2. 服务、进程、可执行文件、日志不是一回事

| 名称 | 含义 | 例子 |
|---|---|---|
| 服务 | systemd 管理规则 | `r1-show-control.service` |
| 进程 | 正在运行的程序实例 | `/usr/local/bin/r1_show_control eth10` |
| 可执行文件 | 真正被运行的程序文件 | `/usr/local/bin/r1_show_control` |
| 日志文件 | 程序输出保存的位置 | `/var/log/r1_show_control.log` |

所以：

- 删除日志，不等于删除服务。
- 停止服务，不等于禁用开机自启。
- 删除服务文件，不等于杀掉当前已经运行的进程。
- 重新编译程序，不等于 systemd 已经运行新版本。

## 三、查看 R1 相关服务

### 1. 查看所有服务

```bash
systemctl list-units --type=service --all
```

这个会列出当前 systemd 知道的所有 service，包括运行中和未运行的。

### 2. 只看 r1 开头的服务

```bash
systemctl list-units --type=service --all 'r1*'
```

这是排查 R1 项目最常用的命令之一。

### 3. 查看已安装的 r1 服务文件

```bash
systemctl list-unit-files 'r1*'
```

区别：

```text
list-units：看当前 systemd 加载到的服务单元状态
list-unit-files：看系统里安装了哪些服务文件，以及是否 enabled
```

### 4. 查看 r1 相关进程

```bash
ps aux | grep '[r]1'
```

这个比 `ps aux | grep r1` 干净一些，因为它不会把 `grep r1` 自己也显示出来。

如果你看到类似：

```text
/usr/local/bin/r1_show_control eth10
```

说明迎宾程序进程正在运行。

## 四、管理当前迎宾服务

### 1. 查看状态

```bash
systemctl status r1-show-control.service
```

常见状态：

| 状态 | 含义 |
|---|---|
| `active (running)` | 服务正在运行 |
| `inactive (dead)` | 服务没有运行 |
| `failed` | 服务启动失败 |
| `enabled` | 已设置开机自启 |
| `disabled` | 没有设置开机自启 |

如果普通用户看不到完整信息，可以用：

```bash
sudo systemctl status r1-show-control.service
```

### 2. 启动服务

```bash
sudo systemctl start r1-show-control.service
```

只启动当前这一次，不代表开机自启。

### 3. 停止服务

```bash
sudo systemctl stop r1-show-control.service
```

只停止当前这一次。如果服务仍然是 `enabled`，下次开机还会自动启动。

### 4. 重启服务

```bash
sudo systemctl restart r1-show-control.service
```

修改了程序、更新了可执行文件后，常用这个让服务重新跑起来。

### 5. 设置开机自启

```bash
sudo systemctl enable r1-show-control.service
```

这表示机器人下次开机后，systemd 会自动启动这个服务。

### 6. 取消开机自启

```bash
sudo systemctl disable r1-show-control.service
```

这只是不让它下次开机自动启动，不一定会停止当前正在运行的服务。

如果想“现在也停掉，以后也不自启”，要连续执行：

```bash
sudo systemctl stop r1-show-control.service
sudo systemctl disable r1-show-control.service
```

### 7. 修改服务文件后重新加载

只要你改了：

```text
/etc/systemd/system/r1-show-control.service
```

就需要执行：

```bash
sudo systemctl daemon-reload
```

然后再重启服务：

```bash
sudo systemctl restart r1-show-control.service
```

`daemon-reload` 只让 systemd 重新读取服务配置，不会自动重启你的程序。

## 五、查看日志

### 1. 查看程序自己的日志文件

当前部署脚本会把程序输出重定向到：

```text
/var/log/r1_show_control.log
```

查看最近 50 行：

```bash
tail -n 50 /var/log/r1_show_control.log
```

实时查看：

```bash
tail -f /var/log/r1_show_control.log
```

退出实时查看按：

```text
Ctrl+C
```

### 2. 查看 systemd 日志

最近 50 行：

```bash
journalctl -u r1-show-control.service -n 50
```

实时查看：

```bash
journalctl -u r1-show-control.service -f
```

只看今天：

```bash
journalctl -u r1-show-control.service --since today
```

如果没有权限：

```bash
sudo journalctl -u r1-show-control.service -f
```

### 3. 为什么会有旧日志残留

你可能会在 `/var/log/` 看到：

```text
r1_show_control.log
r1_show_control_dual.log
r1_showroom.log
```

这不一定代表这些服务还在运行。

原因是：

- 日志文件是普通文件，服务删了以后它不会自动消失。
- 旧程序曾经写过日志，文件会一直留在 `/var/log/`。
- 如果两个服务或两个脚本都把输出写到同一个地方，看起来内容可能很像。

判断服务是否真的还在运行，要看：

```bash
systemctl list-units --type=service --all 'r1*'
ps aux | grep '[r]1'
```

不要只看 `/var/log/` 里有没有日志文件。

## 六、排查常用流程

### 情况 1：机器人开机后迎宾程序没启动

按顺序查：

```bash
systemctl status r1-show-control.service
```

如果没运行，再看是否开机自启：

```bash
systemctl is-enabled r1-show-control.service
```

如果输出 `disabled`，启用：

```bash
sudo systemctl enable r1-show-control.service
sudo systemctl start r1-show-control.service
```

### 情况 2：服务启动失败

先看状态：

```bash
systemctl status r1-show-control.service
```

再看 systemd 日志：

```bash
journalctl -u r1-show-control.service -n 80
```

再看程序日志：

```bash
tail -n 80 /var/log/r1_show_control.log
```

常见原因：

- `/usr/local/bin/r1_show_control` 不存在。
- 可执行文件没有权限。
- DDS 网口写错，比如应该是 `eth10`。
- 程序依赖的 SDK 库路径不对。
- 语音文件或资源路径不存在。

### 情况 3：修改了服务文件但没生效

服务文件改完后必须：

```bash
sudo systemctl daemon-reload
sudo systemctl restart r1-show-control.service
```

只保存文件是不够的。

### 情况 4：怀疑旧服务还在运行

查看所有 R1 服务：

```bash
systemctl list-units --type=service --all 'r1*'
systemctl list-unit-files 'r1*'
```

查看 R1 相关进程：

```bash
ps aux | grep '[r]1'
```

如果发现旧服务还在，比如：

```text
r1-showroom.service
r1-canteen.service
```

先停掉：

```bash
sudo systemctl stop r1-showroom.service
sudo systemctl disable r1-showroom.service
```

确认无误后，再考虑删除对应服务文件。

## 七、清理旧服务和旧日志

### 1. 停止并禁用旧服务

以旧的 `r1-showroom.service` 为例：

```bash
sudo systemctl stop r1-showroom.service
sudo systemctl disable r1-showroom.service
```

### 2. 删除旧服务文件

确认这个服务已经不用了，再删除：

```bash
sudo rm -f /etc/systemd/system/r1-showroom.service
sudo systemctl daemon-reload
```

不要乱用：

```bash
sudo rm -f /etc/systemd/system/*.service
```

这会删除大量系统服务文件，非常危险。

### 3. 删除旧日志

删除日志只会删除文件，不会停止服务：

```bash
sudo rm -f /var/log/r1_show_control_dual.log
sudo rm -f /var/log/r1_showroom.log
```

如果服务还在运行，它可能会再次生成日志文件。

所以正确顺序是：

```text
先停服务 -> 再禁用自启 -> 再删服务文件 -> daemon-reload -> 再删旧日志
```

## 八、当前项目最常用命令速查

### 查看类

```bash
systemctl list-units --type=service --all 'r1*'
systemctl list-unit-files 'r1*'
systemctl status r1-show-control.service
ps aux | grep '[r]1'
tail -f /var/log/r1_show_control.log
journalctl -u r1-show-control.service -f
```

### 操作类

```bash
sudo systemctl start r1-show-control.service
sudo systemctl stop r1-show-control.service
sudo systemctl restart r1-show-control.service
sudo systemctl enable r1-show-control.service
sudo systemctl disable r1-show-control.service
sudo systemctl daemon-reload
```

### 手部服务常用启动命令

如果机器人重启后灵巧手不动，常见原因是 `brainco_hand_server` 没启动。

手动启动示例：

```bash
cd ~/brainco_hand_service/bin
sudo ./brainco_hand_server --network_interface eth10
```

这个目前通常是手动运行，不一定已经做成 systemd 服务。

## 九、高风险命令提醒

### 1. sudo reboot

```bash
sudo reboot
```

会直接重启机器人。实机测试时要确认机器人处于安全姿态。

### 2. sudo rm

```bash
sudo rm -f 文件路径
```

管理员权限删除文件，删错了不一定能恢复。

尤其不要随便执行：

```bash
sudo rm -rf /
sudo rm -rf /*
sudo rm -f /etc/systemd/system/*.service
```

### 3. stop 和 disable 不是一回事

```bash
sudo systemctl stop r1-show-control.service
```

表示现在停。

```bash
sudo systemctl disable r1-show-control.service
```

表示下次开机不要自动启动。

如果你只执行 `stop`，机器人重启后服务可能又会起来。

### 4. 删除日志不是解决服务问题

删除：

```bash
sudo rm -f /var/log/r1_show_control.log
```

只会删日志文件，不会停止程序，也不会修复程序。

排查服务问题优先看：

```bash
systemctl status r1-show-control.service
journalctl -u r1-show-control.service -n 50
ps aux | grep '[r]1'
```

## 十、推荐日常排查顺序

如果你不确定 R1 当前到底跑了什么，按这个顺序来：

```bash
systemctl list-units --type=service --all 'r1*'
systemctl list-unit-files 'r1*'
systemctl status r1-show-control.service
ps aux | grep '[r]1'
tail -n 50 /var/log/r1_show_control.log
journalctl -u r1-show-control.service -n 50
```

如果改了服务文件：

```bash
sudo systemctl daemon-reload
sudo systemctl restart r1-show-control.service
systemctl status r1-show-control.service
```

如果只是重新部署了可执行文件：

```bash
sudo systemctl restart r1-show-control.service
tail -f /var/log/r1_show_control.log
```

## 版本记录

- 更新时间：2026-05-26
- 适用项目：`D:\Unitree-R1-迎宾`
- 当前主服务：`r1-show-control.service`
