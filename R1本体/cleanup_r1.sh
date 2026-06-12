#!/bin/bash
# GitHub display label: 一键删除
# 完整清理 r1_show_control 程序的所有痕迹（不含音频文件）
# 使用方法：
#   1. 上传此脚本到机器人：/home/unitree/
#   2. 执行：chmod +x cleanup_r1.sh && ./cleanup_r1.sh

echo "=== 开始清理 r1_show_control 所有痕迹 ==="

# 1. 停止并禁用服务
echo "[1/6] 停止服务..."
sudo systemctl stop r1-show-control 2>/dev/null
sudo systemctl disable r1-show-control 2>/dev/null
echo "  ✓ 服务已停止并禁用"

# 2. 删除 systemd 服务文件
echo "[2/6] 删除服务文件..."
sudo rm -f /etc/systemd/system/r1-show-control.service
echo "  ✓ 已删除: /etc/systemd/system/r1-show-control.service"
    
# 3. 删除可执行文件
echo "[3/6] 删除可执行文件..."
sudo rm -f /usr/local/bin/r1_show_control
echo "  ✓ 已删除: /usr/local/bin/r1_show_control"

# 4. 删除日志文件
echo "[4/6] 删除日志文件..."
sudo rm -f /var/log/r1_show_control.log
sudo rm -f /var/log/r1_show_control.log.1
sudo rm -f /var/log/r1_show_control.log.1.gz
echo "  ✓ 已删除: /var/log/r1_show_control.log*"

# 5. 删除日志轮转配置
echo "[5/6] 删除日志轮转配置..."
sudo rm -f /etc/logrotate.d/r1-show-control
echo "  ✓ 已删除: /etc/logrotate.d/r1-show-control"

# 6. 重载 systemd 并验证
echo "[6/6] 重载 systemd..."
sudo systemctl daemon-reload

echo ""
echo "=== 清理完成，开始验证 ==="

# 验证
if [ -f /etc/systemd/system/r1-show-control.service ]; then
    echo "✗ 服务文件仍存在"
else
    echo "✓ 服务文件已删除"
fi

if [ -f /usr/local/bin/r1_show_control ]; then
    echo "✗ 可执行文件仍存在"
else
    echo "✓ 可执行文件已删除"
fi

if [ -f /var/log/r1_show_control.log ]; then
    echo "✗ 日志文件仍存在"
else
    echo "✓ 日志文件已删除"
fi

if [ -f /etc/logrotate.d/r1-show-control ]; then
    echo "✗ 日志轮转配置仍存在"
else
    echo "✓ 日志轮转配置已删除"
fi

# 检查是否还有相关进程
if ps aux | grep -v grep | grep r1_show_control > /dev/null; then
    echo "⚠ 警告：仍有 r1_show_control 进程在运行"
    ps aux | grep -v grep | grep r1_show_control
else
    echo "✓ 无相关进程"
fi

echo ""
echo "=== 全部清理完成 ==="
echo "提示：如需删除音频文件，请手动执行："
echo "  rm -rf /home/unitree/voice_pack/audio_show/"
