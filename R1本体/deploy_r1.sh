#!/bin/bash

# =============================================================
# R1 展厅接待程序 - 一键部署/更新脚本
# 功能：
#   1. 首次运行：自动创建systemd服务 + 启用自启动
#   2. 以后运行：编译 + 部署 + 重启服务
#   3. 配置日志文件输出和logrotate自动轮转
# 使用方法：
#   1. 把本脚本和 r1_show_control.cpp 放在同一目录
#   2. 赋予执行权限：chmod +x deploy_r1.sh
#   3. 修改下面的配置区（第23-49行）
#   4. 运行：./deploy_r1.sh
# =============================================================

# ==================== 自动检测脚本所在目录 ====================
# 不管你把脚本放哪，都能正确找到 cpp 文件
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[INFO] 脚本所在目录: ${SCRIPT_DIR}"
# =============================================================


# ==================== 配置区 ====================
# 只需要修改下面这一行：源文件名（相对于脚本所在目录）
SOURCE_FILE="r1_show_control.cpp"

# 自动推导其他路径（通常无需修改）
# 编译输出文件名 = 源文件名去掉 .cpp 后缀
OUTPUT_FILE="${SOURCE_FILE%.cpp}"

# 部署目标路径
DEPLOY_PATH="/usr/local/bin/${OUTPUT_FILE}"

# systemd 服务文件名（把文件名中的 _ 替换成 -）
SERVICE_NAME="${OUTPUT_FILE//_/-}.service"

# 日志文件路径
LOG_FILE="/var/log/${OUTPUT_FILE}.log"

# logrotate 配置文件路径（把文件名中的 _ 替换成 -）
LOGROTATE_CONF="/etc/logrotate.d/${OUTPUT_FILE//_/-}"

# SDK 根目录（如果脚本不在SDK目录，修改这里）
# 如果脚本和 cpp 都在 SDK 根目录，保持默认值即可
SDK_DIR="${SCRIPT_DIR}"

# 网口名称（确认你的机器人是 eth10 还是 eth0）
NETWORK_IF="eth10"
# =============================================================


echo "============================================================"
echo "  R1 展厅接待程序 - 部署/更新脚本"
echo "  日志文件: ${LOG_FILE}"
echo "============================================================"
echo ""

# ==================== 第1步：检查文件 ====================
echo "[1/7] 检查源文件..."
FULL_SOURCE_PATH="${SDK_DIR}/${SOURCE_FILE}"
if [ ! -f "${FULL_SOURCE_PATH}" ]; then
    echo "  [ERROR] 源文件不存在: ${FULL_SOURCE_PATH}"
    echo "  请确认 ${SOURCE_FILE} 和脚本在同一目录"
    exit 1
fi
echo "  [OK] 找到源文件: ${FULL_SOURCE_PATH}"
echo ""

# ==================== 第1.5步：创建日志文件 ====================
echo "[2/7] 创建日志文件..."
sudo touch "${LOG_FILE}"
sudo chmod 666 "${LOG_FILE}"
echo "  [OK] 日志文件已创建: ${LOG_FILE}"
echo ""

# ==================== 第2步：编译 ====================
echo "[3/7] 开始编译..."
cd "${SDK_DIR}" || {
    echo "  [ERROR] 无法进入目录: ${SDK_DIR}"
    exit 1
}

g++ -std=c++17 "${SOURCE_FILE}" \
    -I./include -I./thirdparty/include/ddscxx \
    -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
    -lunitree_sdk2 -lddscxx -lddsc -lpthread \
    -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
    -o "${OUTPUT_FILE}"

if [ $? -ne 0 ]; then
    echo "  [ERROR] 编译失败！请检查代码"
    exit 1
fi
echo "  [OK] 编译成功: ${OUTPUT_FILE}"
echo ""

# ==================== 第3步：部署程序 ====================
echo "[4/7] 部署程序..."
sudo cp "${OUTPUT_FILE}" "${DEPLOY_PATH}"
sudo chmod +x "${DEPLOY_PATH}"
echo "  [OK] 已复制到: ${DEPLOY_PATH}"
echo ""

# ==================== 第4步：检查/创建systemd服务 ====================
echo "[5/7] 检查 systemd 服务..."

SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"

if [ ! -f "${SERVICE_FILE}" ]; then
    echo "  [INFO] 首次部署，创建 systemd 服务..."
    sudo tee "${SERVICE_FILE}" > /dev/null <<EOF
[Unit]
Description=Unitree R1 Showroom Control
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash -c '${DEPLOY_PATH} ${NETWORK_IF} >> ${LOG_FILE} 2>&1'
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

    echo "  [OK] 服务文件已创建: ${SERVICE_FILE}"
    echo "  [INFO] 启用开机自启动..."
    sudo systemctl daemon-reload
    sudo systemctl enable "${SERVICE_NAME}"
    echo "  [OK] 开机自启动已启用"
else
    echo "  [OK] 服务文件已存在，跳过创建"
    sudo systemctl daemon-reload
fi
echo ""

# ==================== 第5步：配置 logrotate ====================
echo "[6/7] 配置日志轮转..."
sudo tee "${LOGROTATE_CONF}" > /dev/null <<EOF
${LOG_FILE} {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 666 root root
}
EOF
echo "  [OK] logrotate 配置已创建: ${LOGROTATE_CONF}"
echo "  [INFO] 日志文件将保留最近7天，自动压缩旧日志"
echo ""

# ==================== 第6步：重启服务 ====================
echo "[7/7] 重启服务..."
sudo systemctl restart "${SERVICE_NAME}"
sleep 2  # 等待服务启动
echo "  [OK] 服务已重启"
echo ""

# ==================== 第7步：显示状态 ====================
echo "[7/7] 查看服务状态..."
echo "============================================================"
sudo systemctl status "${SERVICE_NAME}" --no-pager
echo "============================================================"
echo ""

# 显示最近日志
echo "最近程序日志（最后20行）："
if [ -f "${LOG_FILE}" ]; then
    tail -n 20 "${LOG_FILE}"
else
    echo "  （日志文件尚未生成，请等待程序启动）"
fi
echo ""

echo "============================================================"
echo "  部署/更新完成！"
echo "  开机自启动：已启用"
echo "  程序日志文件：${LOG_FILE}"
echo "  查看实时日志：tail -f ${LOG_FILE}"
echo "  查看服务状态：sudo systemctl status ${SERVICE_NAME}"
echo "  查看服务日志：sudo journalctl -u ${SERVICE_NAME} -f"
echo "============================================================"
