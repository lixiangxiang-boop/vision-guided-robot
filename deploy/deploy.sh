#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# 视觉引导机器人操控系统 — 一键部署脚本
# ═══════════════════════════════════════════════════════════════
# 在服务器上运行此脚本即可完成部署:
#   curl -O https://your-server/deploy/deploy.sh
#   chmod +x deploy.sh && ./deploy.sh
#
# 或手动执行:
#   bash deploy/deploy.sh /www/wwwroot/your-domain.com your-domain.com
# ═══════════════════════════════════════════════════════════════

set -e

# ─── 参数 ───
SITE_ROOT="${1:-/www/wwwroot/robot.example.com}"
DOMAIN="${2:-robot.example.com}"

echo "════════════════════════════════════════"
echo "  视觉引导机器人部署"
echo "  网站路径: $SITE_ROOT"
echo "  域名:     $DOMAIN"
echo "════════════════════════════════════════"

# ─── 1. 创建目录结构 ───
echo "[1/6] 创建目录..."
mkdir -p "$SITE_ROOT"/{web,services,agent,data,logs,models,deploy}
mkdir -p "$SITE_ROOT/web"/{css,js,models/convex,models/low-res}

# ─── 2. 复制文件 ───
echo "[2/6] 复制项目文件..."
SRC="$(cd "$(dirname "$0")/.." && pwd)"
cp -r "$SRC/web"/* "$SITE_ROOT/web/"
cp -r "$SRC/services"/* "$SITE_ROOT/services/"
cp -r "$SRC/agent"/* "$SITE_ROOT/agent/"
cp "$SRC/requirements.txt" "$SITE_ROOT/"
cp "$SRC/yolov8n.pt" "$SITE_ROOT/" 2>/dev/null || echo "    提示: yolov8n.pt 将在首次运行时自动下载"
cp -r "$SRC/data"/* "$SITE_ROOT/data/" 2>/dev/null || true
cp -r "$SRC/models" "$SITE_ROOT/" 2>/dev/null || true
# 不复制 models/lrmate-200id/ (第三方源码，非运行时需要)

# ─── 3. 配置 .env ───
echo "[3/6] 配置环境变量..."
if [ ! -f "$SITE_ROOT/.env" ]; then
    cp "$SRC/deploy/.env.production" "$SITE_ROOT/.env"
    echo "    已创建 $SITE_ROOT/.env，请编辑填入 DEEPSEEK_API_KEY"
else
    echo "    .env 已存在，跳过"
fi

# ─── 4. 安装 Python 依赖 ───
echo "[4/6] 安装 Python 依赖..."
cd "$SITE_ROOT"
if command -v pip3 &> /dev/null; then
    pip3 install -r requirements.txt -q
elif command -v pip &> /dev/null; then
    pip install -r requirements.txt -q
else
    echo "    ⚠ 未找到 pip，请手动安装: pip install -r requirements.txt"
fi

# ─── 5. 配置 Nginx ───
echo "[5/6] 配置 Nginx (宝塔)..."
NGINX_CONF="/www/server/panel/vhost/nginx/${DOMAIN}.conf"
if [ -f "$NGINX_CONF" ]; then
    echo "    检测到宝塔 Nginx 配置: $NGINX_CONF"
    echo "    请手动将 deploy/nginx-baota.conf 中的配置合并进去"
    echo "    或粘贴以下路径到浏览器查看:"
    echo "    file://$SRC/deploy/nginx-baota.conf"
else
    # 标准 Nginx
    NGINX_DIR="/etc/nginx/sites-enabled"
    if [ -d "$NGINX_DIR" ]; then
        sed -e "s|YOUR_DOMAIN.COM|$DOMAIN|g" \
            -e "s|/www/wwwroot/YOUR_DOMAIN.COM|$SITE_ROOT|g" \
            "$SRC/deploy/nginx-baota.conf" > "/tmp/vision-robot-nginx.conf"
        echo "    Nginx 配置已生成: /tmp/vision-robot-nginx.conf"
        echo "    请手动复制到 $NGINX_DIR 并重启 Nginx"
    fi
fi

# ─── 6. 完成 ───
echo "[6/6] 部署完成！"
echo ""
echo "════════════════════════════════════════"
echo "  后续步骤:"
echo ""
echo "  1. 编辑 .env 文件:"
echo "     vim $SITE_ROOT/.env"
echo "     填入 DEEPSEEK_API_KEY"
echo ""
echo "  2. 配置 Nginx 反向代理:"
echo "     参考: $SRC/deploy/nginx-baota.conf"
echo ""
echo "  3. 启动服务:"
echo "     bash $SITE_ROOT/deploy/start-server.sh start"
echo ""
echo "  4. 配置进程守护 (宝塔):"
echo "     软件商店 → Supervisor → 添加守护进程"
echo "     参考: $SRC/deploy/supervisor.ini"
echo ""
echo "  5. 访问网站:"
echo "     http://$DOMAIN"
echo "════════════════════════════════════════"
