#!/usr/bin/env bash
# 代理开关：Clash 内核 + GNOME 系统代理
#   桌面按钮 代理-开启 / 代理-关闭 / 代理-状态 都调用本脚本。
#   on     启动 clash（优先用 systemd 用户服务 clash.service，没有则直接后台拉起）
#          并把 GNOME 系统代理指向 127.0.0.1:7890（浏览器等跟随系统代理的程序立即生效）
#   off    停止 clash + 关闭 GNOME 系统代理（此后浏览器直连）
#   status 显示 clash 进程 / 端口 / 系统代理 现状
#   toggle 在 on / off 之间切换
# 约定：clash 配置固定用 ~/clash/config.yaml（HTTP 7890 / SOCKS 7891 / 控制台 9090）
#       本脚本日志 ~/clash/proxy_ctl.log；clash 内核日志 ~/clash/clash.log
set -u

CLASH_DIR="$HOME/clash"
CLASH_BIN="$CLASH_DIR/clash"
SVC="clash.service"
HTTP_PORT=7890
SOCKS_PORT=7891
LOG="$CLASH_DIR/proxy_ctl.log"

log()    { echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
notify() { command -v notify-send >/dev/null 2>&1 && notify-send -a "代理开关" -i "$3" "$1" "$2" || true; }
have_svc() { systemctl --user cat "$SVC" >/dev/null 2>&1; }
# 精确匹配内核进程：先按进程名 comm=clash 定位，再用 /proc/<pid>/cmdline 确认配置目录。
# 不用 pgrep -f 通配命令行——那样会把"命令行里恰好含该字符串"的 shell/编辑器一起匹配上并误杀。
clash_pids() {
    for p in $(pgrep -x clash 2>/dev/null); do
        tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q -- "-d $CLASH_DIR" && echo "$p"
    done
}
port_up()    { ss -tln 2>/dev/null | grep -q "127.0.0.1:$HTTP_PORT "; }

proxy_set_on() {
    gsettings set org.gnome.system.proxy mode 'manual'
    gsettings set org.gnome.system.proxy.http  host '127.0.0.1'
    gsettings set org.gnome.system.proxy.http  port "$HTTP_PORT"
    gsettings set org.gnome.system.proxy.https host '127.0.0.1'
    gsettings set org.gnome.system.proxy.https port "$HTTP_PORT"
    gsettings set org.gnome.system.proxy.socks host '127.0.0.1'
    gsettings set org.gnome.system.proxy.socks port "$SOCKS_PORT"
    # 本机地址必须绕开代理，否则 codex login 回调(127.0.0.1:1455)/WebUI 会被代理掐断
    gsettings set org.gnome.system.proxy ignore-hosts "['localhost', '127.0.0.0/8', '::1']"
}
proxy_set_off() { gsettings set org.gnome.system.proxy mode 'none'; }

do_on() {
    if [ -n "$(clash_pids)" ]; then
        log "clash 已在运行：pid=$(clash_pids | tr '\n' ' ')"
    elif have_svc; then
        log "启动 systemd 用户服务 $SVC ..."
        systemctl --user start "$SVC" || { log "❌ systemctl start 失败"; notify "❌ 代理开启失败" "systemctl 启动 $SVC 失败，详见 $LOG" "dialog-error"; exit 1; }
    else
        log "无 systemd 服务，直接后台拉起 clash ..."
        ( cd "$CLASH_DIR" && setsid nohup "$CLASH_BIN" -d "$CLASH_DIR" >>"$CLASH_DIR/clash.log" 2>&1 & )
    fi
    for _ in $(seq 1 30); do port_up && break; sleep 0.5; done
    if port_up; then
        proxy_set_on
        log "✅ 代理已开启：http://127.0.0.1:$HTTP_PORT（SOCKS $SOCKS_PORT / 控制台 9090）"
        notify "✅ 代理已开启" "Clash 运行中，系统代理 → 127.0.0.1:$HTTP_PORT" "network-workgroup"
    else
        log "❌ clash 未能在 15s 内监听 $HTTP_PORT"
        notify "❌ 代理开启失败" "clash 没起来（端口 $HTTP_PORT 未监听），详见 $LOG" "dialog-error"
        exit 1
    fi
}

do_off() {
    if have_svc && systemctl --user is-active --quiet "$SVC"; then
        log "停止 systemd 用户服务 $SVC ..."
        systemctl --user stop "$SVC"
    fi
    PIDS="$(clash_pids)"                      # 兜底：清掉非 systemd 启动的残留进程（按 pid 精确 kill，不用 pkill）
    if [ -n "$PIDS" ]; then
        log "关闭残留 clash 进程：$(echo "$PIDS" | tr '\n' ' ')"
        for p in $PIDS; do kill "$p" 2>/dev/null || true; done
        sleep 2
        for p in $(clash_pids); do kill -9 "$p" 2>/dev/null || true; done
    fi
    proxy_set_off
    log "✅ 代理已关闭（clash 已停，系统代理=无）"
    notify "✅ 代理已关闭" "Clash 已停止，浏览器改为直连" "network-offline"
}

do_status() {
    if [ -n "$(clash_pids)" ]; then RUN="运行中 pid=$(clash_pids | tr '\n' ' ')"
    elif have_svc && systemctl --user is-active --quiet "$SVC"; then RUN="运行中（systemd）"
    else RUN="未运行"; fi
    if port_up; then PORT=":$HTTP_PORT 已监听"; else PORT=":$HTTP_PORT 未监听"; fi
    MODE="$(gsettings get org.gnome.system.proxy mode 2>/dev/null || echo '?')"
    MSG="clash: $RUN
端口: $PORT
系统代理: $MODE
自启动: $(have_svc && systemctl --user is-enabled "$SVC" 2>/dev/null || echo '未配置')"
    log "---- 状态 ----"; echo "$MSG" | tee -a "$LOG"
    notify "代理状态" "$MSG" "dialog-information"
}

case "${1:-status}" in
    on)     do_on ;;
    off)    do_off ;;
    status) do_status ;;
    toggle) if [ -n "$(clash_pids)" ]; then do_off; else do_on; fi ;;
    *) echo "用法: $0 {on|off|status|toggle}"; exit 2 ;;
esac
