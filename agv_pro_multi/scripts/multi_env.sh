#!/usr/bin/env bash
set -euo pipefail

# 配置多车 Fast DDS Discovery Server 环境变量 / Configure Fast DDS Discovery Server env for multi-robot.
# robot1 起 server, follower 指向它 / robot1 hosts the server, followers point to it.
# Usage: ./multi_env.sh [robot1|follower|server|del]    # default server(只起Server不改bashrc); robot1=主车配bashrc+起server, follower=从车, del=清除退回组播

BASHRC="$HOME/.bashrc"
IFACE="wlP1p1s0"
PORT="11811"
DOMAIN="42"
MB="# >>> multi discovery >>>"
ME="# <<< multi discovery <<<"

if [ -t 1 ]; then
    C_OK=$'\033[0;32m'; C_ERR=$'\033[0;31m'; C_WARN=$'\033[1;33m'; C_NOTE=$'\033[0;36m'; C_HDR=$'\033[1;36m'; C_RST=$'\033[0m'
else
    C_OK=; C_ERR=; C_WARN=; C_NOTE=; C_HDR=; C_RST=
fi

ensure_trailing_newline() {
    if [ -s "$BASHRC" ] && [ -n "$(tail -c1 "$BASHRC")" ]; then
        printf '\n' >> "$BASHRC"
    fi
}

set_or_replace_var() {
    local name="$1" val="$2"
    if grep -qE "^[[:space:]]*export[[:space:]]+$name=" "$BASHRC"; then
        sed -i "s#^[[:space:]]*export[[:space:]]\+$name=.*#export $name=$val#" "$BASHRC"
    else
        ensure_trailing_newline
        echo "export $name=$val" >> "$BASHRC"
    fi
}

remove_var() {
    sed -i "/^[[:space:]]*export[[:space:]]\+$1=/d" "$BASHRC" 2>/dev/null || true
}

clean_discovery() {
    sed -i '/# >>> .*discovery.*>>>/d' "$BASHRC" 2>/dev/null || true
    sed -i '/# <<< .*discovery.*<<</d' "$BASHRC" 2>/dev/null || true
    remove_var ROS_DISCOVERY_SERVER
    remove_var ROS_SUPER_CLIENT
}

write_bashrc() {
    local ip="$1"
    set_or_replace_var ROS_DOMAIN_ID "$DOMAIN"
    clean_discovery
    ensure_trailing_newline
    {
        echo "$MB"
        echo "export ROS_DISCOVERY_SERVER=\"$ip:$PORT\""
        echo "$ME"
    } >> "$BASHRC"
    echo "${C_OK}[OK]${C_RST} Updated ~/.bashrc:"
    echo "     ROS_DOMAIN_ID=$DOMAIN"
    echo "     ROS_DISCOVERY_SERVER=\"$ip:$PORT\""
    echo "${C_NOTE}[Note] Run 'source ~/.bashrc' or reopen the terminal to apply.${C_RST}"
}

get_iface_ip() {
    ip -4 -o addr show "$IFACE" 2>/dev/null | awk 'NR==1{split($4,a,"/"); print a[1]}'
}

start_server() {
    if ! command -v fastdds >/dev/null 2>&1; then
        echo "${C_ERR}[x]${C_RST} fastdds not found, source ROS2 first."
        exit 1
    fi
    echo "${C_HDR}Starting Discovery Server (foreground, keep this terminal open)...${C_RST}"
    echo "----------------------------------------------------------------"
    exec fastdds discovery -i 0 -l 0.0.0.0 -p "$PORT"
}

ARG="${1:-server}"

case "$ARG" in
del)
    clean_discovery
    echo "${C_OK}[OK]${C_RST} Removed ROS_DISCOVERY_SERVER and ROS_SUPER_CLIENT from ~/.bashrc (back to multicast discovery)."
    echo "${C_NOTE}[Note] Run 'source ~/.bashrc' or reopen the terminal to apply.${C_RST}"
    ;;
server)
    echo "${C_HDR}======== [server] Discovery Server only ========${C_RST}"
    start_server
    ;;
robot1)
    echo "${C_HDR}======== [robot1] leader = Discovery Server ========${C_RST}"
    IP="$(get_iface_ip)"
    if [ -z "$IP" ]; then
        echo "${C_WARN}[!]${C_RST} Failed to auto-detect IP on $IFACE"
        read -rp "    Enter this machine's IP: " IP
    else
        echo "${C_OK}[OK]${C_RST} local IP of $IFACE: $IP"
    fi
    [ -z "$IP" ] && { echo "${C_ERR}[x]${C_RST} IP empty, aborted."; exit 1; }
    write_bashrc "$IP"
    start_server
    ;;
follower)
    echo "${C_HDR}======== [follower] -> robot1's Server ========${C_RST}"
    read -rp "Enter robot1 (leader) IP: " IP
    [ -z "$IP" ] && { echo "${C_ERR}[x]${C_RST} IP required, aborted."; exit 1; }
    write_bashrc "$IP"
    ;;
*)
    echo "${C_ERR}[x]${C_RST} Unknown argument: $ARG"
    echo "    Usage: $0 [robot1|follower|server|del]"
    exit 1
    ;;
esac
