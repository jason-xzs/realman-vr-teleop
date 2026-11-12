#!/bin/bash
# 快速切换旋转映射方案

CONFIG_FILE="$(dirname "$0")/../config/teleop_params.yaml"

if [ -z "$1" ]; then
    echo "用法: $0 <preset_name>"
    echo ""
    echo "可用方案:"
    echo "  current, hmi, rotation_matrix"
    echo "  current_npp, current_pnp, current_ppn, ..."
    echo "  hmi_npp, hmi_pnp, hmi_ppn, ..."
    exit 1
fi

PRESET="$1"

# 使用 sed 替换配置文件中的 rotation_preset
sed -i "s/rotation_preset: \".*\"/rotation_preset: \"$PRESET\"/" "$CONFIG_FILE"

echo "已切换到方案: $PRESET"
echo "请重启 vr_teleop_node 使配置生效"
