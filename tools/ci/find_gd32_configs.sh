#!/bin/bash
# GD32板级配置自动发现脚本
# 扫描boards/arm/gd32*目录下的所有配置

set -e

BOARDS_DIR="boards/arm"
CONFIG_FILE=""
OUTPUT_FORMAT="list"  # list, json, matrix

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --format)
            OUTPUT_FORMAT="$2"
            shift 2
            ;;
        --output)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --help|-h)
            echo "用法: $0 [options]"
            echo ""
            echo "选项:"
            echo "  --format <list|json|matrix>  输出格式 (默认: list)"
            echo "  --output <file>              输出到文件"
            echo "  --help, -h                   显示帮助"
            echo ""
            echo "示例:"
            echo "  $0 --format list             # 输出简单列表"
            echo "  $0 --format json             # 输出JSON数组"
            echo "  $0 --format matrix           # 输出GitHub Actions matrix格式"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            exit 1
            ;;
    esac
done

# 查找所有GD32板级配置
find_gd32_configs() {
    local configs=()

    # 遍历所有gd32*目录
    for chip_dir in "$BOARDS_DIR"/gd32*; do
        if [ ! -d "$chip_dir" ]; then
            continue
        fi

        # 遍历每个芯片下的板子
        for board_dir in "$chip_dir"/*; do
            if [ ! -d "$board_dir" ]; then
                continue
            fi

            # 获取板子名称
            board_name=$(basename "$board_dir")

            # 查找configs目录
            configs_dir="$board_dir/configs"
            if [ ! -d "$configs_dir" ]; then
                continue
            fi

            # 遍历所有配置
            for config_dir in "$configs_dir"/*; do
                if [ ! -d "$config_dir" ]; then
                    continue
                fi

                config_name=$(basename "$config_dir")
                # 格式: board_name:config_name
                configs+=("$board_name:$config_name")
            done
        done
    done

    echo "${configs[@]}"
}

# 输出配置列表
output_configs() {
    local configs=("$@")

    case $OUTPUT_FORMAT in
        list)
            # 简单列表，每行一个
            for config in "${configs[@]}"; do
                echo "$config"
            done
            ;;

        json)
            # JSON数组格式
            echo -n "["
            first=true
            for config in "${configs[@]}"; do
                if [ "$first" = true ]; then
                    first=false
                else
                    echo -n ","
                fi
                echo -n "\"$config\""
            done
            echo "]"
            ;;

        matrix)
            # GitHub Actions matrix格式
            echo "config<<EOF"
            for config in "${configs[@]}"; do
                echo "$config"
            done
            echo "EOF"
            ;;

        *)
            echo "错误: 未知格式 $OUTPUT_FORMAT" >&2
            exit 1
            ;;
    esac
}

# 主逻辑
configs=($(find_gd32_configs))

if [ ${#configs[@]} -eq 0 ]; then
    echo "警告: 未找到任何GD32配置" >&2
    exit 1
fi

# 输出结果
if [ -n "$CONFIG_FILE" ]; then
    output_configs "${configs[@]}" > "$CONFIG_FILE"
    echo "配置已保存到: $CONFIG_FILE" >&2
    echo "找到 ${#configs[@]} 个配置" >&2
else
    output_configs "${configs[@]}"
fi

# 显示统计信息到stderr（如果输出到文件）
if [ -n "$CONFIG_FILE" ]; then
    echo "" >&2
    echo "配置详情:" >&2
    for config in "${configs[@]}"; do
        echo "  - $config" >&2
    done
fi
