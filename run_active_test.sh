#!/bin/bash
# Novint Falcon アクティブ力テスト実行スクリプト

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DYLD_LIBRARY_PATH="${SCRIPT_DIR}/libnifalcon/build/lib:$DYLD_LIBRARY_PATH"

echo "=========================================="
echo " Novint Falcon アクティブ力フィードバック"
echo "=========================================="
echo ""
echo "このプログラムでは、様々な力をリアルタイムで体験できます。"
echo ""

exec "${SCRIPT_DIR}/build/falcon_active_test"
