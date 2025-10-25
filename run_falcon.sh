#!/bin/bash
# スクリプトのディレクトリに移動
cd "$(dirname "$0")"
export DYLD_LIBRARY_PATH="./libnifalcon/build/lib:$DYLD_LIBRARY_PATH"

echo "Novint Falcon テストプログラム"
echo "1. マウスカーソル追従（XY軸・Z軸固定版）"
echo "2. マウスカーソル追従（XY軸フリー版）"
echo "3. マウスカーソル追従（XZ軸・Y軸フリー版）"
echo "4. マウスカーソル追従（XZ軸・Y軸固定版）"
echo "5. Y軸前後運動（高速版）"
echo "6. Z軸上下運動"
echo "7. 基本テスト"
echo "8. アクティブテスト"
echo ""
read -p "実行するプログラムを選択してください (1-8): " choice

case $choice in
    1)
        echo "マウスカーソル追従プログラム（XY軸・Z軸固定版）を実行します..."
        ./build/falcon_cursor_motion
        ;;
    2)
        echo "マウスカーソル追従プログラム（XY軸フリー版）を実行します..."
        ./build/falcon_cursor_xy
        ;;
    3)
        echo "マウスカーソル追従プログラム（XZ軸・Y軸フリー版）を実行します..."
        ./build/falcon_cursor_xz
        ;;
    4)
        echo "マウスカーソル追従プログラム（XZ軸・Y軸固定版）を実行します..."
        ./build/falcon_cursor_xz_fixed
        ;;
    5)
        echo "Y軸前後運動プログラムを実行します..."
        ./build/falcon_y_motion
        ;;
    6)
        echo "Z軸上下運動プログラムを実行します..."
        ./build/falcon_gentle_motion
        ;;
    7)
        echo "基本テストプログラムを実行します..."
        ./build/falcon_test
        ;;
    8)
        echo "アクティブテストプログラムを実行します..."
        ./build/falcon_active_test
        ;;
    *)
        echo "無効な選択です。"
        ;;
esac