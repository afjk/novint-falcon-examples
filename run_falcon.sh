#!/bin/bash
# スクリプトのディレクトリに移動
cd "$(dirname "$0")"
export DYLD_LIBRARY_PATH="./libnifalcon/build/lib:$DYLD_LIBRARY_PATH"

echo "Novint Falcon テストプログラム"
echo "0. 初期化・ファームウェアロードのみ"
echo "1. マウスカーソル追従（XY軸・Z軸固定版）"
echo "2. マウスカーソル追従（XY軸フリー版）"
echo "3. マウスカーソル追従（XZ軸・Y軸フリー版）"
echo "4. マウスカーソル追従（XZ軸・Y軸固定版）"
echo "5. Y軸上下運動"
echo "6. Z軸上下運動"
echo "7. 基本テスト"
echo "8. アクティブテスト"
echo "9. 記録と再生"
echo "10. ざらざらテクスチャテスト（キューブ）"
echo "11. もちもち球体テスト（お餅のようなぷにぷに）"
echo ""
read -p "実行するプログラムを選択してください (0-11): " choice

case $choice in
    0)
        echo "初期化・ファームウェアロード処理を実行します..."
        ./build/falcon_init_firmware
        ;;
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
        echo "Y軸上下運動プログラムを実行します..."
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
    9)
        echo "記録と再生プログラムを実行します..."
        ./build/falcon_record_play
        ;;
    10)
        echo "ざらざらテクスチャテストプログラムを実行します..."
        ./build/falcon_texture_test
        ;;
    11)
        echo "もちもち球体テストプログラムを実行します..."
        ./build/falcon_viscous_sphere
        ;;
    *)
        echo "無効な選択です。"
        ;;
esac