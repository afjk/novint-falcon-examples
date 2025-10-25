#!/bin/bash
# Novint Falcon テスト実行スクリプト

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DYLD_LIBRARY_PATH="${SCRIPT_DIR}/libnifalcon/build/lib:$DYLD_LIBRARY_PATH"

FALCON_CLI="${SCRIPT_DIR}/libnifalcon/build/bin/falcon_test_cli"

echo "Novint Falcon テストプログラム"
echo "=============================="
echo ""
echo "利用可能なテスト:"
echo "  1) キューブテスト (立方体の触覚フィードバック)"
echo "  2) 球体テスト (球体の触覚フィードバック)"
echo "  3) 壁面テスト - X軸 (X軸方向の壁)"
echo "  4) 壁面テスト - Y軸 (Y軸方向の壁)"
echo "  5) 壁面テスト - Z軸 (Z軸方向の壁)"
echo "  6) カラーテスト (位置に応じたLED制御)"
echo "  7) ループタイムテスト (I/O性能テスト)"
echo "  8) デバイス数確認"
echo ""
echo "使用例:"
echo "  $0 cube          # キューブテスト"
echo "  $0 sphere        # 球体テスト"
echo "  $0 wall-x        # X軸壁面テスト"
echo "  $0 device-count  # デバイス数確認"
echo ""

case "$1" in
    cube)
        echo "キューブテストを開始します..."
        echo "注意: Falconをキャリブレーションしてください（完全に外側まで動かしてから、まっすぐ内側に押し込む）"
        exec "$FALCON_CLI" --nvent_firmware --cube_test
        ;;
    sphere)
        echo "球体テストを開始します..."
        echo "注意: Falconをキャリブレーションしてください（完全に外側まで動かしてから、まっすぐ内側に押し込む）"
        exec "$FALCON_CLI" --nvent_firmware --sphere_test
        ;;
    wall-x)
        echo "X軸壁面テストを開始します..."
        echo "注意: Falconをキャリブレーションしてください（完全に外側まで動かしてから、まっすぐ内側に押し込む）"
        exec "$FALCON_CLI" --nvent_firmware --x_wall_test
        ;;
    wall-y)
        echo "Y軸壁面テストを開始します..."
        echo "注意: Falconをキャリブレーションしてください（完全に外側まで動かしてから、まっすぐ内側に押し込む）"
        exec "$FALCON_CLI" --nvent_firmware --y_wall_test
        ;;
    wall-z)
        echo "Z軸壁面テストを開始します..."
        echo "注意: Falconをキャリブレーションしてください（完全に外側まで動かしてから、まっすぐ内側に押し込む）"
        exec "$FALCON_CLI" --nvent_firmware --z_wall_test
        ;;
    color)
        echo "カラーテストを開始します..."
        echo "注意: Falconをキャリブレーションしてください（完全に外側まで動かしてから、まっすぐ内側に押し込む）"
        exec "$FALCON_CLI" --nvent_firmware --color_test
        ;;
    loop)
        echo "ループタイムテストを開始します..."
        exec "$FALCON_CLI" --nvent_firmware --loop_time_test
        ;;
    device-count)
        echo "接続されているFalconデバイスを確認します..."
        exec "$FALCON_CLI" --device_count
        ;;
    *)
        if [ -z "$1" ]; then
            echo "テストを選択してください。"
        else
            echo "不明なテスト: $1"
        fi
        exit 1
        ;;
esac
