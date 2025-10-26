#!/bin/bash
# スクリプトのディレクトリに移動
cd "$(dirname "$0")"
export DYLD_LIBRARY_PATH="./libnifalcon/build/lib:$DYLD_LIBRARY_PATH"

echo "Novint Falcon テストプログラム"
echo "=============================="
echo ""

# キャリブレーション状態を確認
echo "デバイスの状態を確認しています..."
./build/check_calibration 2>/dev/null
calibration_status=$?

if [ $calibration_status -eq 2 ]; then
    echo "エラー: Falconデバイスが見つからないか、接続できません。"
    exit 1
elif [ $calibration_status -eq 1 ]; then
    echo ""
    echo "キャリブレーションが必要です。"
    echo "キャリブレーションを実行します..."
    echo ""
    echo "注意: Falconをキャリブレーションしてください"
    echo "      （完全に外側まで動かしてから、まっすぐ内側に押し込む）"
    echo ""
    ./libnifalcon/build/bin/falcon_test_cli --nvent_firmware --loop_time_test

    # キャリブレーション後の状態を確認
    ./build/check_calibration 2>/dev/null
    if [ $? -ne 0 ]; then
        echo ""
        echo "警告: キャリブレーションが完了していない可能性があります。"
        echo "      プログラム実行前に手動でキャリブレーションを行ってください。"
        echo ""
    else
        echo ""
        echo "キャリブレーション完了！"
        echo ""
    fi
else
    echo "キャリブレーション済みです。"
    echo ""
fi

echo "=============================="
echo "0. キャリブレーション（再実行）"
echo "1. マウスカーソル追従（XY軸・Z軸固定版）"
echo "2. マウスカーソル追従（XY軸フリー版）"
echo "3. マウスカーソル追従（XZ軸・Y軸フリー版）"
echo "4. マウスカーソル追従（XZ軸・Y軸固定版）"
echo "5. Y軸上下運動"
echo "6. Z軸上下運動"
echo "7. 基本テスト"
echo "8. アクティブテスト"
echo "9. 記録と再生"
echo ""
read -p "実行するプログラムを選択してください (0-9): " choice

case $choice in
    0)
        echo "キャリブレーションを再実行します..."
        echo "注意: Falconをキャリブレーションしてください（完全に外側まで動かしてから、まっすぐ内側に押し込む）"
        ./libnifalcon/build/bin/falcon_test_cli --nvent_firmware --loop_time_test
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
    *)
        echo "無効な選択です。"
        ;;
esac