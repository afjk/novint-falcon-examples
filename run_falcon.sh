#!/bin/bash
# スクリプトのディレクトリに移動
cd "$(dirname "$0")"
export DYLD_LIBRARY_PATH="./libnifalcon/build/lib:$DYLD_LIBRARY_PATH"

echo "Novint Falcon テストプログラム"
echo ""
echo "【初期化】"
echo "0. 初期化・ファームウェアロード"
echo ""
echo "【カーソル追従制御】"
echo "1. マウスカーソル追従（XY軸・Z軸固定版）"
echo "2. マウスカーソル追従（XY軸フリー版）"
echo "3. マウスカーソル追従（XZ軸・Y軸固定版）"
echo "4. マウスカーソル追従（XZ軸・Y軸フリー版）"
echo ""
echo "【基本動作】"
echo "5. Y軸上下運動"
echo "6. Z軸上下運動"
echo ""
echo "【デバイステスト】"
echo "7. 基本テスト"
echo "8. アクティブテスト"
echo ""
echo "【記録・再生】"
echo "9. 記録と再生"
echo ""
echo "【触覚シミュレーション】"
echo "10. ざらざらテクスチャテスト（キューブ）"
echo "11. もちもち球体テスト（お餅のようなぷにぷに）"
echo "12. つるつる球体テスト（氷・ガラス）"
echo "13. ふわふわ球体テスト（クッション・綿）"
echo "14. ぬるぬる球体テスト（スライム・ジェル）"
echo "15. バネ球体テスト（ゴムボール）"
echo ""
read -p "実行するプログラムを選択してください (0-15): " choice

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
        echo "マウスカーソル追従プログラム（XZ軸・Y軸固定版）を実行します..."
        ./build/falcon_cursor_xz_fixed
        ;;
    4)
        echo "マウスカーソル追従プログラム（XZ軸・Y軸フリー版）を実行します..."
        ./build/falcon_cursor_xz
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
    12)
        echo "つるつる球体テストプログラムを実行します..."
        ./build/falcon_slippery_sphere
        ;;
    13)
        echo "ふわふわ球体テストプログラムを実行します..."
        ./build/falcon_fluffy_sphere
        ;;
    14)
        echo "ぬるぬる球体テストプログラムを実行します..."
        ./build/falcon_viscous_goo
        ;;
    15)
        echo "バネ球体テストプログラムを実行します..."
        ./build/falcon_spring_sphere
        ;;
    *)
        echo "無効な選択です。"
        ;;
esac