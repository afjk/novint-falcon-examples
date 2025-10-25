#include <iostream>
#include <cmath>
#include <array>
#include <chrono>
#include <thread>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/comm/FalconCommLibUSB.h>

// macOSのマウスカーソル位置取得用
#include <CoreGraphics/CoreGraphics.h>

using namespace libnifalcon;

// マウス座標をFalcon座標に変換する関数
std::pair<double, double> convertMouseToFalcon(CGPoint mousePos, CGSize screenSize) {
    // スクリーン座標系をFalcon座標系に変換
    // マウス座標（0-screen width/height）をFalcon座標（-0.05～+0.05m）にマッピング
    double falconX = ((mousePos.x / screenSize.width) - 0.5) * 0.1;   // -0.05～+0.05m
    double falconY = ((mousePos.y / screenSize.height) - 0.5) * 0.1;  // -0.05～+0.05m
    
    // Y軸を反転（スクリーン座標系とFalcon座標系の違いを調整）
    falconY = -falconY;
    
    return std::make_pair(falconX, falconY);
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  マウスカーソル追従プログラム" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Falconデバイスの初期化
    FalconDevice falcon;
    falcon.setFalconComm<FalconCommLibUSB>();

    unsigned int count;
    falcon.getDeviceCount(count);
    std::cout << "検出されたFalconデバイス: " << count << std::endl;

    if (count == 0) {
        std::cout << "エラー: Falconデバイスが見つかりません。" << std::endl;
        return 1;
    }

    std::cout << "デバイスを開いています..." << std::endl;
    if (!falcon.open(0)) {
        std::cout << "エラー: デバイスを開けません。" << std::endl;
        return 1;
    }

    std::cout << "ファームウェアをロード中..." << std::endl;
    falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
    if (!falcon.loadFirmware(10, false)) {
        std::cout << "警告: ファームウェアのロードに失敗しました。" << std::endl;
    }

    falcon.setFalconKinematic<FalconKinematicStamper>();

    // 制御パラメータ
    const double zFixed = 0.10;          // Z軸固定位置（高さ10cm）
    const double springConstant = 150.0; // バネ定数

    // PID制御パラメータ（振動抑制のため調整）
    const double Kp = 100.0;             // 比例ゲイン（振動抑制のため大幅減少）
    const double Ki = 5.0;               // 積分ゲイン（振動抑制のため減少）
    const double Kd = 8.0;               // 微分ゲイン（振動抑制のため減少）
    const double integralLimit = 0.005;  // 積分項の上限（より厳しく制限）
    
    // ローパスフィルタパラメータ（振動除去用）
    const double alpha = 0.1;            // フィルタ係数（0.1 = 10%の新データ、90%の古いデータ）

    // PID制御用変数
    std::array<double, 3> lastPos = {0.0, 0.0, 0.0};
    double xErrorIntegral = 0.0;
    double yErrorIntegral = 0.0;
    double zErrorIntegral = 0.0;
    
    // フィルタリング用変数
    double filteredTargetX = 0.0;
    double filteredTargetY = 0.0;

    // スクリーンサイズを取得
    CGDirectDisplayID displayID = CGMainDisplayID();
    CGSize screenSize = CGDisplayBounds(displayID).size;

    std::cout << std::endl;
    std::cout << "マウスカーソル追従制御を開始します（振動抑制版）。" << std::endl;
    std::cout << "スクリーンサイズ: " << screenSize.width << " x " << screenSize.height << std::endl;
    std::cout << "Falconグリップがマウスカーソルの位置にスムーズに追従します。" << std::endl;
    std::cout << "Z軸は" << (zFixed * 1000) << "mmに固定されます。" << std::endl;
    std::cout << "制御ゲイン: Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << std::endl;
    std::cout << "ローパスフィルタ係数: " << alpha << " (振動抑制)" << std::endl;
    std::cout << "Ctrl+C で終了してください。" << std::endl;
    std::cout << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();
    int loopCount = 0;

    while (true) {
        if (!falcon.runIOLoop()) {
            continue;
        }

        // 現在のマウスカーソル位置を取得
        CGEventRef event = CGEventCreate(NULL);
        CGPoint mousePos = CGEventGetLocation(event);
        CFRelease(event);

        // マウス座標をFalcon座標に変換
        auto [targetX, targetY] = convertMouseToFalcon(mousePos, screenSize);
        
        // ローパスフィルタを適用（急激な変化を抑制）
        filteredTargetX = alpha * targetX + (1.0 - alpha) * filteredTargetX;
        filteredTargetY = alpha * targetY + (1.0 - alpha) * filteredTargetY;

        // 現在の位置を取得
        auto pos = falcon.getPosition();

        // 経過時間を取得（秒）
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
        double elapsedSeconds = duration / 1000.0;

        // X軸のPID制御（フィルタリング済み目標値を使用）
        double xError = filteredTargetX - pos[0];
        double xVelocity = (pos[0] - lastPos[0]) * 1000.0;
        xErrorIntegral += xError / 1000.0;
        if (xErrorIntegral > integralLimit) xErrorIntegral = integralLimit;
        if (xErrorIntegral < -integralLimit) xErrorIntegral = -integralLimit;
        double xForce = Kp * xError + Ki * xErrorIntegral - Kd * xVelocity;

        // Y軸のPID制御（フィルタリング済み目標値を使用）
        double yError = filteredTargetY - pos[1];
        double yVelocity = (pos[1] - lastPos[1]) * 1000.0;
        yErrorIntegral += yError / 1000.0;
        if (yErrorIntegral > integralLimit) yErrorIntegral = integralLimit;
        if (yErrorIntegral < -integralLimit) yErrorIntegral = -integralLimit;
        double yForce = Kp * yError + Ki * yErrorIntegral - Kd * yVelocity;

        // Z軸のPID制御（固定位置）
        double zError = zFixed - pos[2];
        double zVelocity = (pos[2] - lastPos[2]) * 1000.0;
        zErrorIntegral += zError / 1000.0;
        if (zErrorIntegral > integralLimit) zErrorIntegral = integralLimit;
        if (zErrorIntegral < -integralLimit) zErrorIntegral = -integralLimit;
        double zForce = Kp * zError + Ki * zErrorIntegral - Kd * zVelocity;

        // 力の制限（安全のため最大力を制限）
        const double maxForce = 2.0;  // 最大2N
        xForce = std::max(-maxForce, std::min(maxForce, xForce));
        yForce = std::max(-maxForce, std::min(maxForce, yForce));
        zForce = std::max(-maxForce, std::min(maxForce, zForce));

        // 力を設定
        std::array<double, 3> force = {xForce, yForce, zForce};
        falcon.setForce(force);

        // 現在の位置を記憶
        lastPos = pos;

        // 1秒ごとに状態を表示
        loopCount++;
        if (loopCount % 1000 == 0) {
            double totalError = std::sqrt(xError*xError + yError*yError);

            std::cout << "時間: " << elapsedSeconds << "秒 | "
                      << "マウス: (" << (int)mousePos.x << ", " << (int)mousePos.y << ") | "
                      << "目標XY: (" << (filteredTargetX * 1000) << ", " << (filteredTargetY * 1000) << ")mm | "
                      << "実際XY: (" << (pos[0] * 1000) << ", " << (pos[1] * 1000) << ")mm | "
                      << "誤差: " << (totalError * 1000) << "mm | "
                      << "Z: " << (pos[2] * 1000) << "mm"
                      << std::endl;
        }
    }

    falcon.close();
    return 0;
}