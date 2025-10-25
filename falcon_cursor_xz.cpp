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

// マウス座標をFalcon座標に変換する関数（XZ軸版）
std::pair<double, double> convertMouseToFalconXZ(CGPoint mousePos, CGSize screenSize) {
    // スクリーン座標系をFalcon座標系に変換
    // マウスX座標（0-screen width）をFalconX座標（-0.06～+0.06m）にマッピング
    double falconX = ((mousePos.x / screenSize.width) - 0.5) * 0.12;   // -0.06～+0.06m
    
    // マウスY座標（0-screen height）をFalconZ座標（0.05～0.15m）にマッピング
    // スクリーン上部（Y=0）→高い位置（Z=0.15m）、下部→低い位置（Z=0.05m）
    double falconZ = 0.15 - (mousePos.y / screenSize.height) * 0.10;   // 0.05～0.15m
    
    return std::make_pair(falconX, falconZ);
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  マウスカーソル追従プログラム（XZ軸版）" << std::endl;
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

    // 制御パラメータ（XZ軸版）
    const double springConstant = 50.0;  // Y軸のバネ定数（弱めに設定）

    // PID制御パラメータ（XZ軸専用調整）
    const double Kp = 85.0;              // 比例ゲイン（XZ軸用に調整）
    const double Ki = 4.0;               // 積分ゲイン（適度に設定）
    const double Kd = 7.0;               // 微分ゲイン（振動抑制）
    const double integralLimit = 0.004;  // 積分項の上限（厳しく制限）
    
    // ローパスフィルタパラメータ（振動除去用）
    const double alpha = 0.12;           // フィルタ係数（XZ軸用に調整）

    // PID制御用変数
    std::array<double, 3> lastPos = {0.0, 0.0, 0.0};
    double xErrorIntegral = 0.0;
    double zErrorIntegral = 0.0;
    
    // フィルタリング用変数
    double filteredTargetX = 0.0;
    double filteredTargetZ = 0.10;  // Z軸の初期値

    // スクリーンサイズを取得
    CGDirectDisplayID displayID = CGMainDisplayID();
    CGSize screenSize = CGDisplayBounds(displayID).size;

    std::cout << std::endl;
    std::cout << "マウスカーソル追従制御を開始します（XZ軸版）。" << std::endl;
    std::cout << "スクリーンサイズ: " << screenSize.width << " x " << screenSize.height << std::endl;
    std::cout << "Falconグリップがマウスカーソルの位置にXZ平面で追従します。" << std::endl;
    std::cout << "X軸: マウスの左右移動、Z軸: マウスの上下移動（高さ変化）" << std::endl;
    std::cout << "Y軸は自由に動かせます（弱いバネ力で中央に戻ろうとします）。" << std::endl;
    std::cout << "制御ゲイン: Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << std::endl;
    std::cout << "ローパスフィルタ係数: " << alpha << " (振動抑制)" << std::endl;
    std::cout << "X可動範囲: ±60mm, Z可動範囲: 50-150mm" << std::endl;
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

        // マウス座標をFalcon座標に変換（XZ軸）
        auto [targetX, targetZ] = convertMouseToFalconXZ(mousePos, screenSize);
        
        // ローパスフィルタを適用（急激な変化を抑制）
        filteredTargetX = alpha * targetX + (1.0 - alpha) * filteredTargetX;
        filteredTargetZ = alpha * targetZ + (1.0 - alpha) * filteredTargetZ;

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

        // Y軸は弱いバネ力で中央（0mm）に戻す
        double yTarget = 0.0;  // 中央
        double yForce = -springConstant * (pos[1] - yTarget);

        // Z軸のPID制御（フィルタリング済み目標値を使用）
        double zError = filteredTargetZ - pos[2];
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
            double xzError = std::sqrt(xError*xError + zError*zError);

            std::cout << "時間: " << elapsedSeconds << "秒 | "
                      << "マウス: (" << (int)mousePos.x << ", " << (int)mousePos.y << ") | "
                      << "目標XZ: (" << (filteredTargetX * 1000) << ", " << (filteredTargetZ * 1000) << ")mm | "
                      << "実際XYZ: (" << (pos[0] * 1000) << ", " << (pos[1] * 1000) << ", " << (pos[2] * 1000) << ")mm | "
                      << "XZ誤差: " << (xzError * 1000) << "mm"
                      << std::endl;
        }
    }

    falcon.close();
    return 0;
}