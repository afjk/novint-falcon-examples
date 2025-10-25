#include <iostream>
#include <cmath>
#include <array>
#include <chrono>
#include <thread>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/comm/FalconCommLibUSB.h>

using namespace libnifalcon;

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  スムーズなY軸前後運動プログラム" << std::endl;
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

    // スムーズなY軸前後運動のパラメータ
    const double period = 1.0;           // 周期（秒）：1秒で1往復（速度2倍）
    const double yAmplitude = 0.025;     // Y軸の移動範囲（メートル）：25mm
    const double frequency = 1.0 / period; // 周波数
    const double centeringForce = 100.0;  // X, Z軸を中心に固定する力の強さ
    const double yCenter = 0.0;          // Y軸の中心位置（前後の中央）

    // PID+フィードフォワード制御パラメータ
    const double Kp = 200.0;             // 比例ゲイン
    const double Ki = 50.0;              // 積分ゲイン
    const double Kd = 15.0;              // 微分ゲイン
    const double Kff = 60.0;             // フィードフォワードゲイン（高速化のため2倍）
    const double integralLimit = 0.01;   // 積分項の上限（飽和防止）
    const double phaseCompensation = 0.35; // 位相補償（ラジアン）：約20度先読み

    // PID制御用変数
    std::array<double, 3> lastPos = {0.0, 0.0, 0.0};
    double yErrorIntegral = 0.0;         // Y軸誤差の積分

    std::cout << std::endl;
    std::cout << "スムーズなY軸前後運動のPID+フィードフォワード+位相補償制御を開始します。" << std::endl;
    std::cout << "グリップが目標位置に向かって高速で滑らかに前後に動きます。" << std::endl;
    std::cout << "周期: " << period << "秒、振幅: ±" << (yAmplitude * 1000) << "mm" << std::endl;
    std::cout << "範囲: " << ((yCenter - yAmplitude) * 1000) << "-" << ((yCenter + yAmplitude) * 1000) << "mm" << std::endl;
    std::cout << "制御ゲイン: Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << ", Kff=" << Kff << std::endl;
    std::cout << "位相補償: " << phaseCompensation << " rad (" << (phaseCompensation * 180.0 / M_PI) << " deg)" << std::endl;
    std::cout << "Ctrl+C で終了してください。" << std::endl;
    std::cout << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();
    int loopCount = 0;

    while (true) {
        if (!falcon.runIOLoop()) {
            continue;
        }

        // 位置を取得
        auto pos = falcon.getPosition();

        // 経過時間を取得（秒）
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
        double elapsedSeconds = duration / 1000.0;

        // 正弦波を使った目標位置と目標速度を計算（Y軸方向）
        // 位置: sin(2π * frequency * time + phaseCompensation)
        // 速度: 2π * frequency * amplitude * cos(2π * frequency * time + phaseCompensation)
        double phase = 2.0 * M_PI * frequency * elapsedSeconds;
        double compensatedPhase = phase + phaseCompensation;  // 位相補償を適用
        double targetY = yCenter + yAmplitude * std::sin(compensatedPhase);
        double targetVelocity = yAmplitude * 2.0 * M_PI * frequency * std::cos(compensatedPhase);

        // X, Z軸を中心に固定するバネ力を計算
        double xCenteringForce = -centeringForce * pos[0];
        double zCenteringForce = -centeringForce * (pos[2] - 0.10); // Z軸は高さ10cmを中心に固定

        // Y軸のPID+フィードフォワード制御
        double yError = targetY - pos[1];                    // 誤差（P項）
        double yVelocity = (pos[1] - lastPos[1]) * 1000.0;  // 速度（D項用）

        // 積分項の更新（飽和防止付き）
        yErrorIntegral += yError / 1000.0;  // 1000Hzで積分
        if (yErrorIntegral > integralLimit) yErrorIntegral = integralLimit;
        if (yErrorIntegral < -integralLimit) yErrorIntegral = -integralLimit;

        // PID+フィードフォワード制御則
        double yForceP = Kp * yError;              // 比例項
        double yForceI = Ki * yErrorIntegral;      // 積分項
        double yForceD = -Kd * yVelocity;          // 微分項
        double yForceFf = Kff * targetVelocity;    // フィードフォワード項
        double yForce = yForceP + yForceI + yForceD + yForceFf;

        // 力を設定（Y軸制御、X/Z軸固定）
        std::array<double, 3> force = {xCenteringForce, yForce, zCenteringForce};
        falcon.setForce(force);

        // 現在の位置を記憶
        lastPos = pos;

        // 1秒ごとに状態を表示
        loopCount++;
        if (loopCount % 1000 == 0) {
            double yError = pos[1] - targetY;  // Y軸の誤差
            double xzDistance = std::sqrt(pos[0]*pos[0] + (pos[2]-0.10)*(pos[2]-0.10));  // XZ平面での中心からの距離

            std::cout << "時間: " << elapsedSeconds << "秒 | "
                      << "目標Y: " << (targetY * 1000) << "mm | "
                      << "実際Y: " << (pos[1] * 1000) << "mm | "
                      << "誤差Y: " << (yError * 1000) << "mm | "
                      << "XZ中心距離: " << (xzDistance * 1000) << "mm"
                      << std::endl;
        }
    }

    falcon.close();
    return 0;
}