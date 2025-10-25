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
    std::cout << "  スムーズな上下運動プログラム" << std::endl;
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

    // スムーズな上下運動（Z軸）のパラメータ
    const double period = 2.0;           // 周期（秒）：2秒で1往復（速度4倍）
    const double zAmplitude = 0.025;     // Z軸の移動範囲（メートル）：25mm（可動域内に調整）
    const double frequency = 1.0 / period; // 周波数
    const double centeringForce = 100.0;  // X, Y軸を中心に固定する力の強さ
    const double zCenter = 0.10;         // Z軸の中心位置（範囲：75-125mm）

    // PID+フィードフォワード制御パラメータ
    const double Kp = 200.0;             // 比例ゲイン
    const double Ki = 50.0;              // 積分ゲイン
    const double Kd = 15.0;              // 微分ゲイン
    const double Kff = 30.0;             // フィードフォワードゲイン
    const double integralLimit = 0.01;   // 積分項の上限（飽和防止）
    const double phaseCompensation = 0.35; // 位相補償（ラジアン）：約20度先読み

    // PID制御用変数
    std::array<double, 3> lastPos = {0.0, 0.0, 0.0};
    double zErrorIntegral = 0.0;         // Z軸誤差の積分

    std::cout << std::endl;
    std::cout << "スムーズな上下運動のPID+フィードフォワード+位相補償制御を開始します。" << std::endl;
    std::cout << "グリップが目標位置に向かってゆっくりと滑らかに上下します。" << std::endl;
    std::cout << "周期: " << period << "秒、振幅: ±" << (zAmplitude * 1000) << "mm" << std::endl;
    std::cout << "範囲: " << ((zCenter - zAmplitude) * 1000) << "-" << ((zCenter + zAmplitude) * 1000) << "mm" << std::endl;
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

        // 正弦波を使った目標位置と目標速度を計算（Z軸方向）
        // 位置: sin(2π * frequency * time + phaseCompensation)
        // 速度: 2π * frequency * amplitude * cos(2π * frequency * time + phaseCompensation)
        double phase = 2.0 * M_PI * frequency * elapsedSeconds;
        double compensatedPhase = phase + phaseCompensation;  // 位相補償を適用
        double targetZ = zCenter + zAmplitude * std::sin(compensatedPhase);
        double targetVelocity = zAmplitude * 2.0 * M_PI * frequency * std::cos(compensatedPhase);

        // X, Y軸を中心に固定するバネ力を計算
        double xCenteringForce = -centeringForce * pos[0];
        double yCenteringForce = -centeringForce * pos[1];

        // Z軸のPID+フィードフォワード制御
        double zError = targetZ - pos[2];                    // 誤差（P項）
        double zVelocity = (pos[2] - lastPos[2]) * 1000.0;  // 速度（D項用）

        // 積分項の更新（飽和防止付き）
        zErrorIntegral += zError / 1000.0;  // 1000Hzで積分
        if (zErrorIntegral > integralLimit) zErrorIntegral = integralLimit;
        if (zErrorIntegral < -integralLimit) zErrorIntegral = -integralLimit;

        // PID+フィードフォワード制御則
        double zForceP = Kp * zError;              // 比例項
        double zForceI = Ki * zErrorIntegral;      // 積分項
        double zForceD = -Kd * zVelocity;          // 微分項
        double zForceFf = Kff * targetVelocity;    // フィードフォワード項
        double zForce = zForceP + zForceI + zForceD + zForceFf;

        // 力を設定（全軸とも位置制御）
        std::array<double, 3> force = {xCenteringForce, yCenteringForce, zForce};
        falcon.setForce(force);

        // 現在の位置を記憶
        lastPos = pos;

        // 1秒ごとに状態を表示
        loopCount++;
        if (loopCount % 1000 == 0) {
            double zError = pos[2] - targetZ;  // Z軸の誤差
            double xyDistance = std::sqrt(pos[0]*pos[0] + pos[1]*pos[1]);  // XY平面での中心からの距離

            std::cout << "時間: " << elapsedSeconds << "秒 | "
                      << "目標Z: " << (targetZ * 1000) << "mm | "
                      << "実際Z: " << (pos[2] * 1000) << "mm | "
                      << "誤差Z: " << (zError * 1000) << "mm | "
                      << "XY中心距離: " << (xyDistance * 1000) << "mm"
                      << std::endl;
        }
    }

    falcon.close();
    return 0;
}
