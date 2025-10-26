#include <iostream>
#include <cmath>
#include <array>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/comm/FalconCommLibUSB.h>

using namespace libnifalcon;

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  ざらざらキューブ触覚テスト" << std::endl;
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

    // ランダムシードの初期化
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // キューブの定義（FalconCubeTestと同じ寸法）
    std::array<double, 3> cornerA = {-0.030, -0.030, 0.095};  // (-30mm, -30mm, 95mm)
    std::array<double, 3> cornerB = {0.030, 0.030, 0.155};    // (30mm, 30mm, 155mm)
    const double stiffness = 1000.0;        // 壁の硬さ
    const double textureAmplitude = 0.3;    // テクスチャノイズ強度 [N]
    const double textureFrequency = 150.0;  // テクスチャ振動周波数 [Hz]

    std::cout << std::endl;
    std::cout << "ざらざらした仮想キューブを作成しました。" << std::endl;
    std::cout << "キューブ範囲: ["
              << (cornerA[0] * 1000) << ", " << (cornerA[1] * 1000) << ", " << (cornerA[2] * 1000) << "] mm から" << std::endl;
    std::cout << "            ["
              << (cornerB[0] * 1000) << ", " << (cornerB[1] * 1000) << ", " << (cornerB[2] * 1000) << "] mm" << std::endl;
    std::cout << "硬さ: " << stiffness << " N/m" << std::endl;
    std::cout << "テクスチャ強度: " << textureAmplitude << " N" << std::endl;
    std::cout << std::endl;
    std::cout << "グリップを一番外側まで動かしてから、キューブの中に入れてください。" << std::endl;
    std::cout << "壁に触れると、ざらざらした触覚を感じます。" << std::endl;
    std::cout << "Ctrl+C で終了してください。" << std::endl;
    std::cout << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();
    int loopCount = 0;
    double noisePhase = 0.0;
    bool isInitializing = true;
    bool hasPrintedInitMsg = false;

    while (true) {
        if (!falcon.runIOLoop()) {
            continue;
        }

        // 位置を取得
        auto pos = falcon.getPosition();

        // 初期化フェーズ
        if (isInitializing) {
            if (!hasPrintedInitMsg) {
                std::cout << "エンドエフェクタを一番外側まで動かしてください（z > 170mm）" << std::endl;
                hasPrintedInitMsg = true;
            }
            if (pos[2] > 0.170) {
                std::cout << "キューブシミュレーションを開始します..." << std::endl;
                isInitializing = false;
                startTime = std::chrono::high_resolution_clock::now();
            }
            continue;
        }

        // 経過時間を取得（秒）
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
        double elapsedSeconds = duration / 1000.0;

        std::array<double, 3> force = {0.0, 0.0, 0.0};

        double dist = 10000;
        int closest = -1;
        int outside = 3;

        // 各軸について、エンドエフェクタがキューブの内側にあるかチェック
        for (int axis = 0; axis < 3; axis++) {
            if (pos[axis] > cornerA[axis] && pos[axis] < cornerB[axis]) {
                double dA = pos[axis] - cornerA[axis];
                double dB = pos[axis] - cornerB[axis];
                if (std::fabs(dA) < std::fabs(dist)) {
                    dist = dA;
                    closest = axis;
                }
                if (std::fabs(dB) < std::fabs(dist)) {
                    dist = dB;
                    closest = axis;
                }
                outside--;
            }
        }

        // キューブの内側にいて、壁に近い場合
        if (closest > -1 && !outside) {
            // 基本的なバネ力（最も近い壁から押し返す力）
            force[closest] = -stiffness * dist;

            // ざらざら感の生成
            loopCount++;

            // 時間ベースの周期的振動
            double vibration = std::sin(2.0 * M_PI * textureFrequency * elapsedSeconds);

            // ランダムノイズ (-1.0 ~ 1.0)
            double noise = (static_cast<double>(std::rand()) / RAND_MAX) * 2.0 - 1.0;

            // 低周波ノイズ (ゆっくり変化するざらつき感)
            noisePhase += 0.01;
            double slowNoise = std::sin(noisePhase) * std::cos(noisePhase * 1.7);

            // テクスチャ力の合成
            double textureForce = textureAmplitude * (
                vibration * 0.3 +
                noise * 0.4 +
                slowNoise * 0.3
            );

            // 壁に対して垂直な方向（closest軸）にテクスチャを加える
            force[closest] += textureForce;

            // 他の2軸（接線方向）にもテクスチャ力を加える
            for (int axis = 0; axis < 3; axis++) {
                if (axis != closest) {
                    force[axis] += textureForce * 0.5;
                }
            }

            // 1秒ごとに状態を表示
            if (loopCount % 1000 == 0) {
                std::cout << "時間: " << elapsedSeconds << "秒 | "
                          << "位置: ["
                          << (pos[0] * 1000) << ", "
                          << (pos[1] * 1000) << ", "
                          << (pos[2] * 1000) << "] mm | "
                          << "最近壁: " << (closest == 0 ? "X" : closest == 1 ? "Y" : "Z") << " | "
                          << "距離: " << (std::fabs(dist) * 1000) << " mm"
                          << std::endl;
            }
        } else {
            // キューブの外にいる場合
            if (loopCount % 2000 == 0) {
                std::cout << "キューブの外: 位置 = ["
                          << (pos[0] * 1000) << ", "
                          << (pos[1] * 1000) << ", "
                          << (pos[2] * 1000) << "] mm" << std::endl;
            }
        }

        // 力をデバイスに送信
        falcon.setForce(force);

        loopCount++;
    }

    falcon.close();
    return 0;
}
