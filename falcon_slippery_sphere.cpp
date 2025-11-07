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
    std::cout << "  つるつる球体（氷・ガラス）触覚テスト" << std::endl;
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

    // つるつる球体のパラメータ
    const double radius = 0.040;              // 球の半径: 40mm
    const double stiffness = 1200.0;          // 剛性: 非常に高い（硬い表面）
    const double friction = 0.02;             // 摩擦係数: 非常に低い（つるつる）
    const double viscosity = 0.5;             // 粘性係数: 非常に低い（空気抵抗程度）
    const double zOffset = 0.11;              // Z軸オフセット（球の中心位置）
    const double loopFrequency = 1000.0;      // ループ周波数（約1kHz）

    std::cout << std::endl;
    std::cout << "つるつるで硬い仮想球体を作成しました。" << std::endl;
    std::cout << "球の半径: " << (radius * 1000) << " mm" << std::endl;
    std::cout << "中心位置: z = " << (zOffset * 1000) << " mm" << std::endl;
    std::cout << "剛性: " << stiffness << " N/m（非常に硬い）" << std::endl;
    std::cout << "摩擦係数: " << friction << "（氷のように滑りやすい）" << std::endl;
    std::cout << std::endl;
    std::cout << "グリップを一番外側まで動かしてから、球の表面に触れてください。" << std::endl;
    std::cout << "表面を動かすと、氷やガラスのようにつるつる滑る感触を感じます。" << std::endl;
    std::cout << "Ctrl+C で終了してください。" << std::endl;
    std::cout << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();
    int loopCount = 0;
    bool isInitializing = true;
    bool hasPrintedInitMsg = false;

    // 前回の位置を記憶（速度計算用）
    std::array<double, 3> oldPos = {0.0, 0.0, 0.0};

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
                std::cout << "つるつる球体シミュレーションを開始します..." << std::endl;
                isInitializing = false;
                startTime = std::chrono::high_resolution_clock::now();
                oldPos = pos;
            } else {
                oldPos = pos;
            }
            continue;
        }

        // 経過時間を取得（秒）
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
        double elapsedSeconds = duration / 1000.0;

        std::array<double, 3> force = {0.0, 0.0, 0.0};

        // Z軸オフセットを適用（球の中心位置）
        std::array<double, 3> offsetPos = pos;
        offsetPos[2] -= zOffset;

        // 球の中心からの距離を計算
        double dist = std::sqrt(
            offsetPos[0] * offsetPos[0] +
            offsetPos[1] * offsetPos[1] +
            offsetPos[2] * offsetPos[2]
        );

        // 速度を計算（前回の位置との差分）
        std::array<double, 3> velocity;
        velocity[0] = (pos[0] - oldPos[0]) * loopFrequency;
        velocity[1] = (pos[1] - oldPos[1]) * loopFrequency;
        velocity[2] = (pos[2] - oldPos[2]) * loopFrequency;

        // 球の表面または内側にいる場合
        if (dist < radius + 0.005) {  // 5mmのマージンを持たせる
            double penetration = radius - dist;

            // 1. 強い弾性力（硬い表面）
            if (dist > 0.001) {  // ゼロ除算を防ぐ
                double elasticForce = stiffness * penetration;
                force[0] = (offsetPos[0] / dist) * elasticForce;
                force[1] = (offsetPos[1] / dist) * elasticForce;
                force[2] = (offsetPos[2] / dist) * elasticForce;
            }

            // 2. 非常に低い摩擦力（つるつる滑る）
            // 表面に沿った速度成分を計算
            if (dist > 0.001) {
                // 法線方向の単位ベクトル
                std::array<double, 3> normal = {
                    offsetPos[0] / dist,
                    offsetPos[1] / dist,
                    offsetPos[2] / dist
                };

                // 法線方向の速度成分
                double normalVelocity =
                    normal[0] * velocity[0] +
                    normal[1] * velocity[1] +
                    normal[2] * velocity[2];

                // 接線方向の速度（表面に沿った動き）
                std::array<double, 3> tangentialVelocity = {
                    velocity[0] - normal[0] * normalVelocity,
                    velocity[1] - normal[1] * normalVelocity,
                    velocity[2] - normal[2] * normalVelocity
                };

                // 非常に低い摩擦抵抗（つるつる感）
                double frictionForce = friction * std::max(0.0, penetration * stiffness);
                force[0] -= frictionForce * tangentialVelocity[0];
                force[1] -= frictionForce * tangentialVelocity[1];
                force[2] -= frictionForce * tangentialVelocity[2];
            }

            // 3. 最小限の粘性抵抗（空気抵抗程度）
            force[0] -= viscosity * velocity[0];
            force[1] -= viscosity * velocity[1];
            force[2] -= viscosity * velocity[2];

            // 1秒ごとに状態を表示
            if (loopCount % 1000 == 0) {
                double speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
                std::cout << "時間: " << elapsedSeconds << "秒 | "
                          << "位置: ["
                          << (pos[0] * 1000) << ", "
                          << (pos[1] * 1000) << ", "
                          << (pos[2] * 1000) << "] mm | "
                          << "距離: " << (dist * 1000) << " mm | "
                          << "速度: " << (speed * 1000) << " mm/s"
                          << std::endl;
            }
        } else {
            // 球の外にいる場合
            if (loopCount % 2000 == 0) {
                std::cout << "球の外: 距離 = " << (dist * 1000) << " mm "
                          << "(表面まで: " << ((dist - radius) * 1000) << " mm)" << std::endl;
            }
        }

        // 位置を記憶（次回の速度計算用）
        // ノイズ除去のため、ローパスフィルタを適用
        oldPos[0] = 0.3 * oldPos[0] + 0.7 * pos[0];
        oldPos[1] = 0.3 * oldPos[1] + 0.7 * pos[1];
        oldPos[2] = 0.3 * oldPos[2] + 0.7 * pos[2];

        // 力をデバイスに送信
        falcon.setForce(force);

        loopCount++;
    }

    falcon.close();
    return 0;
}
