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
    std::cout << "  ぬるぬる球体（スライム・ジェル）触覚テスト" << std::endl;
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

    // ぬるぬる球体のパラメータ
    const double radius = 0.045;              // 球の半径: 45mm
    const double stiffness = 80.0;            // 剛性: 低い（柔らかい）
    const double viscosity = 45.0;            // 粘性係数: 非常に高い（ぬるぬる）
    const double damping = 1.5;               // 減衰係数（ゆっくり動く）
    const double nonlinearViscosity = 25.0;   // 非線形粘性（速く動かすと重い）
    const double zOffset = 0.11;              // Z軸オフセット（球の中心位置）
    const double loopFrequency = 1000.0;      // ループ周波数（約1kHz）

    std::cout << std::endl;
    std::cout << "ぬるぬるで粘性の高い仮想球体を作成しました。" << std::endl;
    std::cout << "球の半径: " << (radius * 1000) << " mm" << std::endl;
    std::cout << "中心位置: z = " << (zOffset * 1000) << " mm" << std::endl;
    std::cout << "剛性: " << stiffness << " N/m（柔らかい）" << std::endl;
    std::cout << "粘性: " << viscosity << " N·s/m（非常に高い、ぬるぬる）" << std::endl;
    std::cout << std::endl;
    std::cout << "グリップを一番外側まで動かしてから、球の中に入れてください。" << std::endl;
    std::cout << "球の中を動かすと、スライムやジェルのようなぬるぬるした感触を感じます。" << std::endl;
    std::cout << "動かすと重く、速く動かそうとするとより強い抵抗を感じます。" << std::endl;
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
                std::cout << "ぬるぬる球体シミュレーションを開始します..." << std::endl;
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

        // 速度の大きさ
        double speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);

        // 球の内側にいる場合
        if (dist < radius) {
            double penetration = radius - dist;
            double penetrationRatio = penetration / radius;

            // 1. 弱めの弾性力（柔らかい）
            if (dist > 0.001) {  // ゼロ除算を防ぐ
                double elasticForce = stiffness * penetration;
                force[0] = (offsetPos[0] / dist) * elasticForce;
                force[1] = (offsetPos[1] / dist) * elasticForce;
                force[2] = (offsetPos[2] / dist) * elasticForce;
            }

            // 2. 非常に強い粘性力（ぬるぬる、重い）
            // 侵入が深いほど粘性が強い
            double viscousForce = viscosity * penetrationRatio;
            force[0] -= viscousForce * velocity[0];
            force[1] -= viscousForce * velocity[1];
            force[2] -= viscousForce * velocity[2];

            // 3. 非線形粘性（速く動かすとより重くなる）
            // 速度の二乗に比例した抵抗（流体抵抗）
            double speedFactor = speed * speed;
            double nonlinearForce = nonlinearViscosity * penetrationRatio * speedFactor;
            if (speed > 0.001) {
                force[0] -= nonlinearForce * (velocity[0] / speed);
                force[1] -= nonlinearForce * (velocity[1] / speed);
                force[2] -= nonlinearForce * (velocity[2] / speed);
            }

            // 4. 減衰力（ゆっくり動く特性）
            double dampingForce = damping * penetrationRatio;
            force[0] -= dampingForce * velocity[0];
            force[1] -= dampingForce * velocity[1];
            force[2] -= dampingForce * velocity[2];

            // 5. スティッキー効果（離れにくい感じ）
            // 表面付近で引き戻す力
            if (penetration < radius * 0.3) {  // 表面から30%以内
                double stickyRatio = 1.0 - (penetration / (radius * 0.3));
                double stickyForce = 0.5 * stickyRatio;
                if (dist > 0.001) {
                    force[0] -= (offsetPos[0] / dist) * stickyForce;
                    force[1] -= (offsetPos[1] / dist) * stickyForce;
                    force[2] -= (offsetPos[2] / dist) * stickyForce;
                }
            }

            // 1秒ごとに状態を表示
            if (loopCount % 1000 == 0) {
                std::cout << "時間: " << elapsedSeconds << "秒 | "
                          << "位置: ["
                          << (pos[0] * 1000) << ", "
                          << (pos[1] * 1000) << ", "
                          << (pos[2] * 1000) << "] mm | "
                          << "侵入: " << (penetration * 1000) << " mm | "
                          << "速度: " << (speed * 1000) << " mm/s | "
                          << "抵抗力: " << std::sqrt(force[0]*force[0] + force[1]*force[1] + force[2]*force[2]) << " N"
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
