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
    std::cout << "  バネ球体（ゴムボール）触覚テスト" << std::endl;
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

    // バネ球体のパラメータ
    const double radius = 0.035;              // 球の半径: 35mm（小さめで硬い）
    const double stiffness = 800.0;           // 剛性: 非常に高い（バネのように硬い）
    const double bounceFactor = 1.2;          // バウンス係数（跳ね返る強さ）
    const double damping = 0.15;              // 減衰係数: 低い（エネルギーを保持）
    const double viscosity = 1.0;             // 粘性: 低い
    const double zOffset = 0.11;              // Z軸オフセット（球の中心位置）
    const double loopFrequency = 1000.0;      // ループ周波数（約1kHz）

    std::cout << std::endl;
    std::cout << "弾力のある硬い仮想球体を作成しました。" << std::endl;
    std::cout << "球の半径: " << (radius * 1000) << " mm" << std::endl;
    std::cout << "中心位置: z = " << (zOffset * 1000) << " mm" << std::endl;
    std::cout << "剛性: " << stiffness << " N/m（非常に硬い）" << std::endl;
    std::cout << "バウンス係数: " << bounceFactor << "（強く跳ね返る）" << std::endl;
    std::cout << std::endl;
    std::cout << "グリップを一番外側まで動かしてから、球に触れてください。" << std::endl;
    std::cout << "球を押すと、ゴムボールのように強く跳ね返されます。" << std::endl;
    std::cout << "速く押し込むほど、強い反発力を感じます。" << std::endl;
    std::cout << "Ctrl+C で終了してください。" << std::endl;
    std::cout << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();
    int loopCount = 0;
    bool isInitializing = true;
    bool hasPrintedInitMsg = false;

    // 前回の位置を記憶（速度計算用）
    std::array<double, 3> oldPos = {0.0, 0.0, 0.0};
    // 前回の速度を記憶（加速度計算用）
    std::array<double, 3> oldVelocity = {0.0, 0.0, 0.0};

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
                std::cout << "バネ球体シミュレーションを開始します..." << std::endl;
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

            // 1. 非常に強い弾性力（バネのように硬い）
            if (dist > 0.001) {  // ゼロ除算を防ぐ
                // 非線形バネ力（深く押すほど硬くなる）
                double nonlinearFactor = 1.0 + penetrationRatio * penetrationRatio;
                double elasticForce = stiffness * penetration * nonlinearFactor;
                force[0] = (offsetPos[0] / dist) * elasticForce;
                force[1] = (offsetPos[1] / dist) * elasticForce;
                force[2] = (offsetPos[2] / dist) * elasticForce;
            }

            // 2. 速度ベースのバウンス力（押し込む速度に応じて反発）
            // 内向きの速度（押し込む動き）を検出
            if (dist > 0.001) {
                // 法線方向の単位ベクトル（中心から外向き）
                std::array<double, 3> normal = {
                    offsetPos[0] / dist,
                    offsetPos[1] / dist,
                    offsetPos[2] / dist
                };

                // 法線方向の速度成分（負の値 = 押し込んでいる）
                double normalVelocity =
                    normal[0] * velocity[0] +
                    normal[1] * velocity[1] +
                    normal[2] * velocity[2];

                // 押し込んでいる場合（負の速度）、追加の反発力
                if (normalVelocity < 0) {
                    double bounceForce = bounceFactor * stiffness * penetrationRatio * std::abs(normalVelocity);
                    force[0] += normal[0] * bounceForce;
                    force[1] += normal[1] * bounceForce;
                    force[2] += normal[2] * bounceForce;
                }
            }

            // 3. 最小限の減衰（エネルギーを保持してバウンス）
            double dampingForce = damping * penetrationRatio;
            force[0] -= dampingForce * velocity[0];
            force[1] -= dampingForce * velocity[1];
            force[2] -= dampingForce * velocity[2];

            // 4. 軽い粘性抵抗
            double viscousForce = viscosity * penetrationRatio * 0.2;
            force[0] -= viscousForce * velocity[0];
            force[1] -= viscousForce * velocity[1];
            force[2] -= viscousForce * velocity[2];

            // 5. インパルス効果（衝突時の瞬間的な力）
            // 加速度を計算（速度の変化率）
            std::array<double, 3> acceleration;
            acceleration[0] = (velocity[0] - oldVelocity[0]) * loopFrequency;
            acceleration[1] = (velocity[1] - oldVelocity[1]) * loopFrequency;
            acceleration[2] = (velocity[2] - oldVelocity[2]) * loopFrequency;

            double accelMagnitude = std::sqrt(
                acceleration[0]*acceleration[0] +
                acceleration[1]*acceleration[1] +
                acceleration[2]*acceleration[2]
            );

            // 急激な減速（衝突）を検出して追加の反発
            if (accelMagnitude > 0.5 && dist > 0.001) {
                double impulseFactor = std::min(accelMagnitude / 10.0, 1.0);
                double impulseForce = 2.0 * impulseFactor * penetrationRatio;
                force[0] += (offsetPos[0] / dist) * impulseForce;
                force[1] += (offsetPos[1] / dist) * impulseForce;
                force[2] += (offsetPos[2] / dist) * impulseForce;
            }

            // 1秒ごとに状態を表示
            if (loopCount % 1000 == 0) {
                double forcemagnitude = std::sqrt(force[0]*force[0] + force[1]*force[1] + force[2]*force[2]);
                std::cout << "時間: " << elapsedSeconds << "秒 | "
                          << "位置: ["
                          << (pos[0] * 1000) << ", "
                          << (pos[1] * 1000) << ", "
                          << (pos[2] * 1000) << "] mm | "
                          << "侵入: " << (penetration * 1000) << " mm | "
                          << "速度: " << (speed * 1000) << " mm/s | "
                          << "反発力: " << forcemagnitude << " N"
                          << std::endl;
            }
        } else {
            // 球の外にいる場合
            if (loopCount % 2000 == 0) {
                std::cout << "球の外: 距離 = " << (dist * 1000) << " mm "
                          << "(表面まで: " << ((dist - radius) * 1000) << " mm)" << std::endl;
            }
        }

        // 速度を記憶（次回の加速度計算用）
        oldVelocity[0] = velocity[0];
        oldVelocity[1] = velocity[1];
        oldVelocity[2] = velocity[2];

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
