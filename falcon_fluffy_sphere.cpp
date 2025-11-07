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
    std::cout << "  ふわふわ球体（クッション・綿）触覚テスト" << std::endl;
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

    // ふわふわ球体のパラメータ
    const double radius = 0.050;              // 球の半径: 50mm（大きめで触りやすい）
    const double stiffness = 30.0;            // 剛性: 非常に低い（ふわふわ）
    const double damping = 0.8;               // 減衰係数（ゆっくり戻る）
    const double viscosity = 2.0;             // 軽い粘性
    const double zOffset = 0.12;              // Z軸オフセット（球の中心位置）
    const double loopFrequency = 1000.0;      // ループ周波数（約1kHz）
    const double returnSpeed = 0.3;           // 復元速度係数（ゆっくり）

    std::cout << std::endl;
    std::cout << "ふわふわで柔らかい仮想球体を作成しました。" << std::endl;
    std::cout << "球の半径: " << (radius * 1000) << " mm" << std::endl;
    std::cout << "中心位置: z = " << (zOffset * 1000) << " mm" << std::endl;
    std::cout << "剛性: " << stiffness << " N/m（非常に柔らかい）" << std::endl;
    std::cout << "減衰: " << damping << "（ゆっくり戻る）" << std::endl;
    std::cout << std::endl;
    std::cout << "グリップを一番外側まで動かしてから、球の中に入れてください。" << std::endl;
    std::cout << "球の中を動かすと、クッションや綿のようなふわふわした感触を感じます。" << std::endl;
    std::cout << "深く押し込んでも柔らかく、離すとゆっくり戻ります。" << std::endl;
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
                std::cout << "ふわふわ球体シミュレーションを開始します..." << std::endl;
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

        // 球の内側にいる場合
        if (dist < radius) {
            double penetration = radius - dist;
            double penetrationRatio = penetration / radius;

            // 1. 非常に弱い弾性力（ふわふわ）
            // 深く沈んでもあまり抵抗しない
            if (dist > 0.001) {  // ゼロ除算を防ぐ
                // 非線形の弱い復元力（深く入っても柔らかい）
                double softFactor = 1.0 - 0.5 * penetrationRatio;  // 深いほど柔らかく
                double elasticForce = stiffness * penetration * softFactor;
                force[0] = (offsetPos[0] / dist) * elasticForce;
                force[1] = (offsetPos[1] / dist) * elasticForce;
                force[2] = (offsetPos[2] / dist) * elasticForce;
            }

            // 2. 強めの減衰力（ゆっくり戻る、ふわふわ感）
            double dampingForce = damping * penetrationRatio;
            force[0] -= dampingForce * velocity[0];
            force[1] -= dampingForce * velocity[1];
            force[2] -= dampingForce * velocity[2];

            // 3. 軽い粘性（空気のような抵抗）
            double viscousForce = viscosity * penetrationRatio * 0.3;
            force[0] -= viscousForce * velocity[0];
            force[1] -= viscousForce * velocity[1];
            force[2] -= viscousForce * velocity[2];

            // 4. 追加の復元力（ゆっくり中心に戻ろうとする）
            // 速度とは独立した、位置ベースの弱い力
            double returnForce = returnSpeed * penetrationRatio;
            if (dist > 0.001) {
                force[0] += (offsetPos[0] / dist) * returnForce;
                force[1] += (offsetPos[1] / dist) * returnForce;
                force[2] += (offsetPos[2] / dist) * returnForce;
            }

            // 1秒ごとに状態を表示
            if (loopCount % 1000 == 0) {
                double speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
                std::cout << "時間: " << elapsedSeconds << "秒 | "
                          << "位置: ["
                          << (pos[0] * 1000) << ", "
                          << (pos[1] * 1000) << ", "
                          << (pos[2] * 1000) << "] mm | "
                          << "侵入: " << (penetration * 1000) << " mm ("
                          << (int)(penetrationRatio * 100) << "%) | "
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
