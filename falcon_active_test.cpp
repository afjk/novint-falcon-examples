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

// 力の強度係数（適度な強さに調整）
const double SPRING_CONSTANT = 20.0;     // バネ定数
const double GRAVITY_FORCE = 3.0;        // 重力の強さ
const double DAMPING = 2.0;              // 減衰係数
const double MAGNETIC_FORCE = 15.0;      // 磁力の強さ

// 前回の位置を記憶（速度計算用）
std::array<double, 3> lastPosition = {0.0, 0.0, 0.0};

// ベクトルの長さを計算
double vectorLength(const std::array<double, 3>& vec) {
    return std::sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

// バネ力を計算（中心に引き戻す）
std::array<double, 3> calculateSpringForce(const std::array<double, 3>& pos) {
    std::array<double, 3> force;
    force[0] = -SPRING_CONSTANT * pos[0];
    force[1] = -SPRING_CONSTANT * pos[1];
    force[2] = -SPRING_CONSTANT * pos[2];
    return force;
}

// 重力を計算（下向き）
std::array<double, 3> calculateGravity() {
    return {0.0, 0.0, -GRAVITY_FORCE};
}

// 減衰力を計算（動きに対する抵抗）
std::array<double, 3> calculateDamping(const std::array<double, 3>& pos, const std::array<double, 3>& lastPos) {
    std::array<double, 3> velocity;
    velocity[0] = (pos[0] - lastPos[0]) * 1000.0; // 速度の概算
    velocity[1] = (pos[1] - lastPos[1]) * 1000.0;
    velocity[2] = (pos[2] - lastPos[2]) * 1000.0;

    return {-DAMPING * velocity[0], -DAMPING * velocity[1], -DAMPING * velocity[2]};
}

// 磁力を計算（複数の磁石ポイント）
std::array<double, 3> calculateMagneticForce(const std::array<double, 3>& pos) {
    // 4つの磁石ポイント（四隅に配置）
    std::array<std::array<double, 3>, 4> magnets = {{
        {0.02, 0.02, 0.0},
        {0.02, -0.02, 0.0},
        {-0.02, 0.02, 0.0},
        {-0.02, -0.02, 0.0}
    }};

    std::array<double, 3> totalForce = {0.0, 0.0, 0.0};

    for (const auto& magnet : magnets) {
        std::array<double, 3> diff = {
            magnet[0] - pos[0],
            magnet[1] - pos[1],
            magnet[2] - pos[2]
        };

        double distance = vectorLength(diff);
        if (distance > 0.001) { // ゼロ除算を避ける
            double forceMagnitude = MAGNETIC_FORCE / (distance * distance);
            totalForce[0] += forceMagnitude * diff[0] / distance;
            totalForce[1] += forceMagnitude * diff[1] / distance;
            totalForce[2] += forceMagnitude * diff[2] / distance;
        }
    }

    return totalForce;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Novint Falcon アクティブ力テスト" << std::endl;
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

    std::cout << std::endl;
    std::cout << "力フィードバックモード:" << std::endl;
    std::cout << "  [1] バネ力（中心に引き戻す）" << std::endl;
    std::cout << "  [2] 重力（下向きの力）" << std::endl;
    std::cout << "  [3] バネ + 重力" << std::endl;
    std::cout << "  [4] 磁力（4つの磁石に引き寄せられる）" << std::endl;
    std::cout << "  [5] フルシミュレーション（バネ + 重力 + 減衰）" << std::endl;
    std::cout << std::endl;
    std::cout << "モードを選択してください (1-5): ";

    int mode;
    std::cin >> mode;

    std::cout << std::endl;
    std::cout << "テスト開始！ (Ctrl+Cで終了)" << std::endl;
    std::cout << "グリップを動かして力を感じてください。" << std::endl;
    std::cout << std::endl;

    int loopCount = 0;
    auto startTime = std::chrono::high_resolution_clock::now();

    while (true) {
        if (!falcon.runIOLoop()) {
            continue;
        }

        auto pos = falcon.getPosition();
        std::array<double, 3> force = {0.0, 0.0, 0.0};

        // モードに応じた力を計算
        switch (mode) {
            case 1: // バネ力のみ
                force = calculateSpringForce(pos);
                break;

            case 2: // 重力のみ
                force = calculateGravity();
                break;

            case 3: { // バネ + 重力
                auto springForce = calculateSpringForce(pos);
                auto gravity = calculateGravity();
                force[0] = springForce[0] + gravity[0];
                force[1] = springForce[1] + gravity[1];
                force[2] = springForce[2] + gravity[2];
                break;
            }

            case 4: // 磁力
                force = calculateMagneticForce(pos);
                break;

            case 5: { // フルシミュレーション
                auto springForce = calculateSpringForce(pos);
                auto gravity = calculateGravity();
                auto damping = calculateDamping(pos, lastPosition);
                force[0] = springForce[0] + gravity[0] + damping[0];
                force[1] = springForce[1] + gravity[1] + damping[1];
                force[2] = springForce[2] + gravity[2] + damping[2];
                break;
            }

            default:
                force = calculateSpringForce(pos);
                break;
        }

        // 力を設定
        falcon.setForce(force);

        // 前回の位置を更新
        lastPosition = pos;

        // 1秒ごとに位置と力を表示
        loopCount++;
        if (loopCount % 1000 == 0) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
            double fps = 1000.0 / (duration / 1000.0);

            std::cout << "位置: ["
                      << pos[0] << ", "
                      << pos[1] << ", "
                      << pos[2] << "] | "
                      << "力: ["
                      << force[0] << ", "
                      << force[1] << ", "
                      << force[2] << "] | "
                      << "FPS: " << fps << std::endl;

            startTime = currentTime;
        }
    }

    falcon.close();
    return 0;
}
