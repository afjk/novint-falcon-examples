#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/comm/FalconCommLibUSB.h>

using namespace libnifalcon;

// キーボード入力を非ブロッキングで取得するための設定
class KeyboardInput {
private:
    struct termios original_termios;
    bool terminal_modified;

public:
    KeyboardInput() : terminal_modified(false) {
        struct termios new_termios;
        tcgetattr(STDIN_FILENO, &original_termios);
        new_termios = original_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        terminal_modified = true;
    }

    ~KeyboardInput() {
        if (terminal_modified) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
        }
    }

    int getKey() {
        int ch = getchar();
        if (ch != EOF) {
            return ch;
        }
        return 0;
    }
};

enum class Mode {
    IDLE,
    RECORDING,
    PLAYING
};

// タイムライン記録用の構造体
struct RecordedSample {
    double timestamp;  // 記録開始からの経過時間（秒）
    std::array<double, 3> position;
};

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  記録と再生プログラム" << std::endl;
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

    // ファームウェアがロードされているか確認
    if (!falcon.isFirmwareLoaded()) {
        std::cout << "ファームウェアをロード中..." << std::endl;
        falcon.setFalconFirmware<FalconFirmwareNovintSDK>();

        if (!falcon.loadFirmware(10, false)) {
            std::cout << "警告: ファームウェアのロードに失敗しました。" << std::endl;
            std::cout << "デバイスの電源を入れ直してください。" << std::endl;
        } else {
            std::cout << "ファームウェアをロードしました。" << std::endl;
        }
    } else {
        std::cout << "ファームウェアは既にロードされています。" << std::endl;
        falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
    }

    falcon.setFalconKinematic<FalconKinematicStamper>();

    // PID制御パラメータ
    const double Kp = 100.0;
    const double Ki = 5.0;
    const double Kd = 8.0;
    const double integralLimit = 0.005;
    const double alpha = 0.15;  // ローパスフィルタ係数

    // タイムライン記録データ
    std::vector<RecordedSample> recordedTimeline;
    double recordingStartTime = 0.0;
    double playbackStartTime = 0.0;
    double totalRecordedTime = 0.0;
    Mode currentMode = Mode::IDLE;

    // PID制御用変数
    std::array<double, 3> lastPos = {0.0, 0.0, 0.0};
    std::array<double, 3> errorIntegral = {0.0, 0.0, 0.0};
    std::array<double, 3> filteredTarget = {0.0, 0.0, 0.0};

    // キーボード入力
    KeyboardInput keyboard;

    std::cout << std::endl;
    std::cout << "記録と再生モード" << std::endl;
    std::cout << "  1キー: 押している間、グリップ位置を記録（LED: 赤）" << std::endl;
    std::cout << "  2キー: 記録した位置を再生（ループ）（LED: 青）" << std::endl;
    std::cout << "  ESCキー: 終了" << std::endl;
    std::cout << "制御ゲイン: Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << std::endl;
    std::cout << std::endl;
    std::cout << "デバイスとの通信を開始しています..." << std::endl;
    std::cout << "I/O通信が確立されるまでお待ちください。" << std::endl;
    std::cout << std::endl;

    auto programStartTime = std::chrono::high_resolution_clock::now();
    auto lastKey1Time = programStartTime;
    bool ioLoopWorking = false;
    int failCount = 0;
    bool key1Pressed = false;  // 1キーが押されているか

    while (true) {
        // キーボード入力をチェック（runIOLoopの成功/失敗に関わらず）
        int key = keyboard.getKey();
        auto currentKeyCheckTime = std::chrono::high_resolution_clock::now();

        // 1キーが検出されたら時刻を更新
        if (key == '1') {
            key1Pressed = true;
            lastKey1Time = currentKeyCheckTime;
        } else {
            // 1キーが一定時間（200ms）検出されなければ離されたと判断
            auto timeSinceLastKey1 = std::chrono::duration_cast<std::chrono::milliseconds>(
                currentKeyCheckTime - lastKey1Time).count();
            if (timeSinceLastKey1 > 200) {
                key1Pressed = false;
            }
        }

        if (!falcon.runIOLoop()) {
            // runIOLoopが失敗してもキー入力は処理する
            if (!ioLoopWorking) {
                failCount++;
                if (failCount % 1000 == 0) {
                    std::cout << "I/Oループ待機中... (" << (failCount/1000) << "秒)" << std::endl;
                }
            }
            if (key == 27) { // ESCキー
                std::cout << "終了します。" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // runIOLoop成功
        if (!ioLoopWorking) {
            ioLoopWorking = true;
            std::cout << "I/O通信が確立されました！" << std::endl;
            std::cout << "操作を開始できます。" << std::endl;
            std::cout << std::endl;
        }

        // 現在の位置を取得
        auto pos = falcon.getPosition();

        Mode previousMode = currentMode;

        // 現在時刻を取得
        auto currentTime = std::chrono::high_resolution_clock::now();
        double currentSeconds = std::chrono::duration<double>(currentTime - programStartTime).count();

        // 1キーの状態に応じて記録開始・終了
        if (key1Pressed && currentMode != Mode::RECORDING) {
            // 記録開始
            currentMode = Mode::RECORDING;
            recordedTimeline.clear();
            recordingStartTime = currentSeconds;
            std::cout << "記録開始..." << std::endl;
            // LED赤色
            falcon.getFalconFirmware()->setLEDStatus(FalconFirmware::RED_LED);
        } else if (!key1Pressed && currentMode == Mode::RECORDING) {
            // 記録終了（1キーが離された）
            currentMode = Mode::IDLE;
            totalRecordedTime = currentSeconds - recordingStartTime;
            std::cout << "記録終了。記録時間: " << totalRecordedTime << " 秒, "
                      << recordedTimeline.size() << " サンプル" << std::endl;
            // LED消灯
            falcon.getFalconFirmware()->setLEDStatus(0);
        }

        // 2キーで再生
        if (key == '2') {
            if (currentMode != Mode::PLAYING && !recordedTimeline.empty()) {
                currentMode = Mode::PLAYING;
                playbackStartTime = currentSeconds;
                errorIntegral = {0.0, 0.0, 0.0};
                filteredTarget = recordedTimeline[0].position;
                std::cout << "再生開始... (記録時間: " << totalRecordedTime << " 秒, "
                          << recordedTimeline.size() << " サンプル)" << std::endl;
                // LED青色
                falcon.getFalconFirmware()->setLEDStatus(FalconFirmware::BLUE_LED);
            } else if (recordedTimeline.empty()) {
                std::cout << "記録データがありません。まず1キーで記録してください。" << std::endl;
            }
        }

        // ESCキーで終了
        if (key == 27) {
            std::cout << "終了します。" << std::endl;
            break;
        }

        // モード別の処理
        if (currentMode == Mode::RECORDING) {
            // タイムスタンプと位置を記録
            double recordTime = currentSeconds - recordingStartTime;
            RecordedSample sample;
            sample.timestamp = recordTime;
            sample.position = pos;
            recordedTimeline.push_back(sample);

            // 記録中は力をかけない（自由に動かせる）
            std::array<double, 3> force = {0.0, 0.0, 0.0};
            falcon.setForce(force);

        } else if (currentMode == Mode::PLAYING) {
            // 再生モード：時間軸に沿って記録された位置に追従
            if (!recordedTimeline.empty()) {
                // 再生開始からの経過時間
                double playbackTime = currentSeconds - playbackStartTime;

                // ループ再生：記録時間でモジュロ
                double loopTime = fmod(playbackTime, totalRecordedTime);

                // loopTimeに対応する目標位置を線形補間で取得
                std::array<double, 3> targetPos = {0.0, 0.0, 0.0};

                // 該当する時刻のサンプルを見つける
                for (size_t i = 0; i < recordedTimeline.size() - 1; i++) {
                    if (loopTime >= recordedTimeline[i].timestamp &&
                        loopTime < recordedTimeline[i + 1].timestamp) {
                        // 線形補間
                        double t = (loopTime - recordedTimeline[i].timestamp) /
                                   (recordedTimeline[i + 1].timestamp - recordedTimeline[i].timestamp);
                        for (int axis = 0; axis < 3; axis++) {
                            targetPos[axis] = recordedTimeline[i].position[axis] * (1.0 - t) +
                                              recordedTimeline[i + 1].position[axis] * t;
                        }
                        break;
                    }
                }

                // 最後のサンプルを超えている場合は最後の位置
                if (loopTime >= recordedTimeline.back().timestamp) {
                    targetPos = recordedTimeline.back().position;
                }

                // ローパスフィルタを適用
                for (int i = 0; i < 3; i++) {
                    filteredTarget[i] = alpha * targetPos[i] + (1.0 - alpha) * filteredTarget[i];
                }

                // PID制御で各軸の力を計算
                std::array<double, 3> force;
                double totalError = 0.0;
                for (int i = 0; i < 3; i++) {
                    double error = filteredTarget[i] - pos[i];
                    totalError += error * error;  // 二乗誤差
                    double velocity = (pos[i] - lastPos[i]) * 1000.0;
                    errorIntegral[i] += error / 1000.0;

                    // 積分項の制限
                    if (errorIntegral[i] > integralLimit) errorIntegral[i] = integralLimit;
                    if (errorIntegral[i] < -integralLimit) errorIntegral[i] = -integralLimit;

                    force[i] = Kp * error + Ki * errorIntegral[i] - Kd * velocity;
                }

                // 誤差の大きさを計算（距離）
                double errorDistance = std::sqrt(totalError) * 1000.0; // メートルからミリメートルへ

                // 誤差に応じて最大力を調整（障害物がある場合は強い力で侵入を試みる）
                double maxForce;
                if (errorDistance < 5.0) {
                    maxForce = 2.0;  // 誤差が小さい：通常の力
                } else if (errorDistance < 10.0) {
                    maxForce = 5.0;  // 誤差が中程度：強い力
                } else {
                    maxForce = 9.0;  // 誤差が大きい（障害物）：最大の力（デバイス仕様上限）
                }

                for (int i = 0; i < 3; i++) {
                    force[i] = std::max(-maxForce, std::min(maxForce, force[i]));
                }

                falcon.setForce(force);
            }

        } else {
            // アイドルモード：軽い力で現在位置を保持
            std::array<double, 3> force = {0.0, 0.0, 0.0};
            falcon.setForce(force);

            if (previousMode == Mode::PLAYING) {
                // 再生終了時にLEDを消灯
                falcon.getFalconFirmware()->setLEDStatus(0);
            }
        }

        // 現在の位置を記憶
        lastPos = pos;

        // 少し待つ
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // 終了時にLEDを消灯
    falcon.getFalconFirmware()->setLEDStatus(0);
    falcon.close();
    return 0;
}
