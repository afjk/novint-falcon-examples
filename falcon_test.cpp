#include <iostream>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/util/FalconCLIBase.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/comm/FalconCommLibUSB.h>

using namespace libnifalcon;

int main() {
    std::cout << "Novint Falcon テストプログラム" << std::endl;
    std::cout << "================================" << std::endl;

    // Falconデバイスの作成
    FalconDevice falcon;

    // LibUSB通信を設定
    falcon.setFalconComm<FalconCommLibUSB>();

    // デバイスの数を取得
    unsigned int count;
    falcon.getDeviceCount(count);
    std::cout << "検出されたFalconデバイス数: " << count << std::endl;

    if (count == 0) {
        std::cout << "Falconデバイスが見つかりません。" << std::endl;
        return 1;
    }

    // 最初のデバイスを開く
    std::cout << "デバイスを開いています..." << std::endl;
    if (!falcon.open(0)) {
        std::cout << "エラー: デバイスを開けません。" << std::endl;
        return 1;
    }

    std::cout << "デバイスを正常に開きました！" << std::endl;

    // ファームウェアがロードされているか確認
    if (!falcon.isFirmwareLoaded()) {
        std::cout << "ファームウェアをロード中..." << std::endl;
        falcon.setFalconFirmware<FalconFirmwareNovintSDK>();

        if (!falcon.loadFirmware(10, false)) {
            std::cout << "警告: ファームウェアのロードに失敗しました。" << std::endl;
            std::cout << "エラーコード: " << falcon.getErrorCode() << std::endl;
        } else {
            std::cout << "ファームウェアを正常にロードしました！" << std::endl;
        }
    } else {
        std::cout << "ファームウェアは既にロードされています。" << std::endl;
    }

    // キネマティクスを設定
    falcon.setFalconKinematic<FalconKinematicStamper>();

    // デバイスのI/Oをテスト
    std::cout << "\nデバイスとの通信をテスト中..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        if (!falcon.runIOLoop()) {
            std::cout << "I/Oループエラー" << std::endl;
        } else {
            std::array<double, 3> pos = falcon.getPosition();
            std::cout << "位置 " << i << ": ["
                      << pos[0] << ", "
                      << pos[1] << ", "
                      << pos[2] << "]" << std::endl;
        }
    }

    // デバイスを閉じる
    falcon.close();
    std::cout << "\nデバイスを閉じました。" << std::endl;
    std::cout << "テスト完了！" << std::endl;

    return 0;
}
