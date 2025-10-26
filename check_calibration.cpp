#include <iostream>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/comm/FalconCommLibUSB.h>

using namespace libnifalcon;

int main() {
    FalconDevice falcon;

    // LibUSB通信を設定
    falcon.setFalconComm<FalconCommLibUSB>();

    // デバイスの数を取得
    unsigned int count;
    if (!falcon.getDeviceCount(count) || count == 0) {
        std::cerr << "エラー: Falconデバイスが見つかりません。" << std::endl;
        return 2; // デバイスなし
    }

    // 最初のデバイスを開く
    if (!falcon.open(0)) {
        std::cerr << "エラー: デバイスを開けません。" << std::endl;
        return 2; // デバイスを開けない
    }

    // ファームウェアがロードされているか確認
    if (!falcon.isFirmwareLoaded()) {
        std::cout << "ファームウェアが未ロードです。" << std::endl;
        falcon.close();
        return 1; // ファームウェア未ロード
    }

    // ファームウェアオブジェクトを設定
    falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
    falcon.setFalconKinematic<FalconKinematicStamper>();

    // キャリブレーション状態を確認
    // 数回IOループを実行して状態を取得
    for (int i = 0; i < 5; ++i) {
        falcon.runIOLoop(FalconDevice::FALCON_LOOP_FIRMWARE);
    }

    auto firmware = falcon.getFalconFirmware();
    if (firmware && firmware->isHomed()) {
        std::cout << "キャリブレーション済みです。" << std::endl;
        falcon.close();
        return 0; // キャリブレーション済み
    } else {
        std::cout << "キャリブレーションが必要です。" << std::endl;
        falcon.close();
        return 1; // キャリブレーション未完了
    }
}
