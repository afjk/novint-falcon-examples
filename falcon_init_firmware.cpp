#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "falcon/core/FalconDevice.h"
#include "falcon/core/FalconFirmware.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

namespace
{
constexpr int kMaxFirmwareRetries = 10;
constexpr int kIOLoopFailureLimit = 200;

void printCommError(const std::shared_ptr<libnifalcon::FalconComm>& comm)
{
    if(comm == nullptr)
    {
        std::cerr << "  通信オブジェクトが初期化されていません。" << std::endl;
        return;
    }
    std::cerr << "  通信エラーコード: " << comm->getErrorCode();
    if(comm->isCommOpen())
    {
        std::cerr << ", デバイスエラーコード: " << comm->getDeviceErrorCode();
    }
    std::cerr << std::endl;
}

void printFirmwareError(const std::shared_ptr<libnifalcon::FalconFirmware>& firmware)
{
    if(firmware == nullptr)
    {
        std::cerr << "  ファームウェアオブジェクトが利用できません。" << std::endl;
        return;
    }
    std::cerr << "  ファームウェアエラーコード: " << firmware->getErrorCode() << std::endl;
}

bool calibrateFalcon(libnifalcon::FalconDevice& device)
{
    using namespace libnifalcon;

    auto firmware = device.getFalconFirmware();
    if(firmware == nullptr)
    {
        std::cerr << "ファームウェアオブジェクトが利用できないため、キャリブレーションを実行できません。" << std::endl;
        return false;
    }

    std::cout << "Falcon を完全に外側まで動かしてから、まっすぐ内側に押し込んでください。" << std::endl;

    int consecutiveFailures = 0;
    while(true)
    {
        firmware->setHomingMode(true);
        firmware->setLEDStatus(FalconFirmware::RED_LED);

        if(!device.runIOLoop())
        {
            ++consecutiveFailures;
            if(consecutiveFailures > kIOLoopFailureLimit)
            {
                std::cerr << "I/O ループで連続エラーが発生しました。" << std::endl;
                std::cerr << "  ライブラリエラーコード: " << device.getErrorCode() << std::endl;
                printCommError(device.getFalconComm());
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        consecutiveFailures = 0;

        if(firmware->isHomed())
        {
            std::cout << "キャリブレーションが完了しました。" << std::endl;
            firmware->setLEDStatus(FalconFirmware::GREEN_LED);
            for(int i = 0; i < 5; ++i)
            {
                device.runIOLoop();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
}

int main()
{
    using namespace libnifalcon;

    std::cout << "Novint Falcon の初期化とファームウェアロードを開始します。" << std::endl;

    FalconDevice device;
    device.setFalconFirmware<FalconFirmwareNovintSDK>();

    std::cout << "デバイス 0 をオープンしています..." << std::endl;
    if(!device.open(0))
    {
        std::cerr << "Falcon デバイスを開けませんでした。ライブラリエラーコード: " << device.getErrorCode() << std::endl;
        printCommError(device.getFalconComm());
        return 1;
    }

    auto firmware = device.getFalconFirmware();
    if(firmware == nullptr)
    {
        std::cerr << "Falcon ファームウェアオブジェクトが初期化されていません。" << std::endl;
        device.close();
        return 1;
    }

    bool firmwareReady = false;

    std::cout << "既存のファームウェア状態を確認しています..." << std::endl;
    if(device.isFirmwareLoaded())
    {
        std::cout << "ファームウェアは既にロード済みです。" << std::endl;
        firmwareReady = true;
    }
    else
    {
        std::cout << "ファームウェアがロードされていないため、nVent ファームウェアを転送します。" << std::endl;

        bool loaded = false;
        for(int attempt = 1; attempt <= kMaxFirmwareRetries && !loaded; ++attempt)
        {
            std::cout << "ファームウェア転送 試行 " << attempt << " / " << kMaxFirmwareRetries << std::endl;
            if(firmware->loadFirmware(false, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
            {
                loaded = true;
                break;
            }

            std::cerr << "ファームウェアロードに失敗しました (試行 " << attempt << ")。" << std::endl;
            std::cerr << "  デバイスエラーコード: " << device.getErrorCode() << std::endl;
            printFirmwareError(firmware);
            printCommError(device.getFalconComm());

            std::cerr << "デバイスを再接続してリトライします..." << std::endl;
            device.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if(!device.open(0))
            {
                std::cerr << "再接続に失敗しました。ライブラリエラーコード: " << device.getErrorCode() << std::endl;
                printCommError(device.getFalconComm());
                return 2;
            }
            firmware = device.getFalconFirmware();
        }

        if(!loaded)
        {
            std::cerr << "ファームウェアをロードできませんでした。デバイスを確認して再試行してください。" << std::endl;
            device.close();
            return 3;
        }

        std::cout << "ファームウェアの転送が完了しました。検証のために IO ループを実行します..." << std::endl;
        if(!device.isFirmwareLoaded())
        {
            std::cerr << "ファームウェア検証に失敗しました。" << std::endl;
            printFirmwareError(device.getFalconFirmware());
            device.close();
            return 4;
        }
        firmwareReady = true;
    }

    if(!firmwareReady)
    {
        std::cerr << "ファームウェアの準備に失敗しました。" << std::endl;
        device.close();
        return 4;
    }

    std::cout << "ファームウェアが正常にロードされ、デバイスが応答しています。" << std::endl;

    std::cout << "キャリブレーションを実行します..." << std::endl;
    if(!calibrateFalcon(device))
    {
        std::cerr << "キャリブレーションに失敗しました。" << std::endl;
        device.close();
        return 5;
    }

    std::cout << "初期化・ファームウェアロード・キャリブレーションが完了しました。" << std::endl;
    device.close();
    return 0;
}
