# Novint Falcon セットアップ & テスト環境

このディレクトリにはNovint Falcon触覚デバイスの開発環境とテストプログラムが含まれています。

## セットアップ済みの内容

1. **libnifalcon** - Novint Falcon用のオープンソースライブラリ（gitサブモジュール）
2. **テストプログラム** - デバイスの動作確認用プログラム
3. **実行スクリプト** - 簡単にテストを実行できるシェルスクリプト

## 必要な環境

- Novint Falcon触覚デバイス
- macOS / Linux / Windows
- CMake 3.5以上
- libusb-1.0

## 使い方

### 1. デバイス数の確認

```bash
./run_falcon_test.sh device-count
```

### 2. 触覚テストの実行

#### キューブテスト（立方体の触覚フィードバック）
```bash
./run_falcon_test.sh cube
```

#### 球体テスト
```bash
./run_falcon_test.sh sphere
```

#### 壁面テスト
```bash
./run_falcon_test.sh wall-x  # X軸方向の壁
./run_falcon_test.sh wall-y  # Y軸方向の壁
./run_falcon_test.sh wall-z  # Z軸方向の壁
```

#### カラーテスト（LEDテスト）
```bash
./run_falcon_test.sh color
```

#### 性能テスト
```bash
./run_falcon_test.sh loop
```

## キャリブレーション

テストを実行すると、最初にキャリブレーションが求められます：
1. Falconのグリップを完全に外側まで動かす
2. そのまままっすぐ内側に押し込む

これでキャリブレーションが完了し、テストが開始されます。

## プログラミング

### カスタムプログラムの作成

`falcon_test.cpp`にサンプルプログラムがあります。これを参考に独自のプログラムを作成できます。

#### ビルド方法

```bash
cd build
cmake ..
make
```

#### 実行方法

```bash
DYLD_LIBRARY_PATH=../libnifalcon/build/lib:$DYLD_LIBRARY_PATH ./falcon_test
```

### 基本的な使用例

```cpp
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/comm/FalconCommLibUSB.h>

using namespace libnifalcon;

FalconDevice falcon;
falcon.setFalconComm<FalconCommLibUSB>();

// デバイスを開く
falcon.open(0);

// ファームウェアをロード
falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
falcon.loadFirmware(10, false);

// キネマティクスを設定
falcon.setFalconKinematic<FalconKinematicStamper>();

// 位置を取得
falcon.runIOLoop();
std::array<double, 3> pos = falcon.getPosition();

// 力を設定
std::array<double, 3> force = {0.0, 0.0, -1.0};
falcon.setForce(force);
```

## ディレクトリ構造

```
novint-falcon-examples/
├── README.md                    # このファイル
├── LICENSE                      # ライセンスファイル
├── run_falcon_test.sh          # テスト実行スクリプト
├── falcon_test.cpp             # 基本テストプログラム
├── falcon_active_test.cpp      # アクティブ制御テスト
├── falcon_gentle_motion.cpp    # ジェントルモーション
├── falcon_cursor_*.cpp         # カーソル追従制御プログラム
├── CMakeLists.txt              # ビルド設定
├── build/                      # ビルド出力（gitignore）
└── libnifalcon/                # libnifalconライブラリ（サブモジュール）
```

## トラブルシューティング

### デバイスが見つからない場合

1. デバイスが正しく接続されているか確認
2. USBケーブルを再接続
3. デバイスの電源がオンになっているか確認

### ファームウェアのロードに失敗する場合

libnifalcon付属のテストプログラムを使用してください：
```bash
./run_falcon_test.sh device-count
```

## 参考リンク

- [libnifalcon GitHub](https://github.com/libnifalcon/libnifalcon)
- [libnifalcon ドキュメント](https://github.com/libnifalcon/libnifalcon/wiki)

## セットアップ完了日

2025-10-25
