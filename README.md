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

### 1. 初期化とファームウェアロード（キャリブレーション付き）

Falcon を接続したら、まず初期化ツールを実行してファームウェアをロードし、ホーミング（キャリブレーション）を完了させてください。

```bash
./run_falcon.sh
```

メニューが表示されたら `0` を選択し、画面の指示に従ってグリップを外側 → 内側の順に動かします。LED が緑に変わり、完了メッセージが表示されたら準備完了です。

### 2. デバイス数の確認

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

### 3. カスタムプログラムの実行

このプロジェクトには、複数のカスタム制御プログラムが含まれています。`run_falcon.sh`を使用して対話的に実行できます。

```bash
./run_falcon.sh
```

**利用可能なプログラム:**

0. **初期化・ファームウェアロード（キャリブレーション付き）**
1. **マウスカーソル追従（XY軸・Z軸固定版）**
2. **マウスカーソル追従（XY軸フリー版）**
3. **マウスカーソル追従（XZ軸・Y軸フリー版）**
4. **マウスカーソル追従（XZ軸・Y軸固定版）**
5. **Y軸上下運動**
6. **Z軸上下運動**
7. **基本テスト**
8. **アクティブテスト**
9. **記録と再生**（1キー：記録、2キー：再生、LED 表示付き）

各プログラムは、Falconデバイスとの対話的な制御を実現します。

## キャリブレーション

0 番の初期化ツール、または `run_falcon_test.sh` の各テストで求められたら、以下の手順でホーミングを行ってください：

1. Falcon のグリップを完全に外側まで動かす
2. そのまままっすぐ内側に押し込む

LED が緑色に変わればキャリブレーション完了です。

## セットアップ

### リポジトリのクローン

このプロジェクトはlibnifalconをgitサブモジュールとして使用しています。クローン時にサブモジュールも取得してください：

```bash
git clone --recurse-submodules https://github.com/afjk/novint-falcon-examples.git
cd novint-falcon-examples
```

既にクローン済みの場合は、サブモジュールを初期化してください：

```bash
git submodule update --init --recursive
```

### ビルド方法

#### 1. libnifalconのビルド

```bash
cd libnifalcon
mkdir -p build
cd build
cmake ..
make
cd ../..
```

#### 2. プロジェクトのビルド

```bash
mkdir -p build
cd build
cmake ..
make
```

これで、`build/`ディレクトリに実行ファイルが生成されます。

## プログラミング

### カスタムプログラムの作成

`falcon_test.cpp`にサンプルプログラムがあります。これを参考に独自のプログラムを作成できます。

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
├── run_falcon_test.sh          # libnifalconテスト実行スクリプト
├── run_falcon.sh               # 初期化＋カスタムプログラム実行メニュー
├── run_active_test.sh          # アクティブテスト実行スクリプト
├── falcon_test.cpp             # 基本テストプログラム
├── falcon_active_test.cpp      # アクティブ制御テスト
├── falcon_gentle_motion.cpp    # Z軸上下運動
├── falcon_y_motion.cpp         # Y軸上下運動
├── falcon_cursor_motion.cpp    # カーソル追従（XY軸・Z軸固定）
├── falcon_cursor_xy.cpp        # カーソル追従（XY軸フリー）
├── falcon_cursor_xz.cpp        # カーソル追従（XZ軸・Y軸フリー）
├── falcon_cursor_xz_fixed.cpp  # カーソル追従（XZ軸・Y軸固定）
├── falcon_record_play.cpp      # 記録と再生
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

## ライセンス

このプロジェクトはMIT Licenseの下で公開されています。

**重要な注意事項**: このプロジェクトは以下のライブラリを使用しています：
- **libnifalcon** - BSD 3-Clause License
- **GMTL** - GNU LGPL v2
- **Novint SDK** - 非商用ライセンス（商用利用には制限があります）

詳細は[LICENSE](LICENSE)ファイルを参照してください。
