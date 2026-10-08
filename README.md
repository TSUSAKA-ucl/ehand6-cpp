# `EHand-6 Product User Manual` PDFマニュアルを参考にしてeHand6を動かす

0. CAN FDのハードウェアに配線し、24V3A電源も配線する  
   socket CANのインターフェースが`can0`でない場合は以下適宜よみかえる
1. このリポジトリーをclone
   ```
   git clone https://github.com/TSUSAKA-ucl/ehand6-cpp.git
   cd ehand6-cpp/driver_original/
   ```
1. ビルド
   ```
   make
   ```
2. CANをCAN FDに初期化、ehand6のbps設定
   ```
   ./can-up.sh
   ```
   確認
   ```
   ip -details link show can0
   ```
   `bitrate 1000000 sample-point 0.800`で`dbitrate 5000000 dsample-point 0.750`になっている必要がある3
   `can-up.sh`は全てのsocket CANインターフェーを設定します。特定のインターフェースだけ設定したい場合は
   ```
   source can-up.sh
   can_up can0
   ```
3. 24V電源ON
4. 動作テスト
   ```
   ./ehand_demo can0 right move reset
   ```
5. 動作テスト(別のCANインターフェース、左手)
   ```
   ./ehand_demo can1 left move reset
   ```
6. 二回目以降zeroing不要
   ```
   ./ehand_demo can1 left move
   ```
   Stateを読むだけ
   ```
   ./ehand_demo can1 left
   ```
