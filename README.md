# VSRC003-ConnectR
VS-RC003とOpenCM9.0.4を繋いで、ROBOTISのコマンドサーボをRobovieMaker2から制御するサンプルソースとプロジェクトファイル  
製作者 : Onomichi Hirokazu = みっちー ( http://dream-drive.net )

## もう少し詳しく説明すると・・・
VS-RC003とOpenCM 9.0.4をシリアル通信で接続し、VS-RC003のメモリをOpenCM9.0.4から参照し、最大23個のROBOTIS製シリアルサーボ(Dynamixel)にシンクライトパケットで角度を送信して動作させます。  
モーションデータをRoboviemaker2で生成できるため、Dynamixelを用いたロボットに、PS2コントローラーや音声出力などの機能を加えて、ロボットのモーション開発が楽ちんになる(といいなー)と言うコンセプトで作られた、OpenCM9.0.4用のソースコードと、VS-RC003のサンプルプロジェクトファイルです。  

両者を使い慣れていないと、なかなか触りづらいかもしれません。(汗  

## 接続
VS-RC003のコマンドポートの(TxD、RxD、GND)を、OpenCM 9.0.4のシリアルポート3(TxD、Rxd、GND)に接続する(半田するとかいろいろ)  
Open CM 9.0.4には電源とROBOTISのDynamixel(IDは事前に01～17を設定)を接続する  

## 使い方
1. OpenCM9.0.4にVSRC003ConnectR.inoにそのまま書き込む  
2. VS-RC003には、VSRC2OpenCM.rpjのプロジェクトファイルを開きCPU情報を書き込む

あとは、適当にRobovieMaker2でいじくると、接続されたDynamixelが動きます。

## いじくりかた
とりあえず、ロボットの目をフルカラーLEDで光らせようと思ってアドレス59～61は確保しています。
アドレス56～61は、Dynamixelに振ってもいいし、他の用途で使ってもOK / 最大モーターは23個となります。  

## 注意点
脱力指示はモーター単位ではなく、0～15のアドレスに対応するサーボを１つでも脱力させるとすべてのサーボが脱力します。  
（アドレス62,63を監視して、サーボ１つずつ脱力指示を行うことも出来ましたが、最大個数のサーボ数が減るのと、今のところ必要性がなかったので手抜きですみません。）
