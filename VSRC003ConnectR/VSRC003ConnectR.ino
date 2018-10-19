//
// VSRC003-ConnectR
//
// VS-RC003のメモリをシリアル通信で参照して、アドレスの値をDynamixelにシンクライトパケットで送信する
// 製作者 : Onomichi Hirokazu = みっちー ( http://dream-drive.net )
// 
// (接続) VS-RC003のコマンドポートの(TxD、RxD、GND)を、OpenCM 9.0.4のシリアルポート3(TxD、Rxd、GND)に接続する
// Open CM 9.0.4には電源とROBOTISのDynamixelを接続する
// このプログラムを書き込んで、VS-RC003には、別フォルダのプロジェクトファイルから起動する
// あとは、適当に適宜プログラムを書き換えたり、RobovieMakerでモーションを作ったり。
// アドレス56～61は、Dynamixelに振ってもいいし、他の用途で使ってもOK / 最大モーターは23個となります。
//
// 脱力指示はモーター単位ではなく、0～15のアドレスに対応するサーボを１つでも脱力させるとすべてのサーボが脱力します。
//

#include <stdio.h>

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

#define NUM_ACTUATOR        17 // Number of actuator
#define MAX_POSITION        1023 // 最大値
#define GOAL_SPEED          32  // GOAL_SPEEDのアドレス
#define GOAL_POSITION       30  // GOAL_POSITIONのアドレス
#define TORQUE_ENABLE       24  // TORQUE_ENABLEのアドレス
#define	BUFF_SZ	          (256) //送受信バッファのサイズ

word  AmpPos = 512;    // 初期位置
word  GoalPos[NUM_ACTUATOR]; // 目標位置
byte  id[NUM_ACTUATOR]; // 使用するIDの配列

// 基板上のLEDのフラグ
int led_chk = 0;

// ロボットの目のLED
int eyeLed_R = 0;
int eyeLed_G = 0;
int eyeLed_B = 0;

// 脱力指示のフラグ
unsigned int torqueF = 0;

// Dynamixel宣言
Dynamixel Dxl(DXL_BUS_SERIAL1);

// 関数宣言
void sendmessage(char *wbuf, char *rbuf);
int get_memmap8(unsigned char map_add , short value[]);
short get_memmap(unsigned char map_add);

////////////////////////////////////////////////////////////////
//    setup !
////////////////////////////////////////////////////////////////

void setup() {
  
  int i = 0;
  
  // VS-RC003と接続するシリアルの初期化
  Serial3.begin(115200);
  
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  //Insert dynamixel ID number to array id[]
  for(i=0; i<NUM_ACTUATOR; i++ ){
    id[i] = i + 1;
    GoalPos[i] = AmpPos;  // ゴールポジションを初期位置で初期化
  }
  
  //Set all dynamixels as same condition.
  Dxl.writeWord( BROADCAST_ID, GOAL_SPEED, 0 );
  Dxl.writeWord( BROADCAST_ID, GOAL_POSITION, AmpPos );
  
  //処理速度を測るピン
  pinMode(BOARD_LED_PIN, OUTPUT);
}

////////////////////////////////////////////////////////////////
//    loop !
////////////////////////////////////////////////////////////////

void loop() {
  int error = 0 , i = 0;
  short sv1[8],sv2[8],sv3[8];

  //アドレス:0～7の読み出し
  error = get_memmap8(0,sv1);
  if (error==0){
    for(i=0; i < 8; i++ ){
      GoalPos[i]= (word)sv1[i];
    }
  }

  //アドレス:8～15の読み出し  
  error = get_memmap8(8,sv2);
  if (error==0){
    for(i=0; i < 8; i++ ){
      GoalPos[i+8]= (word)sv2[i];
    }
  }

  //アドレス:55～62の読み出し  
  error = get_memmap8(55,sv3);
  if (error==0){
     GoalPos[16]= (word)sv3[0];
     eyeLed_R = sv3[4];   
     eyeLed_G = sv3[5];
     eyeLed_B = sv3[6];
     torqueF = sv3[7];    // アドレス62の脱力判定アドレス
  }

  // １ループごとに点灯／消灯…(オシロで見たときに1ループの実測処理時間が分かる)
  led_chk = 1 - led_chk;
  digitalWrite(BOARD_LED_PIN, led_chk);


  if (torqueF == 0){  // アドレス62の脱力判定がすべて0(脱力なし)の時
     
    // サーボ書き込み ******************
    Dxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);
    Dxl.pushByte(GOAL_POSITION);
    Dxl.pushByte(2);
    //push individual data length per 1 dynamixel, goal position needs 2 bytes(1word) 
    for( i=0; i<NUM_ACTUATOR; i++ ){
      Dxl.pushByte(id[i]);
      Dxl.pushByte(DXL_LOBYTE(GoalPos[i]));
      Dxl.pushByte(DXL_HIBYTE(GoalPos[i]));
    }
    Dxl.flushPacket();    // 書き込み
    
    //書き込み時のエラー処理
    if(!Dxl.getResult()){
      // なにか処理する場合はここに書く
    }
    // サーボ書き込みここまで ****************** 
    
  }
  else{  // 脱力指定が1つでもあるとき
    
    // サーボ書き込み ******************
    Dxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);
    Dxl.pushByte(TORQUE_ENABLE);
    Dxl.pushByte(1);
    //push individual data length per 1 dynamixel, goal position needs 2 bytes(1word) 
    for( i=0; i<NUM_ACTUATOR; i++ ){
      Dxl.pushByte(id[i]);
      Dxl.pushByte(0x00);
    }
    Dxl.flushPacket();    // 書き込み
    
    //書き込み時のエラー処理
    if(!Dxl.getResult()){
      // なにか処理する場合はここに書く
    }
    // サーボ書き込みここまで ****************** 

  }
    
  //delay(10);  いろいろタイムアウト待ちの処理があるのでわざわざ待ち時間を設けるのを廃止
}

////////////////////////////////////////////////////////////////
//	get_memmap8関数
//	メモリマップのアドレスを指定して、8アドレス分の値を読む関数
//	
//	・引数
//		unsigned char map_add 読み出したいメモリマップのアドレス0～255
//              short value[] 値を返すための配列変数8個分
//
//	・戻り値
//		正常終了は0、異常終了は-1
////////////////////////////////////////////////////////////////

int get_memmap8(unsigned char map_add , short value[]) {
	char rbuf[BUFF_SZ], wbuf[BUFF_SZ], rbuf2[BUFF_SZ];
        char outputc0[5],outputc1[5],outputc2[5],outputc3[5],outputc4[5],outputc5[5],outputc6[5],outputc7[5];
	int pos_sharp = 0, i ,length;
  
	//送受信メッセージバッファのクリア (これやっておかないと悲惨)
       for(i = 0; i < BUFF_SZ; i++) {
            wbuf[i]=0;
            rbuf[i]=0;
            rbuf2[i]=0;
        }

	//rコマンドのメッセージを作成
	sprintf(wbuf, "r 20%04X 16\r\n", (2048 + (map_add * 2)));

	//送受信！
	sendmessage(wbuf, rbuf);

	//受信結果を整形
	if (rbuf[0] == '\0') rbuf[0] = ' ';

	for (i = 0; rbuf[i] != '\0'; i++) {
		if (rbuf[i] == '#') pos_sharp = i;
	}

	for (i = 0; rbuf[i + pos_sharp] != '\0'; i++) {
		rbuf2[i] = rbuf[pos_sharp + i];
	}

        //debug_print(wbuf, rbuf);  // デバッグ表示
        
        length = i;
              
        // エラーチェック（返答の長さが適切かどうか）
        if(length != 58) return -1;
   
        // エラーチェック(送ったアドレスと受信したアドレスが一致しているか)
        if(!((wbuf[2]==rbuf2[1]) && (wbuf[3]==rbuf2[2]) && (wbuf[4]==rbuf2[3]) && (wbuf[5]==rbuf2[4]) && (wbuf[6]==rbuf2[5]) && (wbuf[7]==rbuf2[6]))) return -1;
        
        // ビッグエンディアン／リトルエンディアン並び替え（しかも文字で ()  ()sass！）
	outputc0[0] = rbuf2[11];
	outputc0[1] = rbuf2[12];
	outputc0[2] = rbuf2[8];
	outputc0[3] = rbuf2[9];
	outputc0[4] = 0;

	outputc1[0] = rbuf2[17];
	outputc1[1] = rbuf2[18];
	outputc1[2] = rbuf2[14];
	outputc1[3] = rbuf2[15];
	outputc1[4] = 0;

	outputc2[0] = rbuf2[23];
	outputc2[1] = rbuf2[24];
	outputc2[2] = rbuf2[20];
	outputc2[3] = rbuf2[21];
	outputc2[4] = 0;

	outputc3[0] = rbuf2[29];
	outputc3[1] = rbuf2[30];
	outputc3[2] = rbuf2[26];
	outputc3[3] = rbuf2[27];
	outputc3[4] = 0;

	outputc4[0] = rbuf2[35];
	outputc4[1] = rbuf2[36];
	outputc4[2] = rbuf2[32];
	outputc4[3] = rbuf2[33];
	outputc4[4] = 0;

	outputc5[0] = rbuf2[41];
	outputc5[1] = rbuf2[42];
	outputc5[2] = rbuf2[38];
	outputc5[3] = rbuf2[39];
	outputc5[4] = 0;

	outputc6[0] = rbuf2[47];
	outputc6[1] = rbuf2[48];
	outputc6[2] = rbuf2[44];
	outputc6[3] = rbuf2[45];
	outputc6[4] = 0;

	outputc7[0] = rbuf2[53];
	outputc7[1] = rbuf2[54];
	outputc7[2] = rbuf2[50];
	outputc7[3] = rbuf2[51];
	outputc7[4] = 0;

	//16進数を数値に変換 -> short(2byte)に格納 (ここの処理の時間、一度チェックした方がいいかも)
	value[0] = strtol(outputc0, NULL, 16);
	value[1] = strtol(outputc1, NULL, 16);
	value[2] = strtol(outputc2, NULL, 16);
	value[3] = strtol(outputc3, NULL, 16);
	value[4] = strtol(outputc4, NULL, 16);
	value[5] = strtol(outputc5, NULL, 16);
	value[6] = strtol(outputc6, NULL, 16);
	value[7] = strtol(outputc7, NULL, 16);
      
	return 0;
}


////////////////////////////////////////////////////////////////
//	get_memmap関数
//	メモリマップのアドレスを指定して、値を読む関数
//	
//	・引数
//		HANDLE hCom	通信に使用するハンドル
//		unsigned char map_add 読み出したいメモリマップのアドレス0～255
//
//	・戻り値
//		メモリマップの値(符号付2バイト) ただし、return -32678はエラー
////////////////////////////////////////////////////////////////

short get_memmap(unsigned char map_add) {
	char rbuf[BUFF_SZ], wbuf[BUFF_SZ], rbuf2[BUFF_SZ], outputc[4];
	short value;
	int pos_sharp = 0, i ,length;

	//送受信メッセージバッファのクリア
       for(i = 0; i < BUFF_SZ; i++) {
            wbuf[i]=0;
            rbuf[i]=0;
            rbuf2[i]=0;
        }

	//rコマンドのメッセージを作成
	sprintf(wbuf, "r 20%04X 02\r\n", (2048 + (map_add * 2)));

	//送受信！
	sendmessage(wbuf, rbuf);

	//受信結果を整形
	if (rbuf[0] == '\0') rbuf[0] = ' ';

	for (i = 0; rbuf[i] != '\0'; i++) {
		if (rbuf[i] == '#') pos_sharp = i;
	}

	for (i = 0; rbuf[i + pos_sharp] != '\0'; i++) {
		rbuf2[i] = rbuf[pos_sharp + i];
	}

        length = i;
        // 返答の長さが適切かどうか
        if(length != 16) return -32768;
        
        // エラーチェック(送ったアドレスと受信したアドレスが一致しているか)
        if(!((wbuf[2]==rbuf2[1]) && (wbuf[3]==rbuf2[2]) && (wbuf[4]==rbuf2[3]) && (wbuf[5]==rbuf2[4]) && (wbuf[6]==rbuf2[5]) && (wbuf[7]==rbuf2[6]))) return -32768;
                
	outputc[0] = rbuf2[11];
	outputc[1] = rbuf2[12];
	outputc[2] = rbuf2[8];
	outputc[3] = rbuf2[9];

	//16進数を数値に変換 -> short(2byte)に格納
	value = strtol(outputc, NULL, 16);

	return value;
}


////////////////////////////////////////////////////////////////
//	sendmessage関数
//	メッセージを送受信する関数。
//	
//	・引数
//		char *wbuf	送信メッセージバッファへのポインタ
//		char *rbuf	受信メッセージバッファへのポインタ
////////////////////////////////////////////////////////////////

void sendmessage(char *wbuf, char *rbuf){

//	BOOL werr = FALSE, rerr = FALSE;
	int rn = 1;
	int i = 0, cr_num = 0;
        long j = 0;

        Serial3.flush();  // 受信バッファクリア（前回送受信に失敗してたら残ってる受信バッファのゴミをクリア）

        Serial3.print(wbuf);	//メッセージの送信

	//メッセージの受信
	//メッセージの最後まで1byteずつ読み込みを行なう
	//メッセージの最後の判断は、2回目に改行('\n')が登場する場所。1回目は送信メッセージ、2回目はCPUボードが付記するもの
	//また、受信がタイムアウトを迎えた場合もループを抜ける
	while (rn) {
                if(Serial3.available() > 0){
                  rbuf[i] = (char)Serial3.read();
                  if (rbuf[i] == '\n') cr_num++;
                  if (cr_num >= 2) break;
                  i++;
                  j = 0;  //タイムアウトカウントリセット
                }
              j++;  
              if (j == 100000) {            // 10000ループでは結構タイムアウト×10で適当に数値決め
                //SerialUSB.print("time out");
                break;                      // タイムアウト
              }
        }
        
}

////////////////////////////////////////////////////////////////
//
// debug_print
// 送受信したパケットを2行に表示（改行コードを'R'と'N'に置換)
//
////////////////////////////////////////////////////////////////
void debug_print(char *wbuf, char *rbuf){
 	char wbuf_d[BUFF_SZ],rbuf_d[BUFF_SZ];
	int i;

	//送受信メッセージバッファのクリア
       for(i = 0; i < BUFF_SZ; i++) {
             wbuf_d[i] = 0;
            rbuf_d[i] = 0;  // デバッグ用
        }

	//表示用に整形
	for (i = 0; wbuf[i] != '\0'; i++) {
                wbuf_d[i] = wbuf[i];    // デバッグ用
                if (wbuf[i] == '\n') wbuf_d[i]='N';    // デバッグ用
                if (wbuf[i] == '\r') wbuf_d[i]='R';    // デバッグ用  
	}

	//表示用に整形
	for (i = 0; rbuf[i] != '\0'; i++) {
                rbuf_d[i] = rbuf[i];    // デバッグ用
                if (rbuf[i] == '\n') rbuf_d[i]='N';    // デバッグ用
                if (rbuf[i] == '\r') rbuf_d[i]='R';    // デバッグ用  
	}

        SerialUSB.print("TX >> ");   
        SerialUSB.println(wbuf_d);       
        SerialUSB.print("RX << ");
        SerialUSB.println(rbuf_d); 
}
