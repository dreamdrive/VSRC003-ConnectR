
/* Minimum_Source*/
#include <stdio.h>
/* Dynamixel Basic Position Control Example
 
 Turns left the dynamixel , then turn right for one second,
 repeatedly.
 
                   Compatibility
 CM900                  O
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
/* Dynamixel ID defines */
#define ID_NUM 1
/* Control table defines */
#define GOAL_POSITION 30

Dynamixel Dxl(DXL_BUS_SERIAL1);

/* Control table defines */

//送受信バッファのサイズ
#define	BUFF_SZ	(256)

void sendmessage(char *wbuf, char *rbuf);
int get_memmap4(unsigned char map_add , short value[]);
int get_memmap8(unsigned char map_add , short value[]);
short get_memmap(unsigned char map_add);

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(115200);

  
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.jointMode(ID_NUM); //jointMode() is to use position mode
  
}

void loop() {
  volatile int error1 = 0 , error2 = 0;
  short sv1[8],sv2[8];

  error1 = get_memmap8(43,sv1);
  error2 = get_memmap8(51,sv2);
  
  if ((error1 == 0)&&(error2 == 0)){
  
    
    SerialUSB.print(sv1[0], DEC);
    SerialUSB.print(" | ");       
    SerialUSB.print(sv1[1], DEC); 
    SerialUSB.print(" | ");
    SerialUSB.print(sv1[2], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.print(sv1[3], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.print(sv1[4], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.print(sv1[5], DEC); 
    SerialUSB.print(" | ");
    SerialUSB.print(sv1[6], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.print(sv1[7], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.print(sv2[0], DEC);
    SerialUSB.print(" | ");       
    SerialUSB.print(sv2[1], DEC); 
    SerialUSB.print(" | ");
    SerialUSB.print(sv2[2], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.print(sv2[3], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.print(sv2[4], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.print(sv2[5], DEC); 
    SerialUSB.print(" | ");
    SerialUSB.print(sv2[6], DEC); 
    SerialUSB.print(" | ");       
    SerialUSB.println(sv2[7], DEC); 
    
    
  }
  
    
  //delay(10);
}


////////////////////////////////////////////////////////////////
//	get_memmap4関数
//	メモリマップのアドレスを指定して、8アドレス分の値を読む関数
//      1～4個取得と8個取得でパケットが変わることに注意！！
//	
//	・引数
//		unsigned char map_add 読み出したいメモリマップのアドレス0～255
//              short value[] 値を返すための配列変数8個分
//
//	・戻り値
//		正常終了は0、異常終了は-1
////////////////////////////////////////////////////////////////

int get_memmap4(unsigned char map_add , short value[]) {
	char rbuf[BUFF_SZ], wbuf[BUFF_SZ], rbuf2[BUFF_SZ];
        char outputc0[5],outputc1[5],outputc2[5],outputc3[5];
	//short value;
	int pos_sharp = 0, i ,length;

	//送受信メッセージバッファのクリア
       for(i = 0; i < BUFF_SZ; i++) {
            wbuf[i]=0;
            rbuf[i]=0;
            rbuf2[i]=0;
        }

	//rコマンドのメッセージを作成
	sprintf(wbuf, "r 20%04X 08\r\n", (2048 + (map_add * 2)));

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
        if(length != 34) return -1;
        // 返答の長さが適切かどうか
     
        // エラーチェック(送ったアドレスと受信したアドレスが一致しているか)
        if(!((wbuf[2]==rbuf2[1]) && (wbuf[3]==rbuf2[2]) && (wbuf[4]==rbuf2[3]) && (wbuf[5]==rbuf2[4]) && (wbuf[6]==rbuf2[5]) && (wbuf[7]==rbuf2[6]))) return -1;
        
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

	//16進数を数値に変換 -> short(2byte)に格納
	value[0] = strtol(outputc0, NULL, 16);
	value[1] = strtol(outputc1, NULL, 16);
	value[2] = strtol(outputc2, NULL, 16);
	value[3] = strtol(outputc3, NULL, 16);

      	return 0;
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
	//short value;
	int pos_sharp = 0, i ,length;
        short tempp;

	//送受信メッセージバッファのクリア
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
              
        // 返答の長さが適切かどうか
        if(length != 58) return -1;
   
        // エラーチェック(送ったアドレスと受信したアドレスが一致しているか)
        if(!((wbuf[2]==rbuf2[1]) && (wbuf[3]==rbuf2[2]) && (wbuf[4]==rbuf2[3]) && (wbuf[5]==rbuf2[4]) && (wbuf[6]==rbuf2[5]) && (wbuf[7]==rbuf2[6]))) return -1;
        
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

	//16進数を数値に変換 -> short(2byte)に格納
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

        Serial3.flush();  // 受信バッファクリア

	//メッセージの送信
        Serial3.print(wbuf);
        //if (!WriteFile(hCom, wbuf, (int)strlen(wbuf), &wn, NULL)) werr = TRUE;

	//メッセージの受信
	//メッセージの最後まで1byteずつ読み込みを行なう
	//メッセージの最後の判断は、2回目に改行('\n')が登場する場所。1回目は送信メッセージ、2回目はCPUボードが付記するもの
	//また、受信がタイムアウトを迎えた場合もループを抜ける
	while (rn) {
		//if (!ReadFile(hCom, &rbuf[i], 1, &rn, NULL)) { rerr = TRUE; break; }
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

void debug_print(char *wbuf, char *rbuf){
 	char wbuf_d[BUFF_SZ],rbuf_d[BUFF_SZ];
	int i;

	//送受信メッセージバッファのクリア
       for(i = 0; i < BUFF_SZ; i++) {
             wbuf_d[i] = 0;
            rbuf_d[i] = 0;  // デバッグ用
        }

	for (i = 0; wbuf[i] != '\0'; i++) {
                wbuf_d[i] = wbuf[i];    // デバッグ用
                if (wbuf[i] == '\n') wbuf_d[i]='N';    // デバッグ用
                if (wbuf[i] == '\r') wbuf_d[i]='R';    // デバッグ用  
	}

	for (i = 0; rbuf[i] != '\0'; i++) {
                rbuf_d[i] = rbuf[i];    // デバッグ用
                if (rbuf[i] == '\n') rbuf_d[i]='N';    // デバッグ用
                if (rbuf[i] == '\r') rbuf_d[i]='R';    // デバッグ用  
	}

        SerialUSB.print("TX >> ");   
        SerialUSB.print(wbuf);       
        SerialUSB.print("RX << ");
        SerialUSB.println(rbuf_d); 
}
