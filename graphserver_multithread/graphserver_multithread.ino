#include <M5Core2.h>
#include <map>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "arduinoFFT.h"

#define SMPLPERIOD 100 //IMUのサンプリング周期[ms]
#define SMPLFREQ_FFT 2//FFTのサンプリング周波数
#define FFTsamples 256//FFTする際の窓の大きさ 2のべき乗じゃないとダメ
#define ROWAMOUNT 10//HTMLファイルで表示する行数
#define GRAPHDATASIZE 120//グラフのデータの個数
#define JST (3600L*9)//日本の時差9h

float accX = 0.0F; float accY = 0.0F; float accZ = 0.0F;
float gyroX = 0.0F; float gyroY = 0.0F; float gyroZ = 0.0F;
float pitch = 0.0F; float roll = 0.0F; float yaw = 0.0F;
float temperature = 0.0F;
float rollperiod[ROWAMOUNT]= {0.0};float pitchperiod[ROWAMOUNT]= {0.0};//Rolling/Pitching周期格納用変数
int timeatfft[ROWAMOUNT][3];//FFTが行われた時刻記憶用変数
float gyroOffset[3]={0}; float accOffset[3]={0};//角速度と加速度のオフセット
float rollgdata[GRAPHDATASIZE]= {0.0};float pitchgdata[GRAPHDATASIZE]= {0.0};//グラフ表示用のRoll/Pitch毎回値を更新したい、1s周期にしたいという思惑もあり、vReal*とは別変数にした。
char gtime[GRAPHDATASIZE][10]={""};

WiFiClient client;
double vReal_r[FFTsamples];//FFTするうえでPitchとRollを格納しておくための変数。
double vReal_p[FFTsamples];
double vImag_r[FFTsamples];double vImag_p[FFTsamples];
arduinoFFT FFT_r = arduinoFFT(vReal_r, vImag_r, FFTsamples, SMPLFREQ_FFT);  // FFTオブジェクトを作る
arduinoFFT FFT_p = arduinoFFT(vReal_p, vImag_p, FFTsamples, SMPLFREQ_FFT);  // FFTオブジェクトを作る

static uint16_t j_f=0;//RollrateとPitchrateの配列の添え字カウンタ
static int8_t j_d=0;//ディスプレイ及びグラフデータの格納間隔を0.5/1.0sに管理するための変数
File output_file;//書き出し用csvfileと設定読み込み用txtfile
char ssid[32];  char pass[32];//WiFiのSSIDとパスワードを格納する変数　char型のグローバル変数でないと、エラーになる。
float breadthmld=0.0F;//船の型幅[m]

bool isLogging = true;
int num = 0;
String script_g1 ="";String script_g2 ="";String script_g3 ="";//Webサーバー用のHTMLスクリプト
String script_tb1 ="";
std::map<int, std::string> activityName;
WebServer server(80); //WebServerオブジェクトを作る

void calibration(float* gyroOffset,float* accOffset);
void SetwifiSD();
String getntpTime();
String zeroPadding(int num, int cnt);
void readparticular();
void readhtmlScript();
void computeFFT(double majorpeak[]);


void handleRoot(){// "/"にアクセスされた時の処理関数
  server.send(200,"text/plain","hello from M5stack!");
  return;
}

void handleGraph(){// "/graph"にアクセスされた時の処理関数
  String script_rgdata ="labels:[";
  for(uint8_t ig=0;ig<GRAPHDATASIZE;ig++){
    script_rgdata+="'"+String(gtime[ig])+"'";
    if(ig<GRAPHDATASIZE-1)
      script_rgdata+=",";
  }
  script_rgdata+="],\
              datasets:[{\
              label:'Roll[°]',\
              data:[";
  for(uint8_t ig=0;ig<GRAPHDATASIZE;ig++){
    script_rgdata+=rollgdata[ig];
    if(ig<GRAPHDATASIZE-1)
      script_rgdata+=",";
  }
  script_rgdata+="],";
  
  String script_pgdata ="labels:[";
  for(uint8_t ig=0;ig<GRAPHDATASIZE;ig++){
    script_pgdata+="'"+String(gtime[ig])+"'";
    if(ig<GRAPHDATASIZE-1)
      script_pgdata+=",";
  }
  script_pgdata+="],\
              datasets:[{\
              label:'Pitch[°]',\
              data:[";
  for(uint8_t ig=0;ig<GRAPHDATASIZE;ig++){
    script_pgdata+=pitchgdata[ig];
    if(ig<GRAPHDATASIZE-1)
      script_pgdata+=",";
  }
  script_pgdata+="],";

  String scriptg=script_g1+script_rgdata+script_g2+script_pgdata+script_g3;
  server.send(200,"text/html",scriptg);
  
  return;

}

void handleTable(){// "/table"にアクセスされた時の処理関数
  char tbrow[ROWAMOUNT][100]; //HTMLを編集する文字配列
  String script_tbl=script_tb1;
  for(uint8_t ir=0;ir<ROWAMOUNT;ir++){
    sprintf(tbrow[ir], "<tr><th>%02d:%02d:%02d</th><th>%f</th><th>%f</th><th>%f</th></tr>", timeatfft[ir][0],timeatfft[ir][1],timeatfft[ir][2],rollperiod[ir], pow((float)0.8 * breadthmld / rollperiod[ir], 2), pitchperiod[ir]); 
    script_tbl+=String(tbrow[ir]);
  }

  script_tbl+="</table>\
            </body>\
            </html>";
  server.send(200,"text/html",script_tbl);
  return;
}

void handleNotFound(){//存在しないファイルにアクセスされた時の処理関数
  server.send(404,"text/plain","File Not Found\n\n");
  return;
}

void taskHandleServer(void *arg){
  while(1){
    server.handleClient();
    delay(1);
  }
}

void setup() {
  M5.begin();                        // Init M5Core.
  M5.IMU.Init();                     // Init IMU sensor.

  M5.Lcd.fillScreen(BLACK);          // Set the screen background color to black.
  M5.Lcd.setTextColor(WHITE, BLACK); // Sets the foreground color and background color of the displayed text.
  M5.Lcd.setTextSize(2);             // Set the font size.

  readparticular();//船の寸法読み込み
  readhtmlScript();//HTMLのスクリプト読み込み
  String datetime=getntpTime();//Wifi接続と同時に年月日時取得

  
  String filepath = "/imu_data_" + datetime + ".csv";//csvファイル用意
  output_file = SD.open(filepath.c_str(), FILE_WRITE);
  if (!output_file)
  {
    M5.Lcd.println("ERROR: OPEN FILE");
    while (1);
  }
  output_file.println("JST,ms,num,gyroX,gyroY,gyroZ,accX,accY,accZ,roll,pitch,yaw,temperature,roll period,frequency,pitch period,freq"); // Header

  if(MDNS.begin("m5shipmonitor")){
    M5.Lcd.println("MDNS responder started");
    delay(1000);
  }
  server.on("/",handleRoot);
  server.on("/graph",handleGraph);
  server.on("/table",handleTable);
  server.onNotFound(handleNotFound);
  server.begin();
  M5.Lcd.println("HTTP server started");

  calibration(gyroOffset,accOffset);//IMUキャリブレーション

  //HTTPサーバのマルチスレッド関数を起動
  xTaskCreatePinnedToCore(taskHandleServer,"Task1",8192,NULL,1,NULL,1);

}


void loop() {
  unsigned int iniloopt=millis();//ループに入った時刻を記憶しておく この代入・遅延時間計算に1msかかっていると仮定
  struct tm tm;//日時変数
  double majorpeak[2];//Majorピーク周波数 double
  static bool ispotu0s;
  
  M5.update();
  
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ); // Stores the triaxial accelerometer.
  gyroX-=gyroOffset[0];gyroY-=gyroOffset[1];gyroZ-=gyroOffset[2];
  M5.IMU.getAccelData(&accX, &accY, &accZ); // Stores the triaxial accelerometer.
  accX-=accOffset[0];accY-=accOffset[1];accZ-=accOffset[2];
  M5.IMU.getAhrsData2(&pitch, &roll, &yaw,gyroX,gyroY,gyroZ,accX, accY, accZ);// Stores the inertial sensor attitude.
  M5.IMU.getTempData(&temperature);                // Stores the inertial sensor temperature.

  if(j_d++ == (1000/SMPLPERIOD)/SMPLFREQ_FFT -1){//0.1*5=0.5s毎にLcdへの表示・ファイルへの記録
    

    // Gyroscope output is related
    M5.Lcd.setCursor(0, 20);  // Move the cursor position to (x,y).
    M5.Lcd.println("gyroX,  gyroY"); // Screen printingformatted string.
    M5.Lcd.printf("%6.2f %6.2f", gyroX, gyroY);//初期オフセット除いた値の記入

    // Accelerometer output is related
    M5.Lcd.setCursor(0, 70);
    M5.Lcd.println("accX,   accY,  accZ");
    M5.Lcd.printf("%5.2f  %5.2f  %5.2f G", accX, accY, accZ); // 初期オフセット除いた値の記入

    // Pose output is related
    M5.Lcd.setCursor(0, 120);
    M5.Lcd.println("roll,  pitch");
    M5.Lcd.printf("%5.2f  %5.2f deg", -1 * roll, pitch); // M5Stackの電源スイッチ・USB-C端子を船首に向けて置いた場合
    M5.Lcd.println();                                    // 改行

    if (isLogging && getLocalTime(&tm)){
      if (output_file.size() < (1024 * 1024 * 1024))
      {
        output_file.printf("%2d:%2d:%2d,%d,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",\
        tm.tm_hour,tm.tm_min,tm.tm_sec,iniloopt,num,gyroX,gyroY,gyroZ,accX,accY,accZ,-1*roll,pitch, yaw,temperature,\
        rollperiod[0],majorpeak[0],pitchperiod[0],majorpeak[1]);
        output_file.flush();
        num++;
      }
      else
        output_file.close();
    }

    vReal_r[j_f]=(double)-roll;
    vReal_p[j_f]=(double)pitch;
    vImag_r[j_f] = 0.0; vImag_p[j_f] = 0.0; j_f+=1;

    if(ispotu0s){//1.0s毎にグラフ表示用データを格納
      memmove(rollgdata,&rollgdata[1],sizeof(rollgdata)-sizeof(float));
      memmove(pitchgdata,&pitchgdata[1],sizeof(pitchgdata)-sizeof(float));
      memmove(gtime,&gtime[1],sizeof(gtime)-sizeof(gtime[1]));
      rollgdata[GRAPHDATASIZE-1]=-roll;
      pitchgdata[GRAPHDATASIZE-1]=pitch;
      ispotu0s=false;
      if(getLocalTime(&tm))
        sprintf(gtime[GRAPHDATASIZE-1],"%2d:%2d:%2d",tm.tm_hour,tm.tm_min,tm.tm_sec);
    }else{
      ispotu0s=true;
    }
    j_d=0;
    
  }

  if(j_f>=FFTsamples){//FFT計算
    computeFFT(majorpeak);
    j_f=0;
    if((majorpeak[0]>0&&majorpeak[0]<2.0/SMPLFREQ_FFT)||(majorpeak[1]>0&&majorpeak[1]<2.0/SMPLFREQ_FFT)){
      M5.Lcd.setCursor(0, 165);
      for(int ir=ROWAMOUNT-1;ir>0;ir--){
        rollperiod[ir]=rollperiod[ir-1];pitchperiod[ir]=pitchperiod[ir-1];
        timeatfft[ir][0]=timeatfft[ir-1][0];timeatfft[ir][1]=timeatfft[ir-1][1];timeatfft[ir][2]=timeatfft[ir-1][2];
      }
      rollperiod[0]=(float)1.0/majorpeak[0];pitchperiod[0]=(float)1.0/majorpeak[1];
      M5.Lcd.printf("Period %5.2f  %5.2f s",rollperiod[0],pitchperiod[0]);//Lcdに周期を表示
      if(getLocalTime(&tm))
        timeatfft[0][0]=tm.tm_hour;timeatfft[0][1]=tm.tm_min;timeatfft[0][2]=tm.tm_sec;

      M5.Lcd.println();//改行
    }
    return;
  }

  // Button B  calibration
  if (M5.BtnB.wasPressed()){
    if (isLogging){
      calibration(gyroOffset,accOffset);//IMUキャリブレーション
      return;
    }
  }
 
  // Button C, stop logging
  if (M5.BtnC.wasPressed())
  {
    if (isLogging)
    {
      isLogging = false;
      output_file.close();
      M5.Lcd.setCursor(180, 221);
      M5.Lcd.printf("LOG STOPPED");
    }
  }
  
  if(SMPLPERIOD-(millis()-iniloopt)>1)
    delay(SMPLPERIOD-(millis()-iniloopt)); // Delay 100ms.(コンピューター内の計算にかかった遅延時間も考慮)

}

String zeroPadding(int num, int cnt){
  char tmp[256];
  char prm[5] = {'%', '0', (char)(cnt + 48), 'd', '\0'};
  sprintf(tmp, prm, num);
  return tmp;
}

String getntpTime(){
  SetwifiSD();
  m5.Lcd.setTextSize(2);
  configTime(JST,0,"ntp.nict.jp","time.google.com","ntp.jst.mfeed.ad.jp");//NTPサーバと時刻を同期させる
  struct tm tm;
  if(getLocalTime(&tm)){
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(60,80);
    M5.Lcd.printf("%d/%2d/%2d",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday);
    M5.Lcd.setCursor(80,140);
    M5.Lcd.printf("%02d:%02d:%02d",tm.tm_hour,tm.tm_min,tm.tm_sec);
    delay(1000);M5.Lcd.fillScreen(BLACK);
  } 
  return zeroPadding(tm.tm_year+1900, 4) + zeroPadding(tm.tm_mon+1, 2) + zeroPadding(tm.tm_mday, 2)+ zeroPadding(tm.tm_hour, 2)+ zeroPadding(tm.tm_min, 2);

}

void calibration(float* gyroOffset,float* accOffset){//MPU6886のキャリブレーション(初期オフセット除去)
  M5.Lcd.fillScreen(BLACK); // Set the screen background color to black.
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Calibrating...\n");
  M5.Lcd.print("IP: ");M5.Lcd.println(WiFi.localIP());
  delay(500);

  //補正値を求める
  int COUNTER = 3000;//=60000/20
  float gyroSum[3]={0,0,0};
  float accSum[3]={0,0,0};

  for(int i=0;i<COUNTER;i++){
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    M5.IMU.getAccelData(&accX, &accY, &accZ); // Stores the triaxial accelerometer.
    M5.IMU.getAhrsData2(&pitch, &roll, &yaw,gyroX,gyroY,gyroZ-gyroOffset[2],accX-accOffset[0], accY-accOffset[1], accZ-accOffset[2]);  // Stores the inertial sensor attitude.
    gyroSum[0] += gyroX;
    gyroSum[1] += gyroY;
    gyroSum[2] += gyroZ;
    accSum[0] += accX;
    accSum[1] += accY;
    accSum[2] += accZ;
    delay(20-1);//delay 20ms
  }
  gyroOffset[0] = (float)gyroSum[0]/COUNTER;
  gyroOffset[1] = (float)gyroSum[1]/COUNTER;
  gyroOffset[2] = (float)gyroSum[2]/COUNTER;
  accOffset[0] = (float)accSum[0]/COUNTER;
  accOffset[1] = (float)accSum[1]/COUNTER;
  accOffset[2] = (float)accSum[2]/COUNTER-1.0;//鉛直軸方向では重力加速度1Gが検出される

  M5.Lcd.setCursor(0, 20);
  M5.Lcd.println("Calibration end");
  M5.Lcd.fillScreen(BLACK);
  

}

void readparticular(){
  File rfp;//設定読み込み用ファイル
  unsigned int cnt = 0;
  char data[64]; char *str;

  rfp = SD.open("/read/shipparticular.csv", FILE_READ);
  while(rfp.available()){
    data[cnt++] = rfp.read();
  }
  strtok(data,",");
  str = strtok(NULL,"\r");    // CR

  breadthmld=atof(str);//船の型幅[m]

  M5.Lcd.printf("Breadth: %f[m]\n",breadthmld);
  rfp.close();
  

}

void SetwifiSD(){
  File rfp;
  unsigned int cnt = 0;
  char data[128];
  
  char *str;

  rfp = SD.open("/read/wificloud.csv", FILE_READ);
  while(rfp.available()&&cnt<128){
    data[cnt++] = rfp.read();
  }
  strtok(data,",");
  str = strtok(NULL,"\r");    // CR
  strncpy(&ssid[0], str, strlen(str));
  

  strtok(NULL,",");
  str = strtok(NULL,"\r");    // CR
  strncpy(&pass[0], str, strlen(str));

  M5.Lcd.printf("WIFI-SSID: %s\n",ssid);
  M5.Lcd.printf("WIFI-PASS: %s\n",pass);
  M5.Lcd.println("Connecting...");

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      M5.Lcd.print(".");
  }

  M5.Lcd.print("IP: ");
  M5.Lcd.println(WiFi.localIP());

  rfp.close();
}


void computeFFT(double majorpeak[]){
  FFT_r.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // 窓関数
  FFT_r.Compute(FFT_FORWARD); // FFT処理(複素数で計算)
  vReal_r[0]/=200;vReal_r[1]/=100;vReal_r[2]/=100;vReal_r[3]/=10;
  FFT_r.ComplexToMagnitude();
  majorpeak[0]=FFT_r.MajorPeak();
  FFT_p.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // 窓関数
  FFT_p.Compute(FFT_FORWARD); // FFT処理(複素数で計算)
  vReal_p[0]/=200;vReal_p[1]/=100;vReal_p[2]/=100;vReal_p[3]/=10;
  FFT_p.ComplexToMagnitude();
  majorpeak[1]=FFT_p.MajorPeak();

  for(int i=0;i<2;i++){
    if(majorpeak[i]>SMPLFREQ_FFT/2)
      majorpeak[i]=0;
  }

}

void readhtmlScript(){
  File rfp;
  unsigned int cnt = 0;
  
  rfp = SD.open("/read/script/rollchart1.html", FILE_READ);
  while(rfp.available()){
    script_g1 += rfp.readStringUntil('\n');
  }
  rfp.close();

  rfp = SD.open("/read/script/rollchart2.html", FILE_READ);
  while(rfp.available()){
    script_g2 += rfp.readStringUntil('\n');
  }
  rfp.close();

  rfp = SD.open("/read/script/pitchchart1.html", FILE_READ);
  while(rfp.available()){
    script_g3 += rfp.readStringUntil('\n');
  }
  rfp.close();
  rfp = SD.open("/read/script/table.html", FILE_READ);
  while(rfp.available()){
    script_tb1 += rfp.readStringUntil('\n');
  }
  rfp.close();
  
}


