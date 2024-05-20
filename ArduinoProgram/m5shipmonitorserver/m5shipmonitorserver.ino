#include <M5Core2.h>
#include <map>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "arduinoFFT.h"

#define SMPLPERIOD 100 //IMUのサンプリング周期[ms]
#define SMPLFREQ_FFT 2//FFTのサンプリング周波数
#define FFTsamples 256//FFTする際の窓の大きさ 2のべき乗じゃないとダメ
#define FFTpersamples 128//何sample毎にFFTするか  FFTsamplesの約数である必要ある。
#define ROWAMOUNT 30//HTMLファイルで表示する行数
#define GRAPHDATASIZE 120//グラフのデータの個数

RTC_TimeTypeDef RTCtime;
RTC_DateTypeDef RTCDate;
int8_t timediff=9;

float accX = 0.0F; float accY = 0.0F; float accZ = 0.0F;//IMU本体のX/Y/Z軸の加速度
float gyroX = 0.0F; float gyroY = 0.0F; float gyroZ = 0.0F;//X/Y/Z軸の角速度
float pitch = 0.0F; float roll = 0.0F; float yaw = 0.0F;
float temperature = 0.0F;//IMU内部温度
float rollperiod[GRAPHDATASIZE]= {0.0};float pitchperiod[GRAPHDATASIZE]= {0.0};//Rolling/Pitching周期格納用変数
char timeatfft[GRAPHDATASIZE][10]={""};;//FFTが行われた時刻記憶用変数
float gyroOffset[3]={0}; float accOffset[3]={0};//角速度と加速度のオフセット
float rollgdata[GRAPHDATASIZE]= {0.0};float pitchgdata[GRAPHDATASIZE]= {0.0};//グラフ表示用のRoll/Pitch 1s周期
float rollfftdata[FFTsamples]= {0.0};float pitchfftdata[FFTsamples]= {0.0};//FFTを行うRoll/Pitch(0.5s周期)を格納しておく。FFTを窓長より短い周期で行うために、vReal*とは別変数にした。
char gtime[GRAPHDATASIZE][10]={""};

WiFiClient client;
double vReal_r[FFTsamples];double vReal_p[FFTsamples];//FFTするうえでPitchとRollを格納しておくための変数。
double vImag_r[FFTsamples];double vImag_p[FFTsamples];
ArduinoFFT<double> FFT_r = ArduinoFFT<double>(vReal_r, vImag_r, FFTsamples, SMPLFREQ_FFT);  // FFTオブジェクトを作る
ArduinoFFT<double> FFT_p = ArduinoFFT<double>(vReal_p, vImag_p, FFTsamples, SMPLFREQ_FFT);  // FFTオブジェクトを作る

static uint16_t j_f=0;//RollrateとPitchrateの配列の添え字カウンタ
static int8_t j_d=0;//ディスプレイ及びグラフデータの格納間隔を0.5/1.0sに管理するための変数
File output_file;//書き出し用csvfileと設定読み込み用txtfile
char ssid[32];  char pass[32];//WiFiのSSIDとパスワードを格納する変数　char型のグローバル変数でないと、エラーになる。
float breadthmld=0.0;//船の型幅[m]
float kgm=0.0;//船の慣動半径
int8_t m5putdir=1;//M5Stackを置いた向き
char published_url[256];//GoogleスプレッドシートのデプロイされたURL

bool isLogging = true;//SDカードへ記録しているときにtrue
int num = 0;
String script_g1 ="";String script_g2 ="";String script_g3 ="";String script_g4 ="";String script_g5 ="";String script_g6 ="";//Webサーバー用のHTMLスクリプト
String script_tb1 ="";
std::map<int, std::string> activityName;
WebServer server(80); //WebServerオブジェクトを作る

void calibration(float* gyroOffset,float* accOffset);
void SetwifiSD();
String getrtcTime();
void getntpTime();
void setupTimeNtp();
String zeroPadding(int num, int cnt);
void readparticular();
void readhtmlScript();
void computeFFT(double majorpeak[]);
void PushtoGoogleSheet(float rollingperiod,float pitchingperiod);



void handleRoot(){// "/"にアクセスされた時の処理関数
  server.send(200,"text/plain","hello from M5stack!");
  return;
}

void handleGraph(){// "/graph"にアクセスされた時の処理関数
  String script_rgdata ="labels:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    script_rgdata+="'"+String(gtime[ig])+"'";
    if(ig<GRAPHDATASIZE-1)
      script_rgdata+=",";
  }
  script_rgdata+="],\
              datasets:[{\
              label:'Roll[°]',\
              data:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    script_rgdata+=rollgdata[ig];
    if(ig<GRAPHDATASIZE-1)
      script_rgdata+=",";
  }
  script_rgdata+="],";
  
  String script_pgdata ="labels:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    script_pgdata+="'"+String(gtime[ig])+"'";
    if(ig<GRAPHDATASIZE-1)
      script_pgdata+=",";
  }
  script_pgdata+="],\
              datasets:[{\
              label:'Pitch[°]',\
              data:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    script_pgdata+=pitchgdata[ig];
    if(ig<GRAPHDATASIZE-1)
      script_pgdata+=",";
  }
  script_pgdata+="],";

  String script_rpgdata ="labels:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    if(timeatfft[ig]!=""){
      script_rpgdata+="'"+String(timeatfft[ig])+"'";
      if(ig<GRAPHDATASIZE-1)
        script_rpgdata+=",";
    } 
  }
  script_rpgdata+="],\
              datasets:[{\
              label:'Rolling Period[s]',\
              data:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    if(timeatfft[ig]!=""){
      script_rpgdata+=rollperiod[ig];
      if(ig<GRAPHDATASIZE-1)
        script_rpgdata+=",";
    }
  }
  script_rpgdata+="],";

  String script_ppgdata ="labels:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    if(timeatfft[ig]!=""){
      script_ppgdata+="'"+String(timeatfft[ig])+"'";
      if(ig<GRAPHDATASIZE-1)
        script_ppgdata+=",";
    }
  }
  script_ppgdata+="],\
              datasets:[{\
              label:'Pitching Period[s]',\
              data:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    if(timeatfft[ig]!=""){
      script_ppgdata+=pitchperiod[ig];
      if(ig<GRAPHDATASIZE-1)
        script_ppgdata+=",";
    }
  }
  script_ppgdata+="],";

 String script_gmgdata ="labels:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    if(timeatfft[ig]!=""&& rollperiod[ig]!=0){
      script_gmgdata+="'"+String(timeatfft[ig])+"'";
      if(ig<GRAPHDATASIZE-1)
        script_gmgdata+=",";
    }
  }
  script_gmgdata+="],\
              datasets:[{\
              label:'GM[m]',\
              data:[";
  for(int16_t ig=0;ig<GRAPHDATASIZE;ig++){
    if(timeatfft[ig]!="" && rollperiod[ig]!=0){
      //float gm=2* 0.1 * rollperiod[ig];
      float gm=pow((float) 2*kgm * breadthmld / rollperiod[ig], 2);
      script_gmgdata+=gm;
      if(ig<GRAPHDATASIZE-1)
        script_gmgdata+=",";
    }
  }
  script_gmgdata+="],";
  

  String scriptg=script_g1+script_rgdata+script_g2+script_pgdata+script_g3+script_rpgdata+script_g4+script_ppgdata+script_g5+script_gmgdata+script_g6;
  server.send(200,"text/html",scriptg);
  
  return;

}

void handleTable(){// "/table"にアクセスされた時の処理関数
  char tbrow[100]; //HTMLを編集する文字配列
  String script_tbl=script_tb1;
  for(int8_t ir=GRAPHDATASIZE-1;ir>=GRAPHDATASIZE-ROWAMOUNT;ir--){
    sprintf(tbrow, "<tr><th>%s</th><th>%f</th><th>%f</th><th>%f</th></tr>", timeatfft[ir],rollperiod[ir], pow((float) 2*kgm * breadthmld / rollperiod[ir], 2), pitchperiod[ir]); 
    script_tbl+=String(tbrow);
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

void taskHandleButton(void *arg){
  while (1){

    // Button C, stop logging
    if (M5.BtnC.wasPressed()){
      if (isLogging){
        isLogging = false;
        output_file.close();
        M5.Lcd.setCursor(180, 221);
        M5.Lcd.printf("Log Stopped");
      }
    }
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
  setupTimeNtp();//NTPで時刻合わせ
  String datetime=getrtcTime();//RTCから年月日時取得

  
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

  
  xTaskCreatePinnedToCore(taskHandleServer,"Task1",8192,NULL,2,NULL,1);//HTTPサーバのマルチスレッド関数を起動
  xTaskCreatePinnedToCore(taskHandleButton,"Task2",2048,NULL,1,NULL,1);//記録停止ボタンのマルチスレッド関数を起動

}


void loop() {
  double majorpeak[2];//Majorピーク周波数 double
  static bool ispotu0s;
  unsigned int iniloopt;//時刻を記録する変数
  M5.update();
  
  for(int8_t i_i=0;i_i<(1000/SMPLPERIOD)/SMPLFREQ_FFT;i_i++){
    iniloopt=millis();
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ); // Stores the triaxial accelerometer.
    gyroX-=gyroOffset[0];gyroY-=gyroOffset[1];gyroZ-=gyroOffset[2];
    M5.IMU.getAccelData(&accX, &accY, &accZ); // Stores the triaxial accelerometer.
    accX-=accOffset[0];accY-=accOffset[1];accZ-=accOffset[2];
    M5.IMU.getAhrsData2(&pitch, &roll, &yaw,gyroX,gyroY,gyroZ,accX, accY, accZ);// Stores the inertial sensor attitude.
    if(SMPLPERIOD-(millis()-iniloopt)>1&&i_i<(1000/SMPLPERIOD)/SMPLFREQ_FFT-1)
      delay(SMPLPERIOD-(millis()-iniloopt)); // Delay 100ms.(コンピューター内の計算にかかった遅延時間も考慮)
  }
  
  //Roll/Pitchの向き・符号を船体工学分野の定義に合わせる
  if(m5putdir==1){//M5StackのSDカードslotを船尾に向けて置いた場合 
    roll*=-1;
  }else if(m5putdir==2){//M5StackのUSB-C端子を船首に向けて置いた場合
    float temp_rp=roll;
    roll=pitch;
    pitch=temp_rp;
  }


  M5.Lcd.setCursor(0, 70);
  M5.Lcd.println("roll,  pitch");
  M5.Lcd.printf("%5.2f  %5.2f deg",  roll, pitch); 
  M5.Rtc.GetDate(&RTCDate);//RTCの日付取得
  M5.Rtc.GetTime(&RTCtime);//RTCの時刻取得
  
  if (isLogging){
    if (output_file.size() < (1024 * 1024 * 1024)){
      output_file.printf("%2d:%2d:%2d,%d,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
                         RTCtime.Hours,RTCtime.Minutes,RTCtime.Seconds, iniloopt, num, gyroX, gyroY, gyroZ, accX, accY, accZ, roll, pitch, yaw, temperature,
                        rollperiod[0], majorpeak[0], pitchperiod[0], majorpeak[1]);
      output_file.flush();
      num++;
    }else{
      output_file.close();
    }
  } 

  if(ispotu0s){//1.0s毎にグラフ表示用データを格納
    memmove(rollgdata,&rollgdata[1],sizeof(rollgdata)-sizeof(float));
    memmove(pitchgdata,&pitchgdata[1],sizeof(pitchgdata)-sizeof(float));
    memmove(gtime,&gtime[1],sizeof(gtime)-sizeof(gtime[1]));
    rollgdata[GRAPHDATASIZE-1]=roll;
    pitchgdata[GRAPHDATASIZE-1]=pitch;
    ispotu0s=false;
    sprintf(gtime[GRAPHDATASIZE-1],"%2d:%02d:%02d",RTCtime.Hours,RTCtime.Minutes,RTCtime.Seconds);
  }else{
      ispotu0s=true;
  }

  memmove(rollfftdata,&rollfftdata[1],sizeof(rollfftdata)-sizeof(float));
  memmove(pitchfftdata,&pitchfftdata[1],sizeof(pitchfftdata)-sizeof(float));
  rollfftdata[FFTsamples-1]=roll;
  pitchfftdata[FFTsamples-1]=pitch;j_f+=1;
  
  if(j_f>=FFTpersamples){//FFT計算
    for(int16_t i_f=0;i_f<FFTsamples;i_f++){
      vReal_r[i_f]=(double)rollfftdata[i_f];
      vReal_p[i_f]=(double)pitchfftdata[i_f];
      vImag_r[i_f] = 0.0; vImag_p[i_f] = 0.0; 
    }
    computeFFT(majorpeak);
    j_f=0;

    if((majorpeak[0]>0&&majorpeak[0]<2.0/SMPLFREQ_FFT)||(majorpeak[1]>0&&majorpeak[1]<2.0/SMPLFREQ_FFT)){
      PushtoGoogleSheet((float)1.0/majorpeak[0],(float)1.0/majorpeak[1]);//Google Spread SheetにRolling周期をPush
      memmove(rollperiod,&rollperiod[1],sizeof(rollperiod)-sizeof(float));
      memmove(pitchperiod,&pitchperiod[1],sizeof(pitchperiod)-sizeof(float));
      memmove(timeatfft,&timeatfft[1],sizeof(timeatfft)-sizeof(timeatfft[1]));
      rollperiod[GRAPHDATASIZE-1]=(float)1.0/majorpeak[0];pitchperiod[GRAPHDATASIZE-1]=(float)1.0/majorpeak[1];
      
      M5.Lcd.setCursor(0, 120);
      M5.Lcd.println("rolling,  pitching period");
      M5.Lcd.printf("%5.2f  %5.2f s",  rollperiod[GRAPHDATASIZE-1], pitchperiod[GRAPHDATASIZE-1]); 
      M5.Lcd.setCursor(0, 170);
      M5.Lcd.print("Estimated GM: ");
      M5.Lcd.printf("%3.2f m", pow((float) 2*kgm * breadthmld / rollperiod[GRAPHDATASIZE-1], 2)); //LcdにGM推定値を表示
      M5.Lcd.println();//改行

      sprintf(timeatfft[GRAPHDATASIZE-1],"%2d:%02d:%02d",RTCtime.Hours,RTCtime.Minutes,RTCtime.Seconds);
    }
    return;
  }

  // Button B  calibration
  if (M5.BtnB.wasPressed()){
    if (isLogging){
      calibration(gyroOffset, accOffset); // IMUキャリブレーション
      return;
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


String getrtcTime(){
  m5.Lcd.setTextSize(2);

  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCtime);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(60, 80);
  M5.Lcd.printf("%d/%2d/%2d", RTCDate.Year, RTCDate.Month, RTCDate.Date);
  M5.Lcd.setCursor(80, 140);
  M5.Lcd.printf("%02d:%02d:%02d", RTCtime.Hours, RTCtime.Minutes, RTCtime.Seconds);
  delay(1000);
  M5.Lcd.fillScreen(BLACK);

  return zeroPadding(RTCDate.Year, 4) + zeroPadding(RTCDate.Month, 2) + zeroPadding(RTCDate.Date, 2)+ zeroPadding(RTCtime.Hours, 2)+ zeroPadding(RTCtime.Minutes, 2);

}

void getntpTime(){
  SetwifiSD();
  delay(500);
  M5.Lcd.setTextSize(2);
  configTime(3600*timediff,0,"ntp.nict.jp","time.google.com","ntp.jst.mfeed.ad.jp");//NTPサーバと時刻を同期させる
  struct tm tm;
  if(getLocalTime(&tm)){
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(60,80);
    M5.Lcd.printf("%d/%2d/%2d",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday);
    M5.Lcd.setCursor(80,140);
    M5.Lcd.printf("%02d:%02d:%02d",tm.tm_hour,tm.tm_min,tm.tm_sec);
    delay(1000);M5.Lcd.fillScreen(BLACK);
  } 

}



void setupTimeNtp() {
  struct tm tm;
  getntpTime();
  for(int8_t i=0;i<10;i++){
    getLocalTime(&tm);
    RTCtime.Hours = tm.tm_hour;  // Set the time.  
    RTCtime.Minutes = tm.tm_min;
    RTCtime.Seconds = tm.tm_sec;
    if (!M5.Rtc.SetTime(&RTCtime)) Serial.println("wrong time set!");
    // and writes the set time to the real
    // time clock. 
    RTCDate.Year = tm.tm_year+1900;  // Set the date.  
    RTCDate.Month = tm.tm_mon+1;
    RTCDate.Date = tm.tm_mday;
    if (!M5.Rtc.SetDate(&RTCDate)) Serial.println("wrong date set!");
    M5.Lcd.fillScreen(BLACK);
    delay(200);
  }
  
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
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.print("IP: ");M5.Lcd.println(WiFi.localIP());
  M5.Lcd.setCursor(180, 221);
  M5.Lcd.print("Logging");

}

void readparticular(){
  File rfp;//設定読み込み用ファイル
  unsigned int cnt = 0;
  char data[100]; char *str;

  rfp = SD.open("/read/shipparticular.txt", FILE_READ);
  while(rfp.available()&&cnt<100){
    data[cnt++] = rfp.read();
  }
  strtok(data,":");
  str = strtok(NULL,"\r\n");    // CR
  breadthmld=atof(str);//船の型幅[m]

  strtok(NULL,":");
  str = strtok(NULL,"\r\n");    // CR
  kgm=atof(str);//船の慣動半径の係数

  strtok(NULL,":");
  str = strtok(NULL,"\r\n");    // CR
  m5putdir=atoi(str);//M5Stackを置く向き どの面を船首に向けるか

  M5.Lcd.printf("Breadth: %f[m]\tk:%f\n",breadthmld,kgm);
  M5.Lcd.printf("Put%d\n",m5putdir);
  rfp.close();
}

void SetwifiSD(){
  File rfp;
  unsigned int cnt = 0;
  char data[512];
  
  char *str,*str_1;

  rfp = SD.open("/read/wificloud.txt", FILE_READ);
  while(rfp.available()&&cnt<512){
    data[cnt++] = rfp.read();
  }

  for(int i=0;i<4;i++){
    if(i==0){//１行・1列目の読み込み(読み飛ばし)
      str_1=strtok(data,":");//SSID読み込み
    }else{//2行目～・1列目
      str_1=strtok(NULL,":");//,区切り
    }
    
    str = strtok(NULL,"\r"); // CR区切り
    switch (i){
      case 0://"wifi.csv"１行目2列目にWiFi SSID
        if (str_1[0]=='W'&&str_1[1]=='i'){//1・2文字目だけ確認してこの行にWiFiのSSIDが記載されているか確認
          strncpy(ssid, str, strlen(str));
          M5.Lcd.printf("WIFI-SSID: %s\n",ssid);
        }
        break;

      case 1://2行目にWiFi Password
        strncpy(pass, str, strlen(str));
        M5.Lcd.printf("WIFI-PASS: %s\n",pass);
        break;

      case 2://3行目に時差[h]
        timediff=atoi(str);//時差[h] -12~12
        M5.Lcd.printf("time difference from UTC %d[hour]\n",timediff);
        break;

      case 3://4行目にGoogleスプレッドシートのデプロイされたURL
        strncpy(published_url, str, strlen(str));
        M5.Lcd.printf("To Google Spread Sheet(cloud)\n");
        break;
    }
  }

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
  FFT_r.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // 窓関数
  FFT_r.compute(FFTDirection::Forward); // FFT処理(複素数で計算)
  vReal_r[0]/=200;vReal_r[1]/=100;vReal_r[2]/=100;vReal_r[3]/=10;
  FFT_r.complexToMagnitude();
  majorpeak[0]=FFT_r.majorPeak();
  FFT_p.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // 窓関数
  FFT_p.compute(FFTDirection::Forward); // FFT処理(複素数で計算)
  vReal_p[0]/=200;vReal_p[1]/=100;vReal_p[2]/=100;vReal_p[3]/=10;
  FFT_p.complexToMagnitude();
  majorpeak[1]=FFT_p.majorPeak();

  for(int i=0;i<2;i++){
    if(majorpeak[i]>SMPLFREQ_FFT/2)
      majorpeak[i]=0;
  }

}

void PushtoGoogleSheet(float rollingperiod,float pitchingperiod){
  StaticJsonDocument<500> doc;
  char pubMessage[256];
  char buf[32];

    JsonArray dataValues=doc.createNestedArray("Period");
    dataValues.add(String(rollingperiod,4));
    dataValues.add(String(pow((float)2*breadthmld*kgm/rollingperiod,2),4));
    dataValues.add(String(pitchingperiod,4));
    serializeJson(doc,pubMessage);
    HTTPClient http;//HTTP通信開始
    http.begin(published_url);

    Serial.println("HTTPCommunicationPost");
    int httpCode=http.POST(pubMessage);
    
    if(httpCode>0){
      Serial.printf("HTTP Respose:%d",httpCode);M5.Lcd.println();

      if(httpCode==HTTP_CODE_OK){
       Serial.println("HTTP Success!!");
        String payload=http.getString();
        Serial.println(payload);
      }
    }else{
      Serial.println("FAILED");
      Serial.printf("HTTP failed,error:%s\n",http.errorToString(httpCode).c_str());
     

    http.end();
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
  
  rfp = SD.open("/read/script/periodchart1.html", FILE_READ);
  while(rfp.available()){
    script_g4 += rfp.readStringUntil('\n');
  }
  rfp.close();

  rfp = SD.open("/read/script/periodchart2.html", FILE_READ);
  while(rfp.available()){
    script_g5 += rfp.readStringUntil('\n');
  }
  rfp.close();

  rfp = SD.open("/read/script/gmchart.html", FILE_READ);
  while(rfp.available()){
    script_g6 += rfp.readStringUntil('\n');
  }
  rfp.close();

  rfp = SD.open("/read/script/table.html", FILE_READ);
  while(rfp.available()){
    script_tb1 += rfp.readStringUntil('\n');
  }
  rfp.close();
  
}


