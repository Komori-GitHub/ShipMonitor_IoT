#include <M5Core2.h>
#include <TinyGPS++.h>

RTC_TimeTypeDef RTCtime;
RTC_DateTypeDef RTCDate;
TinyGPSPlus gps;// The TinyGPS++ object


char timeStrbuff[64];
File rfp;//設定読み込み用txtfile
char ssid[32];  char password[32];//WiFiのSSIDとパスワードを格納する変数　グローバル変数でないと、エラーになる
int8_t timediff=9;//時差[h] -12~12

static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (Serial2.available() > 0) gps.encode(Serial2.read());
    } while (millis() - start < ms);
    M5.Lcd.clear();
}


void SetwifiSD(){
  File rfp;
  unsigned int cnt = 0;
  char data[512];
  
  char *str;

  rfp = SD.open("/read/wificloud.txt", FILE_READ);
  while(rfp.available()&&cnt<512){
    data[cnt++] = rfp.read();
  }

  for(int i=0;i<5;i++){
    if(i==0){//１行・1列目の読み込み(読み飛ばし)
      strtok(data,":");//SSID読み込み
    }else{//2行目～・1列目
      strtok(NULL,":");//,区切り
    }
    
    str = strtok(NULL,"\r"); // CR区切り
    switch (i){
      case 0://"wifi.csv"１行目2列目にWiFi SSID
        strncpy(ssid, str, strlen(str));
        M5.Lcd.printf("WIFI-SSID: %s\n",ssid);
        break;
      case 1://2行目にWiFi Password
        strncpy(password, str, strlen(str));
        M5.Lcd.printf("WIFI-PASS: %s\n",password);
        break;
      case 2://3行目に時差
        timediff=atoi(str);//時差[h] -12~12
        M5.Lcd.printf("time difference from UTC %d[hour]\n",timediff);
        break;
    }
  }
  delay(1000);
  rfp.close();
}

void displayInfo() {
  M5.Lcd.setCursor(0, 40, 4);
  M5.Lcd.print(F("Latitude:    "));
   if (gps.location.isValid()) {
     M5.Lcd.print(gps.location.lat(), 6);

   } else {
     M5.Lcd.print(F("INVALID"));
   }

   M5.Lcd.println();
   M5.Lcd.print(F("Longitude:    "));
   if (gps.location.isValid()) {
     M5.Lcd.print(gps.location.lng(), 6);
   } else {
     M5.Lcd.print(F("INVALID"));
   }

  M5.Lcd.println();
  M5.Lcd.print(F("Altitude:    "));
  if (gps.altitude.isValid()) {
    M5.Lcd.print(gps.altitude.meters());
  } else {
    M5.Lcd.print(F("INVALID"));
  }

  M5.Lcd.println();
  M5.Lcd.print(F("Satellites:    "));
  if (gps.satellites.isValid()) {
    M5.Lcd.print(gps.satellites.value());
  } else {
    M5.Lcd.print(F("INVALID"));
  }

  M5.Lcd.println();
  M5.Lcd.print(F("Date: "));
  if (gps.date.isValid()) {
    if(gps.date.day()>1&&gps.date.day()<30){//時差で日付を修正した際に間違った日にちが登録されるかもしれない1日と30・31日は日付修正を行わない。
      if (gps.time.hour() + timediff > 24){ // 時差を調整したときの月日ずれを修正
        RTCDate.Date = gps.date.day() + 1;
      }else if (gps.time.hour() + timediff < 0){
        RTCDate.Date = gps.date.day() - 1;
      }
      RTCDate.Month =gps.date.month();   
      RTCDate.Year =gps.date.year();
    }
    
    M5.Lcd.printf("%d",gps.date.year());
    M5.Lcd.print(F("/"));
    M5.Lcd.printf("%02d",gps.date.month());
    M5.Lcd.print(F("/"));
    M5.Lcd.printf("%02d",gps.date.day());
    
  } else {
    M5.Lcd.print(F("INVALID"));
  }

  M5.Lcd.println();
  M5.Lcd.print(F("Time: "));
  if (gps.time.isValid()) {
    RTCtime.Hours=(gps.time.hour()+timediff)%24;
    RTCtime.Minutes =gps.time.minute();
    RTCtime.Seconds = gps.time.second();

    M5.Lcd.printf("%02d",RTCtime.Hours);//M5.Lcd.print(gps.time.hour());
    M5.Lcd.print(F(":"));
    M5.Lcd.printf("%02d",RTCtime.Minutes);//M5.Lcd.print(gps.time.minute());
    M5.Lcd.print(F(":"));
    M5.Lcd.printf("%02d",RTCtime.Seconds);//M5.Lcd.print(gps.time.second());
    M5.Lcd.print(F("."));
  } else {
    M5.Lcd.print(F("INVALID"));
  }
  if (!M5.Rtc.SetTime(&RTCtime)) M5.Lcd.println("wrong time set!");
  if (!M5.Rtc.SetDate(&RTCDate)) M5.Lcd.println("wrong date set!");
}

void flushTime() {
  M5.Rtc.GetTime(&RTCtime);  // Gets the time in the real-time clock.
                             // 获取实时时钟内的时间
  M5.Rtc.GetDate(&RTCDate);
  sprintf(timeStrbuff, "%d/%02d/%02d %02d:%02d:%02d", RTCDate.Year,
          RTCDate.Month, RTCDate.Date, RTCtime.Hours, RTCtime.Minutes,
          RTCtime.Seconds);
  // Stores real-time time and date data
  // to timeStrbuff.
  // 将实时时间、日期数据存储至timeStrbuff
  M5.lcd.setCursor(10, 100);
  // Move the cursor position to (x,y).  移动光标位置到(x,y)处
  M5.Lcd.println(timeStrbuff);
  // Output the contents of.  输出timeStrbuff中的内容
}

void setupTimeGnss(){
  M5.lcd.setCursor(10, 10);
  if (gps.time.isValid()) {
    RTCtime.Hours=(gps.time.hour()+timediff)%24;
    RTCtime.Minutes =gps.time.minute();
    RTCtime.Seconds = gps.time.second();
    M5.Lcd.println("GNSS time VALID");
    if (!M5.Rtc.SetTime(&RTCtime))
      Serial.println("wrong time set!");

  }  else {
    M5.Lcd.println("GNSS time INVALID");
  }

 
  if (gps.date.isValid()) {

    if(gps.date.day()>1&&gps.date.day()<30){//時差で日付を修正した際に間違った日にちが登録されるかもしれない1日と30・31日は日付修正を行わない。
      if (gps.time.hour() + timediff > 24){ // 時差を調整したときの月日ずれを修正
        RTCDate.Date = gps.date.day() + 1;
      }else if (gps.time.hour() + timediff < 0){
        RTCDate.Date = gps.date.day() - 1;
      }
      RTCDate.Month =gps.date.month();   
      RTCDate.Year =gps.date.year();
      M5.Lcd.println("GNSS date VALID");
    }
    if (!M5.Rtc.SetDate(&RTCDate)) 
      Serial.println("wrong date set!");
      
  } else {
    M5.Lcd.println("GNSS date INVALID");
  }

}


void setup() {
    M5.begin(true, true, true, false, kMBusModeInput);
    /*   kMBusModeOutput,powered by USB or Battery
    kMBusModeInput,powered by outside input need to fill in this Otherwise
    M5Core2 will not work properly
    由外部供电时此项必填,否则M5Core2将无法正常工作 */
    Serial2.begin(9600, SERIAL_8N1, 13, 14);
    M5.Lcd.setTextColor(WHITE, BLACK);
    SetwifiSD();
    smartDelay(1000);

}

void loop() {
    setupTimeGnss();
    flushTime();
    smartDelay(200);
}
