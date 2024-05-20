/*
*******************************************************************************
* Copyright (c) 2021 by M5Stack
*                  Equipped with M5Core2 sample source code
*                          配套  M5Core2 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/core2
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/core2
*
* Describe: RTC--实时时钟示例
* Date: 2022/1/9
*******************************************************************************
*/
#include <M5Core2.h>
#include <WiFi.h>
#include <TinyGPS++.h>

#define JST (3600L*9)//日本の時差9h

RTC_TimeTypeDef RTCtime;
RTC_DateTypeDef RTCDate;

char timeStrbuff[64];
File rfp;//設定読み込み用txtfile
char ssid[32];  char password[32];//WiFiのSSIDとパスワードを格納する変数　グローバル変数でないと、エラーになる

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
    }
  }

  M5.Lcd.println("Connecting...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      M5.Lcd.print(".");
  }

  M5.Lcd.print("IP: ");
  M5.Lcd.println(WiFi.localIP());

  rfp.close();
}

void getntpTime(){
  SetwifiSD();
  WiFi.begin(ssid,password);  //Connect wifi and return connection status.  
  while (WiFi.status() !=WL_CONNECTED) {  //If the wifi connection fails.  
    delay(1000);           //delay 1.0s. 
    M5.Lcd.print(".");
  }
  M5.Lcd.println("\nCONNECTED!");
  M5.Lcd.print("\r\nWifi connected\r\nIP address:");
  M5.Lcd.println(WiFi.localIP());
  delay(500);
  M5.Lcd.setTextSize(2);
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

}




void flushTime() {
  M5.Rtc.GetTime(&RTCtime);  // Gets the time in the real-time clock.
                             // 获取实时时钟内的时间
  M5.Rtc.GetDate(&RTCDate);
  sprintf(timeStrbuff, "%d/%02d/%02d\n %02d:%02d:%02d", RTCDate.Year,
          RTCDate.Month, RTCDate.Date, RTCtime.Hours, RTCtime.Minutes,
          RTCtime.Seconds);
  // Stores real-time time and date data
  // to timeStrbuff.
  M5.lcd.setCursor(10, 100);
  // Move the cursor position to (x,y).  
  M5.Lcd.println(timeStrbuff);
  // Output the contents of.  
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
    WiFi.disconnect();
    if (!M5.Rtc.SetDate(&RTCDate)) Serial.println("wrong date set!");

    delay(200);
  }
  
}
/* After M5Core2 is started or reset
the program in the setUp () function will be run, and this part will only be run
once. 在 M5Core2
启动或者复位后，即会开始执行setup()函数中的程序，该部分只会执行一次。 */
void setup() {
  M5.begin();  // Init M5Core2.  初始化 M5Core2
  M5.Lcd.fillScreen(BLACK);          // Set the screen background color to black.
  M5.Lcd.setTextColor(WHITE, BLACK); // Sets the foreground color and background color of the displayed text.
  M5.Lcd.setTextSize(2);             // Set the font size.
  delay(1000);
  setupTimeNtp();//NTPでの時刻合わせ 
}

/* After the program in setup() runs, it runs the program in loop()
The loop() function is an infinite loop in which the program runs repeatedly
在setup()函数中的程序执行完后，会接着执行loop()函数中的程序
loop()函数是一个死循环，其中的程序会不断的重复运行 */
void loop() {
  flushTime();
  delay(1000);
}

