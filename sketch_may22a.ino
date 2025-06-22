/* 
 * 基于ESP32的物联网综合监测终端
 * 功能特点：
 * 1. 多WiFi网络自动切换
 * 2. MQTT协议通信（巴法云）
 * 3. 环境数据显示（OLED）
 * 4. 报警状态指示（声光）
 * 5. 数据本地存储（SD卡）
 * 6. 远程固件更新（OTA）
 * 7. 历史数据查询（SD卡）
 */
/******************** 库文件引入 ********************/
#include <WiFi.h>           // WiFi通信
#include <PubSubClient.h>   // MQTT客户端
#include <HTTPUpdate.h>     // OTA更新
#include <HTTPClient.h>
#include <ArduinoJson.h>    // JSON处理
#include <U8g2lib.h>        // OLED显示
#include <Wire.h>           // I2C通信
#include "FS.h"             // 文件系统
#include "SD.h"             // SD卡操作
#include "SPI.h"            // SPI通信
#include <time.h>           // 时间处理
#include <EEPROM.h>  // 添加EEPROM库
#include <TFT_eSPI.h> // 硬件专用库
#include <TJpg_Decoder.h> // 包含 jpeg 解码器库
#include <SPI.h>
#include "font_20.h"            //20号中文字体
#define DATA_TOPIC "scada004"    // 默认数据发布主题
#define ERROR_TOPIC "scada004" // 错误信息主题
#define MAX_RECORDS 100      // 最大返回记录数
#define JSON_BUFFER_SIZE 1024 // JSON处理缓冲区大小
#define USE_HSPI_PORT  // 使用HSPI端口
//#define TFT_BROWN 0x38E0
//#define ST7789_DRIVER
//#define TFT_WIDTH  240
//#define TFT_HEIGHT 320
// TFT引脚配置 (HSPI)
//#define TFT_MISO -1    // 主设备输入从设备输出
#define TFT_MOSI 13    // 主设备输出从设备输入
#define TFT_SCLK 14    // 串行时钟
#define TFT_CS   15    // 片选控制引脚
#define TFT_DC   27    // 数据/命令控制引脚
#define TFT_RST  26    // 复位引脚 (可接开发板RST)
//#define TFT_RST  -1   // 如果显示屏复位接ESP32板载RST，则设为-1
// 触摸屏引脚 (HSPI)
//#define TOUCH_DO  12   // 触摸屏数据输出
//#define TOUCH_DIN 13   // 触摸屏数据输入
//#define TOUCH_CLK 14   // 触摸屏时钟
//#define TOUCH_CS  33   // 触摸屏片选引脚(T_CS)
// SD卡引脚 (VSPI)
#define SD_MISO 19     // SD卡主入从出
#define SD_MOSI 23     // SD卡主出从入
#define SD_SCLK 18     // SD卡时钟
#define SD_CS    5     // SD卡片选
// 创建两个SPI实例
SPIClass spiSD(VSPI);   // SD卡使用VSPI
SPIClass spiTFT(HSPI);  // TFT使用HSPI
TFT_eSPI tft = TFT_eSPI();  // 创建TFT对象
// 用于将每个图块渲染到 TFT 上。如果您使用不同的 TFT 库，
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // 如果图像超出屏幕底部，则停止继续解码
  if ( y >= tft.height() ) return 0;
  // 此函数会在 TFT 边界自动裁剪图像块的渲染
  tft.pushImage(x, y, w, h, bitmap); // 推送图像块到 TFT
  // 返回 1 以解码下一个图块
  return 1;
}
/******************** 硬件配置 ********************/
// OLED显示屏配置（SH1106 128x64）
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*SCL=*/22, /*SDA=*/21, U8X8_PIN_NONE);
// 外设引脚定义
const int ledPin = 2;      // LED指示灯
const int buzzerPin = 25;  // 蜂鸣器引脚
/******************** 网络配置 ********************/
// 多WiFi网络配置（优先级排序）
struct WiFiConfig {
  const char* ssid;
  const char* password;
};
WiFiConfig networks[] = {
  {"CMCC-seDU", "aFucTAC4"},      // 主网络
  {"TP-LINK_5A34F0", "1234567890"},// 备用1
  {"MiFi-E103", "1234567890"}     // 备用2
};
const int numNetworks = sizeof(networks)/sizeof(networks[0]);
// 巴法云MQTT配置
const char* mqttServer = "bemfa.com";
const int mqttPort = 9501;
const char* clientID = "a0f1fc0f1eb2df7c877887eb05a81730";
String upUrl = "http://bin.bemfa.com/b/1BcYTBmMWZjMGYxZWIyZGY3Yzg3Nzg4N2ViMDVhODE3MzA=led002.bin";
// API配置
const String api_url = "https://api.bemfa.com/api/device/v1/bin?uid=a0f1fc0f1eb2df7c877887eb05a81730&topic=led002&type=1";
// EEPROM配置
#define EEPROM_SIZE 4      // 分配4字节空间 (足够存储int)
#define EEPROM_ADDR 0      // 存储地址偏移量
int local_value ;           // 本地V值
/******************** 主题配置 ********************/
enum Topics {
  TEMP_TOPIC,    // 温度主题 temp004
  LED_TOPIC,     // LED控制 led002 （新增)SD卡查询
  TOPIC_COUNT
};
const char* subTopics[TOPIC_COUNT] = {
  "temp004",
  "led002"
};
/*=============== 函数前置声明 ===============*/
String getDataByTimeRange(const String &startStr, const String &endStr);
/******************** 全局变量 ********************/
WiFiClient espClient;           // WiFi客户端
PubSubClient client(espClient); // MQTT客户端
// 传感器数据（来自第一段程序）
float temperature = 0.0;        // 当前温度
float humidity = 0.0;           // 当前湿度
// 报警系统
bool dataAlarm = false;         // 数据报警标志
String ALERT = "报警点:查询中...";              // 报警信息
String sd_vol;//SD卡剩余容量
unsigned long lastBlinkTime = 0;// LED闪烁计时
const int BLINK_INTERVAL = 500; // 闪烁间隔(ms)
// 设备状态监测
struct TopicStatus {
  unsigned long lastReceive;    // 最后接收时间
  bool isOnline;                // 在线状态
};
TopicStatus topicStatus[TOPIC_COUNT]; // 主题状态数组
const unsigned long CHECK_INTERVAL = 30000; // 主题在线状态检测间隔
// SD卡相关
unsigned long lastSDWrite = 0;          // 最后写入时间
const int SD_WRITE_INTERVAL = 300000;    // 写入间隔(ms)10分钟600000
String sdQueryResult = "";              // 查询结果缓存
// 系统状态
int icon ;//图标
int userLedState = LOW;         // 用户LED状态
//const int numNetworks = sizeof(networks) / sizeof(networks[0]);
int currentNetworkIndex = 0;       // 当前尝试的网络索引
int cnt1 = 0;                   // 网络重连计数器
bool allOnlineFlag = true;       // 全在线标志
int fmwavl;//固件赋值
long timezone = 8;     // 时区偏移（小时）
byte daysavetime = 0;  // 夏令时设置（0=禁用，1=启用）
// 初始化EEPROM
void initEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  // 从EEPROM读取存储的值
  EEPROM.get(EEPROM_ADDR, local_value);
  // 验证读取值是否有效 (首次运行时EEPROM可能是255)
  if (local_value < 0 || local_value > 200) {
    local_value = 0;  // 默认值
    EEPROM.put(EEPROM_ADDR, local_value);
    EEPROM.commit();
    Serial.println("初始化EEPROM默认值: " + String(local_value));
  } else {
    Serial.println("从EEPROM读取值: " + String(local_value));
  }
}
// 保存值到EEPROM
void saveToEEPROM(int value) {
  EEPROM.put(EEPROM_ADDR, value);
  if (EEPROM.commit()) {
    Serial.println("保存到EEPROM成功: " + String(value));
  } else {
    Serial.println("EEPROM保存失败!");
  }
}
/******************** 初始化设置 ********************/
void setup() {
  Serial.begin(115200);
  Serial.println("\n系统启动中...");
  delay(1000);
  // 初始化硬件外设
  pinMode(ledPin, OUTPUT);      // 配置LED引脚
  ledcSetup(0, 1000, 8);       // 蜂鸣器PWM通道
  ledcAttachPin(buzzerPin, 0); // 绑定蜂鸣器
  digitalWrite(ledPin, userLedState);
  // 初始化EEPROM并读取存储值
  initEEPROM();
  // 初始化OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setPowerSave(0);
   // 初始化SPI总线 (SCK,MISO,MOSI,CS)
  spiSD.begin(18, 19, 23, 5);   // SD卡引脚配置
  spiTFT.begin(14, 12, 13, 15); // TFT引脚配置
  if (!SD.begin()) {
    Serial.println("SD卡初始化失败!");
    sdQueryResult = "SD卡故障";
  } else {
    if (!SD.exists("/data")) {// 确保/data目录存在（新增）
      SD.mkdir("/data");
      Serial.println("已创建/data目录");
    }
  }
   // 显示SD卡信息
    uint8_t cardType = SD.cardType();
    Serial.print("SD卡类型: ");
    if(cardType == CARD_NONE){
        Serial.println("未检测到SD卡");
    } else if(cardType == CARD_MMC){
        Serial.println("MMC卡");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC卡");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC卡");
    } else {
        Serial.println("未知类型");
    }
  // 获取 SD 卡信息
  uint64_t cardSize = SD.cardSize() / (1024 * 1024); // 转换为 MB
  uint64_t usedSpace = SD.usedBytes() / (1024 * 1024);
  uint64_t freeSpace = (SD.cardSize() - SD.usedBytes()) / (1024 * 1024);
  // 打印信息
  Serial.printf("SD 卡总容量: %llu MB\n", cardSize);
  Serial.printf("已用空间: %llu MB\n", usedSpace);
  Serial.printf("剩余空间: %llu MB\n", freeSpace);
  sd_vol = ("SD卡剩余: "+ String(freeSpace) + "MB\n");
  // 初始化主题状态
  for(int i=0; i<TOPIC_COUNT; i++){
    topicStatus[i].lastReceive = millis();
    topicStatus[i].isOnline = true;
  }
  delay(1000);
  // 初始化TFT屏幕
  tft.init();
  tft.setRotation(3);   // 屏幕方向（0-3),1为横向模式
  tft.loadFont(font_20);  //设定读取字体文件名
  tft.fillScreen(TFT_BLACK); // 清屏背景为黑色,白色TFT_WHITE,灰色TFT_DARKGREY,蓝色TFT_BLUE,黑色TFT_BLACK红色RED
  //tft.writecommand(0x20);//关闭颜色反转
  tft.invertDisplay(0);//关闭颜色反转
  tft.setSwapBytes(true); // 我们需要交换颜色字节（字节序问题）
  //TJpgDec.setJpgScale(2); // 设置图片缩放大小为 1（不缩放）
  TJpgDec.setCallback(tft_output); // 设置渲染回调函数
  //tft.setTextSize(1);//英文字体大小设置
  //pinMode(TFT_BL,OUTPUT);
  //digitalWrite(TFT_BL,HIGH);
  // 连接网络
  setupWiFi();
  // 配置MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  tft.fillScreen(TFT_BLACK); // 清屏背景为黑色
}
/******************** 主循环 ********************/
void loop() {
  // MQTT连接维护
  if (!client.connected()) reconnect();
  client.loop();
  // 系统状态检测
  checkTopicsStatus();    // 主题在线状态
  handleSDRecording();    // SD卡数据记录
  //handleAlarm();          // 报警处理
  updateDisplay();        // 更新显示
  handleSerialCommand();  // 串口命令处理
  lcd_tft();//TFT屏显示
}
/******************** 网络连接函数 ********************/
void setupWiFi() {
  Serial.println("\n启动多网络连接...");
  for (int i = 0; i < numNetworks; i++) {
    currentNetworkIndex = i;
    const char* currentSSID = networks[currentNetworkIndex].ssid;
    const char* currentPassword = networks[currentNetworkIndex].password;
    Serial.printf("尝试网络 %d/%d: %s\n", currentNetworkIndex+1, numNetworks, currentSSID);
    WiFi.disconnect(true);
    WiFi.begin(currentSSID, currentPassword);
    unsigned long startTime = millis();
    bool connected = false;
    while (millis() - startTime < 15000) { // 15秒连接超时
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        break;
      }
      delay(1000);
      Serial.print(".");
                    cnt1++;//自加
      String cnt2 = ("重启计数  600 / " + String(cnt1)); 
      updateConnectDisplay();
          if (cnt1 == 180){
          WiFi.mode(WIFI_OFF);//关闭WiFi,
          Serial.println("WiFi已关闭");
          }
          else if (cnt1 >= 600){
          ESP.restart();//重启
          } 
    }
    if (connected) {
      Serial.printf("\n连接成功! 当前网络: %s\nIP地址: ", currentSSID);
      Serial.println(WiFi.localIP());
      sdQueryResult = "网络连接成功";
      cnt1 = 0;//网络连接时间复位
    // 配置网络时间
    Serial.println("正在连接时间服务器");
    configTime(3600*timezone, daysavetime*3600, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
    struct tm tmstruct;
    delay(2000);
    tmstruct.tm_year = 0;
    getLocalTime(&tmstruct, 5000);
    Serial.printf("\n当前时间: %d-%02d-%02d %02d:%02d:%02d\n",
                (tmstruct.tm_year)+1900,
                (tmstruct.tm_mon)+1,
                tmstruct.tm_mday,
                tmstruct.tm_hour,
                tmstruct.tm_min,
                tmstruct.tm_sec);
  // 打印初始化的时间戳
  time_t now;
  time(&now);
  Serial.printf("\n当前UTC时间戳: %lu\n", now);
      return;
    }
  Serial.println("\n连接失败，尝试下一个网络...");
  Serial.println("所有网络连接失败!");
  sdQueryResult = "网络连接失败";
 }
}
/******************** MQTT连接管理 ********************/
void reconnect() {
   while (!client.connected()) {
    // 先检查WiFi连接
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi连接丢失，尝试重新连接...");
      setupWiFi(); // 重新执行多网络连接
    }
    Serial.print("尝试MQTT连接...");
    if (client.connect(clientID)) {//mqttUser, mqttPassword//用户名密码
      Serial.println("连接成功");
      cnt1 = 0;//mqtt重联复位
      for (int i = 0; i < TOPIC_COUNT; i++) {
        if(client.subscribe(subTopics[i])){
          Serial.println("已订阅主题: " + String(subTopics[i]));
        } else {
          Serial.println("订阅失败: " + String(subTopics[i]));
        }
      }
    } else {
      Serial.print("失败, rc=");
      Serial.print(client.state());
      Serial.println(" 5秒后重试...");
      delay(5000);
    }
  }
}
/******************** MQTT回调处理 ********************/
void callback(char* topic, byte* payload, unsigned int length) {
  updateTopicStatus(topic);// 更新主题状态
  char msg[length+1];
  memcpy(msg, payload, length);
  msg[length] = '\0';
  //Serial.print("收到消息 [");
  //Serial.print(topic);
  //Serial.print("]: ");
  //Serial.println(msg);
  icon = 0x0020;//关闭图标
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, msg);
  if(error){
    Serial.print("JSON解析失败: ");
    Serial.println(error.c_str());
    return;
  }
// 在回调函数内部处理led报警：
  else if(doc.containsKey("RTValue")){
    String alertMsg = "";
    for(JsonObject obj : doc["RTValue"].as<JsonArray>()){
        const char* name = obj["name"];
        int value = obj["value"];
        int type = obj["type"];
        if((value == 1) && (type == 1)){
          //  Serial.printf("[ALERT] Topic: %s | Pin: %s\n", topic, name);
            if (alertMsg.length() > 0) {
                alertMsg += ", ";
            }
            alertMsg += String(topic) + "/" + String(name);
        }
    }
    if (alertMsg.length() > 0) {
        ALERT = "报警点: " + alertMsg;
        dataAlarm = true;//嗡鸣器触发
    }
    else if(alertMsg.length() == 0) {
        dataAlarm = false;//嗡鸣器复位
        ALERT = "报警点:无"; // 初始化报警状态
    }
}
  if(strcmp(topic, "temp004") == 0){
    processTempHumi(doc);
  }
  else if(strcmp(topic, "led002") == 0){
        // 调用查询处理器
    handleSDQuery(payload, length);// 处理SD卡查询
  } 
}
void updateTopicStatus(char* topic) {
  for(int i=0; i<TOPIC_COUNT; i++){
    if(strcmp(topic, subTopics[i]) == 0){
      topicStatus[i].lastReceive = millis();
      if(!topicStatus[i].isOnline){
        topicStatus[i].isOnline = true;
        Serial.println("主题恢复在线: " + String(subTopics[i]));
      }
    }
  }
}
/******************** 温湿度数据处理 ********************/
void processTempHumi(JsonDocument& doc) {
  if(doc.containsKey("RTValue")){
    for(JsonObject obj : doc["RTValue"].as<JsonArray>()){
      const char* name = obj["name"];
      int value = obj["value"];
      int type = obj["type"];
      // 更新传感器数据
      if(strcmp(name, "temp") == 0) temperature = value;
      if(strcmp(name, "humi") == 0) humidity = value;
      // 固件更新标志检测
      if(strcmp(name, "fmw") == 0){
        fmwavl = value;
        if(fmwavl == 32){
    // 发送GET请求
    HTTPClient http;//http声明
    http.begin(api_url);
    // 启用重定向跟随
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println("API Response: " + payload);
      // 解析JSON
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);
      if (!error) {
        // 提取API返回的V值
        int api_value = doc["data"]["v"]; // 根据实际JSON结构调整路径
        Serial.println("API V值: " + String(api_value));
        Serial.println("本地V值: " + String(local_value));
        // 比较V值
        if (api_value > local_value) {
          Serial.println("API值 > 本地值");
          // 执行操作后更新本地值
        local_value = api_value;
        saveToEEPROM(local_value);
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_wqy12_t_gb2312);
        u8g2.drawUTF8(0, 35, "触发固件更新请等待!");
        u8g2.sendBuffer();
        Serial.println("触发固件更新...");
        updateBin();
        } else {
          Serial.println("已经是最新版本，无需更新");
          sdQueryResult = "已经是最新版本，无需更新";
        }
        // 添加实际硬件控制代码
        // 例如：digitalWrite(LED_PIN, api_value > local_value);
      } else {
        Serial.println("JSON解析错误: " + String(error.c_str()));
      }
    } else {
      Serial.println("HTTP请求失败, 错误码: " + String(httpCode));
    }
    http.end();
          }
       }
    }
  }
}
/*
/******************** LED控制处理 ********************
void processLEDCommand(JsonDocument& doc) {
  if(doc.containsKey("RTValue")){
    for(JsonObject item : doc["RTValue"].as<JsonArray>()){
      if(strcmp(item["name"], "led1") == 0){
        userLedState = item["value"];
        digitalWrite(ledPin, userLedState);
        Serial.printf("LED状态已更新: %s\n", userLedState ? "ON" : "OFF");
      }
    }
  }
}
/******************** SD卡数据记录 ********************/
void handleSDRecording() {
  if (millis() - lastSDWrite > SD_WRITE_INTERVAL) {
    lastSDWrite = millis();
    // 获取当前时间
    struct tm tmstruct;
    getLocalTime(&tmstruct);
    char timeStr[20];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d",
           tmstruct.tm_year+1900, tmstruct.tm_mon+1, tmstruct.tm_mday,
           tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
    // 构建数据记录
    DynamicJsonDocument doc(128);
    doc["time"] = timeStr;
    doc["temp"] = temperature;
    doc["humi"] = humidity;
    // 写入SD卡
    String data;
    serializeJson(doc, data);
    appendFile(SD, "/data/log.txt", (data + "\n").c_str());
    Serial.println("数据已存储: " + data);
    icon = 0xe070;//显示图标  0xe070
  }
}
/*=============== 时间解析器 ===============*/
// 时间字符串转时间戳（简化版）
time_t parseTime(const char* timeStr) {
    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    // 使用更严格的格式解析
    if (sscanf(timeStr, "%d-%d-%d %d:%d:%d",
            &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
            &tm.tm_hour, &tm.tm_min, &tm.tm_sec) != 6) {
        return 0; // 解析失败
    }
    tm.tm_year -= 1900;// tm_year从1900开始计数
    tm.tm_mon -= 1;// tm_mon范围0-11
   time_t local_time = mktime(&tm);
   return local_time - (timezone * 3600); // 转换为UTC时间戳
}
/*=============== SD卡查询处理器 ===============*/
/*=============== SD卡查询处理器（分批发布版） ===============*/
void handleSDQuery(byte* payload, unsigned int length) {
  /*---- 第1阶段：载荷转换 ----*/
  char msg[length+1];
  memcpy(msg, payload, length);
  msg[length] = '\0';
  /*---- 第2阶段：指令解析 ----*/
  DynamicJsonDocument doc(JSON_BUFFER_SIZE/8);
  DeserializationError error = deserializeJson(doc, msg);
  // JSON解析失败处理
  if (error) {
    String errMsg = "JSON解析失败: " + String(error.c_str());
    sdQueryResult = errMsg;
    client.publish(ERROR_TOPIC, errMsg.c_str());
    return;
  }
  /*---- 第3阶段：参数提取 ----*/
  String pubTopic = DATA_TOPIC; // 默认发布主题
  String startTime, endTime;
  // 提取自定义发布主题
  if (doc.containsKey("pub_topic")) {
    pubTopic = doc["pub_topic"].as<String>();
  }
  // 多格式时间参数提取
  if (doc.containsKey("RTValue")) {
    // 处理复合指令格式
    JsonArray rtArray = doc["RTValue"];
    for (JsonObject obj : rtArray) {
      if (obj.containsKey("start") && obj.containsKey("end")) {
        startTime = obj["start"].as<String>();
        endTime = obj["end"].as<String>();
        break;
      }
    }
  } else if (doc.containsKey("start") && doc.containsKey("end")) {
    // 处理简单指令格式
    startTime = doc["start"].as<String>();
    endTime = doc["end"].as<String>();
  }
  /*---- 第4阶段：参数校验 ----*/
  if (startTime.isEmpty() || endTime.isEmpty()) {
    String errMsg = "时间参数缺失";
    sdQueryResult = errMsg;
    client.publish(ERROR_TOPIC, errMsg.c_str());
    return;
  }
  /*---- 第5阶段：执行查询 ----*/
  String result = getDataByTimeRange(startTime, endTime);
  /*---- 第6阶段：结果处理 ----*/
   // 解析查询结果
  DynamicJsonDocument resultDoc(JSON_BUFFER_SIZE);
  if (deserializeJson(resultDoc, result)) {
    String errMsg = "结果解析失败";
    sdQueryResult = errMsg;
    client.publish(ERROR_TOPIC, errMsg.c_str());
    return;
  }
  // 错误结果处理（保持原样）
  if (resultDoc.containsKey("error")) {
    String fullErr = "查询错误: " + resultDoc["error"].as<String>();
    sdQueryResult = fullErr;
    client.publish(ERROR_TOPIC, fullErr.c_str());
    return;
  }
  // 分批处理逻辑
  if (resultDoc.is<JsonArray>()) {
    JsonArray dataArray = resultDoc.as<JsonArray>();
    int totalRecords = dataArray.size();
    sdQueryResult = "找到 " + String(totalRecords) + " 条记录";
    // 计算分批参数
    const int BATCH_SIZE = 4;
    int totalBatches = (totalRecords + BATCH_SIZE - 1) / BATCH_SIZE; // 向上取整
    int publishedBatches = 0;
    // 分批发布
    for (int batch = 0; batch < totalBatches; batch++) {
      // 创建批次文档
      DynamicJsonDocument batchDoc(1024);
      JsonArray batchArray = batchDoc.to<JsonArray>();
      // 填充当前批次数据
      int startIdx = batch * BATCH_SIZE;
      int endIdx = min((batch+1)*BATCH_SIZE, totalRecords);
      for (int i = startIdx; i < endIdx; i++) {
        batchArray.add(dataArray[i]);
      }
      // 序列化并发布
      String batchPayload;
      serializeJson(batchArray, batchPayload);
      // 添加批次元数据
      batchDoc["batch"] = batch + 1;
      batchDoc["total"] = totalBatches;
      batchDoc["data"] = batchArray;
      if (client.publish(pubTopic.c_str(), batchPayload.c_str())) {
        publishedBatches++;
        Serial.printf("已发布批次 %d/%d (%d 条)\n", 
                     batch+1, totalBatches, endIdx-startIdx);
      } else {
        Serial.println("批次发布失败: " + String(batch+1));
      }
      // 添加短暂延迟确保发送顺序
      delay(50); 
      // 释放内存
      batchDoc.clear();
    }
    // 更新状态信息
    sdQueryResult += "\n分" + String(totalBatches) + "批发布";
    if (publishedBatches < totalBatches) {
      sdQueryResult += " (" + String(publishedBatches) + "批成功)";
    }
  }
}

/*=============== 数据查询引擎 ===============*/
String getDataByTimeRange(const String &startStr, const String &endStr) {
  /*-- 阶段1：准备查询 --*/
  Serial.println("\n=== 启动数据查询 ===");
  Serial.println("时间范围：" + startStr + " - " + endStr);
  /*-- 阶段2：存储介质检查 --*/
  if (!SD.begin()) {
    Serial.println("SD卡重新初始化失败！");
    return "{\"error\":\"存储介质故障\"}";
  }
  /*-- 阶段3：打开数据文件 --*/
  File dataFile = SD.open("/data/log.txt");
  if (!dataFile) {
    Serial.println("无法打开数据文件！");
    return "{\"error\":\"数据文件不可访问\"}";
  }
  /*-- 阶段4：时间转换 --*/
  time_t startTime = parseTime(startStr.c_str());
  time_t endTime = parseTime(endStr.c_str());
  if (!startTime || !endTime) {
    dataFile.close();
    return "{\"error\":\"无效时间格式\"}";
  }
  Serial.printf("转换后时间戳：%ld -> %ld\n", startTime, endTime);
  /*-- 阶段5：数据过滤 --*/
  DynamicJsonDocument doc(JSON_BUFFER_SIZE);
  JsonArray data = doc.to<JsonArray>();
  int matchCount = 0;
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    line.trim();
    // 使用JSON解析代替CSV解析
    DynamicJsonDocument lineDoc(256);
    DeserializationError error = deserializeJson(lineDoc, line);
    if (error) {
      Serial.println("JSON解析失败，跳过此行");
      continue;
    }
    // 提取时间字段
    const char* recordTime = lineDoc["time"];
    time_t recordTs = parseTime(recordTime);
    // 时间范围检查
    if (recordTs >= startTime && recordTs <= endTime) {
      JsonObject record = data.createNestedObject();
      record["time"] = recordTime;
      record["temp"] = lineDoc["temp"];
      record["humi"] = lineDoc["humi"];
      matchCount++;
    }
  }
    // 内存保护
    if (++matchCount >= MAX_RECORDS) {
      Serial.println("达到最大记录限制，停止读取");
      //break;
    }
  dataFile.close();
  /*-- 阶段6：结果包装 --*/
  String output;
  serializeJson(data, output);
  //sdQueryResult = ("查询数据匹配数：" + String(matchCount) + "条");
  //Serial.println("查询完成，匹配记录数：" + String(matchCount));
  return output;
}
/******************** OLED显示更新 ********************/
void updateDisplay() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 1000) return;
  lastUpdate = millis();
  u8g2.clearBuffer();//清除缓冲区
   //绘制图标
  u8g2.setFontPosTop(); //将图标绘制函数位置更改为“顶部”
  u8g2.setFont(u8g2_font_siji_t_6x10);
  {
  u8g2.drawGlyph(0, 0, icon);  //图标文件夹  
  }
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  // 第一行：标题
  u8g2.drawUTF8(40, 1, "环境监测");
  // 第二行：实时数据
  u8g2.drawUTF8(0, 15, ("温度:" + String(temperature,1) + "C").c_str());
  u8g2.drawUTF8(64, 15, ("湿度:" + String(humidity,1) + "%").c_str());
  // 第三行：报警信息
  u8g2.drawUTF8(0, 30, ALERT.c_str());
  // 第四行：查询结果
  //u8g2.drawUTF8(0, 50, String(TopicAlarm).c_str());
  u8g2.drawUTF8(0, 45, sdQueryResult.c_str());
  // 底部状态指示灯
  drawStatusIndicators();
  u8g2.sendBuffer();//显示缓冲区的内容
}
void lcd_tft(){//TFT屏显示
  tft.setCursor(0, 0, 4);//显示坐标
  tft.setTextColor(TFT_RED,TFT_DARKGREY); //字红色背景灰色
  tft.println("Hello World!");
  tft.setTextColor(TFT_MAGENTA,TFT_BLACK); 
  tft.println("老师温湿度压力登录流动设置菜单返回音量修改  大小:10");  //打印文字
  tft.setTextColor(TFT_WHITE,TFT_BLACK,true); //字白色背景黑色 
  tft.print("humidity :");tft.println(humidity);
  tft.print("temp :"); tft.println(temperature); //
  tft.print("random :"); tft.println(random(0,1000)); // 随机数
  tft.fillRect(0,200,320,40,TFT_GREEN);//清除指定区域//绿色
  tft.setCursor(0, 200);//显示坐标
  tft.setTextColor(TFT_BLUE,true);//如果显示异常加入,true
  tft.println(sdQueryResult); //离线主题名
  tft.println(ALERT); //报警点
  delay(1000); 
}
/******************** 其他关键函数 ********************/

// 文件追加写入
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("向文件追加数据: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("打开文件失败");
    sdQueryResult = "打开文件失败";
    return;
  }
  if (file.print(message)) {
    Serial.println("数据已追加");
    sdQueryResult = "数据已追加";
  } else {
    Serial.println("追加失败");
    sdQueryResult = "数据追加失败";
  }
  file.close();
  //sdQueryResult = "文件已关闭";
}

// 报警处理
void handleAlarm() {
  if (dataAlarm || !allOnlineFlag) {
    // LED闪烁
    if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
      digitalWrite(ledPin, !digitalRead(ledPin));
      lastBlinkTime = millis();
    }
    // 蜂鸣器报警
    ledcWriteTone(0, 2000);
  } else {
    digitalWrite(ledPin, userLedState);
    ledcWriteTone(0, 0);
  }
}
// 状态指示灯绘制
void drawStatusIndicators() {
  int yPos = 63;
  int boxSize = 6;
  int spacing = 10;
  for(int i=0; i<TOPIC_COUNT; i++){
    int xPos = 5 + i*(boxSize + spacing);
    u8g2.drawFrame(xPos, yPos-boxSize, boxSize, boxSize);
    if(topicStatus[i].isOnline){
      u8g2.drawBox(xPos+1, yPos-boxSize+1, boxSize-2, boxSize-2);
    }
  }
}
/******************** 辅助函数 ********************/
// 网络连接显示更新
void updateConnectDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  String cntInfo = "网络连接计时: " + String(cnt1);
  u8g2.drawUTF8(0, 15, ("连接: " + String(networks[currentNetworkIndex].ssid)).c_str());
  u8g2.drawUTF8(0, 35, cntInfo.c_str());
  u8g2.drawUTF8(0, 55, sd_vol.c_str());
  u8g2.sendBuffer();
  TJpgDec.setJpgScale(2); // 设置图片缩放大小为 1（不缩放）
  TJpgDec.drawSdJpg(200, 30, "/panda.jpg"); // 从SD卡绘制JPG图片
  tft.fillRect(0,200,320,40,TFT_GREEN);//清除指定区域//绿色
  tft.setCursor(10, 200);//显示坐标
  tft.setTextColor(TFT_RED);//蓝色
  tft.println("正在连接: " + String(networks[currentNetworkIndex].ssid)); //
  tft.println(sd_vol);//sd卡剩余容量
  delay(1000);
}
//串口调试
void handleSerialCommand() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("getDataByTimeRange")) {
            int startIdx = command.indexOf('(');
            int endIdx = command.indexOf(')');
            if (startIdx != -1 && endIdx != -1) {
                String params = command.substring(startIdx + 1, endIdx);
                params.replace("\"", ""); // 去除引号
                int commaPos = params.indexOf(',');
                String startTime = params.substring(0, commaPos);
                String endTime = params.substring(commaPos + 1);
                startTime.trim();
                endTime.trim();
                String result = getDataByTimeRange(startTime, endTime);
                //Serial.println(result); // 输出结果到串口
            } else {
                Serial.println("{\"error\":\"参数格式错误\"}");
            }
        }
    }
}
/******************** 修改后的状态监测函数 ********************/
void checkTopicsStatus() {
  static unsigned long lastCheck = 0;
  unsigned long currentMillis = millis();
  // 每30秒执行一次状态检查
  if (currentMillis - lastCheck >= CHECK_INTERVAL) {
    lastCheck = currentMillis;
    bool anyOffline = false;
    String offlineTopics = "";
    // 检查所有主题状态
    for (int i = 0; i < TOPIC_COUNT; i++) {
      bool currentState = (currentMillis - topicStatus[i].lastReceive) <= CHECK_INTERVAL;
      // 状态发生变化时记录日志
      if (currentState != topicStatus[i].isOnline) {
        Serial.printf("主题 %s %s\n", subTopics[i], currentState ? "恢复在线" : "检测离线");
      }
      // 更新状态
      topicStatus[i].isOnline = currentState;
      // 记录离线主题
      if (!currentState) {
        anyOffline = true;
        offlineTopics += String(subTopics[i]) + " ";
      }
    }
    // 更新报警信息
    if (anyOffline) {
      sdQueryResult = ("离线主题: " + offlineTopics);
      allOnlineFlag = false;
    } else if (!allOnlineFlag) {
      sdQueryResult = "所有主题在线";
      Serial.println("所有主题都在线");
      allOnlineFlag = true;
    }
  }
}

/******************** OTA固件更新 ********************/
void updateBin() {
  Serial.println("开始OTA更新...");
  WiFiClient UpdateClient;
  // 执行HTTP更新
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, upUrl);
  // 处理更新结果
  switch(ret){
    case HTTP_UPDATE_FAILED:
      Serial.printf("更新失败: %s\n", httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("无可用更新");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("更新成功");
      break;
  }
}