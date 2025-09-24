
//#include <bluefruit52.h>
//#include <Adafruit_LittleFS.h>
//#include <InternalFileSystem.h>

#include "src/PowerMeter/PowerMeter.h"

// 虚拟功率计配置
powermeter_config PWRconfig = 
{
  250  // profileUpdateCycle - ANT+数据发送间隔(ms)
};

PowerMeter power(&PWRconfig);

unsigned long newmessage = 0;


void setup(void)
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== BLE PowerMeter Central Starting ===");

  power.begin();
  
  // 启动蓝牙扫描，寻找喜德盛功率计
  Serial.println("Starting BLE scan for Xidesheng power meter...");
  power.startScanning();
  
  newmessage = millis() + 1000;
  Serial.println("PowerMeter setup is finished!");
  Serial.println("Will use BLE data if connected, otherwise virtual data (~100W, ~70RPM)...");
}

void loop(void)
{
  // 更新虚拟功率计数据
  power.update();
  
  // 简单的串口命令处理（可选）
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("power:")) {
      int newPower = command.substring(6).toInt();
      if (newPower > 0 && newPower < 1000) {
        Serial.printf("Base power changed to: %dW\n", newPower);
        // 这里可以添加修改基础功率的功能
      }
    }
    else if (command.startsWith("cadence:")) {
      int newCadence = command.substring(8).toInt();
      if (newCadence > 0 && newCadence < 200) {
        Serial.printf("Base cadence changed to: %dRPM\n", newCadence);
        // 这里可以添加修改基础踏频的功能
      }
    }
    else if (command == "status") {
      Serial.println("PowerMeter Status:");
      Serial.println("- ANT+ broadcasting active");
      if (power.getConnectionStatus()) {
        Serial.println("- Connected to BLE power meter");
        Serial.println("- Using real power/cadence data");
      } else {
        Serial.println("- Not connected to BLE power meter");
        Serial.println("- Generating virtual power around 100W");
        Serial.println("- Generating virtual cadence around 70RPM");
      }
    }
    else if (command == "scan") {
      Serial.println("Starting BLE scan...");
      power.startScanning();
    }
    else if (command == "help") {
      Serial.println("Available commands:");
      Serial.println("- status: Show current status");
      Serial.println("- scan: Start BLE scanning");
      Serial.println("- power:XXX: Set base power (when not connected)");
      Serial.println("- cadence:XXX: Set base cadence (when not connected)");
      Serial.println("- help: Show this help");
    }
  }
  
  // 小延时避免过度占用CPU
  delay(10);
} //loop
