#ifndef PowerMeter_h
#define PowerMeter_h

// #define USE_TINYUSB


#include "../sdant.h"
#include "BicyclePower.h"
#include <bluefruit.h>
#include "stdint-gcc.h"

//#include "list"

// 蓝牙服务和特征值UUID定义
#define MESH_PROXY_SERVICE_UUID         0x1828
#define CYCLING_POWER_MEASUREMENT_UUID  0x2A63

typedef struct powermeter_config
{
    uint16_t profileUpdateCycle;
    BicyclePower* p_power_profile;
} powermeter_config;

// 喜德盛功率计数据结构定义
// 数据包格式：11字节
// Byte 0-1:  总功率 (无符号16位，小端序)
// Byte 2-3:  左腿功率 (有符号16位，小端序)
// Byte 4-5:  右腿功率 (有符号16位，小端序)
// Byte 6-7:  角度 (有符号16位，小端序)
// Byte 8-9:  踏频 (无符号16位，小端序)
// Byte 10:   错误代码 (8位)
typedef struct XdsPowerMeasurementData
{
    uint16_t totalPower;    // 总功率 (瓦特)
    int16_t leftPower;      // 左腿功率 (瓦特)
    int16_t rightPower;     // 右腿功率 (瓦特)
    int16_t angle;          // 角度 (度)
    uint16_t cadence;       // 踏频 (RPM)
    uint8_t errorCode;      // 错误代码
    bool isValid;           // 数据有效性标志
} XdsPowerMeasurementData;

class PowerMeter
{
public:

    

    PowerMeter(powermeter_config*);
    void begin();
    void update();
    void generateVirtualData();
    void simulateHallInterrupt();
    
    // 蓝牙客户端相关方法
    void initBLEClient();
    void startScanning();
    void connectToPowerMeter();
    void onConnect(uint16_t conn_handle);
    void onDisconnect(uint16_t conn_handle, uint8_t reason);
    void onPowerMeasurementNotify(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
    void parsePowerData(uint8_t* data, uint16_t len);
    
    // 喜德盛功率计数据解析相关函数
    XdsPowerMeasurementData parseXdsData(uint8_t* data, uint16_t len);
    uint16_t getUnsignedValue(uint8_t* data, uint16_t offset);
    int16_t getSignedValue(uint8_t* data, uint16_t offset);
    bool validateXdsData(const XdsPowerMeasurementData& data);
    void printXdsDataDetails(const XdsPowerMeasurementData& data, uint8_t* rawData);

    void SetAccPWR(uint16_t val)        { accPWR = val; }
    void SetInstPWR(uint16_t val)       { instPWR = val; }
    void SetInstCAD(uint8_t val)        { instCAD = val; }
    void SetPWREventCount(uint8_t val)  { PWREventCount = val; }
    
    // 获取连接状态
    bool getConnectionStatus() const    { return isConnected; }
    bool getScanningStatus() const      { return isScanning; }

private:
    BicyclePower* pwr;
    powermeter_config config;
    uint16_t accPWR, instPWR;
    uint8_t instCAD, PWREventCount;

    uint32_t nextProfileUpdate;
    uint32_t lastVirtualDataUpdate;
    uint32_t lastCadenceUpdate;
    
    // 虚拟数据生成相关变量
    uint16_t basePower;      // 基础功率 (约100W)
    uint8_t baseCadence;     // 基础踏频 (约70RPM)
    uint32_t virtualDataInterval;  // 虚拟数据更新间隔
    
    // 蓝牙客户端相关变量
    BLEClientService meshProxyService;
    BLEClientCharacteristic powerMeasurementChar;
    bool isConnected;
    bool isScanning;
    uint16_t connectionHandle;
    
    // 错误处理和数据质量监控
    uint16_t invalidDataCount;      // 无效数据包计数
    uint16_t validDataCount;        // 有效数据包计数
    uint32_t lastValidDataTime;     // 最后一次有效数据时间
    uint32_t dataTimeoutMs;         // 数据超时时间 (毫秒)
    bool dataQualityGood;           // 数据质量状态
    
    // 静态回调函数
    static void staticPowerMeasurementNotify(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
    static void staticConnectCallback(uint16_t conn_handle);
    static void staticDisconnectCallback(uint16_t conn_handle, uint8_t reason);
    static void staticScanCallback(ble_gap_evt_adv_report_t* report);
    
    // 静态实例指针，用于在静态回调中访问实例
    static PowerMeter* instance;

};


#endif