#include "./PowerMeter.h"

// 静态实例指针定义
PowerMeter* PowerMeter::instance = nullptr;

void PrintUnhandledANTEvent(ant_evt_t *evt)
{
  Serial.printf("Channel #%d for %s: event %s\n", evt->channel, ANTplus.getAntProfileByChNum(evt->channel)->getName(), AntEventTypeDecode(evt)); 
  if (evt->event != EVENT_CHANNEL_COLLISION 
    && evt->event != EVENT_RX_FAIL
    && evt->event != EVENT_CHANNEL_CLOSED
    )
    Serial.printf("  (%s)\n", AntEventType2LongDescription(evt));
}
void ReopenANTChannel(ant_evt_t *evt)
{
  if (evt->event == EVENT_CHANNEL_CLOSED ) {
    Serial.printf("Channel #%d closed for %s\n", evt->channel,ANTplus.getAntProfileByChNum(evt->channel)->getName()); 
    Serial.printf("Reopening...");
    uint32_t ret;
    ret = sd_ant_channel_open(evt->channel);
    if (ret == NRF_SUCCESS) Serial.println("success!");
    else Serial.printf("failed with code:%#x\n", ret);
  }
}

PowerMeter::PowerMeter(powermeter_config * cfg) : 
    meshProxyService(MESH_PROXY_SERVICE_UUID),
    powerMeasurementChar(CYCLING_POWER_MEASUREMENT_UUID)
{
    config.profileUpdateCycle = cfg->profileUpdateCycle;
    //config.p_power_profile = cfg->p_power_profile;
    pwr = new BicyclePower(TX);
    
    // 设置静态实例指针
    instance = this;
    
    // 初始化虚拟数据参数
    basePower = 100;           // 基础功率100W
    baseCadence = 70;          // 基础踏频70RPM
    virtualDataInterval = 1000; // 1秒更新一次虚拟数据
    lastVirtualDataUpdate = 0;
    lastCadenceUpdate = 0;
    
    // 初始化功率和踏频
    instPWR = basePower;
    instCAD = baseCadence;
    accPWR = 0;
    PWREventCount = 0;
    
    // 初始化蓝牙客户端状态
    isConnected = false;
    isScanning = false;
    connectionHandle = 0;
    
    // 初始化错误处理和数据质量监控
    invalidDataCount = 0;
    validDataCount = 0;
    lastValidDataTime = 0;
    dataTimeoutMs = 5000;      // 5秒数据超时
    dataQualityGood = true;
}

void PowerMeter::begin() {
    Serial.println("Starting Virtual PowerMeter Setup...");
    Serial.println("Adding PWR profile");
    pwr->setUnhandledEventListener(PrintUnhandledANTEvent);
    pwr->setAllEventListener(ReopenANTChannel);
    pwr->setName("PWR");
    ANTplus.AddProfile(pwr);

    Serial.println("Bluefruit52 BLEUART Startup");
    Serial.println("---------------------------\n");
    Bluefruit.autoConnLed(false);
    Bluefruit.configPrphBandwidth(BANDWIDTH_LOW);

    Serial.print("Starting BLE stack as Central. Expecting 'true':");
    bool ret = Bluefruit.begin(0, 1);  // 0 peripheral, 1 central
    Serial.println(ret);
    
    // 初始化蓝牙客户端
    initBLEClient();
    Serial.print("Starting ANT stack. Expecting 'true':");
    ret = ANTplus.begin(1);
    Serial.println(ret);
    ANTProfile* profiles[] = {pwr};
    for (auto i: profiles)
    {
        Serial.printf("Channel number for %s became %d\n", i->getName(), i->getChannelNumber());
    }
    
    // 初始化虚拟数据时间戳
    nextProfileUpdate = millis();
    lastVirtualDataUpdate = millis();
    lastCadenceUpdate = millis();
    
    Serial.println("Virtual PowerMeter initialized successfully!");
    Serial.printf("Base Power: %dW, Base Cadence: %dRPM\n", basePower, baseCadence);
    Serial.printf("Startup is complete.\n");
}

void PowerMeter::generateVirtualData() // 生成虚拟的功率和踏频数据
{
    uint32_t currentTime = millis();
    
    // 每1秒更新一次虚拟数据
    if (currentTime - lastVirtualDataUpdate >= virtualDataInterval) 
    {
        // 生成功率：基础100W，随机浮动±20W
        int powerVariation = random(-20, 21); // -20到+20的随机数
        instPWR = basePower + powerVariation;
        if (instPWR < 0) instPWR = 0; // 确保功率不为负数
        
        // 生成踏频：基础70RPM，随机浮动±10RPM
        int cadenceVariation = random(-10, 11); // -10到+10的随机数
        instCAD = baseCadence + cadenceVariation;
        if (instCAD < 0) instCAD = 0; // 确保踏频不为负数
        
        // 累积功率和事件计数
        accPWR += instPWR;
        PWREventCount++;
        
        lastVirtualDataUpdate = currentTime;
        
        // 输出调试信息
        Serial.printf("Virtual Data - Power: %dW, Cadence: %dRPM\n", instPWR, instCAD);
    }
}

void PowerMeter::simulateHallInterrupt() // 模拟霍尔传感器中断，用于踏频计算
{
    uint32_t currentTime = millis();
    
    // 根据当前踏频计算中断间隔
    // 踏频 = 60 / (间隔秒数)，所以间隔 = 60000ms / 踏频
    uint32_t expectedInterval = (instCAD > 0) ? (60000 / instCAD) : 1000;
    
    // 模拟霍尔传感器中断
    if (currentTime - lastCadenceUpdate >= expectedInterval) 
    {
        lastCadenceUpdate = currentTime;
        
        // 这里可以添加一些踏频相关的处理逻辑
        // 但主要的数据生成在generateVirtualData()中完成
        Serial.printf("Simulated Hall Interrupt - Cadence: %dRPM\n", instCAD);
    }
}

void PowerMeter::update()
{   
    uint32_t currentTime = millis();
    
    // 检查数据超时 (仅在已连接时检查)
    if (isConnected && lastValidDataTime > 0) {
        if (currentTime - lastValidDataTime > dataTimeoutMs) {
            Serial.printf("Warning: No valid data received for %d ms\n", currentTime - lastValidDataTime);
            
            // 如果超时时间过长，考虑重新扫描
            if (currentTime - lastValidDataTime > dataTimeoutMs * 2) {
                Serial.println("Data timeout exceeded, attempting reconnection...");
                // 这里可以添加重新连接逻辑
                // 暂时切换到虚拟数据模式
                if (isConnected) {
                    Serial.println("Switching to virtual data mode due to timeout");
                }
            }
        }
    }
    
    // 如果没有连接到蓝牙功率计，或者数据超时，则生成虚拟数据
    if (!isConnected || (lastValidDataTime > 0 && currentTime - lastValidDataTime > dataTimeoutMs)) {
        generateVirtualData();
        simulateHallInterrupt();
    }
    
    // 定期发送ANT+数据
    if (millis() > nextProfileUpdate)
    {
        nextProfileUpdate += config.profileUpdateCycle;
        pwr->SetInstantPWR(instPWR);
        pwr->SetAccumulatedPWR(accPWR);
        pwr->SetPWREventCount(PWREventCount);
        pwr->SetInstantCadence(instCAD);
        
        // 确定数据源
        bool usingRealData = isConnected && (lastValidDataTime > 0 && currentTime - lastValidDataTime <= dataTimeoutMs);
        
        if (usingRealData) {
            Serial.printf("ANT+ Data Sent (XDS BLE) - Power: %dW, Cadence: %dRPM, AccPWR: %d, Events: %d\n", 
                         instPWR, instCAD, accPWR, PWREventCount);
        } else {
            Serial.printf("ANT+ Data Sent (Virtual) - Power: %dW, Cadence: %dRPM, AccPWR: %d, Events: %d\n", 
                         instPWR, instCAD, accPWR, PWREventCount);
        }
        
        // 定期打印数据质量统计 (每分钟一次)
        static uint32_t lastStatsTime = 0;
        if (currentTime - lastStatsTime > 60000) {  // 60秒
            lastStatsTime = currentTime;
            if (validDataCount > 0 || invalidDataCount > 0) {
                float errorRate = (float)invalidDataCount / (validDataCount + invalidDataCount) * 100.0;
                Serial.printf("=== Data Quality Report ===\n");
                Serial.printf("Valid packets: %d, Invalid packets: %d\n", validDataCount, invalidDataCount);
                Serial.printf("Error rate: %.2f%%, Data quality: %s\n", errorRate, dataQualityGood ? "Good" : "Poor");
                Serial.printf("Last valid data: %d ms ago\n", lastValidDataTime > 0 ? currentTime - lastValidDataTime : 0);
                Serial.printf("Connection status: %s\n", isConnected ? "Connected" : "Disconnected");
                Serial.println("===========================");
            }
        }
    }
}

// 蓝牙客户端方法实现
void PowerMeter::initBLEClient() {
    Serial.println("Initializing BLE Client...");
    
    // 设置设备名称
    Bluefruit.setName("PowerMeter Central");
    
    // 初始化Mesh Proxy服务
    meshProxyService.begin();
    
    // 初始化Cycling Power Measurement特征值
    powerMeasurementChar.setNotifyCallback(staticPowerMeasurementNotify);
    powerMeasurementChar.begin();
    
    // 设置连接回调
    Bluefruit.Central.setConnectCallback(staticConnectCallback);
    Bluefruit.Central.setDisconnectCallback(staticDisconnectCallback);
    
    Serial.println("BLE Client initialized successfully!");
}

void PowerMeter::startScanning() {
    if (isScanning) {
        Serial.println("Already scanning...");
        return;
    }
    
    Serial.println("Starting BLE scan for power meters...");
    
    // 设置扫描回调
    Bluefruit.Scanner.setRxCallback(staticScanCallback);
    
    // 开始扫描
    Bluefruit.Scanner.start(0);  // 0 = 永久扫描直到找到设备
    isScanning = true;
}

void PowerMeter::onConnect(uint16_t conn_handle) {
    Serial.printf("Connected to power meter, handle: %d\n", conn_handle);
    connectionHandle = conn_handle;
    isConnected = true;
    
    // 发现服务
    if (meshProxyService.discover(conn_handle)) {
        Serial.println("Mesh Proxy Service discovered");
        
        // 发现特征值
        if (powerMeasurementChar.discover()) {
            Serial.println("Cycling Power Measurement characteristic discovered");
            
            // 启用通知
            if (powerMeasurementChar.enableNotify()) {
                Serial.println("Power measurement notifications enabled");
            } else {
                Serial.println("Failed to enable power measurement notifications");
            }
        } else {
            Serial.println("Failed to discover Cycling Power Measurement characteristic");
        }
    } else {
        Serial.println("Failed to discover Mesh Proxy Service");
    }
}

void PowerMeter::onDisconnect(uint16_t conn_handle, uint8_t reason) {
    Serial.printf("Disconnected from power meter, handle: %d, reason: 0x%02X\n", conn_handle, reason);
    isConnected = false;
    connectionHandle = 0;
    
    // 重新开始扫描
    Serial.println("Restarting scan...");
    delay(1000);  // 等待1秒后重新扫描
    startScanning();
}

void PowerMeter::onPowerMeasurementNotify(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
    Serial.printf("Received power data (%d bytes): ", len);
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
    
    // 解析功率数据
    parsePowerData(data, len);
}

void PowerMeter::parsePowerData(uint8_t* data, uint16_t len) {
    // 简单的数据解析，假设接收到的是标准的功率测量数据
    if (len >= 4) {
        // 解析功率数据 (假设前2字节是功率，后2字节是踏频)
        instPWR = (data[1] << 8) | data[0];  // 小端序
        if (len >= 4) {
            instCAD = (data[3] << 8) | data[2];  // 小端序
        }
        
        // 更新累积功率
        accPWR += instPWR;
        PWREventCount++;
        
        // 更新最后有效数据时间
        lastValidDataTime = millis();
        
        Serial.printf("Power: %dW, Cadence: %dRPM\n", instPWR, instCAD);
    }
}

// 静态回调函数实现
void PowerMeter::staticPowerMeasurementNotify(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
    if (instance) {
        instance->onPowerMeasurementNotify(chr, data, len);
    }
}

void PowerMeter::staticConnectCallback(uint16_t conn_handle) {
    if (instance) {
        instance->onConnect(conn_handle);
    }
}

void PowerMeter::staticDisconnectCallback(uint16_t conn_handle, uint8_t reason) {
    if (instance) {
        instance->onDisconnect(conn_handle, reason);
    }
}

void PowerMeter::staticScanCallback(ble_gap_evt_adv_report_t* report) {
    if (instance) {
        // 检查是否是喜德盛功率计
        if (Bluefruit.Scanner.checkReportForService(report, instance->meshProxyService)) {
            Serial.print("Found power meter: ");
            Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
            Serial.println();
            
            // 停止扫描并连接
            Bluefruit.Scanner.stop();
            instance->isScanning = false;
            
            // 连接到设备
            Bluefruit.Central.connect(report);
        }
    }
}

// 喜德盛功率计数据解析函数实现
XdsPowerMeasurementData PowerMeter::parseXdsData(uint8_t* data, uint16_t len) {
    XdsPowerMeasurementData result = {0};
    
    // 直接解析数据，不检查长度
    if (len >= 2) result.totalPower = getUnsignedValue(data, 0);      // Byte 0-1: 总功率
    if (len >= 4) result.leftPower = getSignedValue(data, 2);         // Byte 2-3: 左腿功率
    if (len >= 6) result.rightPower = getSignedValue(data, 4);        // Byte 4-5: 右腿功率
    if (len >= 8) result.angle = getSignedValue(data, 6);             // Byte 6-7: 角度
    if (len >= 10) result.cadence = getUnsignedValue(data, 8);        // Byte 8-9: 踏频
    if (len >= 11) result.errorCode = data[10];                       // Byte 10: 错误代码
    
    // 直接标记为有效，不进行验证
    result.isValid = true;
    
    return result;
}

// 读取无符号16位整数 (小端序)
uint16_t PowerMeter::getUnsignedValue(uint8_t* data, uint16_t offset) {
    uint16_t low = data[offset] & 0xFF;
    uint16_t high = data[offset + 1] & 0xFF;
    return (high << 8) | low;
}

// 读取有符号16位整数 (小端序)
int16_t PowerMeter::getSignedValue(uint8_t* data, uint16_t offset) {
    uint16_t value = getUnsignedValue(data, offset);
    // 如果最高位为1，则为负数
    return (value & 0x8000) ? (int16_t)(value - 0x10000) : (int16_t)value;
}

// 验证喜德盛数据有效性
bool PowerMeter::validateXdsData(const XdsPowerMeasurementData& data) {
    // 检查错误代码
    if (data.errorCode != 0) {
        Serial.printf("XDS Error Code: %d\n", data.errorCode);
        // 根据错误代码决定是否继续处理数据
        if (data.errorCode > 10) {  // 严重错误
            return false;
        }
        // 轻微错误，继续验证其他数据
    }
    
    // 基本范围检查 - 总功率
    if (data.totalPower > 2000) {  // 功率不应超过2000W
        Serial.printf("Invalid total power: %dW (max 2000W)\n", data.totalPower);
        return false;
    }
    
    // 踏频范围检查
    if (data.cadence > 200) {  // 踏频不应超过200RPM
        Serial.printf("Invalid cadence: %dRPM (max 200RPM)\n", data.cadence);
        return false;
    }
    
    // 角度范围检查 (-180° 到 +180°)
    if (data.angle < -180 || data.angle > 180) {
        Serial.printf("Invalid angle: %d° (range: -180° to +180°)\n", data.angle);
        return false;
    }
    
    // 左右功率范围检查
    if (data.leftPower < -100 || data.leftPower > 1500) {
        Serial.printf("Invalid left power: %dW (range: -100W to 1500W)\n", data.leftPower);
        return false;
    }
    
    if (data.rightPower < -100 || data.rightPower > 1500) {
        Serial.printf("Invalid right power: %dW (range: -100W to 1500W)\n", data.rightPower);
        return false;
    }
    
    // 检查左右功率之和是否接近总功率 (允许15%误差)
    int16_t calculatedTotal = data.leftPower + data.rightPower;
    int16_t powerDiff = abs(calculatedTotal - (int16_t)data.totalPower);
    if (data.totalPower > 10) {  // 只在有显著功率时检查
        float errorPercent = (float)powerDiff / data.totalPower * 100.0;
        if (errorPercent > 15.0) {
            Serial.printf("Power mismatch: Total=%dW, L+R=%dW, Diff=%dW (%.1f%% error)\n", 
                         data.totalPower, calculatedTotal, powerDiff, errorPercent);
            // 不返回false，只是警告，因为可能是正常的测量误差
        }
    }
    
    // 检查功率和踏频的合理性组合
    if (data.totalPower > 0 && data.cadence == 0) {
        Serial.println("Warning: Power > 0 but cadence = 0");
    }
    
    if (data.totalPower == 0 && data.cadence > 0) {
        Serial.println("Warning: Cadence > 0 but power = 0");
    }
    
    // 检查极端功率值
    if (data.totalPower > 1000) {
        Serial.printf("Warning: Very high power detected: %dW\n", data.totalPower);
    }
    
    return true;
}

// 打印喜德盛数据详细信息
void PowerMeter::printXdsDataDetails(const XdsPowerMeasurementData& data, uint8_t* rawData) {
    Serial.println("=== XDS Power Meter Data ===");
    
    // 打印原始数据
    Serial.print("Raw data: ");
    for (int i = 0; i < 11; i++) {
        Serial.printf("%02X", rawData[i]);
        if (i < 10) Serial.print("-");
    }
    Serial.println();
    
    // 打印解析结果
    Serial.printf("Total Power:  0x%02X%02X = %dW\n", 
                 rawData[1], rawData[0], data.totalPower);
    Serial.printf("Left Power:   0x%02X%02X = %dW\n", 
                 rawData[3], rawData[2], data.leftPower);
    Serial.printf("Right Power:  0x%02X%02X = %dW\n", 
                 rawData[5], rawData[4], data.rightPower);
    Serial.printf("Angle:        0x%02X%02X = %d deg\n", 
                 rawData[7], rawData[6], data.angle);
    Serial.printf("Cadence:      0x%02X%02X = %d RPM\n", 
                 rawData[9], rawData[8], data.cadence);
    Serial.printf("Error Code:   0x%02X = %d\n", 
                 rawData[10], data.errorCode);
    Serial.printf("Data Valid:   %s\n", data.isValid ? "YES" : "NO");
    Serial.println("============================");
}