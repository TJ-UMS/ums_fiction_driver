//
// Created by anaple on 24-4-1.
//

#ifndef BATTERY_H
#define BATTERY_H

#include <deque>

class BatteryMonitor {
public:
    BatteryMonitor(double maxVoltage, double minVoltage);

    bool updateVoltage(double voltage);
    double calculateBatteryPercentage();

private:
    double calculateAverage();
    std::deque<double> voltages_; // 存储最近100次的电压值
    double maxVoltage_; // 最大电压值
    double minVoltage_; // 最小电压值
    double lastAverageVoltage_ = 0; // 上一次计算的平均电压值
};



#endif //BATTERY_H
