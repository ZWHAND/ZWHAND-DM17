#include "ZWHAND.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    ZWHAND hand(1, "COM1", 115200);
    bool open_com_state = hand.open_zwhand();
    
    if (open_com_state) {
        std::cout << "串口打开成功" << std::endl;
        short initialize_state = hand.get_initialize_state();
        
        if (initialize_state != -1) {
            std::cout << "ZWHAND初始化成功" << std::endl;
            bool set_ret = hand.set_all_motor_calibration();
            
            if (set_ret) {
                std::cout << "全手零位校准成功" << std::endl;
            } else {
                std::cout << "全手零位校准失败" << std::endl;
            }
            
            Sleep(13000);
            set_ret = hand.set_single_motor_absolute(1, 1000);

            if (set_ret) {
                std::cout << "设置ZWHAND单个关节电机绝对角度成功" << std::endl;
            }
            else {
                std::cout << "设置ZWHAND单个关节电机绝对角度失败" << std::endl;
            }
            Sleep(500);
            
            std::vector<int> ang = {1000, 1000, 0, 0, 0, 0, 0, 0, 0, 1000, 1000, 1000, 1000, 1000, 1000, 0, 1000};
            // 设置ZWHAND所有关节电机绝对角度-剪刀
            set_ret = hand.set_all_motor_absolute(ang);
            
            if (set_ret) {
                std::cout << "设置ZWHAND所有关节电机绝对角度成功" << std::endl;
            } else {
                std::cout << "设置ZWHAND所有关节电机绝对角度失败" << std::endl;
            }
            
            Sleep(1000);
            
            // 获取ZWHAND的所有关节电机实际角度挡位
            std::vector<short> get_data = hand.get_motor_real_angle();
            if (!get_data.empty()) {
                std::cout << "关节电机实际角度挡位: ";
                for (size_t i = 0; i < get_data.size(); ++i) {
                    std::cout << get_data[i];
                    if (i < get_data.size() - 1) std::cout << ", ";
                }
                std::cout << std::endl;
            } else {
                std::cout << "关节电机实际角度挡位获取失败" << std::endl;
            }
        }
    } else {
        std::cout << "串口打开失败" << std::endl;
    }
    Sleep(5000);
    return 0;
}
