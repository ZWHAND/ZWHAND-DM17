#include "ZWHAND.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

ZWHAND::ZWHAND(int lqs_id, const std::string& port, int baud) 
    : hSerial(INVALID_HANDLE_VALUE),
      port(port),
      baudrate(baud),
      lqs_id(lqs_id),
      motor_count(17),
      initial_lqs_id(1),
      initial_baud(115200),
      BAUD_RATE_LEVELS({9600, 115200, 921600, 2000000}) {
}

ZWHAND::~ZWHAND() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
    }
}

bool ZWHAND::open_zwhand() {
    try {
        std::string fullPort = "\\\\.\\" + port;
        hSerial = CreateFile(fullPort.c_str(),
                            GENERIC_READ | GENERIC_WRITE,
                            0,
                            NULL,
                            OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL,
                            NULL);
        
        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "[串口异常] 无法打开端口 " << port << std::endl;
            return false;
        }
        
        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "[串口异常] 获取串口状态失败" << std::endl;
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            return false;
        }
        
        dcbSerialParams.BaudRate = baudrate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        
        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "[串口异常] 设置串口参数失败" << std::endl;
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            return false;
        }
        
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 200;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        
        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "[串口异常] 设置串口超时失败" << std::endl;
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            return false;
        }
        
        std::cout << "串口<" << port << ">初始化成功" << std::endl;
        return true;
    } catch (...) {
        std::cerr << "[未知错误] 初始化串口时发生异常" << std::endl;
        return false;
    }
}

bool ZWHAND::close_zwhand() {
    try {
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
        return true;
    } catch (...) {
        std::cerr << "[串口异常] 串口关闭时发生异常" << std::endl;
        return false;
    }
}

std::vector<unsigned char> ZWHAND::modbus_data_receive(const std::vector<unsigned char>& detect_data, int receive_data_len) {
    auto start_time = std::chrono::steady_clock::now();
    DWORD available_num = 0;
    std::vector<unsigned char> data(receive_data_len);
    
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        
        if (elapsed > 200) {
            std::cout << "超时 数据接收失败！" << std::endl;
            return {};
        }
        
        if (hSerial != INVALID_HANDLE_VALUE) {
            COMSTAT comStat;
            DWORD dwErrors;
            if (ClearCommError(hSerial, &dwErrors, &comStat)) {
                available_num = comStat.cbInQue;
            }
        } else {
            std::cout << "串口未打开" << std::endl;
            return {};
        }
        
        if (available_num >= receive_data_len) {
            DWORD bytesRead;
            if (ReadFile(hSerial, data.data(), receive_data_len, &bytesRead, NULL)) {
                // 校验数据
                bool valid = true;
                for (size_t i = 0; i < std::min(detect_data.size(), static_cast<size_t>(bytesRead)); ++i) {
                    if (data[i] != detect_data[i]) {
                        std::cout << "异常：接收数据校验失败！" << std::endl;
                        valid = false;
                        break;
                    }
                }
                
                if (!valid) return {};
                
                // 截取数据段
                std::vector<unsigned char> need_data(data.begin() + detect_data.size(), 
                                                    data.end() - 2); // 去掉CRC
                
                if (!need_data.empty()) {
                    
                    std::vector<short> decimal_list;
                    for (size_t i = 0; i < need_data.size(); i += 2) {
                        if (i + 1 >= need_data.size()) break;
                        short decimal_value = (static_cast<short>(need_data[i]) << 8) + 
                                             static_cast<short>(need_data[i + 1]);
                        decimal_list.push_back(decimal_value);
                    }
                    
                    // 转换为unsigned char vector返回
                    std::vector<unsigned char> result(decimal_list.size() * 2);
                    for (size_t i = 0; i < decimal_list.size(); ++i) {
                        result[i*2] = (decimal_list[i] >> 8) & 0xFF;
                        result[i*2 + 1] = decimal_list[i] & 0xFF;
                    }
                    std::cout << "成功接收数据" << result.size() << std::endl;
                    return result;
                } else {
                    return {1}; // 表示成功但无数据
                    std::cout << "成功但无数据" << std::endl;
                }
            }
        }
        
        Sleep(10);
    }
    
    return {};
}

std::vector<unsigned char> ZWHAND::modbus_crc(const std::vector<unsigned char>& data) {
    unsigned short crc = 0xFFFF;
    for (unsigned char byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    unsigned char crc_high = (crc >> 8) & 0xFF;
    unsigned char crc_low = crc & 0xFF;
    
    std::vector<unsigned char> result = data;
    result.push_back(crc_low);
    result.push_back(crc_high);
    
    return result;
}

bool ZWHAND::send_single_data(unsigned char address, short data) {
    if (hSerial == INVALID_HANDLE_VALUE) return false;
    
    PurgeComm(hSerial, PURGE_RXCLEAR);
    
    std::vector<unsigned char> step_list = {
        static_cast<unsigned char>((data >> 8) & 0xFF),
        static_cast<unsigned char>(data & 0xFF)
    };
    
    std::vector<unsigned char> single_motor_cmd = {
        static_cast<unsigned char>(lqs_id), 0x10, 0x00, address, 0x00,
        0x01, 0x02, step_list[0], step_list[1]
    };
    
    std::vector<unsigned char> cmd = modbus_crc(single_motor_cmd);
    
    try {
        DWORD bytesWritten;
        if (WriteFile(hSerial, cmd.data(), cmd.size(), &bytesWritten, NULL)) {
            if (bytesWritten == cmd.size()) {
                std::vector<unsigned char> detect_data(cmd.begin(), cmd.begin() + 6);
                std::vector<unsigned char> receive_data = modbus_data_receive(detect_data, 8);
                return !receive_data.empty();
            } else {
                std::cout << "数据未发送完成！" << std::endl;
            }
        } else {
            std::cout << "发送失败" << std::endl;
        }
    } catch (...) {
        std::cout << "发送失败: 异常" << std::endl;
    }
    
    return false;
}

bool ZWHAND::send_multiple_data(unsigned char start_address, const std::vector<short>& data) {
    if (hSerial == INVALID_HANDLE_VALUE) return false;
    
    PurgeComm(hSerial, PURGE_RXCLEAR);
    
    int address_len = data.size();
    std::vector<unsigned char> cmd_data(7 + address_len * 2);
    
    cmd_data[0] = lqs_id;
    cmd_data[1] = 0x10;
    cmd_data[2] = 0x00;
    cmd_data[3] = start_address;
    cmd_data[4] = 0x00;
    cmd_data[5] = address_len;
    cmd_data[6] = address_len * 2;
    
    for (size_t i = 0; i < data.size(); ++i) {
        cmd_data[i * 2 + 7] = (data[i] >> 8) & 0xFF;
        cmd_data[i * 2 + 8] = data[i] & 0xFF;
    }
    
    std::vector<unsigned char> cmd = modbus_crc(cmd_data);
    
    try {
        DWORD bytesWritten;
        if (WriteFile(hSerial, cmd.data(), cmd.size(), &bytesWritten, NULL)) {
            if (bytesWritten == cmd.size()) {
                std::vector<unsigned char> detect_data(cmd.begin(), cmd.begin() + 6);
                std::vector<unsigned char> receive_data = modbus_data_receive(detect_data, 8);
                return !receive_data.empty();
            } else {
                std::cout << "数据未发送完成！" << std::endl;
            }
        } else {
            std::cout << "发送失败" << std::endl;
        }
    } catch (...) {
        std::cout << "发送失败: 异常" << std::endl;
    }
    
    return false;
}

std::vector<short> ZWHAND::read_multiple_data(unsigned char start_address, unsigned char address_len) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};
    
    PurgeComm(hSerial, PURGE_RXCLEAR);
    
    std::vector<unsigned char> cmd_data = {
        static_cast<unsigned char>(lqs_id), 0x04, 0x00, start_address, 0x00, address_len
    };
    
    std::vector<unsigned char> cmd = modbus_crc(cmd_data);
    
    try {
        DWORD bytesWritten;
        if (WriteFile(hSerial, cmd.data(), cmd.size(), &bytesWritten, NULL)) {
            if (bytesWritten == cmd.size()) {
                std::vector<unsigned char> detect_data = {
                    static_cast<unsigned char>(lqs_id), 0x04, static_cast<unsigned char>(address_len * 2)
                };
                
                std::vector<unsigned char> received_data = modbus_data_receive(detect_data, address_len * 2 + 5);
                
                if (!received_data.empty()) {
                    std::vector<short> result;
                    for (size_t i = 0; i < received_data.size(); i += 2) {
                        if (i + 1 < received_data.size()) {
                            short value = (static_cast<short>(received_data[i]) << 8) + 
                                         static_cast<short>(received_data[i + 1]);
                            result.push_back(value);
                        }
                    }
                    return result;
                }
            } else {
                std::cout << "数据未发送完成！" << std::endl;
                std::cout << "bytesWritten: " << bytesWritten << cmd.size() << std::endl;
            }
        } else {
            std::cout << "发送失败" << std::endl;
        }
    } catch (...) {
        std::cout << "发送失败: 异常" << std::endl;
    }
    
    return {};
}

// 设备控制函数实现
bool ZWHAND::set_id(int lqs_id) {
    if (lqs_id < 1 || lqs_id > 255) {
        std::cout << "设置ID失败: " << lqs_id << "，参数超出有效范围1-255" << std::endl;
        return false;
    }
    
    try {
        bool send_ret = send_single_data(SET_ID_ADDRESS, static_cast<short>(lqs_id));
        if (send_ret) {
            this->lqs_id = lqs_id;
            std::cout << "设置ID成功: " << lqs_id << std::endl;
            return true;
        } else {
            std::cout << "设置ID失败: " << lqs_id << std::endl;
            return false;
        }
    } catch (...) {
        std::cout << "设置ID失败: " << lqs_id << "，发送数据异常" << std::endl;
        return false;
    }
}

bool ZWHAND::set_baud(int baud_order, bool is_restart) {
    if (baud_order < 1 || baud_order > static_cast<int>(BAUD_RATE_LEVELS.size())) {
        std::cout << "设置波特率失败: " << baud_order << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    try {
        bool send_ret = send_single_data(SET_BAUD_ADDRESS, static_cast<short>(baud_order));
        if (send_ret) {
            baudrate = BAUD_RATE_LEVELS[baud_order - 1];
            
            if (is_restart) {
                close_zwhand();
                Sleep(100);
                open_zwhand();
            }
            
            std::cout << "设置波特率成功: " << BAUD_RATE_LEVELS[baud_order - 1] << std::endl;
            return true;
        } else {
            std::cout << "设置波特率失败: " << BAUD_RATE_LEVELS[baud_order - 1] << std::endl;
            return false;
        }
    } catch (...) {
        std::cout << "设置波特率异常: " << BAUD_RATE_LEVELS[baud_order - 1] << std::endl;
        return false;
    }
}

bool ZWHAND::set_error_clear() {
    return send_single_data(CLEAR_ERROR_ADDRESS, 1);
}

bool ZWHAND::set_power_off_save(int save_type) {
    if (save_type < 1 || save_type > 2) {
        std::cout << "设置掉电保存失败: " << save_type << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    try {
        return send_single_data(SET_POWER_OFF_SAVE_ADDRESS, static_cast<short>(save_type));
    } catch (...) {
        std::cout << "设置掉电保存异常" << std::endl;
        return false;
    }
}

bool ZWHAND::set_factory_data_reset() {
    try {
        bool send_ret = send_single_data(FACTORY_DATA_RESET_ADDRESS, 1);
        if (send_ret) {
            int original_lqs_id = lqs_id;
            int original_baudrate = baudrate;
            
            try {
                lqs_id = initial_lqs_id;
                
                if (baudrate != initial_baud) {
                    close_zwhand();
                    baudrate = initial_baud;
                    open_zwhand();
                }
                
                std::cout << "恢复出厂成功" << std::endl;
                return true;
            } catch (...) {
                lqs_id = original_lqs_id;
                baudrate = original_baudrate;
                close_zwhand();
                open_zwhand();
                std::cout << "恢复出厂设置时发生错误" << std::endl;
                return false;
            }
        } else {
            std::cout << "恢复出厂失败" << std::endl;
            return false;
        }
    } catch (...) {
        std::cout << "发送恢复出厂指令时发生错误" << std::endl;
        std::cout << "恢复出厂失败" << std::endl;
        return false;
    }
}

// 电机控制函数实现
bool ZWHAND::set_single_motor_speed(int motor_number, int speed) {
    if (motor_number < 1 || motor_number > 17) {
        std::cout << "设置速度失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    if (speed < 1 || speed > 100) {
        std::cout << "设置速度失败: " << speed << " - 速度超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = SET_SPEED_ADDRESS + motor_number - 1;
    try {
        return send_single_data(address, static_cast<short>(speed));
    } catch (...) {
        std::cout << "设置速度异常" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_speed(int speed) {
    if (speed < 1 || speed > 100) {
        std::cout << "设置所有电机速度失败: " << speed << " - 速度超出有效范围" << std::endl;
        return false;
    }
    
    std::vector<short> speeds(motor_count, static_cast<short>(speed));
    return send_multiple_data(SET_SPEED_ADDRESS, speeds);
}

bool ZWHAND::set_single_motor_current(int motor_number, int current) {
    if (motor_number < 1 || motor_number > 17) {
        std::cout << "设置电流失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    if (current < 1 || current > 100) {
        std::cout << "设置电流失败: " << current << " - 电流超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = SET_CURRENT_ADDRESS + motor_number - 1;
    try {
        return send_single_data(address, static_cast<short>(current));
    } catch (...) {
        std::cout << "设置电流失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_current(int current) {
    if (current < 1 || current > 100) {
        std::cout << "设置所有电机电流失败: " << current << " - 电流超出有效范围" << std::endl;
        return false;
    }
    
    std::vector<short> currents(motor_count, static_cast<short>(current));
    return send_multiple_data(SET_CURRENT_ADDRESS, currents);
}

bool ZWHAND::set_single_motor_stop(int motor_number) {
    if (motor_number < 1 || motor_number > 17) {
        std::cout << "设置电机停止失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = SET_MOTOR_STOP_ADDRESS + motor_number - 1;
    try {
        return send_single_data(address, 1);
    } catch (...) {
        std::cout << "设置电机停止失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_stop() {
    std::vector<short> stops(motor_count, 1);
    return send_multiple_data(SET_MOTOR_STOP_ADDRESS, stops);
}

bool ZWHAND::set_single_motor_absolute(int motor_number, int joint_angle) {
    if (motor_number < 1 || motor_number > 17) {
        std::cout << "控制电机角度失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    if (joint_angle < 0 || joint_angle > 1000) {
        std::cout << "控制电机角度失败: " << joint_angle << " - 角度超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS + motor_number - 1;
    try {
        return send_single_data(address, static_cast<short>(joint_angle));
    } catch (...) {
        std::cout << "控制电机角度失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_absolute(const std::vector<int>& joint_angle_list) {
    if (joint_angle_list.size() != motor_count) {
        std::cout << "控制电机角度失败: 输入参数类型错误!" << std::endl;
        return false;
    }
    
    for (int item : joint_angle_list) {
        if (item < 0 || item > 1000) {
            std::cout << "控制电机角度失败: " << item << " - 输入参数超出有效范围" << std::endl;
            return false;
        }
    }
    
    std::vector<short> angles;
    for (int item : joint_angle_list) {
        angles.push_back(static_cast<short>(item));
    }
    
    return send_multiple_data(CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS, angles);
}

bool ZWHAND::set_single_motor_relative(int motor_number, int joint_angle) {
    if (motor_number < 1 || motor_number > motor_count) {
        std::cout << "控制电机角度失败: " << motor_number << " - 输入参数超出有效范围" << std::endl;
        return false;
    }
    
    if (joint_angle < -1000 || joint_angle > 1000) {
        std::cout << "控制电机角度失败: " << joint_angle << " - 输入参数超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS + motor_number - 1;
    try {
        return send_single_data(address, static_cast<short>(joint_angle));
    } catch (...) {
        std::cout << "控制电机角度失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_relative(const std::vector<int>& joint_angle_list) {
    if (joint_angle_list.size() != motor_count) {
        std::cout << "控制电机角度失败: 输入参数类型错误!" << std::endl;
        return false;
    }
    
    for (int item : joint_angle_list) {
        if (item < 0 || item > 1000) {
            std::cout << "控制电机角度失败: " << item << " - 输入参数超出有效范围" << std::endl;
            return false;
        }
    }
    
    std::vector<short> angles;
    for (int item : joint_angle_list) {
        angles.push_back(static_cast<short>(item));
    }
    
    return send_multiple_data(CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS, angles);
}

bool ZWHAND::set_single_motor_calibration(int motor_number) {
    if (motor_number < 1 || motor_number > 17) {
        std::cout << "单电机校准失败: " << motor_number << " - 输入参数超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = SINGLE_MOTOR_CALIBRATION_ADDRESS + motor_number - 1;
    try {
        return send_single_data(address, 1);
    } catch (...) {
        std::cout << "单电机校准失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_calibration() {
    return send_single_data(ALL_MOTOR_CALIBRATION_ADDRESS, 1);
}

bool ZWHAND::set_all_step_motor_calibration() {
    return send_single_data(ALL_STEP_MOTOR_CALIBRATION_ADDRESS, 1);
}

// 数据读取函数实现
short ZWHAND::get_initialize_state() {
    std::vector<short> result = read_multiple_data(INITIALIZE_DATA_ADDRESS, 1);
    return result.empty() ? -1 : result[0];
}

short ZWHAND::get_bootloader_version() {
    std::vector<short> result = read_multiple_data(BOOTLOADER_VERSION_ADDRESS, 1);
    return result.empty() ? -1 : result[0];
}

short ZWHAND::get_hardware_version() {
    std::vector<short> result = read_multiple_data(HARDWARE_VERSION_ADDRESS, 1);
    return result.empty() ? -1 : result[0];
}

short ZWHAND::get_software_version() {
    std::vector<short> result = read_multiple_data(SOFTWARE_VERSION_ADDRESS, 1);
    return result.empty() ? -1 : result[0];
}

std::vector<short> ZWHAND::get_device_error() {
    return read_multiple_data(DEVICE_ERROR_ADDRESS, 9);
}

short ZWHAND::get_device_voltage() {
    std::vector<short> result = read_multiple_data(DEVICE_VOLTAGE_ADDRESS, 1);
    return result.empty() ? -1 : result[0];
}

std::vector<short> ZWHAND::get_motor_locked_state() {
    return read_multiple_data(MOTOR_LOCK_STATE_ADDRESS, motor_count);
}

std::vector<short> ZWHAND::get_motor_real_angle() {
    return read_multiple_data(MOTOR_ANGLE_ADDRESS, motor_count);
}

std::vector<short> ZWHAND::get_joint_skin_moment() {
    return read_multiple_data(JOINT_SKIN_ADDRESS, 5);
}
