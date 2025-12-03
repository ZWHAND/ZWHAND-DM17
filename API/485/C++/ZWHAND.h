#ifndef ZWHAND_H
#define ZWHAND_H

#include <windows.h>
#include <vector>
#include <string>

class ZWHAND {
public:
    // 构造函数
    ZWHAND(int lqs_id, const std::string& port, int baud);
    
    // 析构函数
    ~ZWHAND();
    
    // 串口操作
    bool open_zwhand();
    bool close_zwhand();
    
    // 基础通信函数
    std::vector<unsigned char> modbus_data_receive(const std::vector<unsigned char>& detect_data, int receive_data_len);
    static std::vector<unsigned char> modbus_crc(const std::vector<unsigned char>& data);
    bool send_single_data(unsigned char address, short data);
    bool send_multiple_data(unsigned char start_address, const std::vector<short>& data);
    std::vector<short> read_multiple_data(unsigned char start_address, unsigned char address_len);
    
    // 设备控制函数
    bool set_id(int lqs_id);                                                    //设置ID
    bool set_baud(int baud_order, bool is_restart = true);                      //设置波特率
    bool set_error_clear();                                                     //清除错误
    bool set_power_off_save(int save_type);                                     //设置断电保存
    bool set_factory_data_reset();                                              //恢复出厂设置
    
    // 电机控制函数
    bool set_single_motor_speed(int motor_number, int speed);                   //设置单个电机速度
    bool set_all_motor_speed(int speed);                                        //设置所有电机速度
    bool set_single_motor_current(int motor_number, int current);               //设置单个电机电流
    bool set_all_motor_current(int current);                                    //设置所有电机电流
    bool set_single_motor_stop(int motor_number);                               //设置单个电机停止
    bool set_all_motor_stop();                                                  //设置所有电机停止
    bool set_single_motor_absolute(int motor_number, int joint_angle);          //设置单个电机绝对角度
    bool set_all_motor_absolute(const std::vector<int>& joint_angle_list);      //设置所有电机绝对角度
    bool set_single_motor_relative(int motor_number, int joint_angle);          //设置单个电机相对角度
    bool set_all_motor_relative(const std::vector<int>& joint_angle_list);      //设置所有电机相对角度
    bool set_single_motor_calibration(int motor_number);                        //设置单个电机校准
    bool set_all_motor_calibration();                                           //设置所有电机校准
    bool set_all_step_motor_calibration();                                      //设置所有步进电机校准
    
    // 数据读取函数
    short get_initialize_state();                                               //获取初始化状态
    short get_bootloader_version();                                             //获取bootloader版本
    short get_hardware_version();                                               //获取硬件版本
    short get_software_version();                                               //获取软件版本
    std::vector<short> get_device_error();                                      //获取设备错误
    short get_device_voltage();                                                 //获取设备电压
    std::vector<short> get_motor_locked_state();                                //获取电机堵转状态
    std::vector<short> get_motor_real_angle();                                  //获取电机实际角度
    std::vector<short> get_joint_skin_moment();                                 //获取指尖力矩

private:
    HANDLE hSerial;
    std::string port;
    int baudrate;
    int lqs_id;
    int motor_count;
    int initial_lqs_id;
    int initial_baud;
    
    // 波特率等级
    std::vector<int> BAUD_RATE_LEVELS;
    
    // 寄存器地址定义
    static const unsigned char SET_ID_ADDRESS = 0x00;
    static const unsigned char SET_BAUD_ADDRESS = 0x01;
    static const unsigned char CLEAR_ERROR_ADDRESS = 0x02;
    static const unsigned char SET_POWER_OFF_SAVE_ADDRESS = 0x03;
    static const unsigned char FACTORY_DATA_RESET_ADDRESS = 0x04;
    static const unsigned char SET_SPEED_ADDRESS = 0x05;
    static const unsigned char SET_CURRENT_ADDRESS = 0x16;
    static const unsigned char SET_MOTOR_STOP_ADDRESS = 0x27;
    static const unsigned char CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS = 0x38;
    static const unsigned char CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS = 0x49;
    static const unsigned char SINGLE_MOTOR_CALIBRATION_ADDRESS = 0x5A;
    static const unsigned char ALL_MOTOR_CALIBRATION_ADDRESS = 0x6C;
    static const unsigned char ALL_STEP_MOTOR_CALIBRATION_ADDRESS = 0x6B;
    static const unsigned char INITIALIZE_DATA_ADDRESS = 0x00;
    static const unsigned char BOOTLOADER_VERSION_ADDRESS = 0x01;
    static const unsigned char HARDWARE_VERSION_ADDRESS = 0x02;
    static const unsigned char SOFTWARE_VERSION_ADDRESS = 0x03;
    static const unsigned char DEVICE_ERROR_ADDRESS = 0x04;
    static const unsigned char DEVICE_VOLTAGE_ADDRESS = 0x0D;
    static const unsigned char MOVING_RANGE_ADDRESS = 0x0E;
    static const unsigned char MOTOR_LOCK_STATE_ADDRESS = 0x1F;
    static const unsigned char MOTOR_ANGLE_ADDRESS = 0x30;
    static const unsigned char JOINT_SKIN_ADDRESS = 0x41;
};

#endif // ZWHAND_H
