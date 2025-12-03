import time
import serial
import serial.tools.list_ports


class ZWHAND:
    def __init__(self, lqs_id, port, baud):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 2

        self.motor_count = 17
        self.initial_lqs_id = 1
        self.initial_baud = 115200
        self.lqs_id = lqs_id
        self.BAUD_RATE_LEVELS = [9600, 115200, 921600, 2000000]
        self.SET_ID_ADDRESS = 0x00
        self.SET_BAUD_ADDRESS = 0x01
        self.CLEAR_ERROR_ADDRESS = 0x02
        self.SET_POWER_OFF_SAVE_ADDRESS = 0x03
        self.FACTORY_DATA_RESET_ADDRESS = 0x04
        self.SET_SPEED_ADDRESS = 0x05
        self.SET_CURRENT_ADDRESS = 0x16
        self.SET_MOTOR_STOP_ADDRESS = 0x27
        self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS = 0x38
        self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS = 0x49
        self.SINGLE_MOTOR_CALIBRATION_ADDRESS = 0x5A
        self.ALL_MOTOR_CALIBRATION_ADDRESS = 0x6C
        self.ALL_STEP_MOTOR_CALIBRATION_ADDRESS = 0x6B
        self.INITIALIZE_DATA_ADDRESS = 0x00
        self.BOOTLOADER_VERSION_ADDRESS = 0x01
        self.HARDWARE_VERSION_ADDRESS = 0x02
        self.SOFTWARE_VERSION_ADDRESS = 0x03
        self.DEVICE_ERROR_ADDRESS = 0x04
        self.DEVICE_VOLTAGE_ADDRESS = 0x0D
        self.MOVING_RANGE_ADDRESS = 0x0E
        self.MOTOR_LOCK_STATE_ADDRESS = 0x1F
        self.MOTOR_ANGLE_ADDRESS = 0x30
        self.JOINT_SKIN_ADDRESS = 0x41
        try:
            self.ser.open()
            if not self.ser.is_open:
                raise serial.SerialException(f"Failed to open serial port {self.ser.port}")
            print(f"串口<{self.ser.port}>初始化成功")
        except serial.SerialException as e:
            print(f"[串口异常] 无法打开端口 {self.ser.port}: {e}")
        except Exception as e:
            print(f"[未知错误] 初始化串口时发生异常: {e}")

    def close_zwhand(self):
        try:
            self.ser.close()
            return True
        except Exception as e:
            print(f"[串口异常] 串口关闭时发生异常: {e}")
            return False

    #  modbus接收数据
    def modbus_data_receive(self, detect_data, receive_data_len):
        """
        接收串口数据函数。

        最长数据接收时间为500ms
        该函数从串口接收数据，并根据数据格式解析数据。如果数据有效，它将处理数据并发出信号。
        """
        start_time = time.time()
        available_num = 0
        while True:
            if time.time() - start_time > 0.5:
                data = self.ser.read(available_num)
                text = ''.join(f"{byte:02X} " for byte in data)
                print('超时', '数据接收失败！接收数据:', text)
                break
            try:
                if self.ser and self.ser.is_open:
                    available_num = self.ser.in_waiting
                else:
                    print("串口未打开")
            except Exception as e:
                # 可记录异常日志，例如：logging.error(f"串口读取失败: {e}")
                print(f"[串口异常] 串口读取失败: {e}")

            # 如果有数据等待读取
            if available_num >= receive_data_len:
                #  读取数据
                data = self.ser.read(available_num)
                #  界面数据更新
                text = ''.join(f"{byte:02X} " for byte in data)
                # print('接收数据:', text)
                #  接受数据校验
                data_list = list(data)
                for num in range(len(detect_data)):
                    if data_list[num] != detect_data[num]:
                        print('异常：接收数据校验失败！')
                        return False
                #  截取数据段
                need_data = data_list[len(detect_data):-2]
                if need_data:
                    decimal_list = []
                    for i in range(0, len(need_data), 2):
                        if i + 1 == len(need_data):
                            break
                        decimal_value = (need_data[i] << 8) + need_data[i + 1]
                        decimal_list.append(decimal_value)
                    if len(decimal_list) == 1:
                        return decimal_list[0]
                    return decimal_list
                else:
                    return True
        return False

    #  modbus的CRC校验和计算
    @staticmethod
    def modbus_crc(data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        crc_high = (crc >> 8) & 0xFF
        crc_low = crc & 0xFF
        data += [crc_low, crc_high]
        cmd = bytes(data)
        return cmd

    #  发送单地址指令
    def send_single_data(self, address, data):
        """
        发送单个地址的数据指令到指定设备。

        参数:
        address -- 寄存器地址，int
        data -- 要写入的数据，int（-32768 ~ 32767 for 2 bytes）

        此函数负责将给定的数据打包成指令，并通过串口发送到指定设备。
        """
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            # 将整数转换为2字节的二进制数据
            bytes_value = data.to_bytes(2, byteorder='big', signed=True)
            step_list = list(bytes_value)
            # 构造单电机控制指令
            single_motor_cmd = [self.lqs_id, 0x10, 0x00, address, 0x00,
                                0x01, 0x02, step_list[0], step_list[1]]
            # 添加CRC校验
            cmd = self.modbus_crc(single_motor_cmd)
            try:
                # print("发送数据:", ''.join(f"{byte:02X} " for byte in list(cmd)))
                send_num = self.ser.write(cmd)
                if send_num == len(single_motor_cmd):
                    # 成功发送后接收数据
                    receive_data = self.modbus_data_receive(single_motor_cmd[:6], 8)
                    return receive_data
                else:
                    print("数据未发送完成！")
            except Exception as e:
                # 异常处理，打印发送失败的原因
                print("发送失败:", e)
            return False

    #  发送多地址指令
    def send_multiple_data(self, start_address, data):
        """
        发送多个数据到指定的灵巧手。

        :param start_address: 数据写入的起始地址，决定了数据在设备内存中的写入位置。
        :param data: 待发送的数据列表，包含多个要写入设备内存的整数数据。
        :return: 如果数据发送成功并正确接收到回应数据，则返回接收的数据；否则返回False。
        """
        # 检查串口是否已打开
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            # 计算地址长度，即需要写入的数据数量
            address_len = len(data)
            # 初始化命令数据数组，长度为7加上两倍的地址长度（每个地址对应两个字节的数据）
            cmd_data = [0] * (7 + address_len*2)

            #  灵巧手ID
            cmd_data[0] = self.lqs_id
            #  写入服务
            cmd_data[1] = 0x10
            #  起始地址
            cmd_data[2] = 0x0
            cmd_data[3] = start_address
            #  地址数
            cmd_data[4] = 0x0
            cmd_data[5] = address_len
            #  数据位
            cmd_data[6] = address_len * 2
            #  数据
            for i in range(len(data)):
                # 将整数转换为2字节的二进制数据，以适应Modbus协议
                bytes_value = data[i].to_bytes(2, byteorder='big', signed=True)
                step_list = list(bytes_value)
                cmd_data[i * 2 + 7] = step_list[0]
                cmd_data[i * 2 + 8] = step_list[1]

            # 添加CRC校验码
            cmd = self.modbus_crc(cmd_data)

            try:
                # print("发送数据:", ''.join(f"{byte:02X} " for byte in list(cmd)))
                # 发送数据
                send_num = self.ser.write(cmd)
                # 检查发送的数据长度是否与预期相符
                if send_num == len(cmd_data):
                    # 接收设备返回的数据，并返回解析后的数据
                    receive_data = self.modbus_data_receive(cmd_data[:6], 8)
                    return receive_data
                else:
                    # 发送失败
                    print("数据未发送完成！")
            except Exception as e:
                # 捕获异常并打印错误信息
                print("发送失败:", e)
            # 如果发送失败或接收数据失败，则返回0
            return False

    #  读取多地址数据指令
    def read_multiple_data(self, start_address, address_len):
        """
        读取多个地址的数据。

        构建一个Modbus指令，用于从指定的起始地址读取指定长度的数据。

        参数:
        - start_address: 起始地址。
        - address_len: 要读取的数据长度。

        返回:
        - 成功读取的数据，如果发生错误或数据未成功发送，则返回False。
        """
        if self.ser and self.ser.is_open:
            # 直接清空输入缓冲区
            self.ser.reset_input_buffer()
            # 构建命令数据
            cmd_data = [0] * 6
            cmd_data[0] = self.lqs_id
            cmd_data[1] = 0x04
            cmd_data[2] = 0x0
            cmd_data[3] = start_address
            cmd_data[4] = 0x0
            cmd_data[5] = address_len
            # 添加CRC校验
            cmd = self.modbus_crc(cmd_data)
            try:
                # print("发送数据:", ''.join(f"{byte:02X} " for byte in list(cmd)))
                # 发送命令
                send_num = self.ser.write(cmd)
                # 检查发送的数据长度是否正确
                if send_num == len(cmd_data):
                    # 接收返回的数据
                    receive_data = self.modbus_data_receive([self.lqs_id, 0x04, address_len*2], address_len*2+5)
                    return receive_data
                else:
                    print("数据未发送完成！")
            except Exception as e:
                print("发送失败:", e)
        return False

    def set_id(self, lqs_id):
        """
        设置设备ID。

        参数:
        - lqs_id: 设备ID。
        - 类型：int
        - 范围：1-255

        返回:
        - 成功:True
        - 失败:False。
        """
        # 参数验证
        if not isinstance(lqs_id, int):
            print(f"设置ID失败: {lqs_id}，参数类型错误")
            return False
        if not (1 <= lqs_id <= 255):
            print(f"设置ID失败: {lqs_id}，参数超出有效范围1-255")
            return False
        try:
            send_ret = self.send_single_data(self.SET_ID_ADDRESS, lqs_id)
            if send_ret:
                self.lqs_id = lqs_id
                print(f"设置ID成功: {lqs_id}")
                return True
            else:
                print(f"设置ID失败: {lqs_id}")
                return False
        except Exception as e:
            print(f"设置ID失败: {lqs_id}，发送数据异常: {str(e)}")
            return False

    def set_baud(self, baud_order, is_restart=True):
        """
        设置设备波特率。

        波特率档位：1-9600, 2-115200, 3-921600, 4-2000000

        参数:
        - baud_order: 波特率顺序。
        - 类型：int
        - 范围：1-4

        - is_restart: 是否重启串口标志
        - 类型：bool
        - 默认值：True

        返回:
        - 成功:True
        - 失败:False。
        """

        # 参数验证
        if not isinstance(baud_order, int):
            print(f"设置波特率失败: {baud_order} - 参数类型错误")
            return False
        if baud_order < 1 or baud_order > len(self.BAUD_RATE_LEVELS):
            print(f"设置波特率失败: {baud_order} - 参数超出有效范围")
            return False
        try:
            send_ret = self.send_single_data(self.SET_BAUD_ADDRESS, baud_order)
            if send_ret:
                # 更新波特率设置
                self.ser.baudrate = self.BAUD_RATE_LEVELS[baud_order - 1]
                # 如果波特率已更改，需要重新配置串口
                if self.ser.baudrate != baud_order and is_restart:
                    # 先关闭串口
                    if self.ser.is_open:
                        self.ser.close()
                    # 重新打开串口
                    self.ser.open()
                print(f"设置波特率成功: {self.BAUD_RATE_LEVELS[baud_order-1]}")
                return True
            else:
                print(f"设置波特率失败: {self.BAUD_RATE_LEVELS[baud_order-1]}")
                return False
        except Exception as e:
            print(f"设置波特率异常: {self.BAUD_RATE_LEVELS[baud_order-1]} - {str(e)}")
            return False

    def set_error_clear(self):
        """
        清除错误。

        返回:
        - 成功:True
        - 失败:False。
        """
        return self.send_single_data(self.CLEAR_ERROR_ADDRESS, 1)

    def set_power_off_save(self, save_type):
        """
        设置掉电保存。

        参数:
        - save_type: 1-配置保存(设备id、波特率等)，2-参数保存(运动速度、电流等)。
        - 类型：int
        - 范围：1-2

        返回:
        - 成功:True
        - 失败:False。
        """
        # 验证参数范围
        if not isinstance(save_type, int) or save_type < 1 or save_type > 2:
            print(f"设置掉电保存失败: {save_type} - 参数超出有效范围")
            return False
        try:
            return self.send_single_data(self.SET_POWER_OFF_SAVE_ADDRESS, save_type)
        except Exception as e:
            print(f"设置掉电保存异常: {str(e)}")
            return False

    def set_factory_data_reset(self):
        """
        恢复出厂设置。

        返回:
        - 成功:True
        - 失败:False。
        """
        try:
            send_ret = self.send_single_data(self.FACTORY_DATA_RESET_ADDRESS, 1)
            if send_ret:
                # 保存原始状态用于回滚
                original_lqs_id = self.lqs_id
                original_baudrate = self.ser.baudrate

                try:
                    # 重置本地状态变量到初始值
                    self.lqs_id = self.initial_lqs_id

                    # 如果波特率已更改，需要重新配置串口
                    if self.ser.baudrate != self.initial_baud:
                        # 先关闭串口
                        if self.ser.is_open:
                            self.ser.close()

                        # 更新波特率设置
                        self.ser.baudrate = self.initial_baud

                        # 重新打开串口
                        self.ser.open()

                    print("恢复出厂成功")
                    return True
                except Exception as e:
                    # 回滚状态
                    self.lqs_id = original_lqs_id
                    self.ser.baudrate = original_baudrate
                    if self.ser.is_open:
                        self.ser.close()
                        self.ser.open()
                    print(f"恢复出厂设置时发生错误: {e}")
                    return False
            else:
                print("恢复出厂失败")
                return False
        except Exception as e:
            print(f"发送恢复出厂指令时发生错误: {e}")
            print("恢复出厂失败")
            return False

    def set_single_motor_speed(self, motor_number, speed):
        """
        设置单个电机速度。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-17
        - speed: 速度档位
        - 类型：int
        - 范围：1-100

        返回:
        - 成功:True
        - 失败:False。
        """
        # 参数类型检查
        if not isinstance(motor_number, int) or not isinstance(speed, int):
            print(f"设置速度失败: 参数类型错误!")
            return False
        # 参数范围检查
        if motor_number < 1 or motor_number > 17:
            print(f"设置速度失败: {motor_number} - 参数超出有效范围")
            return False
        if speed < 1 or speed > 100:
            print(f"设置速度失败: {speed} - 速度超出有效范围")
            return False
        address = self.SET_SPEED_ADDRESS + motor_number - 1
        try:
            result = self.send_single_data(address, speed)
            return result
        except Exception as e:
            print(f"设置速度异常: {str(e)}")
            return False

    def set_all_motor_speed(self, speed):
        """
        设置所有电机速度。

        参数:
        - speed: 速度档位
        - 类型：int
        - 范围：1-100

        返回:
        - 成功:True
        - 失败:False。
        """
        # 验证参数类型和范围
        if not isinstance(speed, int):
            print(f"设置所有电机速度失败: 速度参数类型错误!")
            return False
        if speed < 1 or speed > 100:
            print(f"设置所有电机速度失败: {speed} - 速度超出有效范围")
            return False
        return self.send_multiple_data(self.SET_SPEED_ADDRESS, [speed]*self.motor_count)

    def set_single_motor_current(self, motor_number, current):
        """
        设置单个电机电流。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-17
        - current: 电流档位
        - 类型：int
        - 范围：1-100

        返回:
        - 成功:True
        - 失败:False。
        """
        # 验证参数类型和范围
        if not isinstance(motor_number, int) or not isinstance(current, int):
            print(f"设置电流失败: 参数类型错误!")
            return False
        if motor_number < 1 or motor_number > 17:
            print(f"设置电流失败: {motor_number} - 参数超出有效范围")
            return False
        if current < 1 or current > 100:
            print(f"设置电流失败: {current} - 电流超出有效范围")
            return False
        address = self.SET_CURRENT_ADDRESS + motor_number - 1
        try:
            result = self.send_single_data(address, current)
            return result
        except Exception as e:
            print(f"设置电流失败: {e}")
            return False

    def set_all_motor_current(self, current):
        """
        设置所有电机电流。

        参数:
        - current: 电流档位
        - 类型：int
        - 范围：1-100

        返回:
        - 成功:True
        - 失败:False。
        """
        # 验证参数类型和范围
        if not isinstance(current, int):
            print(f"设置所有电机电流失败: 电流参数类型错误!")
            return False
        if current < 1 or current > 100:
            print(f"设置所有电机电流失败: {current} - 电流超出有效范围")
            return False
        return self.send_multiple_data(self.SET_CURRENT_ADDRESS, [current]*self.motor_count)

    def set_single_motor_stop(self, motor_number):
        """
        设置单个电机紧急停止。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-17

        返回:
        - 成功:True
        - 失败:False。
        """
        if not isinstance(motor_number, int):
            print(f"设置电机停止失败: {motor_number} - 参数类型错误!")
            return False
        if motor_number < 1 or motor_number > 17:
            print(f"设置电机停止失败: {motor_number} - 参数超出有效范围")
            return False
        address = self.SET_MOTOR_STOP_ADDRESS + motor_number - 1
        try:
            result = self.send_single_data(address, 1)
            return result
        except Exception as e:
            print(f"设置电机停止失败: {e}")
            return False

    def set_all_motor_stop(self):
        """
        设置所有电机紧急停止。

        返回:
        - 成功:True
        - 失败:False。
        """
        return self.send_multiple_data(self.SET_MOTOR_STOP_ADDRESS, [1]*self.motor_count)

    def set_single_motor_absolute(self, motor_number, joint_angle):
        """
        设置单个电机绝对位置角度挡位。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-17
        - joint_angle: 角度挡位
        - 类型：int
        - 范围：0-1000

        返回:
        - 成功:True
        - 失败:False。
        """
        if not isinstance(motor_number, int) or not isinstance(joint_angle, int):
            print(f"控制电机角度失败: 输入参数类型错误!")
            return False
        if motor_number < 1 or motor_number > 17:
            print(f"控制电机角度失败: {motor_number} - 参数超出有效范围")
            return False
        if joint_angle < 0 or joint_angle > 1000:
            print(f"控制电机角度失败: {joint_angle} - 角度超出有效范围")
            return False
        address = self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS + motor_number - 1
        try:
            result = self.send_single_data(address, joint_angle)
            return result
        except Exception as e:
            print(f"控制电机角度失败: {e}")
            return False

    def set_all_motor_absolute(self, joint_angle_list):
        """
        设置所有关节电机绝对位置角度挡位。

        参数:
        - joint_angle_list: 关节角度挡位列表
        - 类型：[int]*17
        - 范围：0-1000

        返回:
        - 成功:True
        - 失败:False。
        """
        if not isinstance(joint_angle_list, list) or len(joint_angle_list) != self.motor_count:
            print(f"控制电机角度失败: 输入参数类型错误!")
            return False
        for item in joint_angle_list:
            if not isinstance(item, int):
                print(f"控制电机角度失败: 输入参数类型错误!")
                return False
            if item < 0 or item > 1000:
                print(f"控制电机角度失败: {item} - 输入参数超出有效范围")
                return False
        return self.send_multiple_data(self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS, joint_angle_list)

    def set_single_motor_relative(self, motor_number, joint_angle):
        """
        设置单个关节电机相对位置角度挡位。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-17
        - joint_angle: 角度挡位
        - 类型：int
        - 范围：-1000-1000

        返回:
        - 成功:True
        - 失败:False。
        """
        if not isinstance(motor_number, int) or not isinstance(joint_angle, int):
            print(f"控制电机角度失败: 输入参数类型错误!")
            return False
        if motor_number < 1 or motor_number > self.motor_count:
            print(f"控制电机角度失败: {motor_number} - 输入参数超出有效范围")
            return False
        if joint_angle < -1000 or joint_angle > 1000:
            print(f"控制电机角度失败: {joint_angle} - 输入参数超出有效范围")
            return False
        address = self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS + motor_number - 1
        try:
            result = self.send_single_data(address, joint_angle)
            return result
        except Exception as e:
            print(f"控制电机角度失败: {e}")
            return False

    def set_all_motor_relative(self, joint_angle_list):
        """
        设置所有关节电机相对位置角度挡位。

        参数:
        - joint_angle_list: 关节角度挡位列表
        - 类型：[int]*17
        - 范围：0-1000

        返回:
        - 成功:True
        - 失败:False。
        """
        if not isinstance(joint_angle_list, list) or len(joint_angle_list) != self.motor_count:
            print(f"控制电机角度失败: 输入参数类型错误!")
            return False
        for item in joint_angle_list:
            if not isinstance(item, int):
                print(f"控制电机角度失败: 输入参数类型错误!")
                return False
            if item < 0 or item > 1000:
                print(f"控制电机角度失败: {item} - 输入参数超出有效范围")
                return False
        return self.send_multiple_data(self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS, joint_angle_list)

    def set_single_motor_calibration(self, motor_number):
        """
        单个关节电机零位校准。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-17

        返回:
        - 成功:True
        - 失败:False。
        """
        if not isinstance(motor_number, int):
            print(f"单电机校准失败: {motor_number} - 输入参数类型错误!")
            return False
        if motor_number < 1 or motor_number > 17:
            print(f"单电机校准失败: {motor_number} - 输入参数超出有效范围")
            return False
        address = self.SINGLE_MOTOR_CALIBRATION_ADDRESS + motor_number - 1
        try:
            result = self.send_single_data(address, 1)
            return result
        except Exception as e:
            print(f"单电机校准失败: {e}")
            return False

    def set_all_motor_calibration(self):
        """
        设置全关节电机(全手)零位校准。

        返回:
        - 成功:True
        - 失败:False。
        """
        return self.send_single_data(self.ALL_MOTOR_CALIBRATION_ADDRESS, 1)

    def set_all_step_motor_calibration(self):
        """
        设置全步进关节电机零位校准。

        返回:
        - 成功:True
        - 失败:False。
        """
        return self.send_single_data(self.ALL_STEP_MOTOR_CALIBRATION_ADDRESS, 1)

    def get_initialize_state(self):
        """
        获取初始化状态。

        返回:
        - 获取成功:1-初始化状态
        - 获取失败:False。
        """
        return self.read_multiple_data(self.INITIALIZE_DATA_ADDRESS, 1)

    def get_bootloader_version(self):
        """
        获取bootloader版本。

        返回:
        - 获取成功:XX，XX为十进制格式的bootloader版本
        - 获取失败:False。
        """
        return self.read_multiple_data(self.BOOTLOADER_VERSION_ADDRESS, 1)

    def get_hardware_version(self):
        """
        获取硬件版本。

        返回:
        - 获取成功:XX，XX为十进制格式的硬件版本
        - 获取失败:False
        """
        return self.read_multiple_data(self.HARDWARE_VERSION_ADDRESS, 1)

    def get_software_version(self):
        """
        获取软件版本。

        返回:
        - 获取成功:XX，XX为十进制格式的软件版本
        - 获取失败:False。
        """

        return self.read_multiple_data(self.SOFTWARE_VERSION_ADDRESS, 1)

    def get_device_error(self):
        """
        获取设备错误码。

        返回:
        - 获取成功:数据列表[XX]*9，XX为十进制格式的设备错误码
        - 获取失败:False。
        """
        return self.read_multiple_data(self.DEVICE_ERROR_ADDRESS, 9)

    def get_device_voltage(self):
        """
        获取设备电压。

        电压系数：0.001
        电压单位：V

        返回:
        - 获取成功:XX，XX*0.001为十进制格式的设备电压
        - 获取失败:False。
        """
        return self.read_multiple_data(self.DEVICE_VOLTAGE_ADDRESS, 1)

    def get_motor_locked_state(self):
        """
        获取所有关节电机堵转状态。

        电机堵转状态：1-堵转，0-正常

        返回:
        - 获取成功:数据列表[XX]*17，XX为十进制格式的电机堵转状态
        - 获取失败:False。
        """
        return self.read_multiple_data(self.MOTOR_LOCK_STATE_ADDRESS, self.motor_count)

    def get_motor_real_angle(self):
        """
        获取所有关节电机实际角度挡位。

        返回:
        - 获取成功:数据列表[XX]*17，XX为十进制格式的电机实际角度挡位
        - 获取失败:False。
        """
        return self.read_multiple_data(self.MOTOR_ANGLE_ADDRESS, self.motor_count)

    def get_fingertip_skin_moment(self):
        """
        获取所有指尖皮肤力矩。

        返回:
        - 获取成功:数据列表[XX]*17，XX为十进制格式的关节皮肤力矩
        - 获取失败:False。
        """
        return self.read_multiple_data(self.JOINT_SKIN_ADDRESS, 5)




