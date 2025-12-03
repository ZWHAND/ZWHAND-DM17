import threading
import time
from collections import deque

from zlgcan import *


class ZWHAND:
    def __init__(self, lqs_id=0x01, arb_baud_rate='500000', data_baud_rate='1000000', zlgcan_type=1):
        """
        :param lqs_id: 设备ID
        :param arb_baud_rate: 仲裁波特率
        :param data_baud_rate: 数据波特率
        :param zlgcan_type: ZCAN类型      1-周立功， 0-智嵌物联
        """
        self.lqs_id = lqs_id                                # 设备ID
        self.initial_lqs_id = 0x01                          # 初始设备ID
        self.arb_baud_rate = str(arb_baud_rate)                  # 仲裁波特率
        self.data_baud_rate = str(data_baud_rate)                # 数据波特率

        self.zlg_can = ZCAN(zlgcan_type)                    # 实例化ZCAN类
        self.handle = ''                                    # 设备句柄
        self.chn_handle = ''                                # 通道句柄
        self.device_index = 0                               # 设备索引
        self.reserved = 0                                   # 保留参数
        self.chn_index = 0                                  # 通道索引

        self.can_is_open = False                            # 设备是否打开
        self.receive_angle_id = 0x300 + self.lqs_id         # 接收检测的角度报文ID
        self.receive_locked_id = 0x200 + self.lqs_id        # 接收检测堵转状态报文ID
        self.receive_event_id = 0x100 + self.lqs_id         # 接收检测的事件型报文ID
        self.init_config_data = deque(maxlen=2)             # 初始化配置报文反馈状态队列
        self.boot_loader = deque(maxlen=2)                  # bootloader反馈状态队列
        self.hardware_version = deque(maxlen=2)             # 硬件版本反馈状态队列
        self.software_version = deque(maxlen=2)             # 软件版本反馈状态队列
        self.error_code = deque(maxlen=2)                   # 错误码反馈状态队列
        self.system_voltage = deque(maxlen=2)               # 系统电压反馈状态队列
        self.motor_angles = deque(maxlen=2)                 # 角度反馈状态队列
        self.locked_state = deque(maxlen=2)                 # 堵转状态反馈状态队列
        self.fingertip_skin_data = deque(maxlen=2)          # 传感器数据反馈状态队列
        self.set_lqs_id_state = 0                           # 设置设备ID状态
        self.set_baud_state = 0                             # 设置波特率状态
        self.clear_error_state = 0                          # 清除错误码状态
        self.set_factory_state = 0                          # 恢复出厂设置状态
        self.set_power_off_save_state = 0                   # 设置掉电保存状态
        self.receive_thread = threading.Thread(target=self.receive_messages)
        self.motor_count = 17
        self.write_fun_type = 0x10
        self.read_fun_type = 0x04
        self.receive_detect_time = 2
        self.arb_baud_rate_list = ["500000", "1000000"]         # 仲裁波特率列表
        self.initial_arb_baud_rate = "500000"
        self.data_baud_rate_list = ['500000', '1000000', '2000000', '5000000']      # 数据波特率列表
        self.initial_data_baud_rate = "1000000"
        self.BAUD_RATE_LEVELS = ['500K/500K', '500K/1000K', '500K/2000K', '1000K/5000K', '1000K/1000K']
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
        self.DEVICE_ERROR_ADDRESS = 0x05
        self.DEVICE_VOLTAGE_ADDRESS = 0x04
        self.MOVING_RANGE_ADDRESS = 0x0E
        self.MOTOR_LOCK_STATE_ADDRESS = 0x1F
        self.MOTOR_ANGLE_ADDRESS = 0x30
        self.FINGERTIP_SKIN_ADDRESS = 0X41

        self.open_canfd_device()
        self.start_canfd()

    # 打开CANFD设备，使用其他CANFD硬件驱动时需重构此函数
    def open_canfd_device(self, device_index=0, reserved=0):
        # self.zlg_can = ZCAN()
        self.device_index = device_index
        self.reserved = reserved
        self.handle = self.zlg_can.OpenDevice(ZCAN_USBCANFD_100U, self.device_index, self.reserved)
        if self.handle == INVALID_DEVICE_HANDLE:
            print("Open CANFD Device failed!")
            return False
        print("device handle:%d." % self.handle)
        info = self.zlg_can.GetDeviceInf(self.handle)
        print("Device Information:\n%s" % info)
        return True

    # 启动CANFD通道，使用其他CANFD硬件驱动时需重构此函数
    def start_canfd(self, chn=0):
        self.chn_index = chn
        self.chn_handle = canfd_start(self.zlg_can, self.handle, self.chn_index, self.arb_baud_rate, self.data_baud_rate)
        if self.chn_handle != self.chn_handle:
            print("Start CANFD Channel failed!")
            return False
        print("channel handle:%d." % self.chn_handle)
        self.can_is_open = True
        self.receive_thread.start()
        return True

    # 关闭CANFD驱动，使用其他CANFD硬件驱动时需重构此函数
    def close_device(self):
        self.can_is_open = False
        ret = self.zlg_can.CloseDevice(self.handle)
        if ret == 1:
            print("Close Device success! ")
        return ret

    # 清除CANFD接收缓存区，使用其他CANFD硬件驱动时需重构此函数
    def clear_buffer(self):
        ret = self.zlg_can.ClearBuffer(self.chn_handle)
        if ret == 1:
            print("Clear Buffer success! ")
        return ret

    def reset_canfd(self):
        return self.zlg_can.ResetCAN(self.chn_handle)

    def message_analysis(self, can_id, canfd_data):
        #  关节角度报文接收处理
        if can_id == self.receive_angle_id:
            self.motor_angles.append([int.from_bytes(canfd_data[i:i + 2], byteorder='big') for i in
                                      range(0, len(canfd_data), 2)][:self.motor_count])
        elif can_id == self.receive_event_id:
            int_data = [int.from_bytes(canfd_data[i:i + 1], byteorder='big') for i in range(8)]
            print(f'receive one frame of canFD message: {can_id} ', int_data)
            if int_data[0] == 0x4:  # 获取初始化配置成功
                read_data = int.from_bytes(canfd_data[3:5], byteorder='big')
                if int_data[1] == self.INITIALIZE_DATA_ADDRESS:
                    self.init_config_data.append(read_data)
                elif int_data[1] == self.BOOTLOADER_VERSION_ADDRESS:
                    self.boot_loader.append(read_data)
                elif int_data[1] == self.HARDWARE_VERSION_ADDRESS:
                    self.hardware_version.append(read_data)
                elif int_data[1] == self.SOFTWARE_VERSION_ADDRESS:
                    self.software_version.append(read_data)
                elif int_data[1] == self.DEVICE_ERROR_ADDRESS:
                    self.error_code.append([int.from_bytes(canfd_data[i:i + 2], byteorder='big') for i in
                                            range(0, len(canfd_data), 2)][3:12])
                elif int_data[1] == self.DEVICE_VOLTAGE_ADDRESS:
                    self.system_voltage.append(read_data)
                elif int_data[1] == self.MOTOR_LOCK_STATE_ADDRESS:
                    self.locked_state.append([int.from_bytes(canfd_data[i:i + 2], byteorder='big') for i in
                                              range(0, len(canfd_data), 2)][3:self.motor_count + 3])

            elif int_data[0] == 0x10:
                if int_data[1] == self.SET_ID_ADDRESS:  # 修改设备ID成功
                    self.set_lqs_id_state = int_data[4]
                elif int_data[1] == self.SET_BAUD_ADDRESS:  # 修改波特率成功
                    self.set_baud_state = int_data[4]
                elif int_data[1] == self.CLEAR_ERROR_ADDRESS:  # 清除设备错误成功
                    self.clear_error_state = int_data[4]
                elif int_data[1] == self.SET_POWER_OFF_SAVE_ADDRESS:  # 修改设备下电保存状态成功
                    # self.set_power_off_save_state = int_data[4]
                    pass
                elif int_data[1] == self.FACTORY_DATA_RESET_ADDRESS:  # 恢复出厂成功
                    # self.set_factory_state = int_data[4]
                    pass
                else:
                    pass

    # 接收总线CANFD报文，使用其他CANFD硬件驱动时需重构此函数
    def receive_messages(self):
        """
        接收CAN总线消息的主循环函数

        该函数在一个独立的线程中运行，持续监听CAN总线上接收到的消息。
        当检测到CANFD消息时，会调用message_analysis方法对接收到的数据进行分析处理。
        循环会一直执行直到can_is_open标志被设置为False。

        参数:
            self: 类实例本身

        返回值:
            无返回值
        """
        while self.can_is_open:
            # 获取当前待接收的CANFD消息数量
            rcv_canfd_num = self.zlg_can.GetReceiveNum(self.chn_handle, ZCAN_TYPE_CANFD)
            if rcv_canfd_num:
                # print("Receive CANFD message number:%d" % rcv_canfd_num)
                # 接收CANFD消息数据
                rcv_canfd_msgs, rcv_canfd_num = self.zlg_can.ReceiveFD(self.chn_handle, rcv_canfd_num)
                # 检查接收数据是否有效，避免空数据访问越界
                if not rcv_canfd_msgs or not rcv_canfd_num:
                    continue  # 避免空数据访问越界
                # 遍历所有接收到的消息并进行分析处理
                for message_index in range(rcv_canfd_num):
                    self.message_analysis(rcv_canfd_msgs[message_index].frame.can_id, rcv_canfd_msgs[message_index].frame.data)
            else:
                pass
        print("receive thread exit！！！")

    # 发送CANFD命令帧，需要传入CAN帧ID和数据内容，使用其他CANFD硬件驱动时需重构此函数
    def send_canfd_cmd(self, can_id, canfd_data):
        """
        发送CANFD命令帧

        参数:
            can_id (int): CAN帧ID
            canfd_data (list): CANFD数据内容，字节列表

        返回值:
            bool: 发送成功返回True，失败返回False
        """
        # 准备发送的CANFD消息结构
        transmit_canfd_num = 1
        canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()
        for i in range(transmit_canfd_num):
            canfd_msgs[i].transmit_type = 0  # 0-正常发送，2-自发自收
            canfd_msgs[i].frame.eff = 0  # 0-标准帧，1-扩展帧
            canfd_msgs[i].frame.rtr = 0  # 0-数据帧，1-远程帧
            canfd_msgs[i].frame.brs = 1  # BRS 加速标志位：0不加速，1加速
            canfd_msgs[i].frame.can_id = can_id
            canfd_msgs[i].frame.len = len(canfd_data)
            for j in range(canfd_msgs[i].frame.len):
                canfd_msgs[i].frame.data[j] = canfd_data[j]

        # 执行发送操作
        ret = 0
        for send_num in range(1):
            ret = self.zlg_can.TransmitFD(self.chn_handle, canfd_msgs, transmit_canfd_num)
            if ret:
                break

        # 格式化打印发送的数据
        text = hex(can_id)
        for cmd in canfd_data:
            text += ' ' + format(cmd & 0xFF, '02x')

        # 检查发送结果并返回状态
        if ret == transmit_canfd_num:
            print(f"Send：", text)
            return True
        else:
            print(f"Send Failed：", text)
            return False

    def int_canfd_cmd(self, fun_type, address, canfd_data):
        """
        构造并发送CANFD命令

        参数:
            fun_type: 功能类型标识
            address: 地址参数
            canfd_data: CANFD数据，可以是整数或整数列表

        返回值:
            调用send_canfd_cmd方法的返回结果
        """
        # 判断canfd_data数据类型，如果是列表，则进行转换
        if fun_type == self.read_fun_type and isinstance(canfd_data, int):
            canFD_cmd = [fun_type, address, canfd_data]
        else:
            if isinstance(canfd_data, int):
                canfd_data = [canfd_data]
            elif not isinstance(canfd_data, list):
                print(f"{canfd_data}数据类型错误")
                return 0

            # 构造基础命令结构
            canFD_cmd = [fun_type, address, len(canfd_data)]

            # 将数据转换为字节并添加到命令中
            for c_data in canfd_data:
                bytes_value = c_data.to_bytes(2, byteorder='big', signed=True)  # 将整数转换为2字节的二进制数据
                step_list = list(bytes_value)
                canFD_cmd += step_list

        # 根据命令长度进行填充或截断处理
        canFD_cmd_len = len(canFD_cmd)
        if 24 > canFD_cmd_len > 8:
            canFD_cmd += [0] * (4 - canFD_cmd_len % 4)
        elif 32 > canFD_cmd_len > 24:
            canFD_cmd += [0] * (8 - canFD_cmd_len % 8)
        elif 64 > canFD_cmd_len > 32:
            canFD_cmd += [0] * (16 - canFD_cmd_len % 16)
        elif canFD_cmd_len > 64:
            canFD_cmd = canFD_cmd[:64]
        else:
            pass

        return self.send_canfd_cmd(self.lqs_id, canFD_cmd)

    def set_id(self, new_id):
        """
        设置设备ID。

        参数:
        - new_id: 设置的设备ID。
        - 类型：int
        - 范围：1-255

        返回:
        - 成功:True
        - 失败:False。
        """
        # 参数验证
        if not isinstance(new_id, int):
            print(f"Failed to set ID: <{new_id}>，Input parameter type error!")
            return False
        if not (1 <= new_id <= 255):
            print(f"Failed to set ID: <{new_id}>，The parameter is outside the valid range of 1 to 255!")
            return False
        try:
            send_ret = self.int_canfd_cmd(self.write_fun_type, self.SET_ID_ADDRESS, new_id)
            if send_ret:
                start_time = time.time()
                while time.time() - start_time < self.receive_detect_time:
                    if self.set_lqs_id_state:
                        self.set_lqs_id_state = 0
                        self.lqs_id = new_id
                        self.receive_angle_id = 0x300 + self.lqs_id  # 接收检测的角度报文ID
                        self.receive_locked_id = 0x200 + self.lqs_id  # 接收检测堵转状态报文ID
                        self.receive_event_id = 0x100 + self.lqs_id  # 接收检测的事件型报文ID
                        # print(f"The ID has been set successfully: <{new_id}>")
                        return True
                print(f"Response timeout!")
                return False
            else:
                print(f"Failed to set ID: <{new_id}>")
                return False
        except Exception as e:
            print(f"Failed to set ID: <{new_id}>，Data transmission error: {str(e)}")
            return False

    def set_baud(self, baud_order):
        """
        设置设备波特率。

        波特率档位：1-'500K/500K', 2-'500K/1000K', 3-'500K/2000K', 4-'1000K/5000K', 5-'1000K/1000K'

        参数:
        - baud_order: 波特率顺序。
        - 类型：int
        - 范围：1-4

        返回:
        - 成功:初始化ZWHAND类对象
        - 失败:False。
        """

        # 参数验证
        if not isinstance(baud_order, int):
            print(f"Failed to set the baud rate: {baud_order} - Input parameter type error!")
            return False
        if baud_order < 1 or baud_order > len(self.BAUD_RATE_LEVELS):
            print(f"Failed to set the baud rate: {baud_order} - Parameter out of valid range!")
            return False
        index = baud_order - 1
        # 更新波特率设置
        if baud_order == 5:
            self.data_baud_rate = self.data_baud_rate_list[3]
        else:
            self.data_baud_rate = self.data_baud_rate_list[index]
        if baud_order <= 3:
            self.arb_baud_rate = self.arb_baud_rate_list[0]
        else:
            self.arb_baud_rate = self.arb_baud_rate_list[1]
        try:
            send_ret = self.int_canfd_cmd(self.write_fun_type, self.SET_BAUD_ADDRESS, baud_order)
            if send_ret:
                start_time = time.time()
                while time.time() - start_time < self.receive_detect_time:
                    if self.set_baud_state:
                        self.set_baud_state = 0
                        # self.reset_canfd()
                        self.close_device()
                        hand = ZWHAND(self.lqs_id, self.arb_baud_rate, self.data_baud_rate)
                        return hand
                # print(f"Baud rate set successfully: {self.BAUD_RATE_LEVELS[index]}")
                return False
            else:
                print(f"Failed to set the baud rate: {self.BAUD_RATE_LEVELS[index]}")
                return False
        except Exception as e:
            print(f"Abnormal baud rate setting: {self.BAUD_RATE_LEVELS[index]} - {str(e)}")
            return False

    def set_error_clear(self):
        """
        清除错误。

        返回:
        - 成功:True
        - 失败:False。
        """
        send_ret = self.int_canfd_cmd(self.write_fun_type, self.CLEAR_ERROR_ADDRESS, 1)
        if send_ret:
            start_time = time.time()
            while time.time() - start_time < self.receive_detect_time:
                if self.clear_error_state:
                    self.clear_error_state = 0
                    return True
            print(f"Response timeout!")
            return False
        else:
            print(f"Send fail!")
            return False

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
            print(f"Failed to save function settings: {save_type} - Parameter out of valid range!")
            return False
        try:
            send_ret = self.int_canfd_cmd(self.write_fun_type, self.SET_POWER_OFF_SAVE_ADDRESS, save_type)
            if send_ret:
                # start_time = time.time()
                # while time.time() - start_time < self.receive_detect_time:
                #     if self.set_power_off_save_state:
                #         self.set_power_off_save_state = 0
                #         return True
                # print(f"Response timeout!")
                return send_ret
            else:
                print(f"Failed to save function settings: {save_type}")
                return False
        except Exception as e:
            print(f"Abnormal function settings saving: {str(e)}")
            return False

    def set_factory_data_reset(self):
        """
        恢复出厂设置。

        返回:
        - 成功:初始化ZWHAND类对象
        - 失败:False。
        """
        try:
            send_ret = self.int_canfd_cmd(self.write_fun_type, self.FACTORY_DATA_RESET_ADDRESS, 1)
            if send_ret:
                # 保存原始状态用于回滚
                original_lqs_id = self.lqs_id
                original_arb_baud_rate = self.arb_baud_rate
                original_data_baud_rate = self.data_baud_rate

                try:
                    # 重置本地状态变量到初始值
                    self.lqs_id = self.initial_lqs_id
                    self.arb_baud_rate = self.initial_arb_baud_rate
                    self.data_baud_rate = self.initial_data_baud_rate
                except Exception as e:
                    # 回滚状态
                    self.lqs_id = original_lqs_id
                    self.arb_baud_rate = original_arb_baud_rate
                    self.data_baud_rate = original_data_baud_rate
                    print(f"Error occurred when restoring factory settings: {e}")
                finally:
                    self.close_device()
                    return ZWHAND(self.lqs_id, self.arb_baud_rate, self.data_baud_rate)

            else:
                print("Failed to restore factory settings!")
                return False
        except Exception as e:
            print(f"Error occurred when sending command: {e}")
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
            print(f"Failed to set speed: Input parameter type error!")
            return False
        # 参数范围检查
        if motor_number < 1 or motor_number > 17:
            print(f"Failed to set speed: {motor_number} - Parameter out of valid range!")
            return False
        if speed < 1 or speed > 100:
            print(f"Failed to set speed: {speed} - Speed out of valid range!")
            return False
        address = self.SET_SPEED_ADDRESS + motor_number - 1
        try:
            result = self.int_canfd_cmd(self.write_fun_type, address, speed)
            return result
        except Exception as e:
            print(f"Abnormal speed setting: {str(e)}")
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
            print(f"Failed to set speed: Input parameter type error!")
            return False
        if speed < 1 or speed > 100:
            print(f"Failed to set speed: {speed} - Speed out of valid range!")
            return False
        return self.int_canfd_cmd(self.write_fun_type, self.SET_SPEED_ADDRESS, [speed]*self.motor_count)

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
            print(f"Failed to set current: Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > 17:
            print(f"Failed to set current: {motor_number} - Parameter out of valid range!")
            return False
        if current < 1 or current > 100:
            print(f"Failed to set current: {current} - Current out of valid range!")
            return False
        address = self.SET_CURRENT_ADDRESS + motor_number - 1
        try:
            result = self.int_canfd_cmd(self.write_fun_type, address, current)
            return result
        except Exception as e:
            print(f"Failed to set current: {e}")
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
            print(f"Failed to set current: Input parameter type error!")
            return False
        if current < 1 or current > 100:
            print(f"Failed to set current: {current} - Current out of valid range!")
            return False
        return self.int_canfd_cmd(self.write_fun_type, self.SET_CURRENT_ADDRESS, [current]*self.motor_count)

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
            print(f"Failed to set motor stop: {motor_number} - Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > 17:
            print(f"Failed to set motor stop: {motor_number} - Parameter out of valid range!")
            return False
        address = self.SET_MOTOR_STOP_ADDRESS + motor_number - 1
        try:
            result = self.int_canfd_cmd(self.write_fun_type, address, 1)
            return result
        except Exception as e:
            print(f"Failed to set motor stop: {e}")
            return False

    def set_all_motor_stop(self):
        """
        设置所有电机紧急停止。

        返回:
        - 成功:True
        - 失败:False。
        """
        return self.int_canfd_cmd(self.write_fun_type, self.SET_MOTOR_STOP_ADDRESS, [1]*self.motor_count)

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
            print(f"Failed to set motor angle: Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > 17:
            print(f"Failed to set motor angle: {motor_number} - Input parameter out of valid range")
            return False
        if joint_angle < 0 or joint_angle > 1000:
            print(f"Failed to set motor angle: {joint_angle} - Input parameter out of valid range")
            return False
        address = self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS + motor_number - 1
        try:
            result = self.int_canfd_cmd(self.write_fun_type, address, joint_angle)
            return result
        except Exception as e:
            print(f"Failed to set motor angle: {e}")
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
            print(f"Failed to set motor angle: Input parameter type error!")
            return False
        for item in joint_angle_list:
            if not isinstance(item, int):
                print(f"Failed to set motor angle: Input parameter type error!")
                return False
            if item < 0 or item > 1000:
                print(f"Failed to set motor angle: {item} - Input parameter out of valid range")
                return False
        return self.int_canfd_cmd(self.write_fun_type, self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS, joint_angle_list)

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
            print(f"Failed to set motor angle: Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > self.motor_count:
            print(f"Failed to set motor angle: {motor_number} - Input parameter out of valid range!")
            return False
        if joint_angle < -1000 or joint_angle > 1000:
            print(f"Failed to set motor angle: {joint_angle} - Input parameter out of valid range!")
            return False
        address = self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS + motor_number - 1
        try:
            result = self.int_canfd_cmd(self.write_fun_type, address, joint_angle)
            return result
        except Exception as e:
            print(f"Failed to set motor angle: {e}")
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
            print(f"Failed to set motor angle: Input parameter type error!!")
            return False
        for item in joint_angle_list:
            if not isinstance(item, int):
                print(f"Failed to set motor angle: Input parameter type error!!")
                return False
            if item < 0 or item > 1000:
                print(f"Failed to set motor angle: {item} - Input parameter out of valid range!")
                return False
        return self.int_canfd_cmd(self.write_fun_type, self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS, joint_angle_list)

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
            print(f"Failed to calibrate single motor zero position: {motor_number} - Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > 17:
            print(f"Failed to calibrate single motor zero position: {motor_number} - Input parameter out of valid range!")
            return False
        address = self.SINGLE_MOTOR_CALIBRATION_ADDRESS + motor_number - 1
        try:
            result = self.int_canfd_cmd(self.write_fun_type, address, 1)
            return result
        except Exception as e:
            print(f"Failed to calibrate single motor zero position: {e}")
            return False

    def set_all_motor_calibration(self):
        """
        设置全关节电机(全手)零位校准。

        返回:
        - 成功:True
        - 失败:False。
        """
        return self.int_canfd_cmd(self.write_fun_type, self.ALL_MOTOR_CALIBRATION_ADDRESS, 1)

    def set_all_step_motor_calibration(self):
        """
        设置全步进关节电机零位校准。

        返回:
        - 成功:True
        - 失败:False。
        """
        return self.int_canfd_cmd(self.write_fun_type, self.ALL_STEP_MOTOR_CALIBRATION_ADDRESS, 1)

    def get_initialize_state(self):
        """
        获取设备初始化状态。

        返回:
        - 获取成功:1-初始化状态
        - 获取失败:False。
        """
        try:
            self.init_config_data.clear()
            start_time = time.time()
            send_ret = self.int_canfd_cmd(self.read_fun_type, self.INITIALIZE_DATA_ADDRESS, 1)
            if send_ret:
                initial_state = False
                while time.time() - start_time < self.receive_detect_time:
                    if len(self.init_config_data):
                        initial_state = self.init_config_data.popleft()
                        break
                return initial_state
            else:
                print(f"Failed to get initialization state!")
                return False
        except Exception as e:
            print(f"Failed to get initialization state: {e}")
            return False

    def get_bootloader_version(self):
        """
        获取设备bootloader版本。

        返回:
        - 获取成功:XX，XX为十进制格式的bootloader版本
        - 获取失败:False。
        """
        try:
            self.boot_loader.clear()
            start_time = time.time()
            send_ret = self.int_canfd_cmd(self.read_fun_type, self.BOOTLOADER_VERSION_ADDRESS, 1)
            if send_ret:
                detect_state = False
                while time.time() - start_time < self.receive_detect_time:
                    if len(self.boot_loader):
                        detect_state = self.boot_loader.popleft()
                        break
                return detect_state
            else:
                print(f"Failed to get bootloader version!")
                return False
        except Exception as e:
            print(f"Failed to get bootloader version: {e}")
            return False

    def get_hardware_version(self):
        """
        获取设备硬件版本。

        返回:
        - 获取成功:XX，XX为十进制格式的硬件版本
        - 获取失败:False
        """
        try:
            self.hardware_version.clear()
            start_time = time.time()
            send_ret = self.int_canfd_cmd(self.read_fun_type, self.HARDWARE_VERSION_ADDRESS, 1)
            if send_ret:
                detect_state = False
                while time.time() - start_time < self.receive_detect_time:
                    if len(self.hardware_version):
                        detect_state = self.hardware_version.popleft()
                        break
                return detect_state
            else:
                print(f"Failed to get hardware version!")
                return False
        except Exception as e:
            print(f"Failed to get hardware version: {e}")
            return False

    def get_software_version(self):
        """
        获取设备软件版本。

        返回:
        - 获取成功:XX，XX为十进制格式的软件版本
        - 获取失败:False。
        """
        try:
            self.software_version.clear()
            start_time = time.time()
            send_ret = self.int_canfd_cmd(self.read_fun_type, self.SOFTWARE_VERSION_ADDRESS, 1)
            if send_ret:
                detect_state = False
                while time.time() - start_time < self.receive_detect_time:
                    if len(self.software_version):
                        detect_state = self.software_version.popleft()
                        break
                return detect_state
            else:
                print(f"Failed to get software version!")
                return False
        except Exception as e:
            print(f"Failed to get software version: {e}")
            return False

    def get_device_error(self):
        """
        获取设备错误码。

        返回:
        - 获取成功:数据列表[XX]*9，XX为十进制格式的设备错误码
        - 获取失败:False。
        """
        try:
            self.error_code.clear()
            start_time = time.time()
            send_ret = self.int_canfd_cmd(self.read_fun_type, self.DEVICE_ERROR_ADDRESS, 9)
            if send_ret:
                detect_state = False
                while time.time() - start_time < self.receive_detect_time:
                    if len(self.error_code):
                        detect_state = self.error_code.popleft()
                        break
                return detect_state
            else:
                print("Failed to get device error code!")
                return False
        except Exception as e:
            print(f"Failed to get device error code: {e}")
            return False

    def get_device_voltage(self):
        """
        获取设备电压。

        电压系数：0.001
        电压单位：V

        返回:
        - 获取成功:XX，XX*0.001为十进制格式的设备电压
        - 获取失败:False。
        """
        try:
            self.system_voltage.clear()
            start_time = time.time()
            send_ret = self.int_canfd_cmd(self.read_fun_type, self.DEVICE_VOLTAGE_ADDRESS, 1)
            if send_ret:
                detect_state = False
                while time.time() - start_time < self.receive_detect_time:
                    if len(self.system_voltage):
                        detect_state = self.system_voltage.popleft()
                        break
                return detect_state
            else:
                print("Failed to get device voltage!")
                return False
        except Exception as e:
            print(f"Failed to get device voltage: {e}")
            return False

    def get_motor_locked_state(self):
        """
        获取所有关节电机堵转状态。

        电机堵转状态：1-堵转，0-正常

        返回:
        - 获取成功:数据列表[XX]*17，XX为十进制格式的电机堵转状态
        - 获取失败:False。
        """
        try:
            self.locked_state.clear()
            start_time = time.time()
            detect_state = False
            while time.time() - start_time < self.receive_detect_time:
                if len(self.locked_state):
                    detect_state = self.locked_state.popleft()
                    break
            return detect_state
        except Exception as e:
            print(f"Failed to get motor locked state: {e}")
            return False

    def get_motor_real_angle(self):
        """
        获取所有关节电机实际角度挡位。

        返回:
        - 获取成功:数据列表[XX]*17，XX为十进制格式的电机实际角度挡位
        - 获取失败:False。
        """
        try:
            start_time = time.time()
            detect_state = False
            while time.time() - start_time < self.receive_detect_time:
                if len(self.motor_angles):
                    detect_state = self.motor_angles.popleft()
                    break
            return detect_state
        except Exception as e:
            print(f"Failed to get motor real angle: {e}")
            return False

    def get_fingertip_skin_data(self):
        """

        :return:
        """
        try:
            self.fingertip_skin_data.clear()
            start_time = time.time()
            send_ret = self.int_canfd_cmd(self.read_fun_type, self.FINGERTIP_SKIN_ADDRESS, 5)
            if send_ret:
                detect_state = False
                while time.time() - start_time < self.receive_detect_time:
                    if len(self.fingertip_skin_data):
                        detect_state = self.fingertip_skin_data.popleft()
                        break
                return detect_state
            else:
                print("Failed to get fingertip skin data!")
                return False
        except Exception as e:
            print(f"Failed to get fingertip skin data: {e}")
            return False


# hand = ZWHAND(0x01, arb_baud_rate='1000000', data_baud_rate='5000000', zlgcan_type=1)
# time.sleep(1)
# ret = hand.get_initialize_state()
# print("初始化信号：", ret)
# if ret:
#     ret = hand.get_fingertip_skin_data()
#     print("皮肤数据：", ret)
# time.sleep(1)
# hand.close_device()






