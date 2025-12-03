from zlgcan import *
from DOF17_ZWHAND_API import ZWHAND


class CanfdApi(ZWHAND):
    # 重写父类方法：打开设备
    def open_canfd_device(self, device_index=0, reserved=0):
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

    # 重写父类方法：启动CAN
    def start_canfd(self, chn=0):
        self.chn_index = chn
        self.chn_handle = canfd_start(self.zlg_can, self.handle, self.chn_index, self.arb_baud_rate,
                                      self.data_baud_rate)
        if self.chn_handle != self.chn_handle:
            print("Start CANFD Channel failed!")
            return False
        print("channel handle:%d." % self.chn_handle)
        self.can_is_open = True
        self.receive_thread.start()
        return True

    # 重写父类方法：关闭设备
    def close_device(self):
        self.can_is_open = False
        ret = self.zlg_can.CloseDevice(self.handle)
        if ret == 1:
            print("Close Device success! ")
        return ret

    # 重写父类方法：清除缓存区
    def clear_buffer(self):
        ret = self.zlg_can.ClearBuffer(self.chn_handle)
        if ret == 1:
            print("Clear Buffer success! ")
        return ret

    # 重写父类方法：接收消息
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
                    self.message_analysis(rcv_canfd_msgs[message_index].frame.can_id,
                                          rcv_canfd_msgs[message_index].frame.data)
            else:
                pass
        print("receive thread exit！！！")

    # 重写父类方法：发送CANFD命令帧
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


hand = CanfdApi(0x01)
time.sleep(1)
ret = hand.get_initialize_state()
print(ret)
time.sleep(1)
hand.close_device()


