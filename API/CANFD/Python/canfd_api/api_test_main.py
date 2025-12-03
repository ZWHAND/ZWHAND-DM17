import time

from DOF17_ZWHAND_API import *


# 初始化ZWHAND
hand = ZWHAND(1)
# 获取ZWHAND初始化状态
initialize_state = hand.get_initialize_state()
if initialize_state:
    # 全手零位校准
    set_ret = hand.set_all_motor_calibration()
    if set_ret:
        print('全手零位校准成功')
    else:
        print('全手零位校准失败')
    time.sleep(13)
    ang = [1000, 1000, 0, 0, 0, 0, 0, 0, 0, 1000, 1000, 1000, 1000, 1000, 1000, 0, 1000]
    # 设置ZWHAND所有关节电机绝对角度-剪刀
    set_ret = hand.set_all_motor_absolute(ang)
    if set_ret:
        print('设置ZWHAND所有关节电机绝对角度成功')
    else:
        print('设置ZWHAND所有关节电机绝对角度失败')
    time.sleep(1)
    # 获取ZWHAND的所有关节电机实际角度挡位
    get_data = hand.get_motor_real_angle()
    if get_data:
        print('关节电机实际角度挡位:', get_data)
    else:
        print('关节电机实际角度挡位获取失败')

hand.close_device()



