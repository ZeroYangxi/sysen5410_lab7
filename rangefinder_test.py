from machine import Pin
import time
import utime

# 定义引脚
trigger = Pin(21, Pin.OUT)
echo = Pin(20, Pin.IN)

def get_distance():
    # 确保触发引脚初始状态为低
    trigger.low()
    utime.sleep_us(5)
    
    # 发送10微秒的触发脉冲
    trigger.high()
    utime.sleep_us(10)
    trigger.low()
    
    # 等待回波信号上升沿
    pulse_start = 0
    pulse_end = 0
    
    # 等待回波信号变高 - 设置超时
    start_time = utime.ticks_us()
    timeout = False
    
    while echo.value() == 0:
        pulse_start = utime.ticks_us()
        if utime.ticks_diff(pulse_start, start_time) > 30000:  # 30ms超时
            timeout = True
            break
    
    if timeout:
        return -1
    
    # 等待回波信号变低 - 设置超时
    start_time = utime.ticks_us()
    timeout = False
    
    while echo.value() == 1:
        pulse_end = utime.ticks_us()
        if utime.ticks_diff(pulse_end, start_time) > 30000:  # 30ms超时
            timeout = True
            break
    
    if timeout:
        return -2
    
    # 计算脉冲持续时间
    pulse_duration = utime.ticks_diff(pulse_end, pulse_start)
    
    # 计算距离
    distance_cm = pulse_duration / 58.8
    
    return distance_cm

print("开始测量距离 - 自定义实现...")

while True:
    try:
        dist = get_distance()
        if dist > 0:
            print("距离: {:.1f} 厘米".format(dist))
        elif dist == -1:
            print("⚠️ 等待回波信号上升沿超时")
        elif dist == -2:
            print("⚠️ 等待回波信号下降沿超时")
    except Exception as e:
        print("错误:", e)
    
    time.sleep(0.5)