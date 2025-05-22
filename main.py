import time
import random
from machine import Pin, PWM, I2C, SoftI2C
from XRPLib.rangefinder import Rangefinder
import uasyncio as asyncio

# 新增VCNL4040类定义，使用SoftI2C实现
class VCNL4040:
    # 寄存器地址
    PS_CONF1 = 0x03      # 接近传感器配置寄存器1
    PS_CONF2 = 0x04      # 接近传感器配置寄存器2
    PS_DATA = 0x08       # 接近传感器数据寄存器
    DEVICE_ID = 0x0C     # 设备ID寄存器
    
    def __init__(self, scl_pin=19, sda_pin=18, i2c=None):
        if i2c is not None:
            self.i2c = i2c
        else:
            # use SoftI2C而不是硬件I2C
            self.i2c = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=5000)
        
        self.addr = 0x60
        
        # 检查设备是否存在
        if self.addr not in self.i2c.scan():
            raise RuntimeError("找不到VCNL4040设备")
        
        # 初始化接近传感器
        self.enable_proximity()
    
    def write_register(self, reg_addr, data):
        """写入16位寄存器"""
        buffer = bytearray(2)
        buffer[0] = data & 0xFF         # 低字节
        buffer[1] = (data >> 8) & 0xFF  # 高字节
        self.i2c.writeto_mem(self.addr, reg_addr, buffer)
        
    def read_register(self, reg_addr):
        """读取16位寄存器"""
        data = self.i2c.readfrom_mem(self.addr, reg_addr, 2)
        return (data[1] << 8) | data[0]  # 小端序转换
    
    def enable_proximity(self):
        """启用接近传感器功能"""
        # PS_CONF1: 设置LED电流(100mA), 使能接近测量
        ps_conf1 = 0x000E  # LED电流=100mA, 接近测量使能
        self.write_register(self.PS_CONF1, ps_conf1)
        
        # PS_CONF2: 设置积分时间和接近持续模式
        ps_conf2 = 0x0008  # 积分时间=1T, 接近持续模式
        self.write_register(self.PS_CONF2, ps_conf2)
        
        time.sleep(0.1)  # 等待配置生效
    
    @property
    def proximity(self):
        """读取接近传感器数据"""
        return self.read_register(self.PS_DATA)

class SimpleDrive:
    def __init__(self, left_phase_pin=6, left_pwm_pin=7, right_phase_pin=14, right_pwm_pin=15):
        self.left_phase = Pin(left_phase_pin, Pin.OUT)
        self.left_pwm = PWM(Pin(left_pwm_pin))
        self.left_pwm.freq(100)

        self.right_phase = Pin(right_phase_pin, Pin.OUT)
        self.right_pwm = PWM(Pin(right_pwm_pin))
        self.right_pwm.freq(100)

    def set_speed(self, left_speed, right_speed):
        # direction
        self.left_phase.value(0 if left_speed >= 0 else 1)
        self.right_phase.value(1 if right_speed >= 0 else 0)
        # speed
        self.left_pwm.duty_u16(min(int(abs(left_speed) / 100 * 65535), 65535))
        self.right_pwm.duty_u16(min(int(abs(right_speed) / 100 * 65535), 65535))

    def stop(self):
        self.left_pwm.duty_u16(0)
        self.right_pwm.duty_u16(0)

class RumbaRobot:
    SAFE_DISTANCE_CM = 12
    BACK_PROXIMITY_THRESHOLD = 1000
    FORWARD_SPEED = 40
    TURN_SPEED = 40

    STATE_FORWARD = 0
    STATE_TURN = 1
    STATE_BACK_UP = 2

    def __init__(self):
        self.drive = SimpleDrive()
        self.rangefinder = Rangefinder(21, 20)
        
        # 修改：使用SoftI2C替代I2C
        self.i2c = SoftI2C(sda=Pin(18), scl=Pin(19), freq=10000)
        time.sleep(0.1)
        # 不立即初始化 VCNL4040，等第一次用的时候再实例化
        self._proximity_sensor = None
        
        self.current_state = self.STATE_FORWARD
        self.turn_direction = 1
        self.state_start_time = time.time()
        self.turn_duration = 0
        self.random_counter = 0
        self.random_behavior_threshold = 50
        self.last_progress_time = time.time() #record last time that successfully move forward
        self.previous_distance = 100  # 设一个初始值，单位是 cm
        self.distance_stuck_time = None  # 开始卡住的时间

        
    def get_front_distance(self):
        try:
            return self.rangefinder.distance()
        except Exception as e:
            print("Rangefinder error:", e)
            return 100

    def get_back_proximity(self):
        try:
            if self._proximity_sensor is None:
                # initialize VCNL4040，直接传入已创建的SoftI2C对象
                self._proximity_sensor = VCNL4040(i2c=self.i2c)
                time.sleep(0.1)  # 给 VCNL4040 初始化时间
            return self._proximity_sensor.proximity
        except Exception as e:
            print("Proximity error:", e)
            return 0

    def choose_random_turn(self):
        self.turn_direction = random.choice([-1, 1])
        self.turn_duration = random.uniform(0.5, 2.0)
        self.state_start_time = time.time()

    def update_movement(self):
        front_distance = self.get_front_distance()
        back_proximity = self.get_back_proximity()
        now = time.time()
        
        # stuck detection (front_distance does not change for long)
        if abs(front_distance - self.previous_distance) < 2:  # if the two measurement differences < 2cm
            if self.distance_stuck_time is None:
                self.distance_stuck_time = now  # first time stuck
            elif now - self.distance_stuck_time > 4:  # if 4 seconds passes but front_distance did not change
                print("⚠️ Robot might be stuck on low obstacle. Forcing turn.")
                self.current_state = self.STATE_TURN
                self.choose_random_turn()
                self.distance_stuck_time = None  # reset stuck time
        else:
            self.distance_stuck_time = None  # move forward normally, reset stuck time
        
        #print("Front distance:", front_distance)
        #print("Current state:", self.current_state)
        #print("Turn direction:", self.turn_direction)

        self.random_counter += 1
        if self.random_counter > self.random_behavior_threshold:
            self.random_counter = 0
            if random.random() < 0.3:
                self.current_state = self.STATE_TURN
                self.choose_random_turn()

        if self.current_state == self.STATE_FORWARD:
            if front_distance < self.SAFE_DISTANCE_CM: # if encounter front obstacles, go back
                self.current_state = self.STATE_BACK_UP
                self.state_start_time = now 
                
            elif back_proximity > self.BACK_PROXIMITY_THRESHOLD:
                self.drive.set_speed(self.FORWARD_SPEED * 1.5, self.FORWARD_SPEED * 1.5)
            else:
                self.last_progress_time = now
                self.drive.set_speed(self.FORWARD_SPEED, self.FORWARD_SPEED)

        elif self.current_state == self.STATE_TURN:
            if now - self.state_start_time > self.turn_duration:
                self.current_state = self.STATE_FORWARD
            else:
                l = -self.TURN_SPEED if self.turn_direction > 0 else self.TURN_SPEED
                r = self.TURN_SPEED if self.turn_direction > 0 else -self.TURN_SPEED
                self.drive.set_speed(l, r)

        elif self.current_state == self.STATE_BACK_UP:
            if now - self.state_start_time > 1.0:
                self.current_state = self.STATE_TURN
                self.choose_random_turn()
            else:
                self.drive.set_speed(-self.FORWARD_SPEED, -self.FORWARD_SPEED)
        
        # if stuck for more than 10 seconds in either backward or turn state
        if now - self.last_progress_time > 10:
            print("⚠️ Stuck detected. Resetting to FORWARD.")
            self.current_state = self.STATE_FORWARD
            self.state_start_time = now
            self.last_progress_time = now
        
        self.previous_distance = front_distance

    def stop(self):
        self.drive.stop()

    async def run(self):
        try:
            while True:
                self.update_movement()
                await asyncio.sleep(0.01)
        except KeyboardInterrupt:
            self.stop()
            print("Stopped.")
            
from machine import UART, Pin
import time

# 设置 UART1，RX 引脚为 GP16（也就是 Servo 1）
UART_ID = 0
BAUD_RATE = 115200
RX_PIN = 17

uart = UART(UART_ID, baudrate=BAUD_RATE, rx=Pin(RX_PIN))
led = Pin("LED", Pin.OUT)  # 板载LED

def uart_rx_callback(uart_obj):
    if uart_obj.any():
        try:
            received_data = uart_obj.read()
            if received_data:
                message = received_data.decode('utf-8').strip()
                print("Received:", message)
                if message == "object_detected":
                    led.value(1)
                    time.sleep(1)
                    led.value(0)
        except Exception as e:
            print("UART error:", e)

# 设置 UART 中断回调函数
uart.irq(trigger=UART.IRQ_RXIDLE, handler=uart_rx_callback)
print(f"UART{UART_ID} initialized on RX=GP{RX_PIN}, listening for messages...")


if __name__ == "__main__":
    print("Starting Rumba-style robot WITHOUT IMU...")
    robot = RumbaRobot()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(robot.run())
    finally:
        robot.stop()