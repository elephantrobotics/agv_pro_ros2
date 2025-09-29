#!/usr/bin/env python3                                                                                                           
#coding=UTF-8
import serial
import time

class SerialCANParser:
    def __init__(self, serial_port='/dev/ttyCH341USB0', baudrate=9600, timeout=1):
        self.serial_port = serial_port  # 串口名称
        self.baudrate = baudrate  # 波特率
        self.timeout = timeout  # 超时设置
        self.ser = None  # 串口对象
        self.buffer = bytearray()  # 存储当前读取的字节
        self.max_retries = 3  # 最大重试次数
        # 存储实时数据
        self.x_speed = 0.0
        self.z_speed = 0.0
        self.infrared_bits = []
        
    def open_serial(self):
        """打开串口"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            print(f"串口 {self.serial_port} 已打开，波特率：{self.baudrate}")
        except Exception as e:
            print(f"打开串口失败: {e}")
            
    def close_serial(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭。")
        else:
            print("串口未打开或已关闭。")
            
    def can_id_check(self, date):
        high_byte, low_byte = date[0:2]
        # 高字节左移 3 位
        can_id = (high_byte << 3)
        # 低字节右移 5 位
        can_id |= (low_byte >> 5)
        return can_id
        
    def parse_can_data(self, data):
        """解析8字节CAN数据帧"""
        if len(data) != 8:
            print("数据帧长度不正确")
            return None
            
        # 解析 X、Y 和 Z 速度
        x_speed_raw = ((data[0] << 8) | data[1])  # X速度的原始数据
        z_speed_raw = ((data[4] << 8) | data[5])  # Z速度的原始数据
        
        # 将原始数据转换为浮动数值，并考虑正负
        if x_speed_raw & 0x8000:  # 如果最高位为1，表示负数
            x_speed_raw = -((65536 - x_speed_raw) & 0xFFFF)  # 补码转换为负数
        if z_speed_raw & 0x8000:  # 如果最高位为1，表示负数
            z_speed_raw = -((65536 - z_speed_raw) & 0xFFFF)  # 补码转换为负数
            
        # 转换单位为 m/s 和 rad/s
        self.x_speed = x_speed_raw / 1000.0  # X速度单位为 m/s
        self.y_speed = 0  # Y速度为0
        self.z_speed = z_speed_raw / 1000.0  # Z速度单位为 rad/s
        self.which_mode = data[2]
        self.infrared = data[6]  # 红外数据
        self.raw_current = data[7]  # 电流数据
        
        if self.raw_current > 32767:  # 无符号数大于 32767 表示负值（因为最大值是 65535）
            # 转换为负数
            self.actual_current = -(65536 - self.raw_current) * 30.0
        else:
            # 正数直接转换
            self.actual_current = self.raw_current * 30.0
            
        # 处理红外数据
        self.infrared_bits = [(self.infrared >> (7 - i)) & 0x01 for i in range(8)]
        
        # 打印或处理数据
        print(f"X Speed: {self.x_speed:.3f}, Y Speed: {self.y_speed}, Z Speed: {self.z_speed:.3f}, "
              f"Actual Current: {self.actual_current:.3f} mA, Infrared: {self.infrared}")
              
        # 打印或处理红外位信息
        print(f"L_A: {self.infrared_bits[2]}, L_B: {self.infrared_bits[3]}, R_B: {self.infrared_bits[4]}, "
              f"R_A: {self.infrared_bits[5]}, infrared_flag : {self.infrared_bits[6]}, "
              f"Charging flag: {self.infrared_bits[7]}")
              
    def read_serial_data(self):
        """读取串口数据并解析"""
        while True:  # 修改为简单的无限循环，由上层控制退出
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)   # 读取一个字节
                if len(self.buffer) < 2:
                    self.buffer.extend(byte)  
                    if len(self.buffer) == 2:
                        # 如果帧头为 0x41 0x54，表示为AT帧头，则开始接收数据
                        if self.buffer[0] != 0x41 or self.buffer[1] != 0x54:
                            # 如果不是有效的帧头，则清空缓冲区并跳到下次循环
                            self.buffer.clear()
                            continue  # 继续等待下一个字节                 
                else:
                    self.buffer.extend(byte)
                    # print("缓冲区内容:", ' '.join(f'{b:02x}' for b in self.buffer)) # debug
                    # 如果缓冲区字节长度大于等于 17 字节（数据帧长度）
                    if len(self.buffer) >= 17:
                        # print("Received Frame (Hex):", ' '.join(f'{byte:02x}' for byte in self.buffer)) # debug
                        # 解析帧头、CAN帧ID、格式、类型和数据
                        # at_frame_header = self.buffer[0:2]  # AT帧头
                        can_frame_id = self.can_id_check(self.buffer[2:4])  # CAN标准帧ID
                        # can_frame_format = self.buffer[4]  # CAN帧格式（0,标准帧;1,扩展帧）
                        # can_frame_type = self.buffer[5]  # CAN帧类型（0,数据帧;1,远程帧）
                        data_length = self.buffer[6]  # 数据长度
                        data = self.buffer[7:15]  # 数据帧
                        # print(f"帧ID: 0x{can_frame_id:X}")    # debug
                        if can_frame_id == 0x182 and data_length == 0x08: # 根据can帧id进行判断
                            # 如果帧ID为0x182,校验通过,进行数据赋值
                            self.parse_can_data(data)
                            # 清空缓冲区，准备下一帧数据
                            self.buffer.clear()
                            return self.x_speed, self.z_speed, self.which_mode, self.infrared_bits   # 返回解析后的数据
                        else:
                            # 清空缓冲区，准备下一帧数据
                            self.buffer.clear()
                            
    def read_serial_response(self):
        """读取串口响应数据，直到接收到 '\r\n' 或超时"""
        response = bytearray()  # 使用 bytearray 来存储原始字节流
        while True:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                response += byte
            # 检查是否已接收到完整响应
            if b'\r\n' in response:
                break
            # 超时机制，防止死循环
            if len(response) > 100:
                break
        return bytes(response)  # 返回原始字节流（bytes）
        
    def send_at_commands(self, commands):
        """发送 AT 命令并等待响应"""
        for command in commands:
            retries = 0
            while retries < self.max_retries:
                self.ser.write(command.encode() + b'\r\n')
                print(f"发送命令: {command}")
                # 等待响应并读取数据
                response = self.read_serial_response()
                # 检查响应是否包含 "OK"
                if b"OK" in response:
                    print(f"收到响应: {response}")
                    break  # 如果收到 OK，退出重试循环
                else:
                    retries += 1
                    print(f"未收到预期的响应，收到: {response}")
            if retries == self.max_retries:
                print(f"重试 {self.max_retries} 次后仍未收到有效响应，请检查设备。")
                break
                
    def start(self):
        """开始读取和处理数据"""
        self.open_serial()
        try:
            # 发送AT 命令从透传模式进入AT指令模式
            self.send_at_commands(["AT+CG", "AT+AT"])
            # 开始读取数据
            self.read_serial_data()
        except KeyboardInterrupt:
            print("手动中止程序。")
        finally:
            self.close_serial()

if __name__ == '__main__':
    parser = SerialCANParser('/dev/ttyCH341USB0', 9600, 1)
    parser.start()
