import serial
import serial.tools.list_ports
import math
from time import sleep


class Nooploop_TOFSense:
    def __init__(self, name="TOFSense"):
        """
        name:对应型号
            TOFSense/P/PS:"TOFSense"
            TOFSense-F/FP:"TOFSense-F"
            TOFSense-M/MS:"TOFSense-M"/"TOFSense-M4*4"
        """
        self.Nlink = {
            "_Frame_Header": "57",
            "_Frame_Mark": "00",
            "_Check_bit": 16,
            "_Read_frame": "5710FFFF00FFFF63",
        }  # 对应各型号所使用的协议
        self.data = {}  # 解包后的数据
        self.is_f = False
        self.ser_is_on = True  # 串口是否成功打开
        if "M" in name:  # M系列
            self.Nlink["_Frame_Mark"] = "01"
            self.Nlink["_Check_bit"] = 400
            if "4*4" in name:  # 4*4模式
                self.Nlink["_Check_bit"] = 112
        if "F" in name:  # F系列
            self.is_f = True
        if not self.open_serial(921600):
            print("初始化串口失败")
            self.ser_is_on = False

    def open_serial(self, BaudRate_t=921600):
        ports_list = list(serial.tools.list_ports.comports())  # 获取COM口列表
        if len(ports_list) > 0:
            print("可用串口:")
            for port in ports_list:
                print(list(port)[0], list(port)[1])
            try:
                port_name = input("输入要使用的COM口: ")
                self.ser = serial.Serial(
                    port=port_name,  # 端口号
                    baudrate=BaudRate_t,  # 波特率
                    bytesize=serial.EIGHTBITS,  # 数据位
                    parity=serial.PARITY_NONE,  # 奇偶校验
                    stopbits=serial.STOPBITS_ONE,  # 停止位
                    # timeout  = 0.5)                   #读超时设置
                )
            except Exception:
                print("打开串口失败")
                return False
            if self.ser.isOpen():
                print(self.ser.name + " 已打开...")
                return True
        else:
            print("没有可用串口")
            return False

    def print_data(self):
        if self.Nlink["_Check_bit"] == 16:
            print("id: ", self.data["id"])
            # print("system_time: ", self.data["system_time"])
            print("dis: %0.2f" % self.data["dis"])
            print("dis_status: ", self.data["dis_status"])
            print("signal_strength: ", self.data["signal_strength"])
        else:
            print("id: ", self.data["id"])
            # print("system_time: ", self.data["system_time"])
            print("zone_map: ", self.data["zone_map"])
            print("dis: ")
            row = int(math.sqrt(self.data["zone_map"]))
            for i in range(0, row):
                for j in range(0, row):
                    if j == row - 1:
                        print("%0.2f" % self.data["pixel" + str(i * 8 + j)]["dis"])
                    else:
                        print("%0.2f" % self.data["pixel" + str(i * 8 + j)]["dis"], end=" ")

    def __unpack_data(self,rawdata_str):
        sum = 0
        raw_data = {}
        num = self.Nlink["_Check_bit"]
        rawstr = (self.Nlink["_Frame_Header"] + self.Nlink["_Frame_Mark"] + rawdata_str)
        for i in range(0, num):
            raw_data[i] = rawstr[i * 2 : (i + 1) * 2]
            sum += int(raw_data[i], 16)
            if i == num - 2:
                if hex(sum)[-2:] == rawstr[-2:]:
                    pass
                else:
                    return False
        self.data["id"] = int(raw_data[3], 16)  # 传感器ID
        self.data["system_time"] = int(raw_data[7] + raw_data[6] + raw_data[5] + raw_data[4], 16)  # 传感器上电时间
        if num == 16:
            self.data["dis"] = (int(raw_data[10] + raw_data[9] + raw_data[8], 16)) / 1000  # 测距距离 单位m
            self.data["dis_status"] = int(raw_data[11], 16)  # 距离状态指示
            self.data["signal_strength"] = int(raw_data[13] + raw_data[12], 16)  # 信号强度
            if self.is_f:
                self.data["range_precision"] = int(raw_data[14], 16)  # 测距精度F系列特有参数
        else:
            self.data["zone_map"] = int(raw_data[8], 16)
            for i in range(self.data["zone_map"]):
                self.data["pixel" + str(i)] = {}
                self.data["pixel" + str(i)]["dis"] = (int(
                        raw_data[11 + i * 6]
                        + raw_data[10 + i * 6]
                        + raw_data[9 + i * 6],
                        16,
                    )/ 1000/ 1000)  # 测距距离 单位m
                self.data["pixel" + str(i)]["dis_status"] = int(raw_data[12 + i * 6], 16)  # 距离状态指示
                self.data["pixel" + str(i)]["signal_strength"] = int(
                    raw_data[14 + i * 6] + raw_data[13 + i * 6], 16)  # 信号强度
        self.print_data()
        return True

    def get_data(self):
        if not self.ser_is_on:
            return False
        if self.ser.read(1).hex() == self.Nlink["_Frame_Header"]:  # 帧头
            if self.ser.read(1).hex() == self.Nlink["_Frame_Mark"]:  # 关键字
                if self.__unpack_data(self.ser.read(self.Nlink["_Check_bit"] - 2).hex()):
                    return True
        return False

    def send_and_read(self,num=1):
        if not self.ser_is_on:
            return
        frame = self.Nlink["_Read_frame"]
        if num == 1:
            self.ser.write(bytes.fromhex(frame))
            self.get_data()
        else:
            num = 8
            FH = frame[:8]
            FM = frame[10:14]
            for i in range(0, num):
                sleep(0.01)
                ID = "0" + str(i)
                CHECK = str(hex(99 + i)[-2:])
                frame = str(FH + ID + FM + CHECK)
                self.ser.write(bytes.fromhex(frame))
                self.get_data()


x = Nooploop_TOFSense("TOFSense")
#x = Nooploop_TOFSense("TOFSense-M")
#x = Nooploop_TOFSense("TOFSense-M4*4")

x.get_data()
