import asyncio
import math
import numpy as np
from datetime import datetime
from typing import Callable, Any

# from aioconsole import ainput
from bleak import BleakClient, discover


connection_flag = False

selected_device = []

node = 128  # max length
filterLength = 4

# save the temporary data
qw = np.zeros((node, filterLength))
qx = np.zeros((node, filterLength))
qy = np.zeros((node, filterLength))
qz = np.zeros((node, filterLength))

# save the filtered data
filter_qw = np.zeros(node)
filter_qx = np.zeros(node)
filter_qy = np.zeros(node)
filter_qz = np.zeros(node)

i = 0
threshold = 0.4
error_num = np.zeros((node, 4))
filter_flag = True


def filter(quaternions):
    global i
    sumqw, sumqx, sumqy, sumqz = [], [], [], []

    for num, w, x, y, z in quaternions:
        num = int(num)
        if i < filterLength:
            qw[num][i], qx[num][i], qy[num][i], qz[num][i] = w, x, y, z
        else:
            # remove the abnormal value
            if w > 1.1 or w < -1.1:  # made by noise
                w = qw[num][filterLength-1]
            elif math.fabs(w - qw[num][filterLength-1]) > threshold and error_num[num][0] <= 4:  # made by noise or fast moving
                w = qw[num][filterLength-1]
                error_num[num][0] += 1  # detect error made by fast moving
            else:
                error_num[num][0] = 0

            if x > 1.1 or x < -1.1:
                x = qx[num][filterLength-1]
            elif math.fabs(x - qx[num][filterLength-1]) > threshold and error_num[num][1] <= 4:
                x = qx[num][filterLength-1]
                error_num[num][1] += 1
            else:
                error_num[num][1] = 0

            if y > 1.1 or y < -1.1:
                y = qy[num][filterLength-1]
            elif math.fabs(y - qy[num][filterLength-1]) > threshold and error_num[num][2] <= 4:
                y = qy[num][filterLength-1]
                error_num[num][2] += 1
            else:
                error_num[num][2] = 0

            if z > 1.1 or z < -1.1:
                z = qz[num][filterLength-1]
            elif math.fabs(z - qz[num][filterLength-1]) > threshold and error_num[num][3] <= 4:
                z = qz[num][filterLength-1]
                error_num[num][3] += 1
            else:
                error_num[num][3] = 0

            # update the buffer array / move left 1 unit
            qw[num][:-1], qx[num][:-1], qy[num][:-1], qz[num][:-1] = qw[num][1:], qx[num][1:], qy[num][1:], qz[num][1:]
            qw[num][filterLength-1], qx[num][filterLength-1], qy[num][filterLength-1], qz[num][filterLength-1] = w, x, y, z

            # get the index of the max/min value in arrays
            maxqw, maxqx, maxqy, maxqz = np.argmax(qw[num]), np.argmax(qx[num]), np.argmax(qy[num]), np.argmax(qz[num])
            minqw, minqx, minqy, minqz = np.argmin(qw[num]), np.argmin(qx[num]), np.argmin(qy[num]), np.argmin(qz[num])

            # sum except the max/min value
            for k in range(filterLength):
                if k != maxqw or k != minqw:
                    sumqw.append(qw[num][k])
                if k != maxqx or k != minqx:
                    sumqx.append(qx[num][k])
                if k != maxqy or k != minqy:
                    sumqy.append(qy[num][k])
                if k != maxqz or k != minqz:
                    sumqz.append(qz[num][k])

            # mean
            w, x, y, z = sum(sumqw)/len(sumqw), sum(sumqx)/len(sumqx), sum(sumqy)/len(sumqy), sum(sumqz)/len(sumqz)

            # update the buffer array / remove the max/min value
            qw[num][maxqw], qw[num][minqw] = w, w
            qx[num][maxqx], qx[num][minqx] = x, x
            qy[num][maxqy], qy[num][minqy] = y, y
            qz[num][maxqz], qz[num][minqz] = z, z

            filter_qw[num], filter_qx[num], filter_qy[num], filter_qz[num] = w, x, y, z
            sumqw, sumqx, sumqy, sumqz = [], [], [], []

    if i < filterLength:
        i += 1

start_time = datetime.now()
end_time = datetime.now()

class DataToFile:
    column_names = ["time", "delay", "data_value"]

    def __init__(self):
        pass

    def write_to_txt(self, data_values, addr):
        # print(data_values)
        # print(len(data_values))
        # global start_time, end_time
        print("---------------------------")

        with open('/tmp/sensors-{}.txt'.format(addr), 'a') as f:
            f.write(data_values + '\n')
        try:
            quaternions = data_values.split(';')[:-1]
            ts = int(quaternions[0])
            quaternions = quaternions[1:]
            for i in range(len(quaternions)):
                quaternion = quaternions[i].split(' ')
                quaternions[i] = quaternion[:5]
            
            print('timestamp: {:+}ms'.format(ts))
            
            quaternions = np.array(quaternions)
            quaternions = quaternions.astype(np.float16)
            
#            quaternions = data_values.split(' ')[:-1]
#            quaternions = np.array(quaternions).reshape((-1, 5))
#            quaternions = quaternions.astype(np.float16)

            filter(quaternions)

            for num, w, x, y, z in quaternions:
                num = int(num)

                print("{} {:+4.2f} {:+4.2f} {:+4.2f} {:+4.2f}".format(num, filter_qw[num], filter_qx[num], filter_qy[num], filter_qz[num]))
#                with open("/Users/zzh/Desktop/SenseNet/项目文档/python/蓝牙接收数据/cube{}.txt".format(num), 'w') as f:
                with open("/tmp/sensor-{}-{}.txt".format(num, addr), 'w') as f:
                    f.write("{:+4.2f} {:+4.2f} {:+4.2f} {:+4.2f}".format(filter_qw[num], filter_qx[num], filter_qy[num], filter_qz[num]))
        except Exception as e:
            print(e)
            return

        #
        # end_time = datetime.now()
        # print("-------------- FPS: {:.1f}".format(1e6/(end_time-start_time).microseconds))
        # start_time = datetime.now()


class Connection:
    client: BleakClient = None
    
    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        read_characteristic: str,
        write_characteristic: str,
        data_dump_handler: Callable[[str, Any], None],
        data_dump_size: int = 10,
    ):
        self.loop = loop
        self.read_characteristic = read_characteristic
        self.write_characteristic = write_characteristic
        self.data_dump_handler = data_dump_handler

        self.last_packet_time = datetime.now()
        self.dump_size = data_dump_size
        self.connected = False
        self.connected_device = None

        self.rx_data = ''
        self.rx_timestamps = []
        self.rx_delays = []

#    def on_disconnect(self, client: BleakClient, future: asyncio.Future):
#        self.connected = False
#        # Put code here to handle what happens on disconnet.
#        print(f"Disconnected from {self.connected_device.name}!")

    async def cleanup(self):
        if self.client:
            await self.client.stop_notify(read_characteristic)
            await self.client.disconnect()

    async def manager(self):
        print("Starting connection manager.")
        while True:
            if self.client:
                await self.connect()
            else:
                await self.select_device()
                await asyncio.sleep(5.0)

    async def connect(self):
        if self.connected:
            return
        try:
            print('try connecting')
            await self.client.connect()
            self.connected = await self.client.is_connected()
            if self.connected:
                print(F"Connected to {self.connected_device.name}")
#                self.client.set_disconnected_callback(self.on_disconnect)
                await self.client.start_notify(self.read_characteristic, self.notification_handler,)
                while True:
                    if not self.connected:
                        break
                    await asyncio.sleep(1.0)
            else:
                print(f"Failed to connect to {self.connected_device.name}")
        except Exception as e:
            print(e)

    async def select_device(self):
        print("Bluetooh LE hardware warming up...")
        await asyncio.sleep(2.0)  # Wait for BLE to initialize.
        devices = await discover()

        print("Please select device: ")
        response = -1
        while response == -1:
            for i, device in enumerate(devices):
                print(f"{i}: {device.name}")
                if device.name == 'IMUsMonitor':
                    response = i
            if response == -1:
                devices = await discover()

        # while True:
        print("Select device: ", response)
            # try:
            #     response = int(response.strip())
            # except:
            #     print("Please make valid selection.")
            #
            # if response > -1 and response < len(devices):
            #     break
            # else:
            #     print("Please make valid selection.")

        print(f"Connecting to {devices[response].name}")
        self.connected_device = devices[response]
        self.client = BleakClient(devices[response].address, loop=self.loop)

    def record_time_info(self):
        present_time = datetime.now()
        self.rx_timestamps.append(present_time)
        self.rx_delays.append((present_time - self.last_packet_time).microseconds)
        self.last_packet_time = present_time

    def clear_lists(self):
        self.rx_data = ''

    def notification_handler(self, sender: str, data: Any):
        try:
            self.rx_data = str(data, 'utf-8')
        except:
            print(data)
            return
        # print(self.rx_data)
        # print(len(self.rx_data))
        self.data_dump_handler(self.rx_data, sender)
        self.clear_lists()


#############
# Loops
#############
# async def user_console_manager(connection: Connection):
#     while True:
#         if connection.client and connection.connected:
#             input_str = await ainput("Enter string: ")
#             bytes_to_send = bytearray(map(ord, input_str))
#             await connection.client.write_gatt_char(write_characteristic, bytes_to_send)
#             print(f"Sent: {input_str}")
#         else:
#             await asyncio.sleep(2.0)


async def main():
    while True:
#        connection_flag = self.client.is_connected()
        # YOUR APP CODE WOULD GO HERE.
        # print('nihao')
        await asyncio.sleep(1)


#############
# App Main
#############
read_characteristic = "00001143-0000-1000-8000-00805f9b34fb"
write_characteristic = "00001142-0000-1000-8000-00805f9b34fb"

if __name__ == "__main__":
    
    while True:
        # Create the event loop.
        loop = asyncio.get_event_loop()

        data_to_file = DataToFile()
        connection = Connection(loop, read_characteristic, write_characteristic, data_to_file.write_to_txt)
        try:
            asyncio.ensure_future(connection.manager())
    #        asyncio.ensure_future(user_console_manager(connection))
#            asyncio.ensure_future(main())
            loop.run_forever()
        except KeyboardInterrupt:
            print()
            print("User stopped program.")
            break
        finally:
            print("Disconnecting...")
            loop.run_until_complete(connection.cleanup())
