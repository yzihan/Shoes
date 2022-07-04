#!/usr/bin/env python
import os
from ssl import ALERT_DESCRIPTION_UNKNOWN_PSK_IDENTITY
import time
import asyncio
import traceback
from typing import Any
import warnings
from tqdm.auto import tqdm
import bleak
from bleak import BleakClient, BleakScanner


class DataRecorder:
  def __init__(self, fn = None):
    if fn is None:
      self.f = None
    else:
      self.f = open(fn, 'a')
    self.rt_stats = {}

    if fn is None:
      warnings.warn('Your data will NOT be recorded since log file is not specified')

    asyncio.create_task(self.__main_task__())

  async def __main_task__(self):
    ts_ctr = 0
    while True:

      report = ' '.join(map(lambda k: '{}={}sp/s'.format(k, self.rt_stats[k]),sorted(self.rt_stats.keys())))
      if len(report) > 0:
        self.log('[DATA] ' + report)
        ts_ctr += 1
        if ts_ctr == 5:
          self.log('[DATA] ts={}'.format(time.time_ns()))
          ts_ctr = 0

      self.rt_stats = {}

      await asyncio.sleep(1)

  def log(self, data):
    print(data)
    if self.f is not None:
      self.f.write('+ {}\n'.format(data))

  def record(self, address, data):
    if self.f is not None:
      self.f.write('{}: {}\n'.format(address, data))
    if address in self.rt_stats:
      self.rt_stats[address] += 1
    else:
      self.rt_stats[address] = 1

data_recorder = None

class SerialListener:
  def __init__(self, pattern, scan_only=False):
    self.pattern = pattern
    self.known = set()
    self.scan_only = True
  
  async def monitor_serial(self, fn):
    import serial
    ser = serial.Serial(fn, 2000000)
    data_recorder.log('[SER] Connected to serial port {}'.format(fn))

    while True:
      if ser.inWaiting():
        try:
          recv = ser.readline().decode("gbk", 'ignore')
        except Exception as e:
          data_recorder.log('[SER] Reading {} error: {}'.format(fn, e))
          break
        data_recorder.record(fn, recv)
    
    data_recorder.log('[SER] Closing serial port {}'.format(fn))
    self.known.remove(fn)
  
  async def run(self):
    import glob
    while True:
      lost_devices = set(self.known)
      for f in glob.glob(self.pattern):
        if f not in self.known:
          self.known.add(f)
          data_recorder.log('[SER] serial port {} is found'.format(f))
          if not self.scan_only:
            asyncio.create_task(self.monitor_serial(f))
        if f in lost_devices:
          lost_devices.remove(f)
      for f in lost_devices:
        data_recorder.log('[SER] serial port {} is lost'.format(f))
        self.known.remove(f)
      await asyncio.sleep(1)

class BleConnection:
  def __init__(self, listener, device, ble_characteristic, *kargs, **kwargs):
    self.listener = listener
    self.characteristic = ble_characteristic
    self.address = device.address
    self.name = device.name

    self.client = None

    self.record_name = 'ble:[{}]'.format(device.address, device.name)

    asyncio.create_task(self.__main_task__())

  def notification_handler(self, sender: str, data: Any):
    try:
      rx_data = str(data, 'utf-8')
    except:
      data_recorder.log('[BTH] Undecodable message from {}, sender={}'.format(self.device.name, sender))
      return
    data_recorder.record(self.record_name, rx_data)

  def on_disconnect(self, client: BleakClient):
    if client.address in self.listener.devices:
      name = self.listener.devices[client.address][0]
      data_recorder.log('[BTH] Connection to {} (name={}) is lost.'.format(client.address, name))
      del self.listener.devices[client.address]

  async def __main_task__(self):
    data_recorder.log('[BTH] Connecting to {} (name={})'.format(self.address, self.name))

    self.client = BleakClient(self.address)
    
    try:
      await self.client.connect(timeout=5)
    except Exception as e:
      data_recorder.log('[BTH] Error while connecting to {} (name={}): {}'.format(self.address, self.name, e))
      if self.address in self.listener.devices:
        data_recorder.log('[BTH] Connection to {} (name={}) is removed.'.format(self.address, self.name))
        del self.listener.devices[self.address]
        return

    if self.client.is_connected:
      self.client.set_disconnected_callback(self.on_disconnect)
      data_recorder.log('[BTH] Connected to {} (name={})'.format(self.address, self.name))

      await asyncio.sleep(1)

      try:
        await self.client.start_notify(self.characteristic, self.notification_handler)
      except Exception as e:
        data_recorder.log('[BTH] Error while reading from {} (name={}): {}'.format(self.address, self.name, e))
        try:
          await self.client.disconnect()
        except Exception as e:
          traceback.print_exc(e)
        if self.address in self.listener.devices:
          data_recorder.log('[BTH] Connection to {} (name={}) is closed.'.format(self.address, self.name))
          del self.listener.devices[self.address]
          return

    else:
      if self.address in self.listener.devices:
        data_recorder.log('[BTH] Connection to {} (name={}) is lost.'.format(self.address, self.name))
        del self.listener.devices[self.address]

class BleListener:
  def __init__(self, device_selector, *kargs, **kwargs):
    self.device_selector = device_selector

    self.kargs = kargs
    self.kwargs = kwargs

    self.devices = {}

  async def run(self):
    print("[BTH] Bluetooh LE hardware warming up...")
    await asyncio.sleep(2.0)  # Wait for BLE to initialize.
    data_recorder.log('[BTH] Start scanning...')

    scanner = BleakScanner()

    while True:
      lost_devices = set(self.devices.keys())
      async with scanner:
        await asyncio.sleep(5)
        devices = scanner.discovered_devices
      for device in devices:
        if device.address in self.devices:
          lost_devices.remove(device.address)
        else:
          data_recorder.log('[BTH] {} (name={}) is found, rssi={}'.format(device.address, device.name, device.rssi))

          if self.device_selector(device):
            self.devices[device.address] = [device.name, BleConnection(self, device, *self.kargs, **self.kwargs)]
          else:
            self.devices[device.address] = [device.name, None]
      for dev in lost_devices:
        if self.devices[dev][1] is None:
          data_recorder.log('[BTH] {} (name={}) is lost'.format(dev, self.devices[dev][0]))
          del self.devices[dev]
      # data_recorder.log('[BTH] Scan complete. devices={}'.format(','.join(k for k in self.devices)))

async def main():
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--force_addr', '-addr', help='Force connect to address')
  parser.add_argument('--log_file', '-o', help='Output file')
  parser.add_argument('--scan_only', '-scan', action='store_true', help='Scan only and do not connect')
  parser.add_argument('--no_serial', '-noser', action='store_true', help='Disable serial port')
  parser.add_argument('--no_bluetooth', '-noble', action='store_true', help='Disable bluetooth')
  parser.add_argument('--serial_pattern', help='Serial port pattern', default='/dev/ttyACM?')
  args = parser.parse_args()

  if args.scan_only:
    def sensor_selector(dev):
      return False
  elif args.force_addr:
    def sensor_selector(dev):
      return dev.address == args.force_addr
  else:
    def sensor_selector(dev):
      return dev.name == 'IMUsMonitor'

  global data_recorder
  data_recorder = DataRecorder(args.log_file)

  tasks = []

  if not args.no_bluetooth:
    listener = BleListener(sensor_selector, ble_characteristic = "00001143-0000-1000-8000-00805f9b34fb")
    tasks.append(asyncio.create_task(listener.run()))

  if not args.no_serial:
    listener = SerialListener(args.serial_pattern, scan_only=args.scan_only)
    tasks.append(asyncio.create_task(listener.run()))

  # join all
  for t in tasks:
    await t

if __name__ == '__main__':

  asyncio.run(main())