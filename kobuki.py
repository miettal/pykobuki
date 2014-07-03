#!/usr/bin/env python
# coding:utf-8
#
# kobuki.py
#
# Author:   Hiromasa Ihara (miettal)
# URL:      http://miettal.com
# License:  MIT License
# Created:  2014-06-05
#

from serial import Serial
import threading

feedback_tbl = {
  1  : "Basic Sensor Data",
  2  : "Reserved",
  3  : "Docking IR",
  4  : "Inertial Sensor",
  5  : "Cliff",
  6  : "Current",
  7  : "Reserved",
  8  : "Reserved",
  9  : "Reserved",
  10 : "Hardware Version",
  11 : "Firmware Version",
  12 : "Reserved",
  13 : "Raw data of 3-axis gyro",
  14 : "Reserved",
  15 : "Reserved",
  16 : "General Purpose Input",
  17 : "Reserved",
  18 : "Reserved",
  19 : "Unique Device IDentifier(UDID)",
  20 : "Reserved",
  21 : "Controller Info",
}

class FeedbackListener :
  def onFeedback(self, feedback_id, data) :
    pass

  def onBasicSensorData(self, timestamp, bumper, wheel_drop, cliff,
      left_encoder, right_encoder,
      left_pwm, right_pwm,
      button, charger, battery, overcurrent_flags) :
    pass

  def onDockingIR(self, data) :
    '''not implemented'''
    pass

  def onInertialSensor(self, angle, angle_rate) :
    pass

  def onCliff(self, data) :
    '''not implemented'''
    pass

  def onCurrent(self, data) :
    '''not implemented'''
    pass

  def onHardwareVersion(self, data) :
    '''not implemented'''
    pass

  def onFirmwareVersion(self, data) :
    '''not implemented'''
    pass

  def onRawDataof3AxisGyro(self, frame_id, gyro_data) :
    pass

  def onGeneralPurposeInput(self, data) :
    '''not implemented'''
    pass

  def onUniqueDeviceIDentifier(self, data) :
    '''not implemented'''
    pass

  def onControllerInfo(self, data) :
    '''not implemented'''
    pass

class Kobuki :
  def __init__(self, dev_path) :
    self.serial = Serial(dev_path, 115200)

  def __getattr__(self, name) :
    def f(*args, **kwargs) :
      return self.send([getattr(BuildRequestData, name)(*args, **kwargs)])
    return f

  def send(self, commands) :
    sub_payloads = commands
    payload = []
    for sub_payload in sub_payloads :
      payload += sub_payload
    header = [0xAA, 0x55]
    body = [len(payload)] + payload
  
    checksum = 0
    for x in body:
      checksum ^= x
  
    packets = header+body+[checksum]
    self.serial.write(''.join(map(chr, packets)))
  
    return packets

  def feedback(self, feedback_listener) :
    def func() :
      while True :
        try :
          c = ord(self.serial.read())
          if c == 0xaa :
            header0 = c
            header1 = ord(self.serial.read())
            length = ord(self.serial.read())
            i = 0
            while i < length :
              i += 2
              feedback_id = ord(self.serial.read())
              sub_payload_length = ord(self.serial.read())
              i += sub_payload_length
              sub_payload = map(ord, list(self.serial.read(sub_payload_length)))

              if len(sub_payload) < sub_payload_length : continue

              feedback_listener.onFeedback(feedback_id, sub_payload)

              if feedback_id < 1 : continue
              if 21 < feedback_id : continue

              if feedback_tbl[feedback_id] == "Basic Sensor Data" :
                timestamp_lsb = sub_payload[0]
                timestamp_msb = sub_payload[1]
                timestamp = (timestamp_msb<<8)+timestamp_lsb
                bumper = sub_payload[2]
                wheel_drop = sub_payload[3]
                cliff = sub_payload[4]
                left_encoder_lsb = sub_payload[5]
                left_encoder_msb = sub_payload[6]
                left_encoder = (left_encoder_msb<<8)+left_encoder_lsb
                right_encoder_lsb = sub_payload[7]
                right_encoder_msb = sub_payload[8]
                right_encoder = (right_encoder_msb<<8)+right_encoder_lsb
                left_pwm = sub_payload[9]
                right_pwm = sub_payload[10]
                button = sub_payload[11]
                charger = sub_payload[12]
                battery = sub_payload[13]
                overcurrent_flags = sub_payload[14]

                feedback_listener.onBasicSensorData(timestamp, bumper, wheel_drop, cliff,
                  left_encoder, right_encoder,
                  left_pwm, right_pwm,
                  button, charger, battery, overcurrent_flags)
              elif feedback_tbl[feedback_id] == "Docking IR" :
                feedback_listener.onDockingIR(sub_payload)
              elif feedback_tbl[feedback_id] == "Inertial Sensor" :
                angle_lsb = sub_payload[0]
                angle_msb = sub_payload[1]
                angle_rate_lsb = sub_payload[2]
                angle_rate_msb = sub_payload[3]

                angle = (angle_msb<<8)+angle_lsb
                angle_rate = (angle_rate_msb<<8)+angle_rate_lsb
                if 0x8000 <= angle : angle -= 0x10000
                if 0x8000 <= angle_rate : angle_rate -= 0x10000

                angle /= 100.0
                angle_rate /= 100.0

                feedback_listener.onInertialSensor(angle, angle_rate)
              elif feedback_tbl[feedback_id] == "Cliff" :
                feedback_listener.onCliff(sub_payload)
              elif feedback_tbl[feedback_id] == "Current" :
                feedback_listener.onCurrent(sub_payload)
              elif feedback_tbl[feedback_id] == "Hardware Version" :
                feedback_listener.onHardwareVersion(sub_payload)
              elif feedback_tbl[feedback_id] == "Firmware Version" :
                feedback_listener.onFirmwareVersion(sub_payload)
              elif feedback_tbl[feedback_id] == "Raw data of 3-axis gyro" :
                frame_id = sub_payload[0]
                gyro_data = []
                for (x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb) in zip(*[iter(sub_payload[2:])]*6) :
                  x = (x_msb<<8)+x_lsb
                  y = (y_msb<<8)+y_lsb
                  z = (z_msb<<8)+z_lsb
                  if 0x8000 <= x : x -= 0x10000
                  if 0x8000 <= y : y -= 0x10000
                  if 0x8000 <= z : z -= 0x10000

                  x =  -0.00875 * y;
                  y =   0.00875 * x;
                  z =   0.00875 * z;

                  gyro_data.append((x,y,z))
                feedback_listener.onRawDataof3AxisGyro(frame_id, gyro_data)
              elif feedback_tbl[feedback_id] == "General Purpose Input" :
                feedback_listener.onGeneralPurposeInput(sub_payload)
              elif feedback_tbl[feedback_id] == "Unique Device IDentifier(UDID)" :
                feedback_listener.onUniqueDeviceIDentifier(sub_payload)
              elif feedback_tbl[feedback_id] == "Controller Info" :
                feedback_listener.onControllerInfo(sub_payload)

            checksum = ord(self.serial.read())
        except IndexError :
          pass

    t = threading.Thread(target=func)
    t.daemon = True
    t.start()

class BuildRequestData :
  @classmethod
  def base_control(self, speed, radius) :
    speed_lsb = 0xff & speed
    speed_msb = 0xff & (speed>>8)
    radius_lsb = 0xff & radius 
    radius_msb = 0xff & (radius>>8)
  
    return [0x01, 0x04, speed_lsb, speed_msb, radius_lsb, radius_msb]
  
  @classmethod
  def sound(self, note, duration) :
    note_lsb = 0xff & note
    note_msb = 0xff & (note>>8)
  
    return [0x03, 0x03, note_lsb, note_msb, duration]
  
  @classmethod
  def sound_sequence(self, n) :
    return [0x04, 0x01, n]
