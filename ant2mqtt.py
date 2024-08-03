#!/usr/bin/python3

import sys
import time
import serial
import struct
import paho.mqtt.client as mqtt
import json

#Define RS485 serial port
ser = serial.Serial(
    port='/dev/rfcomm0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout = 0)

PacketKeys = ("by1",'by2','by3','by4','batteryPackVoltage',
              "cell01","cell02","cell03","cell04","cell05","cell06","cell07","cell08",
              "cell09","cell10","cell11","cell12","cell13","cell14","cell15","cell16",
              "cell17","cell18","cell19","cell20","cell21","cell22","cell23","cell24",
              "cell25","cell26","cell26","cell28","cell29","cell30","cell31","cell32",
              "current","batteryPackCapacityPercent","batteryPackCap","remainingBatteryPackCap","cycleCapacity",
              "uptime","temperatureMOSFET1","temperatureMOSFET2","temperatureExt1","temperatureExt2","temperatureExt3",
              "temperatureExt4","chargeMOSFETstatus","dischargeMOSFETstatus","balanceStatus",
              "tireLenght","pulsesNumberWeek","relaySwitch","currentPowerWatt","cellNumberHighestVoltage",
              "cellHighestVoltage","cellNumberLowestVoltage","cellLowestVoltage","averageVoltage",
              "batteryQuantity","dischargeMOSFETVd-g","driveVoltageDischargeMOSFET","driveVoltageChargeMOSFET","comparator",
              "statusBits","syslog"
              )

def stringify(something):
    if type(something) == list:
        return [stringify(x) for x in something]
    elif type(something) == tuple:
        return tuple(stringify(list(something)))
    else:
        return str(something)


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))


mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.username_pw_set("ant-bms", "xxx")

mqttc.connect("openhab.homelab.ru")
mqttc.loop_start()


while True :
 askStat=b'\xDB\xDB\x00\x00\x00\x00'
 try:
   ser.write (askStat)
 except: 
   ser.close()
 time.sleep(1)
 if(ser.isOpen() == False):
    ser.open()
 Antw33 = ser.read(140)

 if(int(len(Antw33)) == 140):
#  print(list(Antw33))
  Packet = struct.unpack(">4B33HiB4I6h3B2HbiBHB2HB4HI2H",Antw33)
  crc = 0
  for x in range(4, 137): crc = crc + Antw33[x]

  if( Packet[68] == crc ):
      # print("crc ok")
      us = dict(zip(PacketKeys,Packet))
      us.pop("by1",None);us.pop("by2",None);us.pop("by3",None);us.pop("by4",None)
      
      # Our application produce some messages
      mqttc.publish("ant-bms", json.dumps(us), qos=1)
 
  time.sleep(10)
  #break;
 
mqttc.disconnect()
mqttc.loop_stop()
ser.close()
