#!/usr/bin/python3
# -*- coding: utf-8 -*-

# import ptvsd
# ptvsd.enable_attach(address = ('0.0.0.0', 5678))
# ptvsd.wait_for_attach()

import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from config import config as cfg

# 输入输出点定义，使用BCM GPIO编号
pinOut = [4, 17, 27, 22]
pinIn = [5, 6, 13, 26]
# 主题
topicSub = 'node/' + cfg['nodeID'] + '/cmd'
topicPub = 'node/' + cfg['nodeID'] + '/status'

global mqttClient
counter = 0


def pin_setup():
    # 采用BCM编号
    GPIO.setmode(GPIO.BCM)
    # 设置输出点GPIO为输出状态，且默认输出低电平
    for pin in pinOut:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    # 设置输入点GPIO为上拉输入模式，且默认设为下降沿触发
    for pin in pinIn:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if cfg['trigger'] == 0:
            GPIO.add_event_detect(pin, GPIO.FALLING, callback=on_interrupt, bouncetime=200)
        elif cfg['trigger'] == 1:
            GPIO.add_event_detect(pin, GPIO.RISING, callback=on_interrupt, bouncetime=200)
        elif cfg['trigger'] == 2:
            GPIO.add_event_detect(pin, GPIO.BOTH, callback=on_interrupt, bouncetime=200)


def on_interrupt(pin):
    print('GPIO.%d triggered' % pin)
    index = pinIn.index(pin)
    global counter
    if counter % 2 == 0:
        GPIO.output(pinOut[index], GPIO.HIGH)
    else:
        GPIO.output(pinOut[index], GPIO.LOW)
    global mqttClient
    if mqttClient is not None:
        mqttClient.publish(topicPub, str(index + 1), qos=2)
    counter += 1


# 连接成功回调函数
def on_connect(client, userdata, flags, rc):
    print('Connected with result code ' + str(rc))
    # 连接完成之后订阅主题
    client.subscribe(topicSub, qos=2)
    # 并发布已连接消息
    client.publish(topicPub, 'connected', qos=2)


# 消息推送回调函数
def on_message(client, userdata, msg):
    print('Received: ' + msg.topic + " " + str(msg.payload))
    # 获得负载中的pin和value
    payload = str(msg.payload, encoding='UTF-8')
    if len(payload) == 2:
        status = ''
        if payload == 'OP':
            for pin in pinOut:
                status += str(GPIO.input(pin))
            print('Output status: ' + status)
            client.publish(topicPub, status, qos=2)
        elif payload == 'IP':
            for pin in pinIn:
                status += str(GPIO.input(pin))
            print('Input status: ' + status)
            client.publish(topicPub, status, qos=2)
        else:
            pin = payload[0]
            value = payload[1]
            print('Output point: ' + pin + ', value: ' + value)
            if value == '0':
                GPIO.output(pinOut[int(pin) - 1], GPIO.LOW)
            elif value == '1':
                GPIO.output(pinOut[int(pin) - 1], GPIO.HIGH)
            print('GPIO.%s: %s' % (pinOut[int(pin) - 1], GPIO.input(pinOut[int(pin) - 1])))


if __name__ == '__main__':
    global mqttClient
    mqttClient = mqtt.Client(cfg['nodeID'])
    mqttClient.on_connect = on_connect
    mqttClient.on_message = on_message
    pin_setup()
    try:
        mqttClient.connect(cfg['host'], cfg['port'])
        mqttClient.loop_forever()
        # mqttClient.loop_start()
        # while True:
        #     pass
    except KeyboardInterrupt:
        print('User pressed CTRL+C')
    except:
        print('Other error or exception occurred!')
    finally:
        # mqttClient.loop_stop()
        mqttClient.disconnect()
        GPIO.cleanup()
