#!/usr/bin/env python
# -*- coding: utf-8 -*-
import datetime
import json
import time
import uuid

from kafka import KafkaProducer
from kafka.errors import KafkaError

producer = KafkaProducer(bootstrap_servers='192.168.2.91:9092')
topic = 'test'


def test():
    print('begin')
    try:
        n = 0
        while True:
            dic = {}
            dic['id'] = n
            n = n + 1
            dic['myuuid'] = str(uuid.uuid4().hex)
            dic['time'] = datetime.datetime.now().strftime("%Y%m%d %H:%M:%S")
            print(json.dumps(dic).encode())
            producer.send(topic, json.dumps(dic).encode())
            print("send:" + json.dumps(dic))
            time.sleep(0.5)
    except KafkaError as e:
        print(e)
    finally:
        producer.close()
        print('done')


if __name__ == '__main__':
    test()
