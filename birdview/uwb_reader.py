# bridge received around 10hz from each tag, no data from anchor

import paho.mqtt.client as mqtt
import json
import base64
import struct
import time


gateway_ip = '192.168.5.51' # pi address (MQTT server connect to bridge sensor)
port = 1883

tag1 = '0c39' # left
tag2 = '879c' # right 
anc = '442a'

fn1 = 'hummer_path/uwb_d1.txt'
fn2 = 'hummer_path/uwb_d2.txt'

topic1 = 'dwm/node/' + tag1 + '/uplink/data'
topic2 = 'dwm/node/' + tag2 + '/uplink/data'
# topic3 = 'dwm/node/' + anc + '/uplink/data' # anchor no data

def on_connect(client, userdata, flags, rc, properties=None):
    if rc==0:
        client.subscribe(topic1)
        client.subscribe(topic2)
        # client.subscribe(topic3)
        print('conn sub distance topic')
    else:
        print('conn fail')

def on_message(client, userdata, msg):
    try:
    # if True:
        time_ms = int(time.time()*1000)
        payload = json.loads(msg.payload.decode('utf-8'))
        raw_data = payload.get('data')
        dec_byte = base64.b64decode(raw_data)
        addr_l, addr_h, d1, d2, d3, d4 = struct.unpack('BBBBBB', dec_byte)
        # print(raw_data, dec_byte, addr_h, addr_l, d1, d2, d3, d4)
        addr = f'{addr_h:02x}{addr_l:02x}'
        dis = (d1+d2*256+d3*256*256+d4*256*256*256)*1.0/1000
        print(msg.topic, time_ms, addr, dis)
        
        # update file/log
        fn = 'hummer_path/uwb_d0.txt'
        if tag1 in msg.topic:
            fn = fn1
        elif tag2 in msg.topic:
            fn = fn2
        
        with open(fn, 'w') as f:
            f.write('[%d,%.3f]'%(time_ms, dis))
        
    except:
        print('msg err')
    

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(gateway_ip, port, 60)

client.loop_forever()

    