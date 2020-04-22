
#  Copyright (c) Prokura Innovations. All rights reserved.
#  Licensed under the MIT license. See LICENSE file in the project root for full license information.

import socketio
import json

sio = socketio.Client()


@sio.on('connect', namespace='/')
def on_connect():
    print('connection established')
    sio.emit('message', {"dfa": "dfsa"})


@sio.on('reply', namespace='/')
def on_message(data):
    print(json.dumps(data))
    #sio.emit('message', {'chor':'shovan daka'})


@sio.on('angle', namespace='/')
def on_angle(data):
    print(json.dumps(data))
    with open('test.txt', 'w') as f:
        json.dump(data, f)
    sio.emit('message', {'chor': 'shovan daka'})

# @sio.on('message1',namespace='/')
# def on_message1(data):
    # print(data)


@sio.on('disconnect', namespace='/')
def on_disconnect():
    print('disconnected from server')


sio.connect('http://10.42.0.1:8080')
# sio.connect('http://192.168.43.135:8080')
# sio.emit('message1',{'dfa':"sdf"})
# sio.connect('http://localhost:8080')
sio.wait()
