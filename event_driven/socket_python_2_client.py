import socketio
import json

sio = socketio.Client()

@sio.on('connect',namespace='/')
def on_connect():
    print('connection established')

@sio.on('reply',namespace='/')
def on_message(data):
    print(json.dumps(data))
    sio.emit('message', {'chor':'shovan daka'})

@sio.on('disconnect',namespace='/')
def on_disconnect():
    print('disconnected from server')

#sio.connect('http://192.168.1.69:8080')
sio.connect('http://localhost:8080')
sio.emit('message',{'chor':'shovan is daka'})
sio.wait()
