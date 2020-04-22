//
// Copyright (c) Prokura Innovations. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
//
import socketio
import eventlet
import json

sio = socketio.Server()
app = socketio.WSGIApp(sio, static_files={
    '/': {'content_type': 'text/html', 'filename': 'index.html'}
})

@sio.on('connect',namespace='/')
def connect(sid, environ):
    print('connect ', sid)

@sio.on('message',namespace='/')
def message(sid, data):
    print(json.dumps(data))
    sio.emit('reply',{'kamina':'go to hell'},room=sid)

@sio.on('disconnect',namespace='/')
def disconnect(sid):
    print('disconnect ', sid)

if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen(('', 8080)), app)
