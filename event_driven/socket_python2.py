import socketio
import eventlet

sio = socketio.Server()
app = socketio.WSGIApp(sio, static_files={
    '/': {'content_type': 'text/html', 'filename': 'index.html'}
})

@sio.on('connect',namespace='/')
def connect(sid, environ):
    print('connect ', sid)

@sio.on('message',namespace='/')
def message(sid, data):
    print('message ', data)
    sio.emit('reply','welcome',room=sid)

@sio.on('disconnect',namespace='/')
def disconnect(sid):
    print('disconnect ', sid)

if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen(('', 8080)), app)
