from flask import Flask, render_template
from flask_socketio import SocketIO
from pyngrok import ngrok


app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/')
def index():
    return render_template('index.html')


@socketio.on('video_stream')
def handle_video_stream(data):
    print("Received video frame of length:", len(data))

