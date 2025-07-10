from flask import Flask, request

app = Flask(__name__)

@app.route('/log', methods=['POST'])
def log_data():
    data = request.data.decode()
    print("📥 Received from ESP32:", data)
    return "OK", 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
