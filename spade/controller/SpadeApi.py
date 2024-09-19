from flask import request

from main import app


@app.route('/sensorLocation', methods=['POST'])
def add_value():
    """
    Send location values to the multi-agent system regarding one specific sensor.
    """

    global value
    data = request.get_json()
    if 'reading' in data:
        value = data['reading']
        print(value)

    return "Received reading for sensor Location: " + value, 200
