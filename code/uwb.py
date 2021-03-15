import serial
import time

#load_dsettings
def load():
    with open('settings.json', 'r') as r:
        uwb_data = json.load(r)
        return uwb_data["uwb"]["port"], uwb["uwb"]["baudrate"]


port,baudrate = load()

command = serial.Serial(port, baudrate, bytesize=8, parity='N', stopbits=1)

command.write('AT+switchdis=0\r\n'.encode())
time.sleep(1)
command.write('AT+switchdis=1\r\n'.encode())