import serial
import time


#load_settings
def load():
    with open('settings.json', 'r') as r:
        settings_data = json.load(r)
        return settings_data["uno"]["port"],settings_data["uno"]["baudrate"]

port,baudrate = load()

command = serial.Serial(port, baudrate)

reset_power = command.write('!p1600\n'.encode())
time.sleep(1)
reset_stress = command.write('!s1500\n'.encode())
time.sleep(1)
start_power = command.write('!p1710\n'.encode())