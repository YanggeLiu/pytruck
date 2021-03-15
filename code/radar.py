import serial
import uno_contrl


#load_settings
def load():
    with open('settings.json', 'r') as r:
        setting_data = json.load(r)
        return settings_data["radar"]["port"],setting_data["radar"]["baudrate"]
port,baudrate = load()

ser = serial.Serial(port,baudrate)

def getTFminiData():
    while True:
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)
            ser.reset_input_buffer()

            if recv[0] == 0x59 and recv[1] == 0x59:
                distance = recv[2] + recv[3] * 256
                strength = recv[4] + recv[5] * 256
                distance = float(distance)
                strength = float(strength)
                print('distance: %f, strength: %f'%(distance,strength))
                ser.reset_input_buffer()
                if distance > 400:
                    uno_contrl.command.write('!s1500\n'.encode())
                    break

'''
if __name__=='__main__':
    try:
        if ser.is_open == False:
            ser.open()
        getTFminiData()
    except KeyboardInterrupt: # Crtl+C
        if ser != None:
            ser.close()
'''