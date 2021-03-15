import time
import json
import radar
import uwb
import uno_contrl

#

#radar
turn_on_radar = 0

#get_location
def load():
    with open('location.json', 'r') as r:
        location_data = json.load(r)
        return location_data["locations_tag"]

n0_distance = 0
n1_distance = 0
location_data = load()
num = 0
while True:
    num = num + 1
    print(num)
    try:
        if(uwb.command.read()):
            back_data = uwb.command.readline().decode()
            if back_data[:2] == 'n0':
                n0_distance = float(back_data[3:7])
                print('this is 0: %f' % (n0_distance))
            else:
                n1_distance = float(back_data[3:7])
                print("this is 1: %f" % (n1_distance))
            for i in range(len(location_data)):
                # print(location_data[i])
                less_range_0 = location_data[i]["range"]["less_range_0"]
                greater_range_0 = location_data[i]['range']['greater_range_1']
                less_range_1 = location_data[i]['range']['less_range_1']
                greater_range_1 = location_data[i]['range']['greater_range_1']
                if (n0_distance > less_range_0 and n0_distance < greater_range_0) and (n1_distance > less_range_1 and n1_distance < greater_range_1):
                    if (location_data[i]['name'] == 'C_tag'):
                        print('====================')
                        print(location_data[i]['name'] + ' stop')
                        print('====================')
                        uno_contrl.command.write('!s1500\n'.encode())
                        #time.sleep(1)
                        uno_contrl.command.write('!p1600\n'.encode())
                    else:
                        # arduino_command.write('!s1500\n'.encode())
                        print('========================')
                        print(location_data[i]['name'] + ' turn right')
                        print('========================')
                        uno_contrl.command.write('!s2000\n'.encode())
                        turn_on_radar = 1
            if turn_on_radar == 1:
                radar.getTFminiData()
                turn_on_radar = 0
        '''
        if(arduino_command.read()):
            call_data = arduino_command.readline().decode()
            print(call_data)
        '''
    except Exception as e:
        print(e)
