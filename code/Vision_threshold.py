import cv2
import numpy as np
import uno_contrl
import time
import threading
from threading import Timer

thread_lock = threading.Lock()

cap = cv2.VideoCapture(0)
kernel = np.ones((3, 3), dtype=np.uint8)


class revised_route_thread(threading.Thread):
    def __init__(self, start, end,reverse):
        threading.Thread.__init__(self)
        self.start_second = start
        self.end_second = end
        self.reverse = reverse

    def run(self):
        thread_lock.acquire()
        revised_route(self.start_second, self.end_second,self.reverse)
        thread_lock.release()


class second_data():
    def __init__(self):
        self.command = ''
        self.reverse = ''
        self.start_second = 0
        self.end_second = 0
        self.second_lock = 0
        self.reverse_lock = 0

    def record_seconds(self):
        if self.command == 'start':
            if self.second_lock == 0:
                self.start_second = time.process_time()
                self.second_lock = 1
        elif self.command == 'end':
            if self.second_lock == 1:
                self.end_second = time.process_time()
                print('start_time: %f'%self.start_second)
                print('end_time: %f'%self.end_second)
                reverse_time = Timer(3,revised_route,args=(self.start_second,self.end_second,self.reverse))
                print('!!!!!!!!!!!!!!')
                print('#############')
                reverse_time.start()
                #revised_route(self.start_second,self.end_second,self.reverse)
                self.second_lock = 0
                self.reverse_lock = 1

    def return_to_zero(self):
        print('okokokokokokokokokokokokokoko')
        uno_contrl.command.write('!s1500\n'.encode())
        self.start_second = 0
        self.end_second = 0
        self.command = ''
        self.reverse = ''
        second.reverse_lock = 0 


second = second_data()


class frame_data():
    def __init__(self):
        self.left_part_contours_road = 0
        self.right_part_contours_road = 0
        self.left_part_contours_safety = 0
        self.right_part_contours_safety = 0

    def match_data(self):
        if self.left_part_contours_safety > 2 and self.left_part_contours_road:
            uno_contrl.command.write('!s2000\n'.encode())
            second.command = 'start'
            second.reverse = 'left'
            second.record_seconds()
        elif self.right_part_contours_safety > 2 and self.right_part_contours_road:
            uno_contrl.command.write('!s1000\n'.encode())
            second.command='start'
            second.reverse = 'right'
            second.record_seconds()
        else:
            if second.reverse_lock == 0:
                if second.command == 'start':
                    second.command= 'end'
                    second.record_seconds()
                else:
                    uno_contrl.command.write('!s1500\n'.encode())


#green = np.uint8([[[134,143,146 ]]])
#hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
#print( hsv_green )


def turn_target():
    print('coding')

def revised_route(start: float, end: float,reverse:str):
    correct_seconds = end - start
    if reverse == 'left':
        print('***********')
        print(correct_seconds)
        print('left')
        uno_contrl.command.write('!s1000\n'.encode())
        left_Timer = Timer(correct_seconds, second.return_to_zero)
        left_Timer.start()
    else:
        print('**********')
        print(correct_seconds)
        print('right')
        uno_contrl.command.write('!s2000\n'.encode())
        right_Timer = Timer(correct_seconds, second.return_to_zero)
        right_Timer.start()



# handle_frame
def handle_frame_road(part_side, frame):
    frame_road = frame.copy()
    # road
    road_low = [0, 0, 80]
    road_uper = [0, 0, 255]
    Gray_frame = cv2.cvtColor(frame_road, cv2.COLOR_BGR2GRAY)
    Blur_frame = cv2.GaussianBlur(Gray_frame, (15, 15), 0)
    Back_BGR_frame = cv2.cvtColor(Blur_frame, cv2.COLOR_GRAY2BGR)
    hsv_frame = cv2.cvtColor(Back_BGR_frame, cv2.COLOR_BGR2HSV)
    road_lower_hsv = np.array(road_low)
    road_uper_hsv = np.array(road_uper)
    road_mask = cv2.inRange(hsv_frame, road_lower_hsv, road_uper_hsv)
    dst = cv2.adaptiveThreshold(
        road_mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 3, 10)
    contours, hierarchy = cv2.findContours(
        dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print('this is %s: %d' % (part_side, len(contours)))
    draw_block = cv2.drawContours(frame, contours, -1, (57, 0, 199), 3)
    return part_side, len(contours), road_mask


def handle_frame_safety(part_side, frame):
    frame_safety = frame.copy()
    # safety_line
    safety_line_low = [35, 43, 46]
    safety_line_uper = [77, 255, 255]
    Blur_safety_line = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv_safety_line_frame = cv2.cvtColor(Blur_safety_line, cv2.COLOR_BGR2HSV)
    #dilate = cv2.dilate(threshold_frame, kernel, 1)
    #erosion = cv2.erode(threshold_frame, kernel, 20)
    safety_line_lower_hsv = np.array(safety_line_low)
    safety_line_uper_hsv = np.array(safety_line_uper)
    safety_line_mask = cv2.inRange(
        hsv_safety_line_frame, safety_line_lower_hsv, safety_line_uper_hsv)
    #dst_safety_line = cv2.adaptiveThreshold(safety_line_mask,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,3,10)
    contours, hierarchy = cv2.findContours(
        safety_line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        print(len(contours))
        boxes = [cv2.boundingRect(c) for c in contours]
        for box in boxes:
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
    #draw_safetyline = cv2.drawContours(frame,contours,-1,(0,0,255),3)

    return part_side, len(contours), safety_line_mask

# draw_safetyline


def safety_line(frame):
    height, width = frame.shape[0:2]
    radius = 420
    center = (int(width/2), int(height+260))
    axes = (radius, radius)
    angle = 180
    startAngle = 0
    endAngle = 180
    thickness = 1

    #point_0 = (int(width/2),int(height-100))
    #point_1 = (int(width/2),height)

    left_part = frame[height-160:height, 0:int(width/2)]
    right_part = frame[height-160:height, int(width/2):width]

    green = (0, 255, 0)


    cv2.ellipse(frame, center, axes, angle,
                startAngle, endAngle, green, thickness)
    # cv2.line(image,point_0,point_1,green,thickness)

match_frame = frame_data()


def check_road(frame):
    height, width = frame.shape[0:2]
    left_part = frame[height - 160:height, 0:int(width / 2)]
    right_part = frame[height - 160:height, int(width / 2):width]

    part_side_road, left_part_contours_road, left_part_out = handle_frame_road(
        'left_part', left_part)
    part_side_road, right_part_contours_road, right_part_out = handle_frame_road(
        'right_part', right_part)
    match_frame.left_part_contours_road = left_part_contours_road
    match_frame.right_part_contours_road = right_part_contours_road
    cv2.imshow('left_part', left_part_out)
    cv2.imshow('right_part', right_part_out)


def check_safety(frame):
    height, width = frame.shape[0:2]
    left_part = frame[height - 160:height, 0:int(width / 2)]
    right_part = frame[height - 160:height, int(width / 2):width]

    part_side, left_part_contours, left_part_hsv = handle_frame_safety(
        'left_part', left_part)
    part_side, right_part_contours, right_part_hsv = handle_frame_safety(
        'right_part', right_part)
    match_frame.left_part_contours_safety = left_part_contours
    match_frame.right_part_contours_safety = right_part_contours
    match_frame.match_data()
    cv2.imshow('left_part_hsv', left_part_hsv)
    cv2.imshow('right_part_hsv', right_part_hsv)


def main():
    while True:
        ret, frame = cap.read()
        check_road(frame)
        safety_line(frame)
        check_safety(frame)
        cv2.imshow('result', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
