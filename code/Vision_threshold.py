import cv2
import numpy as np
import uno_contrl

cap = cv2.VideoCapture(0)
kernel = np.ones((3, 3), dtype=np.uint8)

#green = np.uint8([[[134,143,146 ]]])
#hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
#print( hsv_green )

#handle_frame
def handle_frame_road(part_side,frame):
    #road
    road_low = [0,0,80]
    road_uper = [0,0,255]
    Gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    Blur_frame = cv2.GaussianBlur(Gray_frame, (5, 5), 0)
    Back_BGR_frame = cv2.cvtColor(Blur_frame, cv2.COLOR_GRAY2BGR)
    hsv_frame = cv2.cvtColor(Back_BGR_frame, cv2.COLOR_BGR2HSV)
    road_lower_hsv = np.array(road_low)
    road_uper_hsv = np.array(road_uper)
    road_mask = cv2.inRange(hsv_frame, road_lower_hsv, road_uper_hsv)
    dst = cv2.adaptiveThreshold(
        road_mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 3, 10)
    contours, hierarchy = cv2.findContours(
        dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print('this is %s: %d'%(part_side,len(contours)))
    draw_block = cv2.drawContours(frame, contours, -1, (57, 0, 199), 3)
    return draw_block

def handle_frame_safety(part_side,frame):
    #safety_line
    safety_line_low = [35,43,46]
    safety_line_uper = [99,255,255]
    Blur_safety_line = cv2.GaussianBlur(frame,(5,5),0)
    hsv_safety_line_frame = cv2.cvtColor(Blur_safety_line,cv2.COLOR_BGR2HSV)
    #dilate = cv2.dilate(threshold_frame, kernel, 1)
    #erosion = cv2.erode(threshold_frame, kernel, 20)
    safety_line_lower_hsv = np.array(safety_line_low)
    safety_line_uper_hsv = np.array(safety_line_uper)
    safety_line_mask = cv2.inRange(hsv_safety_line_frame,safety_line_lower_hsv,safety_line_uper_hsv)
    #dst_safety_line = cv2.adaptiveThreshold(safety_line_mask,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,3,10)
    contours,hierarchy = cv2.findContours(safety_line_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        print(len(contours))
        boxes = [cv2.boundingRect(c) for c in contours]
        for box in boxes:
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
    #draw_safetyline = cv2.drawContours(frame,contours,-1,(0,0,255),3)

    return part_side,len(contours),frame


def judge_point():
    print('test')

def calculate_area():
    print('test')

# draw_safetyline
def safety_line(image):
    height, width = image.shape[0:2]
    radius = 420
    center = (int(width/2), int(height+260))
    axes = (radius, radius)
    angle = 180
    startAngle = 0
    endAngle = 180
    thickness = 1

    point_0 = (int(width/2),int(height-100))
    point_1 = (int(width/2),height)

    left_part = image[height-160:height,0:int(width/2)]
    right_part = image[height-160:height,int(width/2):width]

    green = (0,255,0)


    cv2.ellipse(image, center, axes, angle, startAngle, endAngle, green, thickness)
    #cv2.line(image,point_0,point_1,green,thickness)

    left_part_out = handle_frame_road('left_part',left_part)
    right_part_out = handle_frame_road('right_part',right_part)
    part_side,left_part_contours,left_part_hsv = handle_frame_safety('left_part',left_part)
    part_side,right_part_contours,right_part_hsv = handle_frame_safety('right_part',right_part)
    if left_part_contours > 2:
        print('turn right')
        uno_contrl.command.write('!s2000\n'.encode())
    elif right_part_contours > 2:
        print('turn left')
        uno_contrl.command.write('!s1000\n'.encode())
    else:
        print('fine')
        uno_contrl.command.write('!s1500\n'.encode())
    #cv2.imshow('left_part',left_part_out)
    #cv2.imshow('right_part',right_part_out)
    cv2.imshow('left_part_hsv',left_part_hsv)
    cv2.imshow('right_part_hsv',right_part_hsv)


while True:
    ret, frame = cap.read()
    safety_line(frame)
    cv2.imshow('result', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
