from yolov4.tflite import YOLOv4
from cv2 import cv2
import numpy as np

cap = cv2.VideoCapture(0)
yolo = YOLOv4()

yolo.config.parse_names("../coco.names")
yolo.config.parse_cfg("../yolov4-tiny.cfg")

yolo.load_tflite("../yolov4-tiny-float16.tflite")
#yolo.summary()


while True:
    ret, frame = cap.read()
    pred_bboxes = yolo.predict(frame,0.25)
    print(pred_bboxes)
    frame = yolo.draw_bboxes(frame,pred_bboxes)
    cv2.imshow('frame',frame)
    print(pred_bboxes)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
#Dim(-1, (x, y, w, h, cls_id, prob))
'''
yolo.inference(
    "/dev/video0",
    is_image=False,
    cv_apiPreference=cv2.CAP_V4L2,
    cv_frame_size=(640, 480),
    cv_fourcc="YUYV",
)
'''