from ctypes import *
import math
import random
import numpy as np
import cv2


def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1


def c_array(ctype, values):
    arr = (ctype*len(values))()
    arr[:] = values
    return arr


class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]


class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]


class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]


#lib = CDLL("/home/pjreddie/documents/darknet/libdarknet.so", RTLD_GLOBAL)
lib = CDLL("./libdarknet.so", RTLD_GLOBAL)
lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

set_gpu = lib.cuda_set_device
set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int,
                              c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)


def classify(net, meta, im):
    out = predict_image(net, im)
    res = []
    for i in range(meta.classes):
        res.append((meta.names[i], out[i]))
    res = sorted(res, key=lambda x: -x[1])
    return res


def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
    im = load_image(image, 0, 0)
    num = c_int(0)
    pnum = pointer(num)
    predict_image(net, im)
    dets = get_network_boxes(net, im.w, im.h, thresh,
                             hier_thresh, None, 0, pnum)
    num = pnum[0]
    if (nms):
        do_nms_obj(dets, num, meta.classes, nms)

    res = []
    for j in range(num):
        for i in range(meta.classes):
            if dets[j].prob[i] > 0:
                b = dets[j].bbox
                res.append(
                    (meta.names[i], dets[j].prob[i], (b.x, b.y, b.w, b.h)))
    res = sorted(res, key=lambda x: -x[1])
    free_image(im)
    free_detections(dets, num)
    return res


# def hor_distance(p1, p2):
#     np.linalg.


if __name__ == "__main__":
    #net = load_net("cfg/densenet201.cfg", "/home/pjreddie/trained/densenet201.weights", 0)
    #im = load_image("data/wolf.jpg", 0, 0)
    #meta = load_meta("cfg/imagenet1k.data")
    #r = classify(net, meta, im)
    # print r[:10]
    # net = load_net("../cfg/yolov3.cfg", "../yolov3.weights", 0)
    # meta = load_meta("../cfg/coco.data")

    net = load_net("cfg/yolov3-tiny-obj.cfg",
                   "weights/yolov3-tiny-obj_21000.weights", 0)
    meta = load_meta("cfg/obj.data")
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture('sw2.mp4')

    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        cv2.imwrite('dummy.jpg', frame)
        # print(cap.get(3), cap.get(4))
        # Display the resulting frame
        frame_height, frame_width, _ = frame.shape
        frame_center = (int(frame_width/2),
                        int(frame_height/2))
        r = detect(net, meta, 'dummy.jpg')
        # print r

        if len(r) != 0:
            box_x = r[0][2][0]
            box_y = r[0][2][1]
            box_w = r[0][2][2]
            box_h = r[0][2][3]
            box_top_left = (int(box_x - box_w/2), int(box_y - box_h/2))
            box_bottom_right = (int(box_x + box_w/2), int(box_y + box_h/2))
            box_center = (int(box_x), int(box_y))
            quad_center = frame_center
            # let us consider quad_center as origin
            disp = np.array(box_center) - np.array(quad_center)

            # find distance
            eq_dist = np.linalg.norm(disp)          
            cv2.putText(frame, 'dist: '+str(eq_dist), (280, 80), 0, 0.5, (0, 0, 255))
            
            # find angle
            angle = np.arctan2(-disp[1], disp[0])
            angle_deg = np.degrees(angle)
            if angle_deg < 0:
                angle_deg += 360
            cv2.putText(frame, 'ang: '+str(angle_deg), (280, 90), 0, 0.5, (0, 0, 255))
            

            print(str(eq_dist)+' pix', str(angle_deg) + ' deg')
            cv2.rectangle(frame, box_top_left,
                          box_bottom_right, (0, 255, 0), 3)
            cv2.circle(frame, box_center,
                       10, (0, 0, 0), -1)
            cv2.circle(frame, frame_center, 10, (255, 0, 0), -1)
            cv2.line(frame, frame_center, box_center,(255,0,0),5)
            


            
            # cv2.line(frame,(0,0),(int(box_x),int(box_y)),(255,0,0),5)
            # cv2.line(frame, (0,0),(int(box_w),0),(0,0,255),5)
            # cv2.line(frame, (0,0),(0,int(box_h)),(0,255,0),5)
        # cv2.imshow('frame', cv2.imread('dummy.jpg'))
        # print type(frame)
        # this is the cv coordinate
        # draw the origin of opencv
        cv2.circle(frame, (0,0), 15, (0, 255, 0), -1)
        # draw the y axis
        cv2.line(frame, (0, 0), (0, frame_center[1]-50),(255,0,0),3)
        cv2.putText(frame, 'y-axis', (0, frame_center[1]-100), 0, 0.8, (0, 0, 0))
        # draw the x axis
        cv2.line(frame, (0, 0), (frame_center[0]-50, 0),(255,0,0),3)
        cv2.putText(frame, 'x-axis', (frame_center[0]-100, 10), 0, 0.8, (0, 0, 0))

        # draw the body coordinate
        cv2.circle(frame, frame_center, 10, (255, 0, 0), -1)
        # draw the y axis
        cv2.line(frame, (frame_center[0], 0), (frame_center[0], int(frame_height)),(0,0,255),1)
        # cv2.putText(frame, 'y-axis-body', (0, frame_center[1]), 0, 0.5, (0, 0, 255))
        # draw the x axis
        cv2.line(frame, (0, frame_center[1]), (int(frame_width), frame_center[1]),(0,0, 255),1)
        # cv2.putText(frame, 'x-axis-body', (frame_center[0], 0), 0, 0.5, (0, 0, 255))


        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # print r
