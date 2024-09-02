
import rospy
import os
import cv2
import time
import argparse
import sys
import rospkg

# 获取功能包的路径
currentRospackPath = rospkg.RosPack().get_path('yolo')
yolo_path = currentRospackPath + "/scripts/Yolo-FastestV2"
sys.path.append(yolo_path)


import torch
import model.detector
import utils.utils
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from threading import Thread



def img_forward_process(ros_img):
    global yolo_model, device, cfg, yolo_classes, yolo_center, yolo_result_img

    ori_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
    #数据预处理
    # ori_img = cv2.imread(img)
    res_img = cv2.resize(ori_img, (cfg["width"], cfg["height"]), interpolation = cv2.INTER_LINEAR) 
    img = res_img.reshape(1, cfg["height"], cfg["width"], 3)
    img = torch.from_numpy(img.transpose(0,3, 1, 2))
    img = img.to(device).float() / 255.0

    #模型推理
    start = time.time()
    preds = yolo_model(img)
    end = time.time()
    spend_time = (end - start) * 1000.
    # print("forward time:%fms"%spend_time, end='---')

    #特征图后处理
    output = utils.utils.handel_preds(preds, cfg, device)
    output_boxes = utils.utils.non_max_suppression(output, conf_thres = 0.3, iou_thres = 0.4)

    #加载label names
    LABEL_NAMES = []
    with open(cfg["names"], 'r') as f:
        for line in f.readlines():
            LABEL_NAMES.append(line.strip())

    h, w, _ = ori_img.shape
    scale_h, scale_w = h / cfg["height"], w / cfg["width"]

    yolo_classes = []
    yolo_center = []

    #绘制预测框
    for box in output_boxes[0]:
        box = box.tolist()
        obj_score = box[4]
        if float(obj_score) > 0.95:
            category = LABEL_NAMES[int(box[5])]
            
            x1, y1 = int(box[0] * scale_w), int(box[1] * scale_h)
            x2, y2 = int(box[2] * scale_w), int(box[3] * scale_h)
            center = [(x1 + x2) / 2, (y1 + y2) / 2]
            # print("class: %s, center: (%d, %d)" %(category, center[0], center[1]), end='')

            yolo_classes.append(category)
            yolo_center.append([center[0], center[1]])

            cv2.rectangle(ori_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(ori_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)	
            cv2.putText(ori_img, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
    # print('')
    
    yolo_result_img = bridge.cv2_to_imgmsg(ori_img, encoding="bgr8")
    # cv2.imwrite("test_result.png", ori_img)
    # cv2.namedWindow("result",0)
    # cv2.resizeWindow("result", 640, 360)
    # cv2.imshow("result",ori_img)
    # cv2.waitKey(10)


# 发布识别结果
def publishYoloResult():
    global yolo_result_pub, yolo_classes, yolo_center, yolo_result_img_pub, yolo_result_img
    
    frequence = 50
    cout = 0
    rate = rospy.Rate(frequence)
    while not rospy.is_shutdown():
        yolo_result = String()
        yolo_result.data = "classes:"
        for name in yolo_classes:
            yolo_result.data += "%s," %(name)
        yolo_result.data += "center:"
        for center in yolo_center:
            yolo_result.data += "[%s,%s]," %(str(int(center[0])), str(int(center[1])))

        # print(yolo_result.data)
        yolo_result.data.replace("\n", "")
        yolo_result.data.replace("\r", "")
        yolo_result.data.replace(" ", "")
        yolo_result_pub.publish(yolo_result)

        if cout % frequence == 0:
            # print(all_classes_name.data)
            yolo_info_pub.publish(all_classes_name)
        yolo_result_img_pub.publish(yolo_result_img)
        cout += 1
        rate.sleep()



if __name__ == '__main__':

    rospy.init_node("yolo_detect")
    # 指定训练配置文件
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--data', type=str, default='', 
    #                     help='Specify training profile *.data')
    # parser.add_argument('--weights', type=str, default='', 
    #                     help='The path of the .pth model to be transformed')
    # parser.add_argument('--img', type=str, default='',
    #                     help='The path of test image')
    # opt = parser.parse_args()
    opt = argparse.Namespace()

    # opt.data = 'data/arm.data'
    # opt.weights = 'weights/arm-90-epoch-0.992992ap-model.pth'
    # opt.img = 'datasets/val/063.jpg'

    opt.data = currentRospackPath + '/scripts/Yolo-FastestV2/data/number.data'
    opt.weights = currentRospackPath + '/scripts/Yolo-FastestV2/weights/number.pth'

    cfg = utils.utils.load_datafile(opt.data)
    assert os.path.exists(opt.weights), "请指定正确的模型路径"

    cfg["names"] = currentRospackPath + "/scripts/Yolo-FastestV2/" + cfg["names"]

    #加载获取所有classes的name
    classes_name = []
    with open(cfg["names"], 'r') as f:
        for line in f.readlines():
            classes_name.append(line.strip())
    all_classes_name = String()
    all_classes_name.data = "all classes name:"
    for i in range(0, len(classes_name)):
        all_classes_name.data += classes_name[i] + ","

    # 模型加载
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    yolo_model = model.detector.Detector(cfg["classes"], cfg["anchor_num"], True).to(device)
    yolo_model.load_state_dict(torch.load(opt.weights, map_location=device))

    # sets the module in eval node
    yolo_model.eval()
    bridge = CvBridge()

    sub = rospy.Subscriber("/gripper_camera/image_raw", Image, img_forward_process, queue_size=10)

    yolo_info_pub = rospy.Publisher("/yolo_info", String, queue_size=10)
    yolo_result_pub = rospy.Publisher("/yolo_result", String, queue_size=10)
    yolo_result_img_pub = rospy.Publisher("/yolo_result_img", Image, queue_size=10)
    yolo_result_img = Image()
    
    yolo_classes = []
    yolo_center = []

    # print("1")

    pub_thread = Thread(target=publishYoloResult)
    pub_thread.daemon = True
    pub_thread.start()


    rospy.spin()


    

        

