import argparse

import cv2
import numpy as np
import torch

from models.with_mobilenet import PoseEstimationWithMobileNet
from modules.keypoints import extract_keypoints, group_keypoints
from modules.load_state import load_state
from modules.pose import Pose, propagate_ids
from val import normalize, pad_width

import base64
import urllib

import sys
sys.path.append('../')
import time
from Robotarm import Robotarm

arm = None

class ImageReader(object):
    def __init__(self, file_names):
        self.file_names = file_names
        self.max_idx = len(file_names)

    def __iter__(self):
        self.idx = 0
        return self

    def __next__(self):
        if self.idx == self.max_idx:
            raise StopIteration
        img = cv2.imread(self.file_names[self.idx], cv2.IMREAD_COLOR)
        if img.size == 0:
            raise IOError('Image {} cannot be read'.format(self.file_names[self.idx]))
        self.idx = self.idx + 1
        return img


class VideoReader(object):
    def __init__(self, file_name):
        self.file_name = file_name
        try:  # OpenCV needs int to read from webcam
            self.file_name = int(file_name)
        except ValueError:
            pass

    def __iter__(self):
        self.cap = cv2.VideoCapture(self.file_name)
        self.cap.set(cv2.CAP_PROP_FPS, 0.3)
        if not self.cap.isOpened():
            raise IOError('Video {} cannot be opened'.format(self.file_name))
        return self

    def __next__(self):
        was_read, img = self.cap.read()
        if not was_read:
            raise StopIteration
        return img


def infer_fast(net, img, net_input_height_size, stride, upsample_ratio, cpu,
               pad_value=(0, 0, 0), img_mean=(128, 128, 128), img_scale=1/256):
    height, width, _ = img.shape
    scale = net_input_height_size / height

    scaled_img = cv2.resize(img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    scaled_img = normalize(scaled_img, img_mean, img_scale)
    min_dims = [net_input_height_size, max(scaled_img.shape[1], net_input_height_size)]
    padded_img, pad = pad_width(scaled_img, stride, pad_value, min_dims)

    tensor_img = torch.from_numpy(padded_img).permute(2, 0, 1).unsqueeze(0).float()
    if not cpu:
        tensor_img = tensor_img.cuda()

    stages_output = net(tensor_img)

    stage2_heatmaps = stages_output[-2]
    heatmaps = np.transpose(stage2_heatmaps.squeeze().cpu().data.numpy(), (1, 2, 0))
    heatmaps = cv2.resize(heatmaps, (0, 0), fx=upsample_ratio, fy=upsample_ratio, interpolation=cv2.INTER_CUBIC)

    stage2_pafs = stages_output[-1]
    pafs = np.transpose(stage2_pafs.squeeze().cpu().data.numpy(), (1, 2, 0))
    pafs = cv2.resize(pafs, (0, 0), fx=upsample_ratio, fy=upsample_ratio, interpolation=cv2.INTER_CUBIC)

    return heatmaps, pafs, scale, pad

def wait():
    while 1:
        data = arm.readline()
        if data!=b'' and data[0]==102:
            print(data)
            break

class StateMachine:
    
    def __init__(self, id, pose, arm):
        self.id = id
        self.r_move_cnt = 0
        self.r_still_cnt = 0
        self.l_move_cnt = 0
        self.l_still_cnt = 0
        self.pose = pose
        self.move = False
        self.arm = arm

        self.limb_length = 100.0
        if self.pose[3][0] != -1 and self.pose[4][0] != -1:
            self.limb_length = np.linalg.norm(self.pose[3]-self.pose[4])
        elif self.pose[6][0] != -1 and self.pose[7][0] != -1:
            self.limb_length = np.linalg.norm(self.pose[6]-self.pose[7])
    
    def update(self, new_pose, img):
        # r_elb-3, r-wri-4, l_elb-6, l_wri-7
        if new_pose[4][0] != -1 and new_pose[3][0] != -1:
            self.limb_length = np.linalg.norm(new_pose[4]-new_pose[3])
        elif new_pose[6][0] != -1 and new_pose[7][0] != -1:
            self.limb_length = np.linalg.norm(new_pose[6]-new_pose[7])

        move_thrd = self.limb_length / 20
        if new_pose[4][0] != -1:
            r_move_dist = np.linalg.norm(new_pose[4] - self.pose[4])
        else:
            r_move_dist = -1
        if new_pose[7][0] != -1:
            l_move_dist = np.linalg.norm(new_pose[7] - self.pose[7])
        else:
            l_move_dist = -1

        if r_move_dist != -1:
            if self.r_move_cnt < 5:
                if r_move_dist > move_thrd:
                    self.r_move_cnt += 1
                    if self.r_move_cnt == 3 and new_pose[2][1] < new_pose[4][1]:
                        self.random_act()
                else:
                    self.r_move_cnt = 0
                    if self.move:
                        self.arm.prepare()
                        self.move = False

            else:
                if r_move_dist <= move_thrd:
                    self.r_still_cnt += 1
                #else:
                    #self.r_move_cnt = 0
                    #self.r_still_cnt = 0
                if self.r_still_cnt == 5:
                    centerx, centery = new_pose[4]
                    minx = max(0, int(centerx-1.5*self.limb_length))
                    maxx = min(img.shape[1]-1, int(centerx+1.5*self.limb_length))
                    miny = max(0, int(centery-1.5*self.limb_length))
                    maxy = min(img.shape[0]-1, int(centery+1.5*self.limb_length))
                    if new_pose[2][1] < new_pose[4][1]: # wrist below shoulder
                        self.trigger(img[miny:maxy, minx:maxx, :])
                    else:
                        self.r_move_cnt = 0
                        self.r_still_cnt = 0
        
        if l_move_dist != -1:
            if self.l_move_cnt < 5:
                if l_move_dist > move_thrd:
                    self.l_move_cnt += 1
                    if self.l_move_cnt == 3 and new_pose[5][1] < new_pose[7][1]:
                        self.random_act()
                else:
                    self.l_move_cnt = 0
                    if self.move:
                        self.arm.prepare()
                        self.move = False

            else:
                if l_move_dist <= move_thrd:
                    self.l_still_cnt += 1
                else:
                    self.l_move_cnt = 0
                    self.l_still_cnt = 0
                if self.l_still_cnt == 5:
                    centerx, centery = new_pose[7]
                    minx = max(0, int(centerx-1.5*self.limb_length))
                    maxx = min(img.shape[1]-1, int(centerx+1.5*self.limb_length))
                    miny = max(0, int(centery-1.5*self.limb_length))
                    maxy = max(img.shape[0]-1, int(centery+1.5*self.limb_length))
                    if new_pose[5][1] < new_pose[7][1]:
                        self.trigger(img[miny:maxy, minx:maxx, :])
                    else:
                        self.r_move_cnt = 0
                        self.r_still_cnt = 0
        
        self.pose = new_pose
        #print('ID {}: r_move_cnt {}, r_still_cnt {}, l_move_cnt {}, l_still_cnt {}'.format(self.id, self.r_move_cnt, self.r_still_cnt, self.l_move_cnt, self.l_still_cnt))


    def random_act(self):
        self.arm.prepare2()
        wait()
        act = self.arm.st_jd_b()
        wait()
        if act == 0:
            self.act = 0
        elif act == 1:
            self.act == 2
        elif act == 2:
            self.act == 5
        self.move = True
        

    def trigger(self, img):
        print(self.id, 'triggered')

        self.r_move_cnt = 0
        self.r_still_cnt = 0
        self.l_move_cnt = 0
        self.l_still_cnt = 0
        
        request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/gesture"

        cv2.imwrite('hand.jpg', img)
        base64_str = cv2.imencode('.jpg', img)[1].tostring()
        base64_str = base64.b64encode(base64_str)

        params = {"image":base64_str}
        params = urllib.parse.urlencode(params).encode(encoding='UTF8')

        access_token = '24.c277ba8aff1df28b001179eb81038f18.2592000.1569241320.282335-17077804'
        request_url = request_url + "?access_token=" + access_token
        request = urllib.request.Request(url=request_url, data=params)
        request.add_header('Content-Type', 'application/x-www-form-urlencoded')
        response = urllib.request.urlopen(request)
        content = response.read()
        if content:
            gesture = self.get_gesture(content)
        else: 
            print('No response')

        self.arm.prepare()
        wait()
        time.sleep(3)

    def get_gesture(self, response):
        """
        0-rock, 2-scissor, 5-paper
        """
        
        output_number = None
        output_cls = None
        results = response['result']        
        for res in results:
            classname = res['classname']
            if classname in StateMachine.word2num.keys():
                output_number = StateMachine.word2num[classname]
                output_cls = classname
        if output_number == None:
            print("No gesture recognized: ", response)
        print(output_number, output_cls)
        return output_number


def run_demo(net, image_provider, height_size, cpu, track_ids, arm):
    net = net.eval()
    if not cpu:
        net = net.cuda()

    stride = 8
    upsample_ratio = 4
    num_keypoints = Pose.num_kpts
    previous_poses = []

    stateMachines = {}

    for img in image_provider:
        orig_img = img.copy()
        heatmaps, pafs, scale, pad = infer_fast(net, img, height_size, stride, upsample_ratio, cpu)

        total_keypoints_num = 0
        all_keypoints_by_type = []
        for kpt_idx in range(num_keypoints):  # 19th for bg
            total_keypoints_num += extract_keypoints(heatmaps[:, :, kpt_idx], all_keypoints_by_type, total_keypoints_num)

        pose_entries, all_keypoints = group_keypoints(all_keypoints_by_type, pafs, demo=True)
        for kpt_id in range(all_keypoints.shape[0]):
            all_keypoints[kpt_id, 0] = (all_keypoints[kpt_id, 0] * stride / upsample_ratio - pad[1]) / scale
            all_keypoints[kpt_id, 1] = (all_keypoints[kpt_id, 1] * stride / upsample_ratio - pad[0]) / scale
        current_poses = []
        for n in range(len(pose_entries)):
            if len(pose_entries[n]) == 0:
                continue
            pose_keypoints = np.ones((num_keypoints, 2), dtype=np.int32) * -1
            for kpt_id in range(num_keypoints):
                if pose_entries[n][kpt_id] != -1.0:  # keypoint was found
                    pose_keypoints[kpt_id, 0] = int(all_keypoints[int(pose_entries[n][kpt_id]), 0])
                    pose_keypoints[kpt_id, 1] = int(all_keypoints[int(pose_entries[n][kpt_id]), 1])

            """
            kpt_names = ['nose', 'neck',
                 'r_sho', 'r_elb', 'r_wri', 'l_sho', 'l_elb', 'l_wri',
                 'r_hip', 'r_knee', 'r_ank', 'l_hip', 'l_knee', 'l_ank',
                 'r_eye', 'l_eye',
                 'r_ear', 'l_ear']
            r_elb-3, r-wri-4, l_elb-6, l_wri-7
            """
            # print('ID: {}'.format(n))
            # print('\tRight elbow: {}, right wrist: {}'.format(pose_keypoints[3], pose_keypoints[4]))
            # print('\tLeft elbow: {}, left wrist: {}'.format(pose_keypoints[6], pose_keypoints[7]))
    
            pose = Pose(pose_keypoints, pose_entries[n][18])
            current_poses.append(pose)
            pose.draw(img)

        img = cv2.addWeighted(orig_img, 0.6, img, 0.4, 0)
        if track_ids == True:
            propagate_ids(previous_poses, current_poses, threshold=3)
            previous_poses = current_poses
            for pose in current_poses:
                cv2.rectangle(img, (pose.bbox[0], pose.bbox[1]),
                              (pose.bbox[0] + pose.bbox[2], pose.bbox[1] + pose.bbox[3]), (0, 255, 0))
                cv2.putText(img, 'id: {}'.format(pose.id), (pose.bbox[0], pose.bbox[1] - 16),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
        
        for pose in current_poses:
            if pose.id not in stateMachines.keys():
                stateMachines[pose.id] = StateMachine(pose.id, pose.keypoints, arm)
                print('ID {} detected'.format(pose.id))
                continue
            stateMachines[pose.id].update(pose.keypoints, img)


        cv2.imshow('Lightweight Human Pose Estimation Python Demo', img)
        key = cv2.waitKey(33)
        if key == 27:  # esc
            return


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='''Lightweight human pose estimation python demo.
                       This is just for quick results preview.
                       Please, consider c++ demo for the best performance.''')
    parser.add_argument('--checkpoint-path', type=str, required=True, help='path to the checkpoint')
    parser.add_argument('--height-size', type=int, default=256, help='network input layer height size')
    parser.add_argument('--video', type=str, default='', help='path to video file or camera id')
    parser.add_argument('--images', nargs='+', default='', help='path to input image(s)')
    parser.add_argument('--cpu', action='store_true', help='run network inference on cpu')
    parser.add_argument('--track-ids', default=True, help='track poses ids')
    parser.add_argument("-p", "--port",help="Input serial port",type=str,default="/dev/ttyACM0")
    parser.add_argument('-r', "--rate",help="Input baudrate",type=int, default=9600)
    args = parser.parse_args()

    if args.video == '' and args.images == '':
        raise ValueError('Either --video or --image has to be provided')

    net = PoseEstimationWithMobileNet()
    checkpoint = torch.load(args.checkpoint_path, map_location='cpu')
    load_state(net, checkpoint)

    frame_provider = ImageReader(args.images)
    if args.video != '':
        frame_provider = VideoReader(args.video)

    
    
    angles=[60,10,15,15,55,90,0,0,90,30,1] #angels is the vector representing the angels of different servos []
    control_index=[90,10.5,11,90,1,1,1,1,1,30,1]
    arm = Robotarm(args.port, args.rate, angles)

    try:
        while 1:
            arm.control(control_index)
            data = arm.readline()
            if data!=b'':
                print(data[0])
                if data[0]==98:
                    break
        arm.prepare()
        wait()
        print('start')
        run_demo(net, frame_provider, args.height_size, args.cpu, args.track_ids, arm)
    except KeyboardInterrupt:
        final_index=[90,10.5,7,90,0,0,0,0,0,30,1]
        arm.control(final_index)
        arm.close()
        print('close')
