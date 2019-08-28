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
if sys.platform.lower().find('linux') != -1 or sys.platform.lower().find('darwin') != -1:
    prefix = '../'
elif sys.platform.lower().find('win32') != -1:
    prefix = '..\\'
sys.path.append(prefix)
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

class StateMachine:
    
    def __init__(self, id, pose, arm):
        self.id = id
        self.r_move_cnt = 0
        self.r_still_cnt = 0
        self.l_move_cnt = 0
        self.l_still_cnt = 0
        self.pose = pose
        self.move = False   # if the arm is making action
        self.arm = arm
        self.act = None     # the action the arm does
        self.word2num = {"Fist":0, "Two":2, "Three": 2, "Five":5, "Rock":4, "ILY":3, "Insult":1, "Thumb up":6, "Unknown":7}
        self.tracking_hand = None   # which hand to track
        self.l_track = True     # tracking left hand or not
        self.r_track = True     # tracking right hand or not
        self.lock = 0

        self.limb_length = 100.0
        if self.pose[3][0] != -1 and self.pose[4][0] != -1:
            self.limb_length = np.linalg.norm(self.pose[3]-self.pose[4])
        elif self.pose[6][0] != -1 and self.pose[7][0] != -1:
            self.limb_length = np.linalg.norm(self.pose[6]-self.pose[7])
    
    def update(self, new_pose, img):
        # r_elb-3, r-wri-4, l_elb-6, l_wri-7
        #print("self.move =", self.move)

        if self.lock != 0:
            self.lock += 1
        if self.lock == 20:
            self.lock = 0
            self.arm.prepare()


        if new_pose[4][0] != -1 and new_pose[3][0] != -1:
            self.limb_length = np.linalg.norm(new_pose[4]-new_pose[3])
        elif new_pose[6][0] != -1 and new_pose[7][0] != -1:
            self.limb_length = max(self.limb_length, np.linalg.norm(new_pose[6]-new_pose[7]))

        move_thrd = max(self.limb_length / 10, 5)
        move_frame_thrd = 4
        still_frame_thrd = 3
        if new_pose[4][0] != -1 and self.pose[4][0]:
            r_move_dist = np.linalg.norm(new_pose[4] - self.pose[4])
        else:
            r_move_dist = -1
        if new_pose[7][0] != -1 and self.pose[7][0]:
            l_move_dist = np.linalg.norm(new_pose[7] - self.pose[7])
        else:
            l_move_dist = -1

        """
    pseudo-code
    ver 1.0
        if left detected and right not detected:
            if left hand moves for 5 frames:
                robotarm act
            after that and left hand doesn't move for 5 frames:
                send to baidu and detect gesture

        if right detected and left not detected
            if right hand moves for 5 frames:
                robotarm act
            after that and right hand doesn't move for 5 frames:
                send to baidu and detect gesture

        if both detected:
            find which moves
            robotarm act
        

    ver 2.0
        need sign: self.act, self.tracking_hand
        if not act:
            if left hand move detected:
                record left hand move
                if move > 5 consecutive frames:
                    if tracking_hand == None:
                        tracking_hand = left
                    else:
                        tracking_hand = both
            if right hand move detected:
                record right hand move
                if move > consecutive frames:
                    if tracking_hand == None:
                        tracking_hand = right
                    else:
                        tracking_hand = both
            if tracking_hand != None
                robotarm act
                left hand stay frame cnt = 0
                right hand stay frame cnt = 0
        elif act:

            # after that find the hand that stays more than 5 consecutive frames
            # send the hand image to baidu and detect gesture
            # robotarm react due to the gesture

            if left hand detected:
                if left hand moves:                 # threshold bigger
                    stop tracking left hand         # then in later iterations it all stops
                    if tracking_hand == left:
                        react(Unknown gesture)
                        act = False
                        return
                else:
                    record left hand stays
            if right hand detected:
                if right hand moves:                # threshold bigger
                    stop tracking right hand        # then in later iterations it all stops
                    if tracking_hand == right
                        react(Unknown gesture)
                        act = False
                        return
                else:
                    record right hand stays

            if (left hand stays more than 5 consecutive frames) and !(right hand stays more than 5 consecutive frames):
                # at this time tracking_hand cannot be 'right'
                send left hand to baidu and return gesture
                react(gesture)
                act = False
                return
            elif !(left hand stays more than 5 consecutive frames) and (right hand stays more than 5 consecutive frames):
                # at this time tracking_hand cannot be 'left'
                right - same as above
            elif (left hand stays more than 5 consecutive frames) and (right hand stays more than 5 consecutive frames):
                # at this time tracking_hand can be any one
                if tracking_hand == left:
                    left -- same as above
                elif tracking_hand == right:
                    right -- same as above
                else:
                    send both to baidu and get both gestures            # bigger image part
                    if gesture[0] and gesture[1] are all in (0,2,5):
                        react(break-the-rule)
                    elif gesture[0] or gesture[1] in (0,2,5):
                        react(gesture-in-game)
                    elif gesture[0] and gesture[1] not in (7):
                        react(random(gesture[0], gesture[1])
                    elif gesture[0] and gesture[1] in (7):
                        react(Unknown-gesture)
                    else:
                        react(gesture-not-in-(7))
                    act = False
                    return
        """
        """
        if self.move is False:   # robot arm is not moving
            # if right limb detected
            if r_move_dist != -1:
                if r_move_dist > move_thrd:
                    self.r_move_cnt += 1
                else:
                    self.r_move_cnt = 0
                if self.r_move_cnt >= 5 and new_pose[2][1] < new_pose[4][1]:    # moving down
                    if self.tracking_hand is not None:
                        self.tracking_hand = 'right'
                    else:
                        self.tracking_hand = 'both'
            else:
                self.r_move_cnt = 0
            # if left limb detected
            if l_move_dist != -1:
                if l_move_dist > move_thrd:
                    self.l_move_cnt += 1
                else:
                    self.l_move_cnt = 0
                if self.l_move_cnt >= 5 and new_pose[5][1] < new_pose[7][1]:    # moving down
                    if self.tracking_hand is not None:
                        self.tracking_hand = 'left'
                    else:
                        self.tracking_hand = 'both'
            else:
                self.l_move_cnt = 0
            if self.tracking_hand is not None:
                self.random_act()   # this will set self.move to be True
                self.r_still_cnt = 0
                self.l_still_cnt = 0
        else:       # robot arm has already been moving
            # left hand detected
            if l_move_dist != -1:
                if l_move_dist > move_thrd:
                    self.l_track = False
                    if self.tracking_hand == 'left':
                        self.react(self.word2num['Unknown'])    # do not set self.move in this method
                        self.arm.prepare()
                        self.move = False       # set self.move here, in the position where calls the method
                        self.tracking_hand = None
                        return
                else:
                    self.l_still_cnt += 1
            else:
                self.l_track = False
                if self.tracking_hand == 'left':
                    self.react(self.word2num['Unknown'])    # do not set self.move in this method
                    self.arm.prepare()
                    self.move = False       # set self.move here, in the position where calls the method
                    self.tracking_hand = None
                    return
            # right hand detected
            if r_move_dist != -1:
                if r_move_dist > move_thrd:
                    self.r_track = False
                    if self.tracking_hand == 'right':
                        self.react(self.word2num['Unknown'])    # do not set self.move in this method
                        self.arm.prepare()
                        self.move = False       # set self.move here, in the position where calls the method
                        self.tracking_hand = None
                        return
                else:
                    self.r_still_cnt += 1
            else:
                self.r_track = False
                if self.tracking_hand == 'right':
                    self.react(self.word2num['Unknown'])    # do not set self.move in this method
                    self.arm.prepare()
                    self.move = False       # set self.move here, in the position where calls the method
                    self.tracking_hand = None
                    return
            # left hand move
            if self.l_still_cnt >= 5 and self.r_still_cnt < 5:
                # at this time tracking_hand cannot be 'right'
                centerx, centery = new_pose[7]  # l_wri position
                minx = max(0,                int(centerx - 2*self.limb_length))
                maxx = min(img.shape[1] - 1, int(centerx + 2*self.limb_length))
                miny = max(0,                int(centery - 2*self.limb_length))
                maxy = max(img.shape[0] - 1, int(centery + 2*self.limb_length))
                gesture = self.trigger(img[miny:maxy, minx:maxx, :])
                self.react(gesture)
                self.arm.prepare()
                self.move = False
                self.tracking_hand = None
            elif self.l_still_cnt < 5 and self.r_still_cnt >= 5:
                # at this time tracking_hand cannot be 'left'
                centerx, centery = new_pose[4]  # r_wri position
                minx = max(0,                int(centerx - 2*self.limb_length))
                maxx = min(img.shape[1] - 1, int(centerx + 2*self.limb_length))
                miny = max(0,                int(centery - 2*self.limb_length))
                maxy = max(img.shape[0] - 1, int(centery + 2*self.limb_length))
                gesture = self.trigger(img[miny:maxy, minx:maxx, :])    
                self.react(gesture)
                self.arm.prepare()
                self.move = False
                self.tracking_hand = None
            elif self.l_still_cnt >= 5 and self.r_still_cnt >= 5:
                if self.tracking_hand == 'left':
                    centerx, centery = new_pose[7]  # l_wri position
                elif self.tracking_hand == 'right':
                    centerx, centery = new_pose[4]  # r_wri position
                else:   # tracking_hand == 'both
                    centerx, centery = (new_pose[4] + new_pose[7]) / 2
                minx = max(0,                int(centerx - 2*self.limb_length))
                maxx = min(img.shape[1] - 1, int(centerx + 2*self.limb_length))
                miny = max(0,                int(centery - 2*self.limb_length))
                maxy = max(img.shape[0] - 1, int(centery + 2*self.limb_length))
                gesture = self.trigger(img[miny:maxy, minx:maxx, :])
                self.react(gesture)
                self.arm.prepare()
                self.move = False
                self.tracking_hand = None
        """        
        
        # if right hand detected
        if r_move_dist != -1 and (self.lock == 0 or not self.r_move_cnt == 0):
            if self.r_move_cnt < move_frame_thrd:
                if r_move_dist > move_thrd and new_pose[4][1] > self.pose[4][1]:
                    self.r_move_cnt += 1
                    if self.r_move_cnt == move_frame_thrd and new_pose[2][1] < new_pose[4][1]:    # moves down
                        self.random_act('right')
                else:
                    self.r_move_cnt = 0
                    # if self.move:
                    #     self.arm.prepare()
                    #     self.move = False
            else:
                if r_move_dist <= move_thrd:
                    self.r_still_cnt += 1
                #else:
                    #self.r_move_cnt = 0
                    #self.r_still_cnt = 0
                if self.r_still_cnt == still_frame_thrd:
                    centerx, centery = new_pose[4]
                    minx = max(0, int(centerx-1.5*self.limb_length))
                    maxx = min(img.shape[1]-1, int(centerx+1.5*self.limb_length))
                    miny = max(0, int(centery-1.5*self.limb_length))
                    maxy = min(img.shape[0]-1, int(centery+1.5*self.limb_length))
                    if new_pose[2][1] < new_pose[4][1]:     # wrist below shoulder
                        gesture = self.trigger(img[miny:maxy, minx:maxx, :])
                        self.react(gesture)
                        self.arm.prepare()
                        self.move = False
                    else:
                        self.r_move_cnt = 0
                        self.r_still_cnt = 0

        # if left limb detected
        if l_move_dist != -1 and (self.lock == 0 or not self.l_move_cnt == 0):
            if self.l_move_cnt < move_frame_thrd:
                if l_move_dist > move_thrd and new_pose[7][1] > self.pose[7][1]: 
                    self.l_move_cnt += 1
                    if self.l_move_cnt == move_frame_thrd and new_pose[5][1] < new_pose[7][1]:
                        self.random_act('left')
                else:
                    self.l_move_cnt = 0
                    # if self.move:
                    #     self.arm.prepare()
                    #     self.move = False
            else:
                if l_move_dist <= move_thrd:
                    self.l_still_cnt += 1
                #else:
                #    self.l_move_cnt = 0
                #    self.l_still_cnt = 0
                if self.l_still_cnt == still_frame_thrd:
                    centerx, centery = new_pose[7]
                    minx = max(0, int(centerx-1.5*self.limb_length))
                    maxx = min(img.shape[1]-1, int(centerx+1.5*self.limb_length))
                    miny = max(0, int(centery-1.5*self.limb_length))
                    maxy = max(img.shape[0]-1, int(centery+1.5*self.limb_length))
                    if new_pose[5][1] < new_pose[7][1]:     # wrist below shoulder
                        gesture = self.trigger(img[miny:maxy, minx:maxx, :])
                        self.react(gesture)
                        self.arm.prepare()
                        self.move = False
                    else:
                        self.r_move_cnt = 0
                        self.r_still_cnt = 0
        
        self.pose = new_pose
        #print('ID {}: r_move_cnt {}, r_still_cnt {}, l_move_cnt {}, l_still_cnt {}'.format(self.id, self.r_move_cnt, self.r_still_cnt, self.l_move_cnt, self.l_still_cnt))


    def random_act(self, hand):
        if self.lock != 0:
            return
        if hand == 'right': # stop tracking left hand
            self.l_move_cnt = 0 
            self.l_still_cnt = 0
        else: # stop tracking right hand
            self.r_move_cnt = 0
            self.r_still_cnt = 0
        self.lock = 1
        self.arm.prepare2()
        act = self.arm.st_jd_b()        # rock-paper-scissors
        if act == 0:
            self.act = 0
        elif act == 1:
            self.act == 2
        elif act == 2:
            self.act == 5
        self.move = True

    def react(self, gesture):
        """
        react to player's gesture
        support: Fist, Two, Five, ILY, Rock, Insult, Thumb up, Unknown
        """
        if gesture in (0,2,5) and gesture == self.act:
            print('Draw')
        elif (gesture == 2 and self.act == 0) or (gesture == 0 and self.act == 5) or (gesture == 5 and self.act == 2):
            print('You Lose!')
            # act(V-shape)
            # img = cv2.imread('img/lose.jpg')
            # cv2.imshow('Win or Lose', img)
            # cv2.waitKey(1000)
            #time.sleep(3)
        elif (gesture == 2 and self.act == 5) or (gesture == 0 and self.act == 2) or (gesture == 5 and self.act == 0):
            print('You Win!')
            # act(thumb-up)
            # img = cv2.imread('img/win.jpg')
            # cv2.imshow('Win or Lose', img)
            # cv2.waitKey(1000)
            #time.sleep(3)
        elif gesture == 4 or gesture == 3:      # Rock or ILY
            print("Yo-yo!!")
            # act(rock)
        elif gesture == 1:                      # Insult
            print("No-no-no!!")
            # act(random(insult or no-no-no))
        else:                                   # Unknown or None
            print("I don't know what you mean...")
            # act(don-know)

    def trigger(self, img):
        """
        send to baidu an image containing a gesture and react
        """
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
            try:
                gesture = self.get_gesture(content)
            except Exception as e:
                print(e)
                gesture = None
        else: 
            gesture = None
            print('No response')
        self.lock = 0
        self.arm.prepare()
        return gesture

    def get_gesture(self, response):
        """
        0-fist, 2-scissor, 5-paper, "Rock":4, "ILY":3, "Insult":1, "Thumb up":6, "Unknown":7
        """
        output_number = None
        output_cls = None
        response = eval(response)       # bytes to str to dict
        print("response from baidu:", response) 
        results = response['result']        
        for res in results:
            classname = res['classname']
            print('your hand:', classname)
            if classname in self.word2num.keys():
                output_number = self.word2num[classname]
                output_cls = classname
        if output_number == None:
            print("No gesture recognized!!!")
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
                #print('ID {} detected'.format(pose.id))
                continue
            # call stateMachine methods
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
    parser.add_argument('--checkpoint-path', type=str, default='/media/bob-lytton/MyData/repos/torch_pose/checkpoint_iter_370000.pth', help='path to the checkpoint')
    parser.add_argument('--height-size', type=int, default=256, help='network input layer height size')
    parser.add_argument('--video', type=str, default='0', help='path to video file or camera id')
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

    
    
    angles=[60,10,15,15,55,90,0,0,90,30,1] #angles is the vector representing the angles of different servos []
    control_index=[90,10.5,11,90,1,1,1,1,1,30,1]
    arm = Robotarm(args.port, args.rate, angles)

    try:
        # while 1:
        #     arm.control(control_index)
        #     data = arm.readline()
        #     if data!=b'':
        #         print(data[0])
        #         if data[0]==98:
        #             break
        # arm.prepare()
        print('start')
        run_demo(net, frame_provider, args.height_size, args.cpu, args.track_ids, arm)
    except KeyboardInterrupt:
        final_index=[90,10.5,7,90,0,0,0,0,0,30,1]
        arm.control(final_index)
        arm.close()
        print('close')
