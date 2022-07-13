import numpy as np
import time
import cv2
import torch
import math
from op_models.with_mobilenet import PoseEstimationWithMobileNet
from modules.keypoints import extract_keypoints, group_keypoints
from modules.load_state import load_state
from modules.pose import Pose, track_poses
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective
from models.common import DetectMultiBackend

MODEL_DIR = 'checkpoints/checkpoint_iter_1600-2.pth'
YOLO_DIR = 'checkpoints/Nsz624bs128ep120.pt'


def normalize(img, img_mean, img_scale):
    img = np.array(img, dtype=np.float32)
    img = (img - img_mean) * img_scale
    return img


def pad_width(img, stride, pad_value, min_dims):
    h, w, _ = img.shape
    h = min(min_dims[0], h)
    min_dims[0] = math.ceil(min_dims[0] / float(stride)) * stride
    min_dims[1] = max(min_dims[1], w)
    min_dims[1] = math.ceil(min_dims[1] / float(stride)) * stride
    pad = []
    pad.append(int(math.floor((min_dims[0] - h) / 2.0)))
    pad.append(int(math.floor((min_dims[1] - w) / 2.0)))
    pad.append(int(min_dims[0] - h - pad[0]))
    pad.append(int(min_dims[1] - w - pad[1]))
    padded_img = cv2.copyMakeBorder(img, pad[0], pad[2], pad[1], pad[3],
                                    cv2.BORDER_CONSTANT, value=pad_value)
    return padded_img, pad


def infer_fast(net, img, net_input_height_size, stride, upsample_ratio, cpu,
               pad_value=(0, 0, 0), img_mean=np.array([128, 128, 128], np.float32), img_scale=np.float32(1 / 256)):
    height, width, _ = img.shape
    scale = net_input_height_size / height

    scaled_img = cv2.resize(img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
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


class ObjectTracker:
    global MODEL_DIR

    def __init__(self):
        # construct the argument parse and parse the arguments

        self.device = select_device('')
        yolo = DetectMultiBackend(YOLO_DIR, device=self.device, dnn=False, data=None, fp16=False)
        self.stride, self.names, pt = yolo.stride, yolo.names, yolo.pt
        self.imgsz = check_img_size((960, 720), s=self.stride)

        yolo.warmup(imgsz=(1, 3, *self.imgsz))
        self.yolo = yolo

        self.model = PoseEstimationWithMobileNet(num_heatmaps=6, num_pafs=16)
        checkpoint = torch.load(MODEL_DIR, map_location='cpu')
        load_state(self.model, checkpoint)
        self.model = self.model.eval()

        self.stride = 8
        self.upsample_ratio = 4
        self.num_keypoints = Pose.num_kpts
        self.previous_poses = []
        self.input_height_size = 256

        print("model loaded...")
        self.rect = ()

    def get_sub_frame(self, frame):
        ENLARGEMENT = 1.5
        img = letterbox(frame, self.imgsz, stride=self.stride)[0]
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.yolo.fp16 else img.float()
        img /= 255
        if len(img.shape) == 3:
            img = img[None]
        print(img.shape)
        pred = self.yolo(img)
        pred = non_max_suppression(pred, conf_thres=0.3, iou_thres=0.45, classes=None, max_det=1)
        det = pred[0]
        im0 = frame.copy()
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        annotator = Annotator(im0, line_width=3, example=str(self.names))

        crop_top_left = 0, 0
        size = frame.shape[0], frame.shape[1]

        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            *xyxy, conf, cls = det[0]
            c = int(cls)  # integer class
            label = f'{self.names[c]} {conf:.2f}'
            annotator.box_label(xyxy, label, color=colors(c, True))
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()

            crop_top_left = (xywh[1] - xywh[3] / 2 * ENLARGEMENT, xywh[0] - xywh[2] / 2 * ENLARGEMENT)
            img = img[crop_top_left[0] : crop_top_left[0] + xywh[3] * ENLARGEMENT,
                      crop_top_left[1] : crop_top_left[1] + xywh[2] * ENLARGEMENT]
            size = xywh[2]*ENLARGEMENT, xywh[3]*ENLARGEMENT

        return img, crop_top_left, size

    def get_rect(self, frame):
        t1 = time.time()

        img, crop_top_left, size = self.get_sub_frame(frame)

        heatmaps, pafs, scale, pad = infer_fast(self.model, img, self.input_height_size, self.stride,
                                                self.upsample_ratio, True)

        total_keypoints_num = 0
        all_keypoints_by_type = []
        for kpt_idx in range(self.num_keypoints):
            total_keypoints_num += extract_keypoints(heatmaps[:, :, kpt_idx], all_keypoints_by_type,
                                                     total_keypoints_num)

        pose_entries, all_keypoints = group_keypoints(all_keypoints_by_type, pafs)
        for kpt_id in range(all_keypoints.shape[0]):
            all_keypoints[kpt_id, 0] = (all_keypoints[kpt_id, 0] * self.stride / self.upsample_ratio - pad[1]) / scale
            all_keypoints[kpt_id, 1] = (all_keypoints[kpt_id, 1] * self.stride / self.upsample_ratio - pad[0]) / scale

        current_poses = []
        for n in range(len(pose_entries)):
            if len(pose_entries[n]) == 0:
                continue
            pose_keypoints = np.ones((self.num_keypoints, 2), dtype=np.int32) * -1
            for kpt_id in range(self.num_keypoints):
                if pose_entries[n][kpt_id] != -1.0:  # keypoint was found
                    pose_keypoints[kpt_id, 0] = int(all_keypoints[int(pose_entries[n][kpt_id]), 0])
                    pose_keypoints[kpt_id, 1] = int(all_keypoints[int(pose_entries[n][kpt_id]), 1])
            pose = Pose(pose_keypoints, pose_entries[n][5])
            current_poses.append(pose)

        for p in current_poses:
            p.bbox[0] += crop_top_left[0]
            p.bbox[1] += crop_top_left[1]
            for kpt in p.keypoints:
                kpt[0] += crop_top_left[1]
                kpt[1] += crop_top_left[0]

        track_poses(self.previous_poses, current_poses, smooth=True)

        if len(current_poses) != 0:
            self.previous_poses = current_poses
        if len(self.previous_poses) == 0:
            return frame, None

        pose = max(self.previous_poses, key=lambda x: x.confidence)
        pose.draw(frame)
        cv2.rectangle(frame, (pose.bbox[0], pose.bbox[1]),
                      (pose.bbox[0] + pose.bbox[2], pose.bbox[1] + pose.bbox[3]), (0, 255, 0))
        cv2.putText(frame, 'id: {}'.format(pose.id), (pose.bbox[0], pose.bbox[1] - 16),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))

        def dist(x, y):
            return math.sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2)

        bbox_x = pose.bbox[0] + pose.bbox[2]/2
        bbox_y = pose.bbox[1] + pose.bbox[3]/2
        bbox_wh = (pose.bbox[2], pose.bbox[3])
        # from atop, direction is positive if the posecard is turning counter-clockwise
        tilt_dir = 1 if dist(pose.keypoints[0], pose.keypoints[1]) > dist(pose.keypoints[3], pose.keypoints[4]) else -1

        t2 = time.time()
        print(f'{MODEL_DIR}: {1 / (t2 - t1)}fps')

        return frame, [bbox_x, bbox_y, bbox_wh, tilt_dir]
