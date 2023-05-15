#!/usr/bin/env python

"""
Tensorflow Faster R-CNN object detection model used in ShapeShifter (stop sign attack)
Ref: https://arxiv.org/abs/1804.05810
"""
from __future__ import print_function

import os
import tarfile
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import tensorflow as tf
import six.moves.urllib as urllib

from achilles.autoagents.modules.obstacle_detector import ObstacleDetector, Obstacle
from object_detection.utils import label_map_util
# from object_detection.utils.visualization_utils import visualize_boxes_and_labels_on_image_array
# from object_detection.core import target_assigner

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib import cm


class FasterRCNNInception(ObstacleDetector):

    """
    TODO: document me!
    """

    def __init__(self):

        self.achilles_root = os.environ.get('ACHILLES_ROOT')
        self.log_dir = os.environ.get('ACHILLES_LOGDIR', '/tmp/svl_log')
        DATA_DIR = os.path.join(self.achilles_root, 'achilles/data')

        # Model preparation
        DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
        MODEL_NAME = 'faster_rcnn_inception_v2_coco_2017_11_08'
        MODEL_TAR = MODEL_NAME + '.tar.gz'
        LOCAL_MODEL_DIR = os.path.join(DATA_DIR, MODEL_NAME)
        LOCAL_MODEL_TAR = os.path.join(DATA_DIR, MODEL_TAR)
        REMOTE_MODEL_TAR = DOWNLOAD_BASE + MODEL_TAR

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = os.path.join(LOCAL_MODEL_DIR, 'frozen_inference_graph.pb')

        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = os.path.join(DATA_DIR, 'mscoco_label_map.pbtxt')

        NUM_CLASSES = 90
        VICTIM_CLASS = 13 # victim class: stop sign

        if not os.path.exists(LOCAL_MODEL_DIR):
            # Download Model
            opener = urllib.request.URLopener()
            opener.retrieve(REMOTE_MODEL_TAR, LOCAL_MODEL_TAR)
            tar_file = tarfile.open(LOCAL_MODEL_TAR)
            for file in tar_file.getmembers():
                file_name = os.path.basename(file.name)
                if 'frozen_inference_graph.pb' in file_name:
                    tar_file.extract(file, DATA_DIR)
            print(f"Downloaded model at {LOCAL_MODEL_DIR}")

        # Loading label map
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        # size of the perturbation we want to generate
        psize_h = 1080
        psize_w = 1920

        # Loading the model for inference
        inference_graph = tf.Graph()
        with inference_graph.as_default():
            image_tensor = tf.placeholder(tf.float32, shape=(None, psize_h, psize_w, 3), name='image_tensor')
            inference_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                inference_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(inference_graph_def, name='',
                        input_map={'Preprocessor/map/TensorArrayStack/TensorArrayGatherV3:0':image_tensor})

        self.inference_sess = tf.Session(graph=inference_graph)
        self.input_tensor = inference_graph.get_tensor_by_name('image_tensor:0')
        self.output_tensors = [
                inference_graph.get_tensor_by_name('SecondStagePostprocessor/Reshape_4:0'),
                inference_graph.get_tensor_by_name('SecondStagePostprocessor/convert_scores:0') ]

        # A list of color tuples for annotation
        cmap = plt.get_cmap('Spectral')
        self.colors = []
        for i in range(NUM_CLASSES):
            clr = cmap(i/NUM_CLASSES)
            clr = [int(255 * clr[i]) for i in range(0, len(clr) - 1)]
            self.colors.append(tuple(clr))

    def run_step(self, input_data, timestamp):

        img = input_data['main_camera']
        fid = input_data['frame_id']
        bboxes, scores, classes = self._inference(img)

        obstacles = []
        for i, clss in enumerate(classes):
            obs = Obstacle(bboxes[i], coco_type=clss, score=scores[i])
            obstacles.append(obs)

        if len(classes):
            img_viz = img.copy()
            for i, clss in enumerate(classes):
                if clss in self.category_index.keys():
                    bbox = bboxes[i]
                    score = scores[i]
                    label = "{}: {:.2f}".format(self.category_index[clss]['name'], score)
                    img_viz = self._plot_one_box(bbox, img_viz, self.colors[clss], label)
        else:
            img_viz = img.copy()

        Image.fromarray(img_viz).save(
                os.path.join(self.log_dir, f'frames/main_camera_anno-{fid}.png'))

        return obstacles

    def _inference(self, img):

        feed_dict = { self.input_tensor: np.expand_dims(img, axis=0) }
        bboxes, scores = self.inference_sess.run(self.output_tensors, feed_dict)

        # Apply NMS, and need to get rid of the batch dimension
        output_bboxes, output_scores, output_lables = self._get_detection_results(bboxes[0], scores[0], min_threshold=0.6)

        return output_bboxes, output_scores, output_lables

    def _get_detection_results(self, raw_bboxes, raw_scores, min_threshold=0.6):
        bboxes = []
        scores = []
        labels = []
        for i in range(raw_scores.shape[0]):
            label = raw_scores[i].argmax()
            if label != 0 and raw_scores[i,label] >= min_threshold:
                bboxes.append(raw_bboxes[i, label])
                scores.append(raw_scores[i, label])
                labels.append(label)

        final_bboxes = []
        final_scores = []
        final_labels = []
        for label in range(1, 91):
            if label in labels:
                bboxes_nms = []
                scores_nms = []
                for i in range(len(labels)):
                    if labels[i] == label:
                        bboxes_nms.append(bboxes[i])
                        scores_nms.append(scores[i])
                picked_bboxes, picked_scores = self._nms(bboxes_nms, scores_nms, min_threshold)

                final_bboxes += picked_bboxes
                final_scores += picked_scores
                final_labels += [label for _ in range(len(picked_scores))]

        return np.array(final_bboxes), np.array(final_scores), np.array(final_labels)

    @staticmethod
    def _plot_one_box(box, im, color=(128, 128, 128), label=None, line_thickness=None):
        # PIL bbox takes the fomat of [ w, h, w, h] rather than [ h, w, h, w ]
        new_box = [box[1], box[0], box[3], box[2]]
        im = Image.fromarray(im)
        draw = ImageDraw.Draw(im)
        line_thickness = line_thickness or max(int(min(im.size) / 200), 2)
        draw.rectangle(new_box, width=line_thickness, outline=color)
        if label:
            font = ImageFont.truetype(fm.findfont(fm.FontProperties(family='DejaVu Sans')), size=max(round(max(im.size) / 60), 12))
            txt_width, txt_height = font.getsize(label)
            draw.rectangle([new_box[0], new_box[1] - txt_height + 4, new_box[0] + txt_width, new_box[1]], fill=color)
            draw.text((new_box[0], new_box[1] - txt_height + 1), label, fill=(255, 255, 255), font=font)
        return np.asarray(im)
