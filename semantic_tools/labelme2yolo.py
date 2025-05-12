import argparse
import json
import matplotlib.pyplot as plt
import skimage.io as io
import cv2
#from labelme import utils
from image import img_b64_to_arr
#
import numpy as np
import glob
import PIL.Image
import PIL.ImageDraw
import shutil
from sklearn.model_selection import train_test_split
import yaml

import os


class labelme2yolo(object):
    def __init__(self, labelme_json=[], output_dir="."):
        '''
        Args: labelme_json: paths of labelme json files
        : save_json_path: saved path 
        '''
        self.labelme_json = labelme_json
        self.output_dir = output_dir
        self.dataset = []
        self.categories = []
        # self.data_coco = {}
        self.label = []
        self.annID = 1
        self.height = 1200
        self.width = 1920
        self.val_ratio = 0.2

  
    
    def data_transfer(self):

        N = len(self.labelme_json)
        for num, json_file in enumerate(self.labelme_json):
            with open(json_file, 'r') as fp:
                item = {}
                item['annotations'] = []
                data = json.load(fp)
                (prefix, res) = os.path.split(json_file)
                (file_name, extension) = os.path.splitext(res)

                item['image'] = self.image(data, num, file_name, prefix)
                for shapes in data['shapes']:
                    
                    label = shapes['label']
                    if label not in self.label:
                        categorie = self.categorie(label)
                        self.categories.append(categorie)
                        self.label.append(label)
                    points = shapes['points']
                    annotation = self.annotation(points, label, num)
                    item['annotations'].append(annotation)
                    self.annID += 1
                self.dataset.append(item)
            # if num > 20:
            #     break
            print(f"we are processing the {num}/{N} data")


        self.train_set, self.val_set = train_test_split(self.dataset, test_size=self.val_ratio, random_state=42, shuffle=False)


    def normalize_points(self, points):
        for point in points:
            point[0] = point[0]/self.width
            point[1] = point[1]/self.width
        return points
     

    def image(self, data, num, file_name, dir):
        image = {}
        #img = img_b64_to_arr(data['imageData'])

    
        #img = None
        image['id'] = int(num)
        image['file_name'] = file_name
        image['path'] = os.path.join(dir, data['imagePath'])


        return image

    def categorie(self, label):
        categorie = {}
        categorie['id'] = int(len(self.label))
        categorie['label'] = label

        return categorie

    def annotation(self, points, label, num):
        annotation = {}
        # TODO: check the segementation result:OK
        annotation['detection'] = list(map(float, self.getbbox(points)))
        annotation['detection'][0] = annotation['detection'][0]/self.width
        annotation['detection'][1] = annotation['detection'][1]/self.height
        annotation['detection'][2] = annotation['detection'][2]/self.width
        annotation['detection'][3] = annotation['detection'][3]/self.height

        # annotation['segmentation'] = [
        #     [x, y, x+w, y, x+w, y+h, x, y+h]]  # at least 6 points
        annotation['segmentation'] = list(np.asarray(self.normalize_points(points)).flatten())
        annotation['category_id'] = self.getcatid(label) 
        annotation['label'] = label
        annotation['id'] = int(self.annID)
        # add area info
        # the area is not used for detection
        return annotation
    

    def getcatid(self, label):
        for categorie in self.categories:
            if label == categorie['label']:
                return categorie['id']
            # if label[1]==categorie['name']:
            #     return categorie['id']
        return -1

    def getbbox(self, points):
        # img = np.zeros([self.height,self.width],np.uint8)
        # cv2.polylines(img, [np.asarray(points)], True, 1, lineType=cv2.LINE_AA)
        # cv2.fillPoly(img, [np.asarray(points)], 1)
        polygons = points
        mask = self.polygons_to_mask([self.height, self.width], polygons)
        return self.mask2box(mask)

    def mask2box(self, mask):
        # np.where(mask==1)
        index = np.argwhere(mask == 1)
        rows = index[:, 0]
        clos = index[:, 1]

        left_top_y = np.min(rows)  # y
        left_top_x = np.min(clos)  # x

        right_bottom_y = np.max(rows)
        right_bottom_x = np.max(clos)

        w = right_bottom_x-left_top_x
        h = right_bottom_y-left_top_y
        cx = left_top_x + w/2
        cy = left_top_y + h/2

        # [cx,cy,w,h] for coco box format
        return [cx, cy, w, h]



    def polygons_to_mask(self, img_shape, polygons):
        mask = np.zeros(img_shape, dtype=np.uint8)
        mask = PIL.Image.fromarray(mask)
        xy = list(map(tuple, polygons))
        PIL.ImageDraw.Draw(mask).polygon(xy=xy, outline=1, fill=1)
        mask = np.array(mask, dtype=bool)
        return mask

    def data_save(self, itmes, dirname, type):

        if not os.path.exists(os.path.join(self.output_dir, type)):
            os.makedirs(os.path.join(self.output_dir, type))

        if not os.path.exists(os.path.join(self.output_dir, type, 'images')):
            os.makedirs(os.path.join(self.output_dir, type, 'images'))

        image_dir = os.path.join(self.output_dir, type, 'images', dirname)
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)

        if not os.path.exists(os.path.join(self.output_dir, type, 'labels')):
            os.makedirs(os.path.join(self.output_dir, type, 'labels'))

        label_dir = os.path.join(self.output_dir, type, 'labels', dirname)
        if not os.path.exists(label_dir):
            os.makedirs(label_dir)

        
        for item in itmes:
            image_name = item['image']['file_name']
            image_path = item['image']['path']
            label_filename = f"{image_name}.txt"

            img = PIL.Image.open(image_path)
            img = img.convert("RGB")
            img.save(os.path.join(image_dir, image_name+'.jpg'), "JPEG")


            open(os.path.join(label_dir, label_filename), "w").close()

            annotations = item['annotations']
            for annotation in annotations:
                category_id = annotation['category_id']
                anno = list(map(str, annotation[type]))
                line = str(category_id) + " " + " ".join(anno)

                with open(os.path.join(label_dir, label_filename), "a") as f:
                    f.write(line+"\n")

    def create_yaml(self, dir):
        
        
        data = {
            'path': '../datasets/coco8-seg',
            'train': 'images/train',
            'val': 'images/val',
            'test': None,
            'names': {}
        }

        for categore in self.categories:
            data['names'][categore['id']] = categore['label']


        with open(os.path.join(dir, 'data.yaml'), 'w') as f:
            yaml.dump(data, f, sort_keys=False)

    def run(self):
        self.data_transfer()

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

     
  
        
        self.data_save(self.train_set, 'train', 'segmentation')
        self.data_save(self.val_set, 'val', 'segmentation')

        self.create_yaml(os.path.join(self.output_dir, 'segmentation'))
        
        self.data_save(self.train_set, 'train', 'detection')
        self.data_save(self.val_set, 'val', 'detection')

        self.create_yaml(os.path.join(self.output_dir, 'detection'))



# type check when save json files


class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(MyEncoder, self).default(obj)


# you need to modify the path according to your environment
labelme_json = glob.glob('/media/spiderman/zhipeng_usb/datasets/BotanicGarden/1005-07/imagezip/semantic_1005_07/07/*.json')

convert = labelme2yolo(labelme_json, '/media/spiderman/zhipeng_8t/datasets/BotanicGarden/1005-07/imagezip/YOLO_1005_07')


convert.run()