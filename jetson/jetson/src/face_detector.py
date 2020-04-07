import cv2
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
import math
from sklearn.decomposition import PCA
import os
import time
import glob
import face_detection.src.facenet as facenet
import face_detection.src.align.detect_face as detect_face
from json_socket import Server
import msgpack

BIND_IP = "127.0.0.1"
BIND_PORT = 9879 

class FaceDetector:

    def __init__(self):
        self.socket = Server(BIND_IP, BIND_PORT)
        self.socket.accept()

    def load_model(self, pb_path, image_size=(160,160)):
        tf.reset_default_graph()

        single_image = tf.placeholder(tf.int32, (None,None,3))
        float_image = tf.cast(single_image, tf.float32)
        float_image = float_image / 255
        batch_image = tf.expand_dims(float_image, 0)
        resized_image = tf.image.resize(batch_image, image_size)

        phase_train = tf.placeholder_with_default(False, shape=[])

        input_map = {'image_batch':resized_image, 'phase_train':phase_train}
        model = facenet.load_model(pb_path, input_map)

        embeddings = tf.get_default_graph().get_tensor_by_name("embeddings:0")

        return single_image, embeddings


    def load_image(self, image_path):
        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        return image


    def calc_distance(self, embedding1, embedding2):
        # Euclidian distance
        diff = np.subtract(embedding1, embedding2)
        dist = np.sum(np.square(diff),0)

        return dist


    def crop_faces(self, image, pnet, rnet, onet):
        minsize = 20 # minimum size of face
        threshold = [0.6, 0.7, 0.7] # three steps's threshold
        factor = 0.709 # scale factor

        margin = 44
        image_size = 160
        h,w,_ = np.shape(image)

        bounding_boxes, points = detect_face.detect_face(image, minsize, pnet, rnet, onet, threshold, factor)

        faces = []
        for box in bounding_boxes:
            box = np.int32(box)
            bb = np.zeros(4, dtype=np.int32)
            bb[0] = np.maximum(box[0]-margin/2, 0)
            bb[1] = np.maximum(box[1]-margin/2, 0)
            bb[2] = np.minimum(box[2]+margin/2, w)
            bb[3] = np.minimum(box[3]+margin/2, h)
            cropped = image[bb[1]:bb[3], bb[0]:bb[2],:]
            scaled = cv2.resize(cropped, (image_size, image_size), interpolation=cv2.INTER_LINEAR)

            faces.append(scaled)

        return faces, bounding_boxes

    def check_face(self, frame):
        tf.reset_default_graph()
        
        model_path = os.path.dirname(os.path.abspath(__file__)) + "/face_detection/models/20180402-114759.pb"
        single_image, embeddings = self.load_model(model_path)

        sess = tf.Session()
        pnet, rnet, onet = detect_face.create_mtcnn(sess,None)


        path_me = glob.glob("./data/faces/lsh/*")

        embed_me = []

        for path in path_me:
            img = self.load_image(path)
            result = sess.run(embeddings, feed_dict={single_image:img})
            result = result[0]
            embed_me.append(result)

        embed_me = np.array(embed_me)

        path_other1 = glob.glob("./data/faces/other1/*")
        embed_other1 = []

        for path in path_other1:
            img = self.load_image(path)
            result = sess.run(embeddings, feed_dict={single_image:img})
            result = result[0]
            embed_other1.append(result)

        embed_other1 = np.array(embed_other1)

        path_other2 = glob.glob("./data/faces/other2/*")

        embed_other2 = []

        for path in path_other2:
            img = self.load_image(path)
            result = sess.run(embeddings, feed_dict={single_image:img})
            result = result[0]
            embed_other2.append(result)

        embed_other2 = np.array(embed_other2)

        all_embeddings = np.concatenate((embed_me, embed_other1, embed_other2), axis=0)

        pca = PCA(n_components=2)

        pca.fit(all_embeddings)

        xy_me = pca.transform(embed_me)
        xy_other1 = pca.transform(embed_other1)
        xy_other2 = pca.transform(embed_other2)

        ax = plt.figure()

        sc1 = plt.scatter(xy_me[:,0], xy_me[:,1], color = (1,0,0))
        sc2 = plt.scatter(xy_other1[:,0], xy_other1[:,1], color=(0,1,0.4))
        sc3 = plt.scatter(xy_other2[:,0], xy_other2[:,1], color=(0,0,1))

        plt.legend([sc1,sc2,sc3], ["me", "other1", "other2"], loc="upper right")

        cv_frame = cv2.resize(cv_frame,(400,225))

        boxed_frame = cv_frame.copy()
        faces, bounding_boxes = self.crop_faces(image_frame, pnet, rnet, onet)

        for box in bounding_boxes:
            box = np.int32(box)
            p1 = (box[0], box[1])
            p2 = (box[2], box[3])
            result_frame = sess.run(embeddings, feed_dict={single_image:image_frame})
            result_frame = result_frame[0]

            distance_th = 1.0

            distance1 = self.calc_distance(embed_me[0], result_frame)
            distance2 = self.calc_distance(embed_me[3], result_frame)

            avg_distance = (distance1 + distance2) / 2
            if(avg_distance < distance_th):
                cv2.rectangle(boxed_frame, p1, p2, color=(0,255,0))
            else:
                cv2.rectangle(boxed_frame, p1, p2, color=(0,0,255))

                return boxed_frame

if __name__=="__main__":
    fd = FaceDetector()
    while True:
        recv_data = fd.socket.recv()
        if not recv_data: # recv_data == None, So break
            continue 
        
        video_cap = msgpack.unpackb(recv_data)

        send_image = video_cap

        fd.socket.send(send_image)

        time.sleep(1.0)

    fd.socket.close()
