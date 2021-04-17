import sys
import socket
import numpy as np
from chainer import cuda
from chainer import serializers

sys.path.append('.')
from model import FCN

class Predictor(object):
    def __init__(self, port=19000, max_recv_size=4096):
        self.gpu = True
        self.port = port
        self.max_recv_size = max_recv_size
        self.sock = socket.socket()
        self.sock.bind(('', self.port))
        self.image_width = 256
        self.image_height = 256
        self.image_channels = 3
        self.image_size = (self.image_height, self.image_width, self.image_channels)
        self.classes = 21
        self.model = FCN(self.classes)
        serializers.load_npz('weight/chainer_fcn.weight', self.model)
        if self.gpu:
            cuda.get_device(0).use()
            self.model.to_gpu()

    def __call__(self):
        while True:
            self.sock.listen(5)
            c, addr = self.sock.accept()
            data = bytes()
            while True:
                recv_data = c.recv(self.max_recv_size)
                if len(recv_data) == 0:
                    break
                data += recv_data
            c.close()
            image_data = np.fromstring(data, dtype=np.uint8)
            print len(data)
            if image_data.shape != (self.image_width * self.image_height * self.image_channels, ):
                continue
            image_data = np.reshape(image_data, self.image_size)
            image_array = image_data.transpose(2, 0, 1).astype(np.float32)
            image_array = np.expand_dims(image_array, axis=0)
            if self.gpu:
                image_array = cuda.to_gpu(image_array)
            pred = self.model(image_array).data[0].argmax(axis=0).astype(np.uint16)
            if self.gpu:
                pred = cuda.to_cpu(pred)
            c, addr = self.sock.accept()
            c.sendall(pred.tostring())
            c.close()

if __name__ == '__main__':
    predictor = Predictor()
    predictor()

