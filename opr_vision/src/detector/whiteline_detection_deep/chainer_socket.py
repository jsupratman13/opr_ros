import sys
import argparse
import socket
import numpy as np
from chainer import cuda
from chainer import serializers

sys.path.append('.')
from model import MLP

def parser():
    usage = 'Usage: python {} [--thre] [--npz] [--help]'.format(__file__)
    argparser = argparse.ArgumentParser(usage=usage)
    argparser.add_argument('-t','--thre',dest='threshold',type=int,default=-3,help='whiteline detection threshold. [default=-3]')
    argparser.add_argument('-w','--weight',dest='weight',type=str,default='wl_npz/wl_2layer_20190622_142021.npz',help='whiteline detection weight.')
    args = argparser.parse_args()
    return args

class Predictor(object):
    def __init__(self, port=19000, max_recv_size=4096):
        self.gpu = True
        self.port = port
        self.max_recv_size = max_recv_size
        self.sock = socket.socket()
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        self.image_width = 320
        self.image_height = 240
        self.image_channels = 3
        self.image_size = (self.image_height, self.image_width, self.image_channels)
        self.model = MLP(4, 2)
        serializers.load_npz(WEIGHT, self.model)
        self.threshold = THRESHOLD
        if self.gpu:
            cuda.get_device(0).use()
            self.model.to_gpu()

    def __call__(self):
        try:
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
                image_array = image_data.transpose(2, 0, 1).astype(np.float32) / 255.
                image_array = np.expand_dims(image_array, axis=0)
                if self.gpu:
                    image_array = cuda.to_gpu(image_array)
                pred = self.model(image_array).data[0][0] > self.threshold
                if self.gpu:
                    pred = cuda.to_cpu(pred)
                pred = pred * 255.
                pred = pred.astype(np.uint8)
                c, addr = self.sock.accept()
                c.sendall(pred.tostring())
                c.close()
        except (KeyboardInterrupt, SystemExit):
            self.sock.shutdown(socket.SHUT_RDWR)

if __name__ == '__main__':
    args = parser()
    THRESHOLD = args.threshold
    WEIGHT = args.weight
    predictor = Predictor()
    predictor()

