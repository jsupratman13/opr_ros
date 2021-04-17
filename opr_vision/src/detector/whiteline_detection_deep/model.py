# White Line Detection Network
#
# Reference: https://github.com/kiyoshiiriemon/chainer_wrapper_cpp

import chainer
import numpy as np
import glob
import chainer
import chainer.functions as F
import chainer.links as L
from chainer.backends import cuda 

from chainer import training
from chainer.training import extensions

from chainer import serializers

chainer.config.train = False
chainer.config.enable_backprop = False

class MLP(chainer.Chain):

    def __init__(self, n_units1, n_units2):
        super(MLP, self).__init__()
        with self.init_scope():
            initializer = chainer.initializers.HeNormal()
            self.l1 = L.Convolution2D(3, n_units1, 5, stride=1, pad=2, initialW=initializer)
            self.l3 = L.Convolution2D(None, 1, 1)

    def __call__(self, *args):
        x = args[0]
        self.y = self.forward(x)
        return self.y

    def forward(self, x):
        h1 = F.relu(self.l1(x))
        h3 = self.l3(h1)
        return h3

    def lossfun(self, y, t):
        return (y * (t - (y >= 0)) - F.log1p(F.exp(-F.abs(y))))

