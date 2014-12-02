from __future__ import print_function

from collections import Counter, deque
import sys
import time

import numpy as np

try:
    from sklearn import neighbors, svm
    HAVE_SK = True
except ImportError:
    HAVE_SK = False

from common import *
from myo_raw import MyoRaw

SUBSAMPLE = 3
K = 15

class NNClassifier(object):
    '''A wrapper for sklearn's nearest-neighbor classifier that stores
    training data in vals0, ..., vals9.dat.'''

    def __init__(self):
        for i in range(10):
            with open('vals%d.dat' % i, 'ab') as f: pass
        self.read_data()

    def store_data(self, cls, vals):
        with open('vals%d.dat' % cls, 'ab') as f:
            f.write(pack('8H', *vals))

        self.train(np.vstack([self.X, vals]), np.hstack([self.Y, [cls]]))

    def read_data(self):
        X = []
        Y = []
        for i in range(10):
            X.append(np.fromfile('vals%d.dat' % i, dtype=np.uint16).reshape((-1, 8)))
            Y.append(i + np.zeros(X[-1].shape[0]))

        self.train(np.vstack(X), np.hstack(Y))

    def train(self, X, Y):
        self.X = X
        self.Y = Y
        if HAVE_SK and self.X.shape[0] >= K * SUBSAMPLE:
            self.nn = neighbors.KNeighborsClassifier(n_neighbors=K, algorithm='kd_tree')
            self.nn.fit(self.X[::SUBSAMPLE], self.Y[::SUBSAMPLE])
        else:
            self.nn = None

    def nearest(self, d):
        dists = ((self.X - d)**2).sum(1)
        ind = dists.argmin()
        return self.Y[ind]

    def classify(self, d):
        if self.X.shape[0] < K * SUBSAMPLE: return 0
        if not HAVE_SK: return self.nearest(d)
        return int(self.nn.predict(d)[0])


class Myo(MyoRaw):
    '''Adds higher-level pose classification and handling onto MyoRaw.'''

    HIST_LEN = 25

    def __init__(self, cls, tty=None):
        MyoRaw.__init__(self, tty)
        self.cls = cls

        self.history = deque([0] * Myo.HIST_LEN, Myo.HIST_LEN)
        self.history_cnt = Counter(self.history)
        self.add_emg_handler(self.emg_handler)
        self.last_pose = None

        self.pose_handlers = []

    def emg_handler(self, emg, moving):
        y = self.cls.classify(emg)
        self.history_cnt[self.history[0]] -= 1
        self.history_cnt[y] += 1
        self.history.append(y)

        r, n = self.history_cnt.most_common(1)[0]
        if self.last_pose is None or (n > self.history_cnt[self.last_pose] + 5 and n > Myo.HIST_LEN / 2):
            self.on_raw_pose(r)
            self.last_pose = r

    def add_raw_pose_handler(self, h):
        self.pose_handlers.append(h)

    def on_raw_pose(self, pose):
        for h in self.pose_handlers:
            h(pose)

if __name__ == '__main__':
    import subprocess
    m = Myo(NNClassifier(), sys.argv[1] if len(sys.argv) >= 2 else None)
    m.add_raw_pose_handler(print)

    def page(pose):
        if pose == 5:
            subprocess.call(['xte', 'key Page_Down'])
        elif pose == 6:
            subprocess.call(['xte', 'key Page_Up'])

    m.add_raw_pose_handler(page)

    m.connect()

    while True:
        m.run()
