from __future__ import print_function

from collections import Counter
import struct
import sys
import time

import numpy as np
import pygame
from pygame.locals import *

pygame.init()

try:
    from sklearn import neighbors, svm
    HAVE_SK = True
except ImportError:
    HAVE_SK = False

import myo



def pack(fmt, *args):
    return struct.pack('<' + fmt, *args)
def unpack(fmt, *args):
    return struct.unpack('<' + fmt, *args)

class Classifier(object):
    def __init__(self):
        for i in range(10):
            with open('vals%d.dat' % i, 'ab') as f: pass
        self.read_data()

    def store_data(self, cls, vals):
        with open('vals%d.dat' % cls, 'ab') as f:
            f.write(pack('8H', *vals))

        self.X = np.vstack([self.X, vals])
        self.Y = np.hstack([self.Y, [cls]])

        self.train()

    def read_data(self):
        X = []
        Y = []
        for i in range(10):
            X.append(np.fromfile('vals%d.dat' % i, dtype=np.uint16).reshape((-1, 8)))
            Y.append(i + np.zeros(X[-1].shape[0]))

        self.X = np.vstack(X)
        self.Y = np.hstack(Y)

        self.train()

    def train(self):
        if HAVE_SK and self.X.shape[0] >= 20:
            self.nn = neighbors.KNeighborsClassifier(n_neighbors=5, algorithm='kd_tree').fit(self.X[::3], self.Y[::3])
            #self.sv = svm.SVC(class_weight='auto').fit(self.X[::5], self.Y[::5])
        else:
            self.nn = None

    def nearest(self, d):
        dists = ((self.X - d)**2).sum(1)
        ind = dists.argmin()
        return self.Y[ind]

    def classify(self, d):
        if self.X.shape[0] < 20: return 0
        if self.nn is None: return self.nearest(d)
        return self.nn.predict(d)[0]
        #return sv.predict(d)[0]


class Handler(object):
    def __init__(self, cl):
        self.recording = -1
        self.history = [0] * 25
        self.cl = cl

    def __call__(self, emg, moving):
        if self.recording >= 0:
            self.cl.store_data(self.recording, emg)

        y = self.cl.classify(emg)
        self.history.pop(0)
        self.history.append(y)

if __name__ == '__main__':
    w, h = 1200, 400
    scr = pygame.display.set_mode((w, h))

    m = myo.Myo(sys.argv[1] if len(sys.argv) >= 2 else None)
    hnd = Handler(Classifier())

    m.add_emg_handler(hnd)
    m.connect()

    font = pygame.font.Font(None, 30)

    try:
        while True:
            m.run()
            # data = np.random.uniform(0, 65536, size=8).astype(np.uint16)
            # hnd(data, 0)
            # time.sleep(.02)

            for ev in pygame.event.get():
                if ev.type == QUIT or (ev.type == KEYDOWN and ev.unicode == 'q'):
                    raise KeyboardInterrupt()
                elif ev.type == KEYDOWN:
                    if 48 <= ev.key < 58:
                        hnd.recording = ev.key - 48
                    elif ev.unicode == 'r':
                        hnd.cl.read_data()
                elif ev.type == KEYUP:
                    if 48 <= ev.key < 58:
                        hnd.recording = -1

            scr.fill((0, 0, 0), (0, 0, w, h))
            c = Counter(hnd.history)
            r = c.most_common(1)[0][0]
            for i in range(10):
                x = 0
                y = 0 + 30 * i

                clr = (0,200,0) if i == r else (255,255,255)

                txt = font.render('%5d' % (hnd.cl.Y == i).sum(), True, (255,255,255))
                scr.blit(txt, (x + 20, y))

                txt = font.render('%d' % i, True, clr)
                scr.blit(txt, (x + 110, y))


                scr.fill((0,0,0), (x+130, y + txt.get_height() / 2 - 10, len(hnd.history) * 20, 20))
                scr.fill(clr, (x+130, y + txt.get_height() / 2 - 10, c[i] * 20, 20))

            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    finally:
        m.disconnect()
        print()

    pygame.quit()
