from __future__ import print_function

from collections import Counter
import struct
import sys
import time

import numpy as np

try:
    from sklearn import neighbors, svm
    HAVE_SK = True
except ImportError:
    HAVE_SK = False

try:
    import pygame
    from pygame.locals import *
    HAVE_PYGAME = True
except ImportError:
    HAVE_PYGAME = False

from common import *
import myo

class EMGHandler(object):
    def __init__(self, m):
        self.recording = -1
        self.m = m
        self.emg = (0,) * 8

    def __call__(self, emg, moving):
        self.emg = emg
        if self.recording >= 0:
            self.m.cls.store_data(self.recording, emg)

if __name__ == '__main__':
    if HAVE_PYGAME:
        pygame.init()
        w, h = 800, 320
        scr = pygame.display.set_mode((w, h))
        font = pygame.font.Font(None, 30)

    m = myo.Myo(myo.NNClassifier(), sys.argv[1] if len(sys.argv) >= 2 else None)
    hnd = EMGHandler(m)
    m.add_emg_handler(hnd)
    m.connect()

    try:
        while True:
            m.run()

            r = m.history_cnt.most_common(1)[0][0]

            if HAVE_PYGAME:
                for ev in pygame.event.get():
                    if ev.type == QUIT or (ev.type == KEYDOWN and ev.unicode == 'q'):
                        raise KeyboardInterrupt()
                    elif ev.type == KEYDOWN:
                        if K_0 <= ev.key <= K_9:
                            hnd.recording = ev.key - K_0
                        elif K_KP0 <= ev.key <= K_KP9:
                            hnd.recording = ev.key - K_Kp0
                        elif ev.unicode == 'r':
                            hnd.cl.read_data()
                    elif ev.type == KEYUP:
                        if K_0 <= ev.key <= K_9 or K_KP0 <= ev.key <= K_KP9:
                            hnd.recording = -1

                scr.fill((0, 0, 0), (0, 0, w, h))

                for i in range(10):
                    x = 0
                    y = 0 + 30 * i

                    clr = (0,200,0) if i == r else (255,255,255)

                    txt = font.render('%5d' % (m.cls.Y == i).sum(), True, (255,255,255))
                    scr.blit(txt, (x + 20, y))

                    txt = font.render('%d' % i, True, clr)
                    scr.blit(txt, (x + 110, y))


                    scr.fill((0,0,0), (x+130, y + txt.get_height() / 2 - 10, len(m.history) * 20, 20))
                    scr.fill(clr, (x+130, y + txt.get_height() / 2 - 10, m.history_cnt[i] * 20, 20))

                if HAVE_SK and m.cls.nn is not None:
                    dists, inds = m.cls.nn.kneighbors(hnd.emg)
                    for i, (d, ind) in enumerate(zip(dists[0], inds[0])):
                        y = m.cls.Y[myo.SUBSAMPLE*ind]
                        text(scr, font, '%d %6d' % (y, d), (650, 20 * i))

                pygame.display.flip()
            else:
                for i in range(10):
                    if i == r: sys.stdout.write('\x1b[32m')
                    print(i, '-' * m.history_cnt[i], '\x1b[K')
                    if i == r: sys.stdout.write('\x1b[m')
                sys.stdout.write('\x1b[11A')
                print()

    except KeyboardInterrupt:
        pass
    finally:
        m.disconnect()
        print()

    if HAVE_PYGAME:
        pygame.quit()
