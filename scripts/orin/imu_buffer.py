"""Recent IMU samples on the VOXL clock, linearly interpolated at a query time.

A VOXL reboot (a battery swap, say) brings its clock back earlier than before.
The buffer then starts over, instead of rejecting every newer-but-earlier stamp
as out of order, which is what left a consumer silent after a reboot.
"""
import bisect

import numpy as np

CLOCK_JUMP = 1.0                                    # s backwards in stamps that means a restart


class IMUBuffer(object):
    def __init__(self, horizon=2.0):
        self.t, self.x = [], []
        self.horizon = horizon
        self.resets = 0

    def add(self, t, sample):
        """Append a sample; False for an out-of-order or repeated stamp."""
        if self.t and t <= self.t[-1]:
            if t >= self.t[-1] - CLOCK_JUMP:
                return False
            self.t, self.x = [], []                 # the clock jumped back: start over
            self.resets += 1
        self.t.append(t)
        self.x.append(np.asarray(sample, float))
        if len(self.t) > 8192:
            k = bisect.bisect_left(self.t, self.t[-1] - self.horizon)
            del self.t[:k], self.x[:k]
        return True

    def at(self, t, max_gap=0.1):
        """The sample interpolated at t, or None outside the buffer or across a gap."""
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i == len(self.t):
            return None
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 - t0 > max_gap:
            return None
        a = (t - t0) / (t1 - t0)
        return (1.0 - a) * self.x[i - 1] + a * self.x[i]
