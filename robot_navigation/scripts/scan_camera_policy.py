"""Pure policy shared by the scan camera and offline timing tests."""
import math

class ScanCameraPolicy:
    def __init__(self, mode='auto', timeout=5.0, continuous=False):
        if mode not in ('auto', 'low', 'high'):
            raise ValueError('scan mode must be auto, low or high')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('switch timeout must be positive and finite')
        self.mode = mode
        self.timeout = timeout
        self.continuous = continuous
        self.qr = continuous
        self.bed = 0
        self.bed_done = False
        self.started = None
        self.high = mode == 'high'

    @property
    def active(self):
        return self.continuous or self.qr or (self.bed in (1, 3) and not self.bed_done)

    def reset(self, now):
        self.started = now
        self.high = self.mode == 'high'

    def target(self, bed, now):
        if bed not in (0, 1, 3):
            return
        self.bed = bed
        self.bed_done = False
        if bed:
            self.reset(now)

    def enable_qr(self, enabled, now):
        if enabled and not self.qr:
            self.reset(now)
        self.qr = bool(enabled)

    def decoded(self, kind, now):
        if kind == 'qr':
            self.qr = False
        elif kind == 'barcode':
            self.bed_done = True
        # Standalone always-on mode keeps watching for a different QR.
        # Fixed modes never change resolution.
        if self.continuous:
            self.reset(now)

    def requested(self, now):
        if not self.active:
            return None
        if self.started is None:
            self.reset(now)
        if self.mode == 'auto' and now-self.started >= self.timeout:
            self.high = True
        return 'high' if self.high else 'low'
