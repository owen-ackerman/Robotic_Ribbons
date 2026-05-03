import time
import json


class MotionRecorderEXT:
    """
    Records per-robot theta/phi motion from RobotControllerEXT into named clips.
    Each clip is registered as a dynamic behavior on the controller so it can be
    weighted, blended, and faded like any built-in behavior.

    Expects a sibling op named 'controller_comp'.
    Call Update() every frame AFTER the controller's Update() so last_states is fresh.

    Execute DAT order:
        op('sequencer_comp').ext.ShowSequencerEXT.Update()
        op('controller_comp').ext.RobotControllerEXT.Update()
        op('recorder_comp').ext.MotionRecorderEXT.Update()
    """

    def __init__(self, ownerComp):
        self.ownerComp = ownerComp

        self._recording      = False
        self._current_name   = None
        self._current_frames = []
        self._sample_rate    = 30.0
        self._record_start   = 0.0
        self._include_led    = False

        self.clips = {}   # name -> clip dict

    # -------------------------
    # Update (call every frame, after controller)
    # -------------------------

    def Update(self):
        if not self._recording:
            return
        ctrl = self._getControllerExt()
        if not ctrl or not ctrl.last_states:
            return

        now      = time.time()
        rec_time = now - self._record_start
        interval = 1.0 / self._sample_rate

        if self._current_frames and (rec_time - self._current_frames[-1]['time']) < interval:
            return

        frame = {
            'time':   rec_time,
            'states': [{'theta': s['theta'], 'phi': s['phi']} for s in ctrl.last_states],
        }
        if self._include_led:
            for i, s in enumerate(ctrl.last_states):
                frame['states'][i]['led'] = list(s.get('led', [0.0, 0.0, 0.0]))

        self._current_frames.append(frame)

    # -------------------------
    # Recording control
    # -------------------------

    def startRecording(self, name, sample_rate=30.0, include_led=False):
        """Begin capturing robot states under the given name."""
        if self._recording:
            self.stopRecording()
        self._recording      = True
        self._current_name   = name
        self._current_frames = []
        self._sample_rate    = max(1.0, sample_rate)
        self._include_led    = include_led
        self._record_start   = time.time()
        print(f'[Recorder] Recording "{name}" at {self._sample_rate} fps...')

    def stopRecording(self):
        """Finalize the recording and register it as a behavior on the controller."""
        if not self._recording:
            return None
        self._recording = False

        if len(self._current_frames) < 2:
            print(f'[Recorder] "{self._current_name}" too short — discarded.')
            self._current_name   = None
            self._current_frames = []
            return None

        duration = self._current_frames[-1]['time']
        clip = {
            'name':        self._current_name,
            'duration':    duration,
            'sample_rate': self._sample_rate,
            'frames':      self._current_frames,
        }
        name = self._current_name
        self.clips[name]     = clip
        self._current_name   = None
        self._current_frames = []

        self._registerClip(name)
        print(f'[Recorder] Saved "{name}" — {len(clip["frames"])} frames, {duration:.2f}s')
        return name

    @property
    def isRecording(self):
        return self._recording

    @property
    def recordingTime(self):
        """Elapsed seconds since startRecording(), or 0 if not recording."""
        return (time.time() - self._record_start) if self._recording else 0.0

    # -------------------------
    # Clip playback controls
    # -------------------------

    def resetClip(self, name):
        """Rewind clip playback cursor to t=0."""
        fn = self._getClipFn(name)
        if fn:
            fn.state['t'] = 0.0

    def seekClip(self, name, seconds):
        """Jump clip playback cursor to an arbitrary time."""
        fn = self._getClipFn(name)
        if fn:
            fn.state['t'] = max(0.0, seconds)

    def setClipSpeed(self, name, speed):
        """Playback speed multiplier: 1.0 = realtime, 0.5 = half, -1.0 = reverse."""
        fn = self._getClipFn(name)
        if fn:
            fn.state['speed'] = speed

    def setClipLoop(self, name, loop):
        """True = loop continuously, False = hold last frame when done."""
        fn = self._getClipFn(name)
        if fn:
            fn.state['loop'] = bool(loop)

    def getClipState(self, name):
        """Return the live playback state dict for a clip (t, speed, loop)."""
        fn = self._getClipFn(name)
        return dict(fn.state) if fn else None

    # -------------------------
    # Clip management
    # -------------------------

    def deleteClip(self, name):
        self.clips.pop(name, None)
        ctrl = self._getControllerExt()
        if ctrl:
            ctrl.unregisterBehavior(name)

    def listClips(self):
        return list(self.clips.keys())

    def getClipInfo(self, name):
        if name not in self.clips:
            return None
        c = self.clips[name]
        return {
            'name':        c['name'],
            'duration':    c['duration'],
            'frames':      len(c['frames']),
            'sample_rate': c['sample_rate'],
        }

    # -------------------------
    # Persistence
    # -------------------------

    def saveClips(self, filepath):
        """Serialize all clips to a JSON file."""
        with open(filepath, 'w') as f:
            json.dump(self.clips, f)
        print(f'[Recorder] {len(self.clips)} clip(s) saved to {filepath}')

    def loadClips(self, filepath):
        """Load clips from JSON and re-register each as a behavior on the controller."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        for name, clip in data.items():
            self.clips[name] = clip
            self._registerClip(name)
        print(f'[Recorder] {len(data)} clip(s) loaded from {filepath}')

    # -------------------------
    # Internal
    # -------------------------

    def _registerClip(self, name):
        ctrl = self._getControllerExt()
        if not ctrl:
            print(f'[Recorder] Cannot register "{name}" — controller_comp not found.')
            return
        clip = self.clips[name]
        fn   = self._makeClipFn(clip['frames'], clip['duration'])
        ctrl.registerBehavior(name, fn)

    def _makeClipFn(self, frames, duration):
        """Return a behavior-compatible callable with an attached mutable state dict."""
        state  = {'t': 0.0, 'speed': 1.0, 'loop': True}
        interp = MotionRecorderEXT._interpolateFrames

        def _play(dt):
            state['t'] += dt * state['speed']
            if duration <= 0:
                return None
            t = (state['t'] % duration) if state['loop'] else min(state['t'], duration)
            return interp(frames, t)

        _play.state = state
        return _play

    @staticmethod
    def _interpolateFrames(frames, t):
        """Binary-search + linear interpolation between recorded frames at time t."""
        if not frames:
            return None

        if t <= frames[0]['time']:
            return MotionRecorderEXT._frameToStates(frames[0])
        if t >= frames[-1]['time']:
            return MotionRecorderEXT._frameToStates(frames[-1])

        lo, hi = 0, len(frames) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if frames[mid]['time'] <= t:
                lo = mid
            else:
                hi = mid

        f0, f1  = frames[lo], frames[hi]
        span    = f1['time'] - f0['time']
        alpha   = (t - f0['time']) / span if span > 1e-9 else 0.0

        result = []
        for s0, s1 in zip(f0['states'], f1['states']):
            td    = ((s1['theta'] - s0['theta'] + 180.0) % 360.0) - 180.0
            theta = (s0['theta'] + td * alpha) % 360.0
            phi   = s0['phi'] + (s1['phi'] - s0['phi']) * alpha
            led0  = s0.get('led', [0.5, 0.0, 1.0])
            led1  = s1.get('led', [0.5, 0.0, 1.0])
            led   = [led0[j] + (led1[j] - led0[j]) * alpha for j in range(3)]
            result.append({'theta': theta, 'phi': phi, 'led': led})
        return result

    @staticmethod
    def _frameToStates(frame):
        """Copy a frame's states, ensuring led key is always present."""
        return [
            {'theta': s['theta'], 'phi': s['phi'], 'led': s.get('led', [0.5, 0.0, 1.0])}
            for s in frame['states']
        ]

    def _getControllerExt(self):
        op_ref = self.ownerComp.parent().op('controller_comp')
        return op_ref.ext.RobotControllerEXT if op_ref else None

    def _getClipFn(self, name):
        ctrl = self._getControllerExt()
        if not ctrl:
            return None
        fn = ctrl._behavior_fns.get(name)
        return fn if fn and hasattr(fn, 'state') else None
