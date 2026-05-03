import time


class ShowSequencerEXT:
    """
    Table-driven show sequencer. Reads a Table DAT where each row defines a segment.

    Table columns (first row must be a header row with these exact names):
      time_start   — float, seconds; when this segment begins
      duration     — float, seconds; how long params interpolate from start to end
      fade         — float, seconds; fadeTo transition duration when entering this segment
                     (0 = instant switch via setWeight)
      behavior     — single behavior name
      params_start — space-separated key=value pairs applied to behavior at segment entry
      params_end   — space-separated key=value pairs; params interpolate toward these over duration

    Transitions:
      On entering a new segment, params_start is applied immediately so fadeTo captures the
      correct target state, then fadeTo(behavior, fade) fires. The controller owns weights
      during the fade — the sequencer only drives params each frame after that.

    Execute DAT order:
        op('sequencer_comp').ext.ShowSequencerEXT.Update()
        op('controller_comp').ext.RobotControllerEXT.Update()
        op('recorder_comp').ext.MotionRecorderEXT.Update()
    """

    def __init__(self, ownerComp):
        self.ownerComp       = ownerComp
        self.controller_comp = ownerComp.parent().op('controller_comp')

        self.seq_time      = 0.0
        self._prev_wall    = None
        self.playing       = False
        self.loop          = False
        self.duration      = 0.0

        self._segments     = []
        self._dat_name     = 'sequencer_table'
        self._last_seg_idx = -1

    # -------------------------
    # Update loop
    # -------------------------

    def Update(self):
        now = time.time()
        if self._prev_wall is None:
            self._prev_wall = now
            return
        dt = now - self._prev_wall
        if dt < 0.001:
            return
        self._prev_wall = now

        if not self.playing:
            return

        self.seq_time += dt

        if self.duration > 0 and self.seq_time >= self.duration:
            if self.loop:
                self.seq_time      %= self.duration
                self._last_seg_idx  = -1
            else:
                self.seq_time  = self.duration
                self.playing   = False

        self._evaluateSegments()

    # -------------------------
    # Playback controls
    # -------------------------

    def play(self):
        """Start or resume sequencer playback. Rewinds to t=0 if already at the end."""
        if self.duration > 0 and self.seq_time >= self.duration:
            self.seek(0.0)
        self._prev_wall = time.time()
        self.playing    = True

    def resume(self):
        """Resume playback from the current playhead position without rewinding."""
        self._prev_wall = time.time()
        self.playing    = True

    def pause(self):
        """Freeze sequencer playhead."""
        self.playing = False

    def stop(self):
        """Pause sequencer and rewind to t=0."""
        self.playing       = False
        self.seq_time      = 0.0
        self._last_seg_idx = -1

    def seek(self, seconds):
        """Jump the playhead to an arbitrary time and immediately apply that segment."""
        self.seq_time      = max(0.0, float(seconds))
        self._last_seg_idx = -1
        self._evaluateSegments()

    def setLoop(self, loop):
        self.loop = bool(loop)

    # -------------------------
    # Table loading
    # -------------------------

    def setDATName(self, name):
        """Set the op name of the Table DAT to read. Default: 'sequencer_table'."""
        self._dat_name = name

    def loadFromDAT(self, dat_op_name=None):
        """
        Parse the Table DAT and rebuild the segment list.
        Safe to call every frame for live editing.
        """
        name = dat_op_name or self._dat_name
        dat  = self.ownerComp.op(name) or self.ownerComp.parent().op(name)
        if not dat:
            print(f'[Sequencer] Table DAT "{name}" not found.')
            return

        segments = []
        for row in range(1, dat.numRows):
            def cell(col):
                try:
                    v = str(dat[row, col])
                    return v if v not in ('', 'None') else ''
                except Exception:
                    return ''

            time_start_s = cell('time_start')
            duration_s   = cell('duration')
            behavior_s   = cell('behavior').strip()

            if not time_start_s or not behavior_s:
                continue

            try:
                time_start = float(time_start_s)
                duration   = float(duration_s) if duration_s else 0.0
            except ValueError:
                print(f'[Sequencer] Row {row}: invalid time_start or duration — skipped.')
                continue

            try:
                fade = float(cell('fade') or '0')
            except ValueError:
                fade = 0.0

            segments.append({
                'time_start':   time_start,
                'duration':     max(0.0, duration),
                'fade':         max(0.0, fade),
                'behavior':     behavior_s,
                'params_start': self._parseParams(cell('params_start')),
                'params_end':   self._parseParams(cell('params_end')),
            })

        self._segments     = sorted(segments, key=lambda s: s['time_start'])
        self._last_seg_idx = -1

        if self._segments:
            last = self._segments[-1]
            self.duration = last['time_start'] + last['duration']
        else:
            self.duration = 0.0

        print(f'[Sequencer] Loaded {len(self._segments)} segment(s), '
              f'duration={self.duration:.2f}s')

    def reloadDAT(self):
        """Re-parse the table DAT without changing playback state."""
        self.loadFromDAT()

    # -------------------------
    # Internal — evaluation
    # -------------------------

    def _evaluateSegments(self):
        if not self._segments:
            return
        idx = self._findActiveSegment(self.seq_time)
        if idx < 0:
            return

        seg      = self._segments[idx]
        elapsed  = self.seq_time - seg['time_start']
        progress = min(1.0, elapsed / seg['duration']) if seg['duration'] > 0 else 1.0

        if idx != self._last_seg_idx:
            self._transitionToSegment(seg)
            self._last_seg_idx = idx

        self._updateParams(seg, progress)

    def _findActiveSegment(self, t):
        active = -1
        for i, seg in enumerate(self._segments):
            if seg['time_start'] <= t:
                active = i
            else:
                break
        return active

    def _transitionToSegment(self, seg):
        """
        Called once on entering a new segment.
        Applies params_start to the behavior so fadeTo captures the correct target,
        then fires fadeTo (or instant switch if fade=0).
        'motor_homing' is a special keyword: fires startMotorHoming() and returns.
        """
        ctrl = self._getControllerExt()
        if not ctrl:
            print(f'[Sequencer] _transitionToSegment: no controller — skipping "{seg["behavior"]}"')
            return

        if seg['behavior'] == 'motor_homing':
            raw = seg['params_start'].get('phi_target', None)
            phi_target = raw if isinstance(raw, (int, float)) else None
            ctrl.startMotorHoming(phi_target=phi_target)
            print(f'[Sequencer] → motor_homing  phi_target={phi_target}  t={self.seq_time:.2f}s')
            return
        if seg['behavior'] == 'zeroAll':
            ctrl.zeroAll()
            print(f'[Sequencer] → ctrl zero all')
            return

        # Configure the target behavior before fadeTo samples it
        for key, value in seg['params_start'].items():
            if key == 'robot_mask':
                ctrl.setRobotMask(seg['behavior'], value)
            else:
                ctrl.setParam(seg['behavior'], key, value)

        if seg['fade'] > 0:
            ctrl.fadeTo(seg['behavior'], seg['fade'])
        else:
            ctrl.zeroAll()
            ctrl.setWeight(seg['behavior'], 1.0)

        print(f'[Sequencer] → "{seg["behavior"]}"  fade={seg["fade"]}s  t={self.seq_time:.2f}s')

    def _updateParams(self, seg, progress):
        """Called every frame — interpolates params_start → params_end over duration.
        List params (e.g. robot_mask) are not interpolated — start value holds until
        progress reaches 1.0, then end value takes over.
        """
        if seg['behavior'] == 'motor_homing' :
            return
        if seg['behavior'] == 'zeroAll' :
            return
        ctrl = self._getControllerExt()
        if not ctrl:
            return
        p_start  = seg['params_start']
        p_end    = seg['params_end']
        all_keys = set(p_start.keys()) | set(p_end.keys())
        for key in all_keys:
            v_start = p_start.get(key, p_end.get(key, 0.0))
            v_end   = p_end.get(key,   p_start.get(key, 0.0))
            if key == 'robot_mask':
                ctrl.setRobotMask(seg['behavior'], v_end if progress >= 1.0 else v_start)
            elif isinstance(v_start, list) or isinstance(v_end, list):
                pass  # unknown list param — skip
            elif isinstance(v_start, str) or isinstance(v_end, str):
                pass  # string params are set once at transition, not interpolated
            else:
                ctrl.setParam(seg['behavior'], key, v_start + (v_end - v_start) * progress)

    @staticmethod
    def _parseParams(s):
        """Parse 'key=value key2=value2 ...' into {key: float | list[float]}.
        Comma-separated values become a list: robot_mask=1,0,0,0,0,0 → [1.0, 0.0, ...]
        """
        if not s or not s.strip():
            return {}
        result = {}
        for pair in s.split():
            if '=' in pair:
                key, _, val = pair.partition('=')
                key = key.strip()
                val = val.strip()
                if ',' in val:
                    try:
                        result[key] = [float(v) for v in val.split(',')]
                    except ValueError:
                        pass
                else:
                    try:
                        result[key] = float(val)
                    except ValueError:
                        result[key] = val  # keep as string for non-numeric params (e.g. serial_dat, done_string)
        return result

    # -------------------------
    # Diagnostics
    # -------------------------

    def listSegments(self):
        for i, s in enumerate(self._segments):
            print(f'  [{i}] t={s["time_start"]}  dur={s["duration"]}  '
                  f'fade={s["fade"]}  behavior={s["behavior"]}')

    # -------------------------
    # Op references
    # -------------------------

    def _getControllerExt(self):
        if not self.controller_comp:
            self.controller_comp = self.ownerComp.parent().op('controller_comp')
        if not self.controller_comp:
            print('[Sequencer] controller_comp not found')
            return None
        return self.controller_comp.ext.RobotControllerEXT

    def _getRecorderExt(self):
        op_ref = self.ownerComp.parent().op('recorder_comp')
        return op_ref.ext.MotionRecorderEXT if op_ref else None
