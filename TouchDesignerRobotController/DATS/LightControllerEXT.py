import time


class LightControllerEXT:
    """
    Independent lighting controller for 12 DMX lights (6 robots × 2 lights each).

    Each light has 7 DMX channels: Brightness, R, G, B, 0, 0, 0.
    DMX channel layout (1-based) — robots run in reverse order (Robot 6 first):
      Robot 6 Light A: ch1–ch7     Robot 6 Light B: ch8–ch14
      Robot 5 Light A: ch15–ch21   Robot 5 Light B: ch22–ch28
      Robot 4 Light A: ch29–ch35   Robot 4 Light B: ch36–ch42
      Robot 3 Light A: ch43–ch49   Robot 3 Light B: ch50–ch56
      Robot 2 Light A: ch57–ch63   Robot 2 Light B: ch64–ch70
      Robot 1 Light A: ch71–ch77   Robot 1 Light B: ch78–ch84

    Formula (0-based): base = (5 - robot_idx) * 14 + light_within * 7
      where robot_idx = light_idx // 2, light_within = light_idx % 2

    Outputs to a Constant CHOP (default name: 'light_dmx') writing parameters
    const0value–const83value (→ TD channel names chan1–chan84).
    The Constant CHOP must be pre-configured with 84 channels.

    Table DAT format (first row is header with exact column names):
      time_start        — float, seconds; when segment becomes active
      duration          — float, seconds; interpolation window (0 = instant snap)
      lights            — target: 'all', 'robot1'–'robot6', 'robot1a'/'robot1b',
                          or comma-separated 1-based indices ('1,3,5') or ranges ('1-6')
      brightness_start  — 0–255 (omit → 0)
      r_start           — 0–255 (omit → 0)
      g_start           — 0–255 (omit → 0)
      b_start           — 0–255 (omit → 0)
      brightness_end    — 0–255 (omit → hold brightness_start)
      r_end             — 0–255 (omit → hold r_start)
      g_end             — 0–255 (omit → hold g_start)
      b_end             — 0–255 (omit → hold b_start)
      easing            — 'linear' (default), 'ease', 'ease_in', 'ease_out'

    Multiple segments can be active simultaneously for different lights.
    For overlapping lights the most-recently-started segment wins.

    Execute DAT order:
        op('light_comp').ext.LightControllerEXT.Update()
        op('sequencer_comp').ext.ShowSequencerEXT.Update()
        op('controller_comp').ext.RobotControllerEXT.Update()
        op('recorder_comp').ext.MotionRecorderEXT.Update()
    """

    NUM_LIGHTS         = 12  # 6 robots × 2 lights
    CHANNELS_PER_LIGHT = 7   # brightness, r, g, b, 0, 0, 0

    def __init__(self, ownerComp):
        self.ownerComp = ownerComp

        # Per-light DMX state: [brightness, r, g, b, 0, 0, 0], values 0–255
        self.light_states = [[0] * self.CHANNELS_PER_LIGHT
                             for _ in range(self.NUM_LIGHTS)]

        self.seq_time   = 0.0
        self._prev_wall = None
        self.playing    = False
        self.loop       = True
        self.duration   = 0.0

        self._segments  = []
        self._dat_name  = 'light_table'
        self._chop_name = 'light_dmx'

    # -------------------------
    # Update loop
    # -------------------------

    def Update(self):
        now = time.time()
        if self._prev_wall is None:
            self._prev_wall = now
            return
        dt = now - self._prev_wall
        if dt < 0.001:   # double-cook guard
            return
        self._prev_wall = now

        if self.playing:
            self.seq_time += dt
            if self.duration > 0 and self.seq_time >= self.duration:
                if self.loop:
                    self.seq_time %= self.duration
                else:
                    self.seq_time = self.duration
                    self.playing  = False

        if self._segments and self.playing:
            self._evaluateSegments()

        self.PushToCHOP()

    # -------------------------
    # CHOP output
    # -------------------------

    def PushToCHOP(self):
        """Write all 84 channel values to the Constant CHOP."""
        chop = (self.ownerComp.op(self._chop_name)
                or self.ownerComp.parent().op(self._chop_name))
        if not chop:
            return
        for light_idx in range(self.NUM_LIGHTS):
            robot_idx    = light_idx // 2
            light_within = light_idx % 2
            base = (5 - robot_idx) * (2 * self.CHANNELS_PER_LIGHT) + light_within * self.CHANNELS_PER_LIGHT
            for ch in range(self.CHANNELS_PER_LIGHT):
                try:
                    chop.par[f'const{base + ch}value'] = self.light_states[light_idx][ch]
                except Exception:
                    pass

    # -------------------------
    # Sequencer evaluation
    # -------------------------

    def _evaluateSegments(self):
        """
        For each light, find the most-recently-started active segment and apply
        its interpolated color. Later segments (higher time_start) override earlier
        ones for the same lights.
        """
        t        = self.seq_time
        resolved = [None] * self.NUM_LIGHTS  # (seg, progress 0–1) per light

        for seg in self._segments:
            if seg['time_start'] > t:
                break  # sorted by time_start; nothing further is active yet
            dur      = seg['duration']
            elapsed  = t - seg['time_start']
            progress = min(1.0, elapsed / dur) if dur > 0 else 1.0
            for light_idx in seg['lights']:
                resolved[light_idx] = (seg, progress)

        for light_idx, entry in enumerate(resolved):
            if entry is None:
                continue
            seg, progress = entry
            eased = self._applyEasing(progress, seg['easing'])
            s    = self.light_states[light_idx]
            s[0] = int(round(self._lerp(seg['brightness_start'], seg['brightness_end'], eased)))
            s[1] = int(round(self._lerp(seg['r_start'],          seg['r_end'],          eased)))
            s[2] = int(round(self._lerp(seg['g_start'],          seg['g_end'],          eased)))
            s[3] = int(round(self._lerp(seg['b_start'],          seg['b_end'],          eased)))
            # channels 4–6 remain 0

    @staticmethod
    def _lerp(a, b, t):
        return a + (b - a) * t

    @staticmethod
    def _applyEasing(t, easing):
        if easing == 'ease':
            return 3*t*t - 2*t*t*t       # cubic smoothstep
        if easing == 'ease_in':
            return t * t * t
        if easing == 'ease_out':
            t2 = t - 1.0
            return t2 * t2 * t2 + 1.0
        return t                          # linear

    # -------------------------
    # Playback controls
    # -------------------------

    def play(self):
        """Start or resume sequencer playback. Rewinds if already at the end."""
        if self.duration > 0 and self.seq_time >= self.duration:
            self.seq_time = 0.0
        self._prev_wall = time.time()
        self.playing    = True

    def pause(self):
        """Freeze sequencer playhead; lights hold current state."""
        self.playing = False

    def stop(self):
        """Pause and rewind to t=0."""
        self.playing  = False
        self.seq_time = 0.0

    def seek(self, seconds):
        """Jump playhead to an arbitrary time and immediately apply that state."""
        self.seq_time = max(0.0, float(seconds))
        if self._segments:
            self._evaluateSegments()
            self.PushToCHOP()

    def setLoop(self, loop):
        self.loop = bool(loop)

    # -------------------------
    # Table loading
    # -------------------------

    def setDATName(self, name):
        """Set the Table DAT op name to read from. Default: 'light_table'."""
        self._dat_name = name

    def loadFromDAT(self, dat_op_name=None):
        """Parse the Table DAT and rebuild the segment list."""
        name = dat_op_name or self._dat_name
        dat  = self.ownerComp.op(name) or self.ownerComp.parent().op(name)
        if not dat:
            print(f'[LightController] Table DAT "{name}" not found.')
            return

        segments = []
        for row in range(1, dat.numRows):
            ts       = self._cellStr(dat, row, 'time_start')
            dur_s    = self._cellStr(dat, row, 'duration')
            lights_s = self._cellStr(dat, row, 'lights').strip()

            if not ts or not lights_s:
                continue

            try:
                time_start = float(ts)
                duration   = float(dur_s) if dur_s else 0.0
            except ValueError:
                print(f'[LightController] Row {row}: invalid time_start/duration — skipped.')
                continue

            brightness_s = self._parseFloat(self._cellStr(dat, row, 'brightness_start'), 0.0)
            r_s          = self._parseFloat(self._cellStr(dat, row, 'r_start'),          0.0)
            g_s          = self._parseFloat(self._cellStr(dat, row, 'g_start'),          0.0)
            b_s          = self._parseFloat(self._cellStr(dat, row, 'b_start'),          0.0)
            brightness_e = self._parseFloat(self._cellStr(dat, row, 'brightness_end'),   brightness_s)
            r_e          = self._parseFloat(self._cellStr(dat, row, 'r_end'),            r_s)
            g_e          = self._parseFloat(self._cellStr(dat, row, 'g_end'),            g_s)
            b_e          = self._parseFloat(self._cellStr(dat, row, 'b_end'),            b_s)
            easing       = self._cellStr(dat, row, 'easing') or 'linear'

            lights = self._parseLights(lights_s)
            if not lights:
                print(f'[LightController] Row {row}: unrecognised lights "{lights_s}" — skipped.')
                continue

            segments.append({
                'time_start':       time_start,
                'duration':         max(0.0, duration),
                'lights':           lights,
                'brightness_start': brightness_s,
                'r_start':          r_s,
                'g_start':          g_s,
                'b_start':          b_s,
                'brightness_end':   brightness_e,
                'r_end':            r_e,
                'g_end':            g_e,
                'b_end':            b_e,
                'easing':           easing,
            })

        self._segments = sorted(segments, key=lambda s: s['time_start'])
        if self._segments:
            last = self._segments[-1]
            self.duration = last['time_start'] + last['duration']
        else:
            self.duration = 0.0

        print(f'[LightController] Loaded {len(self._segments)} segment(s), '
              f'duration={self.duration:.2f}s')

    def reloadDAT(self):
        """Re-parse the table DAT without changing playback state."""
        self.loadFromDAT()

    # -------------------------
    # Direct control
    # -------------------------

    def setLight(self, light_idx, brightness, r, g, b):
        """Set a single light by 1-based index (1–12). Values 0–255."""
        idx = int(light_idx) - 1
        if 0 <= idx < self.NUM_LIGHTS:
            s    = self.light_states[idx]
            s[0] = max(0, min(255, int(round(brightness))))
            s[1] = max(0, min(255, int(round(r))))
            s[2] = max(0, min(255, int(round(g))))
            s[3] = max(0, min(255, int(round(b))))
            self.PushToCHOP()

    def setRobotLights(self, robot_idx, brightness, r, g, b):
        """Set both lights on a robot by 1-based robot index (1–6)."""
        base = (int(robot_idx) - 1) * 2
        self.setLight(base + 1, brightness, r, g, b)
        self.setLight(base + 2, brightness, r, g, b)

    def setAll(self, brightness, r, g, b):
        """Set all 12 lights to the same color."""
        for i in range(1, self.NUM_LIGHTS + 1):
            self.setLight(i, brightness, r, g, b)

    def blackout(self):
        """Zero all channels immediately and push to CHOP."""
        self.light_states = [[0] * self.CHANNELS_PER_LIGHT
                             for _ in range(self.NUM_LIGHTS)]
        self.PushToCHOP()

    # -------------------------
    # Config
    # -------------------------

    def setCHOPName(self, name):
        """Set the Constant CHOP op name to write to. Default: 'light_dmx'."""
        self._chop_name = name

    # -------------------------
    # Diagnostics
    # -------------------------

    def listSegments(self):
        for i, s in enumerate(self._segments):
            print(f'  [{i}] t={s["time_start"]:.2f}  dur={s["duration"]:.2f}'
                  f'  lights={s["lights"]}  easing={s["easing"]}'
                  f'  start=[{s["brightness_start"]},{s["r_start"]},{s["g_start"]},{s["b_start"]}]'
                  f'  end=[{s["brightness_end"]},{s["r_end"]},{s["g_end"]},{s["b_end"]}]')

    def getState(self):
        """Return current light states as a list of {brightness, r, g, b} dicts."""
        return [
            {'brightness': ls[0], 'r': ls[1], 'g': ls[2], 'b': ls[3]}
            for ls in self.light_states
        ]

    # -------------------------
    # Static helpers
    # -------------------------

    @staticmethod
    def _cellStr(dat, row, col):
        try:
            v = str(dat[row, col])
            return v if v not in ('', 'None') else ''
        except Exception:
            return ''

    @staticmethod
    def _parseFloat(s, default=0.0):
        try:
            return float(s) if s else default
        except ValueError:
            return default

    @staticmethod
    def _parseLights(s):
        """
        Parse a lights specifier into a list of 0-based light indices (0–11).

          'all'       → [0..11]
          'robot1'    → [0, 1]      both lights on robot 1
          'robot1a'   → [0]         first light on robot 1
          'robot1b'   → [1]         second light on robot 1
          '1,3,5'     → [0, 2, 4]   comma-separated 1-based indices
          '1-6'       → [0..5]      inclusive range, 1-based
        """
        s = s.strip().lower()
        if s == 'all':
            return list(range(12))

        if s.startswith('robot'):
            rest = s[5:]
            try:
                if rest.endswith('a'):
                    return [(int(rest[:-1]) - 1) * 2]
                if rest.endswith('b'):
                    return [(int(rest[:-1]) - 1) * 2 + 1]
                robot = int(rest) - 1
                return [robot * 2, robot * 2 + 1]
            except ValueError:
                return []

        result = []
        for part in s.split(','):
            part = part.strip()
            if '-' in part:
                try:
                    lo, hi = part.split('-', 1)
                    result.extend(range(int(lo) - 1, int(hi)))
                except ValueError:
                    pass
            elif part.isdigit():
                result.append(int(part) - 1)
        return result
