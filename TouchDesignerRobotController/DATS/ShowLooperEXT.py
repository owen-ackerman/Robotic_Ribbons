import time


class ShowLooperEXT:
    """
    Timed show looper. Drives the controller, sequencer, and audio through
    a fixed repeating schedule:

      t =   0 s  →  home:  stop all, reload audio, home motors, load show DAT
      t =  10 s  →  play:  seq.play(), ctrl.resume(), audio play=1
      t = 475 s  →  stop:  seq.stop(), ctrl.stop()+zeroAll(), audio play=0
      t = 595 s  →  loop:  restart cycle from t=0

    Setup:
      1. Attach this extension to a COMP (e.g. 'looper_comp').
      2. Call Update() from a DAT Execute or CHOP Execute each frame:
             op('looper_comp').ext.ShowLooperEXT.Update()
      3. Call start(show_name) to begin, stop() to halt.

    The extension resolves controller_comp, sequencer_comp, and audiofilein1
    from the parent() of ownerComp — put them at the same level.
    """

    HOMING_TIME = 0
    PLAY_TIME   = 5
    STOP_TIME   = 475
    LOOP_PERIOD = 595  # 475 s show + 120 s cooldown

    def __init__(self, ownerComp):
        self.ownerComp        = ownerComp
        self._running         = False
        self._show_name       = 'final_show'
        self._light_show_name = 'final_light_show'  # Table DAT name for LightControllerEXT; None → uses light controller's default
        self._light_ambient_show_name = 'light_lobby'
        #self._empty_sequencer = 'empty_sequencer'
        self._looper_time     = 0.0
        self._prev_wall       = None
        self._current_phase   = None  # 'homing' | 'playing' | 'cooling'
        self._time_chop       = 'const_looper_time'  # Constant CHOP inside ownerComp

    # -------------------------
    # Update loop
    # -------------------------

    def Update(self):
        if not self._running:
            return

        now = time.time()
        if self._prev_wall is None:
            self._prev_wall = now
            return

        dt = now - self._prev_wall
        if dt < 0.001:
            return
        self._prev_wall    = now
        self._looper_time += dt

        if self._looper_time >= self.LOOP_PERIOD:
            self._looper_time = 0.0 
            self._current_phase = None  # allow re-entry into homing on next eval

        t = self._looper_time

        if t < self.PLAY_TIME:
            new_phase = 'homing'
        elif t < self.STOP_TIME:
            new_phase = 'playing'
        else:
            new_phase = 'cooling'

        if new_phase != self._current_phase:
            self._current_phase = new_phase
            if new_phase == 'homing':
                self._onHoming()
            elif new_phase == 'playing':
                self._onPlay()
            elif new_phase == 'cooling':
                self._onStop()

        self._pushToCHOP()

    # -------------------------
    # Public controls
    # -------------------------

    def start(self, show_name=None):
        """Begin the loop from t=0. show_name is the Table DAT op name for the sequencer."""
        if show_name:
            self._show_name = show_name
        self._looper_time   = 0.0
        self._prev_wall     = None
        self._current_phase = None
        self._running       = True
        print(f'[ShowLooper] Starting — show: {self._show_name}')

    def stop(self):
        """Halt the looper and stop all playback immediately."""
        self._running = False
        self._onStop()
        print('[ShowLooper] Stopped.')

    def pause(self):
        """Freeze the looper clock without stopping playback."""
        self._running = False
        print(f'[ShowLooper] Paused at t={self._looper_time:.1f}s.')

    def resume(self):
        """Resume the looper clock from the current position."""
        self._prev_wall = None
        self._running   = True
        print(f'[ShowLooper] Resumed at t={self._looper_time:.1f}s.')

    def setShowName(self, show_name):
        """Change the show DAT name used on the next homing phase."""
        self._show_name = show_name

    def setLightShowName(self, name):
        """Change the light Table DAT name loaded on the next homing phase. None → light controller's own default."""
        self._light_show_name = name

    def setTimeCHOP(self, chop_name):
        """Set the name of the Constant CHOP inside ownerComp that receives elapsed time."""
        self._time_chop = chop_name

    @property
    def elapsed(self):
        """Current position within the loop cycle (seconds)."""
        return self._looper_time

    # -------------------------
    # Phase handlers
    # -------------------------

    def _onHoming(self):
        print(f'[ShowLooper] t={self._looper_time:.1f}s — homing')
        audio = self._audio()
        ctrl  = self._ctrl()
        seq   = self._seq()
        if audio:
            audio.par.play = 0
            audio.par.reloadpulse.pulse()
        if ctrl:
            ctrl.stop()
            ctrl.zeroAll()
            ctrl.startMotorHoming(phi_target=85)
        if seq:
            seq.stop()
            if self._show_name:
                seq.loadFromDAT(self._show_name)
        light = self._light()
        if light:
            print('Starting ambient LIGHT')
            light.stop()
            light.loadFromDAT(self._light_show_name)


    def _onPlay(self):
        print(f'[ShowLooper] t={self._looper_time:.1f}s — play')
        ctrl  = self._ctrl()
        seq   = self._seq()
        audio = self._audio()
        if ctrl:
            ctrl.play()
        if seq:
            seq.resume()
        if audio:
            audio.par.reloadpulse
            audio.par.play = 1
        light = self._light()
        if light:
            light.play()

    def _onStop(self):
        print(f'[ShowLooper] t={self._looper_time:.1f}s — stop')
        ctrl  = self._ctrl()
        seq   = self._seq()
        audio = self._audio()
        #if seq:
            #seq.stop()
        if ctrl:
            ctrl.stop()
            ctrl.zeroAll()
        if audio:
            audio.par.play = 0
        light = self._light()
        if light:
            light.stop()
            light.loadFromDAT(self._light_ambient_show_name)
            time.sleep(0.5)
            light.play()

            

    # -------------------------
    # CHOP output
    # -------------------------

    def _pushToCHOP(self):
        chop = self.ownerComp.op(self._time_chop)
        if chop:
            chop.par.const0value = self._looper_time

    # -------------------------
    # Op helpers
    # -------------------------

    def _ctrl(self):
        c = self.ownerComp.parent().op('controller_comp')
        if not c:
            print('[ShowLooper] controller_comp not found')
            return None
        return c.ext.RobotControllerEXT

    def _seq(self):
        s = self.ownerComp.parent().op('sequencer_comp')
        if not s:
            print('[ShowLooper] sequencer_comp not found')
            return None
        return s.ext.ShowSequencerEXT

    def _audio(self):
        a = self.ownerComp.parent().op('audiofilein1')
        if not a:
            print('[ShowLooper] audiofilein1 not found')
        return a

    def _light(self):
        try:
            l = self.ownerComp.parent().op('light_comp')
            if not l:
                print('[ShowLooper] light_comp not found')
                return None
            return l.ext.LightControllerEXT
        except Exception as e:
            print(f'[ShowLooper] light_comp error: {e}')
            return None
