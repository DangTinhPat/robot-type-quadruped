"""Tkinter control panel for the main_bot Gazebo sim and RViz view.

Replaces the "open one terminal per task" workflow (launch the sim in one
terminal, RViz in another, kill leftover gz/ros processes in a third, ...)
with a single window: set the spawn parameters, start/stop `ros2 launch
main_bot sim.launch.py` and `rz.launch.py`, watch their log, and clear
stale gz/ros processes on demand.
"""

import collections
import math
import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

import rclpy
from sensor_msgs.msg import Imu
from unitree_guide_controller.msg import Inputs

STOP_GRACE_SECONDS = 5

# Balance chart: how much roll/pitch history to keep and how often to sample/
# redraw it. 10Hz is plenty for a human-readable trend line and cheap to redraw;
# the IMU itself publishes much faster (1000Hz) but we only need the latest
# value each tick, not every sample.
CHART_TICK_MS = 100
CHART_HISTORY_SECONDS = 20
CHART_MAX_SAMPLES = CHART_HISTORY_SECONDS * 1000 // CHART_TICK_MS
# Fixed +/-pi/2 vertical scale (not auto-scaled) - the point of this chart is to
# see how close roll/pitch are getting to "about to fall over", so the scale
# should stay consistent rather than rescaling small wobbles to look large.
CHART_SCALE_RAD = math.pi / 2

# Max |lx/ly/rx| change per movement tick (see _move_tick) - ramps to a full 0.3
# command over ~6 ticks (600ms at the 100ms tick interval below) instead of
# jumping there instantly. Added because instant step changes in commanded
# velocity/turn rate were visibly destabilizing the (untuned) gait controller,
# especially when rotating - the robot would lurch and sometimes fall.
MOVE_RAMP_STEP = 0.05
MOVE_TICK_MS = 100

BG = '#1e1e1e'
BG_PANEL = '#2d2d2d'
BG_ENTRY = '#3c3c3c'
BORDER = '#3c3c3c'
FG = '#d4d4d4'
FG_MUTED = '#9d9d9d'
ACCENT_START = '#2ea043'
ACCENT_START_HOVER = '#3fb950'
ACCENT_STOP = '#da3633'
ACCENT_STOP_HOVER = '#f85149'
ACCENT_KILL = '#9e6a03'
ACCENT_KILL_HOVER = '#bb8009'
LOG_BG = '#0c0c0c'
LOG_FG = '#d4d4d4'

_STATUS_STYLES = {
    'idle': 'BadgeIdle.TLabel',
    'running': 'BadgeRunning.TLabel',
    'stopping': 'BadgeStopping.TLabel',
}


class SimControlGui:

    def __init__(self, root):
        self.root = root
        self.root.title('main_bot sim control')
        self.root.geometry('900x820')
        self.root.minsize(760, 600)

        self.procs = {'sim': None, 'rviz': None}
        self.proc_widgets = {}
        self.move_widgets = []
        # Repeating D-pad movement: root.after() id of the next scheduled publish;
        # _target_* is what the current button click asked for, _cur_* is what's
        # actually being published this tick (ramped toward the target - see
        # _move_tick/MOVE_RAMP_STEP). None = not currently ticking.
        self._move_after_id = None
        self._move_target_lx = self._move_target_ly = self._move_target_rx = 0.0
        self._move_cur_lx = self._move_cur_ly = self._move_cur_rx = 0.0
        self.log_queue = queue.Queue()

        # A persistent rclpy node/publisher, created once and reused for every
        # FSM/movement command. Earlier version spawned a fresh `ros2 topic pub`
        # subprocess per click, which pays rclpy-interpreter-startup + DDS
        # discovery cost (can be 1s+) every single time - visibly laggy. publish()
        # on an already-discovered publisher is near-instant.
        rclpy.init()
        self.ros_node = rclpy.create_node('gui_control_panel')
        self.control_input_pub = self.ros_node.create_publisher(Inputs, '/control_input', 10)

        # Balance chart: subscribe to the same /imu bridged topic used elsewhere
        # (config/gz_bridge.yaml), keep a rolling roll/pitch history, redraw on
        # a timer. _latest_roll/pitch are updated by the subscription callback;
        # the chart timer samples them at a fixed rate rather than redrawing on
        # every single IMU message (which arrives much faster than any human
        # needs to see it redraw).
        self.imu_sub = self.ros_node.create_subscription(Imu, '/imu', self._on_imu_msg, 10)
        self._latest_roll = 0.0
        self._latest_pitch = 0.0
        self._chart_buffer = collections.deque(maxlen=CHART_MAX_SAMPLES)

        self._build_widgets()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        # IDs of the two recurring after() timers, so _on_close can cancel them
        # explicitly - otherwise a timer can fire after root.destroy() and Tk
        # raises "invalid command name ..." trying to run the dead callback.
        self._poll_log_after_id = self.root.after(100, self._poll_log_queue)
        self._chart_after_id = self.root.after(CHART_TICK_MS, self._chart_tick)

    def _setup_style(self):
        self.root.configure(bg=BG)
        tkfont.nametofont('TkDefaultFont').configure(family='TkDefaultFont', size=10)

        style = ttk.Style(self.root)
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass

        style.configure('TFrame', background=BG)
        style.configure('TLabelframe', background=BG, bordercolor=BORDER, relief='flat')
        style.configure('TLabelframe.Label', background=BG, foreground=FG_MUTED,
                         font=('TkDefaultFont', 9, 'bold'))
        style.configure('TLabel', background=BG, foreground=FG)
        style.configure('Header.TLabel', background=BG, foreground=FG,
                         font=('TkDefaultFont', 15, 'bold'))
        style.configure('RowLabel.TLabel', background=BG, foreground=FG_MUTED,
                         font=('TkDefaultFont', 10, 'bold'))
        style.configure('TEntry', fieldbackground=BG_ENTRY, foreground=FG,
                         insertcolor=FG, bordercolor=BORDER,
                         lightcolor=BG_ENTRY, darkcolor=BG_ENTRY, padding=5)

        style.configure('TButton', background=BG_PANEL, foreground=FG,
                         font=('TkDefaultFont', 10), padding=(14, 8), borderwidth=0,
                         focusthickness=0)
        style.map('TButton',
                  background=[('active', '#3c3c3c'), ('disabled', '#2a2a2a')],
                  foreground=[('disabled', '#6a6a6a')])

        for name, base, hover in (
            ('Start.TButton', ACCENT_START, ACCENT_START_HOVER),
            ('Stop.TButton', ACCENT_STOP, ACCENT_STOP_HOVER),
            ('Kill.TButton', ACCENT_KILL, ACCENT_KILL_HOVER),
        ):
            style.configure(name, background=base, foreground='white',
                             font=('TkDefaultFont', 10, 'bold'), padding=(14, 8),
                             borderwidth=0, focusthickness=0)
            style.map(name,
                      background=[('active', hover), ('disabled', '#3c3c3c')],
                      foreground=[('disabled', '#6a6a6a')])

        for name, bg in (
            ('BadgeIdle.TLabel', '#3c3c3c'),
            ('BadgeRunning.TLabel', ACCENT_START),
            ('BadgeStopping.TLabel', ACCENT_KILL),
        ):
            style.configure(name, background=bg, foreground='white',
                             font=('TkDefaultFont', 10, 'bold'), padding=(10, 5))

    def _build_widgets(self):
        self._setup_style()

        header = ttk.Frame(self.root, padding=(16, 14, 16, 4))
        header.pack(fill='x')
        ttk.Label(header, text='main_bot Sim Control', style='Header.TLabel').pack(side='left')

        params = ttk.LabelFrame(self.root, text='SPAWN PARAMETERS', padding=(14, 10))
        params.pack(fill='x', padx=16, pady=(10, 6))

        self.world_var = tk.StringVar(value='')
        self.robot_name_var = tk.StringVar(value='go1')
        self.x_var = tk.StringVar(value='0.0')
        self.y_var = tk.StringVar(value='0.0')
        self.z_var = tk.StringVar(value='0.32')

        fields = [
            ('World (blank = default)', self.world_var, 16),
            ('Robot name', self.robot_name_var, 10),
            ('X', self.x_var, 6),
            ('Y', self.y_var, 6),
            ('Z', self.z_var, 6),
        ]
        for col, (label, var, width) in enumerate(fields):
            ttk.Label(params, text=label).grid(
                row=0, column=2 * col, padx=(0 if col == 0 else 4, 6), pady=4, sticky='w')
            ttk.Entry(params, textvariable=var, width=width).grid(
                row=0, column=2 * col + 1, padx=(0, 14), pady=4, sticky='w')

        sim_row = ttk.Frame(self.root, padding=(16, 4))
        sim_row.pack(fill='x')
        ttk.Label(sim_row, text='Sim', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            sim_row, key='sim', start_text='▶  Start sim', stop_text='■  Stop sim')
        ttk.Label(
            sim_row, foreground=FG_MUTED,
            text='  (Gazebo, robot đi được thật - dùng MOVEMENT bên dưới)'
        ).pack(side='left')

        rviz_row = ttk.Frame(self.root, padding=(16, 4))
        rviz_row.pack(fill='x')
        ttk.Label(rviz_row, text='RViz', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            rviz_row, key='rviz', start_text='▶  Open RViz', stop_text='■  Close RViz')
        ttk.Label(
            rviz_row, foreground=FG_MUTED,
            text='  (xem riêng, không vật lý - Sim ở trên đã tự mở RViz kèm theo rồi)'
        ).pack(side='left')

        self._build_movement_panel()
        self._build_balance_chart()

        util_row = ttk.Frame(self.root, padding=(16, 4))
        util_row.pack(fill='x')
        ttk.Label(util_row, text='', width=6).pack(side='left')
        self.kill_button = ttk.Button(
            util_row, text='Kill gz/ros traces', style='Kill.TButton', command=self.kill_traces)
        self.kill_button.pack(side='left')

        self.shutdown_button = ttk.Button(
            util_row, text='⏻  Tắt hết & Thoát', style='Stop.TButton',
            command=self.shutdown_everything)
        self.shutdown_button.pack(side='left', padx=(10, 0))

        log_frame = ttk.LabelFrame(self.root, text='LOG', padding=(1, 1))
        log_frame.pack(fill='both', expand=True, padx=16, pady=(6, 16))

        # Kept in 'normal' state (not 'disabled') so mouse selection and Ctrl+C
        # copy work reliably across Tk versions; _block_edit below blocks
        # actual typing while still letting programmatic .insert() calls through.
        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap='word', font=('DejaVu Sans Mono', 10),
            bg=LOG_BG, fg=LOG_FG, insertbackground=LOG_FG,
            selectbackground='#264f78', selectforeground='white',
            relief='flat', borderwidth=0, padx=10, pady=8)
        self.log_text.pack(fill='both', expand=True)
        self.log_text.bind('<Key>', self._block_edit)
        self.log_text.vbar.configure(
            bg=BG_PANEL, troughcolor=BG, activebackground=BORDER,
            bd=0, highlightthickness=0)

    def _build_process_controls(self, parent, key, start_text, stop_text):
        start_button = ttk.Button(
            parent, text=start_text, style='Start.TButton',
            command=lambda: self.start_process(key))
        start_button.pack(side='left')

        stop_button = ttk.Button(
            parent, text=stop_text, style='Stop.TButton',
            command=lambda: self.stop_process(key), state='disabled')
        stop_button.pack(side='left', padx=(10, 0))

        status_label = ttk.Label(parent, style='BadgeIdle.TLabel')
        status_label.pack(side='right')

        self.proc_widgets[key] = {'start': start_button, 'stop': stop_button, 'status': status_label}
        self._set_badge(status_label, 'Idle', 'idle')

    def _build_movement_panel(self):
        move_frame = ttk.LabelFrame(
            self.root, text='MOVEMENT (needs Sim running)', padding=(14, 10))
        move_frame.pack(fill='x', padx=16, pady=(6, 6))

        # FSM buttons: publish unitree_guide_controller/Inputs 'command' field directly.
        # Stand automates the Passive->FixedDown->FixedStand sequence (2, wait, 2
        # again) discovered while testing the gait controller - it needs the
        # first press to settle before the second one is meaningful.
        fsm_row = ttk.Frame(move_frame)
        fsm_row.pack(fill='x', pady=(0, 8))
        stand_button = ttk.Button(fsm_row, text='Đứng lên', command=self.send_stand)
        stand_button.pack(side='left')
        trot_button = ttk.Button(
            fsm_row, text='Đi (trot)',
            command=lambda: self._publish_control_input(command=4))
        trot_button.pack(side='left', padx=(8, 0))
        # Trotting -> FixedStand is command=2 from that state (verified from
        # StateTrotting::checkChange source) - actually stops the gait and settles
        # back into a stable stand, unlike the D-pad's Dừng which only zeroes
        # velocity and leaves the robot trotting in place.
        stop_walk_button = ttk.Button(
            fsm_row, text='Dừng đi (đứng lại)', command=self.send_stop_walking)
        stop_walk_button.pack(side='left', padx=(8, 0))
        passive_button = ttk.Button(
            fsm_row, text='Nằm',
            command=lambda: self._publish_control_input(command=1))
        passive_button.pack(side='left', padx=(8, 0))

        # D-pad: click a direction to start publishing that Inputs command on a
        # repeating root.after() timer (~10Hz - a single publish doesn't sustain
        # motion), click Dừng to stop. Deliberately plain command= callbacks, same
        # mechanism as the FSM buttons above - an earlier version used
        # <ButtonPress-1>/<ButtonRelease-1> bindings for press-and-hold, which
        # turned out unreliable on ttk.Button (events silently dropped some
        # clicks). All 6 directions confirmed against real robot motion (gz
        # ground-truth pose/yaw delta while clicking each button). lx and rx signs
        # are both flipped from the naive guess because StateTrotting.cpp negates
        # both: v_cmd_body_y = -invNormalize(lx, ...) and
        # d_yaw_cmd_ = -invNormalize(rx, ...) - ly (forward/back) is the only axis
        # that isn't negated.
        dpad_row = ttk.Frame(move_frame)
        dpad_row.pack(fill='x')
        directions = [
            ('▲ Tiến', 0.0, 0.3, 0.0),
            ('▼ Lùi', 0.0, -0.3, 0.0),
            ('◀ Trái', -0.3, 0.0, 0.0),
            ('▶ Phải', 0.3, 0.0, 0.0),
            ('↺ Xoay trái', 0.0, 0.0, -0.3),
            ('↻ Xoay phải', 0.0, 0.0, 0.3),
        ]
        dpad_buttons = []
        for text, lx, ly, rx in directions:
            btn = ttk.Button(
                dpad_row, text=text,
                command=lambda lx=lx, ly=ly, rx=rx: self._start_move(lx, ly, rx))
            btn.pack(side='left', padx=(0 if not dpad_buttons else 6, 0))
            dpad_buttons.append(btn)

        stop_move_button = ttk.Button(dpad_row, text='⏹ Dừng', command=self._stop_move)
        stop_move_button.pack(side='left', padx=(14, 0))
        dpad_buttons.append(stop_move_button)

        hint = ttk.Label(
            move_frame, foreground=FG_MUTED, justify='left',
            text=(
                'Bấm một hướng để đi liên tục theo hướng đó, bấm Dừng để ngừng.\n'
                'Muốn xoay sau khi đã đi thẳng: bấm "Dừng đi (đứng lại)" rồi "Đi '
                '(trot)" lại trước khi xoay - xoay ngay sau khi đi thẳng (chưa qua '
                'FixedStand) dễ làm robot ngã, do trạng thái nội bộ bộ điều khiển '
                'bị lệch (chưa rõ nguyên nhân sâu, xoay từ trạng thái đứng vừa '
                'trot lại thì luôn ổn định).'
            ))
        hint.pack(anchor='w', pady=(6, 0))

        self.move_widgets = [
            stand_button, trot_button, stop_walk_button, passive_button, *dpad_buttons]
        self._set_move_controls_enabled(False)

    def _set_move_controls_enabled(self, enabled):
        state = 'normal' if enabled else 'disabled'
        for widget in self.move_widgets:
            widget.configure(state=state)

    def _build_balance_chart(self):
        # Plain tk.Canvas, not matplotlib: this machine's apt-installed
        # matplotlib is broken (compiled against numpy 1.x, a numpy 2.x is
        # what actually resolves at import time) - a canvas avoids that
        # entirely and needs no extra dependency.
        chart_frame = ttk.LabelFrame(
            self.root, text='CÂN BẰNG - roll/pitch theo thời gian thực (từ /imu)',
            padding=(14, 10))
        chart_frame.pack(fill='x', padx=16, pady=(6, 6))

        self.chart_canvas = tk.Canvas(
            chart_frame, height=140, bg=LOG_BG, highlightthickness=0)
        self.chart_canvas.pack(fill='x')

        legend = ttk.Frame(chart_frame)
        legend.pack(fill='x', pady=(4, 0))
        ttk.Label(legend, text='● roll', foreground=self._CHART_ROLL_COLOR).pack(side='left')
        ttk.Label(legend, text='   ● pitch', foreground=self._CHART_PITCH_COLOR).pack(side='left')
        ttk.Label(
            legend, foreground=FG_MUTED,
            text=f'   (thang đo cố định ±{CHART_SCALE_RAD:.2f} rad - gần mép là sắp ngã)'
        ).pack(side='left')

        self.chart_canvas.bind('<Configure>', lambda _e: self._redraw_chart())

    _CHART_ROLL_COLOR = ACCENT_STOP_HOVER
    _CHART_PITCH_COLOR = ACCENT_START_HOVER
    _CHART_GRID_COLOR = '#3c3c3c'

    def _on_imu_msg(self, msg):
        x, y, z, w = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self._latest_roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        self._latest_pitch = math.asin(sinp)

    def _chart_tick(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        self._chart_buffer.append((time.time(), self._latest_roll, self._latest_pitch))
        self._redraw_chart()
        self._chart_after_id = self.root.after(CHART_TICK_MS, self._chart_tick)

    def _redraw_chart(self):
        canvas = self.chart_canvas
        canvas.delete('all')
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        if width <= 1:
            return

        mid_y = height / 2

        def value_to_y(value):
            return mid_y - (value / CHART_SCALE_RAD) * mid_y

        # Gridlines at 0 and +/-CHART_SCALE_RAD (canvas edges).
        canvas.create_line(0, mid_y, width, mid_y, fill=self._CHART_GRID_COLOR)
        canvas.create_text(
            4, mid_y, text='0', fill=FG_MUTED, anchor='w', font=('TkDefaultFont', 8))
        canvas.create_text(
            4, 8, text=f'+{CHART_SCALE_RAD:.1f}', fill=FG_MUTED, anchor='w',
            font=('TkDefaultFont', 8))
        canvas.create_text(
            4, height - 8, text=f'-{CHART_SCALE_RAD:.1f}', fill=FG_MUTED, anchor='w',
            font=('TkDefaultFont', 8))

        if len(self._chart_buffer) < 2:
            return

        t_newest = self._chart_buffer[-1][0]
        t_oldest = t_newest - CHART_HISTORY_SECONDS

        def time_to_x(t):
            return width * (t - t_oldest) / (t_newest - t_oldest) if t_newest > t_oldest else width

        roll_points = []
        pitch_points = []
        for t, roll, pitch in self._chart_buffer:
            x = time_to_x(t)
            roll_points.extend((x, value_to_y(max(-CHART_SCALE_RAD, min(CHART_SCALE_RAD, roll)))))
            pitch_points.extend((x, value_to_y(max(-CHART_SCALE_RAD, min(CHART_SCALE_RAD, pitch)))))

        canvas.create_line(*roll_points, fill=self._CHART_ROLL_COLOR, width=2)
        canvas.create_line(*pitch_points, fill=self._CHART_PITCH_COLOR, width=2)

    def send_stand(self):
        self._publish_control_input(command=2)
        self.root.after(2000, lambda: self._publish_control_input(command=2))

    def send_stop_walking(self):
        self._stop_move_process()
        self._publish_control_input(command=2, lx=0.0, ly=0.0, rx=0.0, ry=0.0)

    def _publish_control_input(self, command=0, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        msg = Inputs()
        msg.command = command
        msg.lx = lx
        msg.ly = ly
        msg.rx = rx
        msg.ry = ry
        self.control_input_pub.publish(msg)

    def _start_move(self, lx, ly, rx):
        self._move_target_lx, self._move_target_ly, self._move_target_rx = lx, ly, rx
        self._ensure_move_ticking()

    def _stop_move(self):
        """D-pad Dừng: ramp smoothly down to zero rather than an instant stop -
        a sudden full-to-zero step is its own kind of abrupt command."""
        self._move_target_lx = self._move_target_ly = self._move_target_rx = 0.0
        self._ensure_move_ticking()

    def _ensure_move_ticking(self):
        if self._move_after_id is None:
            self._move_tick()

    @staticmethod
    def _ramp_toward(current, target):
        if current < target:
            return min(current + MOVE_RAMP_STEP, target)
        if current > target:
            return max(current - MOVE_RAMP_STEP, target)
        return current

    def _move_tick(self):
        self._move_cur_lx = self._ramp_toward(self._move_cur_lx, self._move_target_lx)
        self._move_cur_ly = self._ramp_toward(self._move_cur_ly, self._move_target_ly)
        self._move_cur_rx = self._ramp_toward(self._move_cur_rx, self._move_target_rx)
        self._publish_control_input(
            command=0, lx=self._move_cur_lx, ly=self._move_cur_ly, rx=self._move_cur_rx)

        at_rest = (self._move_cur_lx, self._move_cur_ly, self._move_cur_rx) == (0.0, 0.0, 0.0)
        at_target = (self._move_target_lx, self._move_target_ly, self._move_target_rx) == (0.0, 0.0, 0.0)
        if at_rest and at_target:
            self._move_after_id = None
        else:
            self._move_after_id = self.root.after(MOVE_TICK_MS, self._move_tick)

    def _stop_move_process(self):
        """Hard/immediate stop, no ramp - used when Sim itself is stopping (no
        point easing toward a controller that's going away) or when handing off
        to a different FSM command (send_stop_walking)."""
        if self._move_after_id is not None:
            self.root.after_cancel(self._move_after_id)
            self._move_after_id = None
        self._move_cur_lx = self._move_cur_ly = self._move_cur_rx = 0.0
        self._move_target_lx = self._move_target_ly = self._move_target_rx = 0.0

    def _block_edit(self, event):
        ctrl_held = bool(event.state & 0x4)
        if ctrl_held and event.keysym.lower() in ('c', 'a'):
            return None
        if event.keysym in ('Left', 'Right', 'Up', 'Down', 'Prior', 'Next', 'Home', 'End'):
            return None
        return 'break'

    def _append_log(self, text):
        self.log_text.insert('end', text)
        self.log_text.see('end')

    def _set_badge(self, label, text, kind):
        label.configure(text='●  ' + text, style=_STATUS_STYLES[kind])

    def _command_for(self, key):
        if key == 'sim':
            cmd = [
                'ros2', 'launch', 'main_bot', 'sim.launch.py',
                'robot_name:=' + self.robot_name_var.get(),
                'x:=' + self.x_var.get(),
                'y:=' + self.y_var.get(),
                'z:=' + self.z_var.get(),
                # sim.launch.py opens its own RViz by default; the GUI keeps a
                # separate dedicated RViz row/button, so skip the bundled one
                # here to avoid two RViz windows opening at once.
                'rviz:=false',
            ]
            world = self.world_var.get().strip()
            if world:
                cmd.append('world:=' + world)
            return cmd
        if key == 'rviz':
            return ['ros2', 'launch', 'main_bot', 'rz.launch.py']
        raise ValueError(key)

    def start_process(self, key):
        if self.procs[key] is not None:
            return

        cmd = self._command_for(key)
        self._append_log(f'$ [{key}] ' + ' '.join(cmd) + '\n')

        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except OSError as exc:
            self._append_log(f'Failed to start {key}: {exc}\n')
            return

        self.procs[key] = proc
        widgets = self.proc_widgets[key]
        widgets['start'].configure(state='disabled')
        widgets['stop'].configure(state='normal')
        self._set_badge(widgets['status'], f'Running (pid {proc.pid})', 'running')
        if key == 'sim':
            self._set_move_controls_enabled(True)

        threading.Thread(target=self._read_proc_output, args=(key, proc), daemon=True).start()

    def _read_proc_output(self, key, proc):
        for line in proc.stdout:
            self.log_queue.put(('log', key, line))
        returncode = proc.wait()
        self.log_queue.put(('exited', key, returncode))

    def _poll_log_queue(self):
        try:
            while True:
                kind, key, payload = self.log_queue.get_nowait()
                if kind == 'log':
                    self._append_log(payload)
                elif kind == 'exited':
                    self._append_log(f'--- [{key}] exited (code {payload}) ---\n')
                    self.procs[key] = None
                    widgets = self.proc_widgets[key]
                    widgets['start'].configure(state='normal')
                    widgets['stop'].configure(state='disabled')
                    self._set_badge(widgets['status'], 'Idle', 'idle')
                    if key == 'sim':
                        self._set_move_controls_enabled(False)
                        self._stop_move_process()
                elif kind == 'kill_done':
                    self.kill_button.configure(state='normal')
        except queue.Empty:
            pass
        self._poll_log_after_id = self.root.after(100, self._poll_log_queue)

    def stop_process(self, key):
        proc = self.procs[key]
        if proc is None:
            return
        self._set_badge(self.proc_widgets[key]['status'], 'Stopping...', 'stopping')
        if key == 'sim':
            self._set_move_controls_enabled(False)
            self._stop_move_process()
        pid = proc.pid
        self._send_signal_to_group(pid, signal.SIGINT)
        self.root.after(STOP_GRACE_SECONDS * 1000, lambda: self._escalate_stop(key, pid))

    def _escalate_stop(self, key, pid):
        proc = self.procs[key]
        if proc is None or proc.pid != pid:
            return
        self._append_log(f'--- [{key}] still alive after SIGINT, sending SIGTERM ---\n')
        self._send_signal_to_group(pid, signal.SIGTERM)

    def _send_signal_to_group(self, pid, sig):
        try:
            os.killpg(os.getpgid(pid), sig)
        except ProcessLookupError:
            pass

    def kill_traces(self):
        self.kill_button.configure(state='disabled')
        threading.Thread(target=self._run_kill_traces, daemon=True).start()

    def _run_kill_traces(self):
        self.log_queue.put(('log', None, '$ ros2 run main_bot kill_gz.sh\n'))
        try:
            result = subprocess.run(
                ['ros2', 'run', 'main_bot', 'kill_gz.sh'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            self.log_queue.put(('log', None, result.stdout))
        except OSError as exc:
            self.log_queue.put(('log', None, f'Failed to run kill_gz.sh: {exc}\n'))
        self.log_queue.put(('kill_done', None, None))

    def shutdown_everything(self):
        """One-click full shutdown: stop every running launch, sweep any stray
        gz/ros processes with kill_gz.sh, then close the app - so quitting
        never requires switching to the terminal the GUI was started from."""
        self.shutdown_button.configure(state='disabled', text='Đang tắt...')
        self.kill_button.configure(state='disabled')
        self._stop_move_process()
        for proc in self.procs.values():
            if proc is not None:
                self._send_signal_to_group(proc.pid, signal.SIGINT)
        self.root.after(STOP_GRACE_SECONDS * 1000, self._finish_shutdown)

    def _finish_shutdown(self):
        try:
            subprocess.run(
                ['ros2', 'run', 'main_bot', 'kill_gz.sh'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            pass
        self._on_close()

    def _on_close(self):
        self._stop_move_process()
        for proc in self.procs.values():
            if proc is not None:
                self._send_signal_to_group(proc.pid, signal.SIGINT)
        # Cancel the two recurring timers explicitly - otherwise one can fire
        # after root.destroy() below and Tk raises "invalid command name ..."
        # trying to run a callback tied to a now-dead widget.
        self.root.after_cancel(self._poll_log_after_id)
        self.root.after_cancel(self._chart_after_id)
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main():
    root = tk.Tk()
    SimControlGui(root)
    root.mainloop()


if __name__ == '__main__':
    main()
