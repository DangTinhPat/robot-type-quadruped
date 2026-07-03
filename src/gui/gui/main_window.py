"""Tkinter control panel for the main_bot Gazebo sim and RViz view.

Replaces the "open one terminal per task" workflow (launch the sim in one
terminal, RViz in another, kill leftover gz/ros processes in a third, ...)
with a single window: set the spawn parameters, start/stop `ros2 launch
main_bot sim.launch.py` and `rz.launch.py`, watch their log, and clear
stale gz/ros processes on demand.
"""

import os
import queue
import signal
import subprocess
import threading
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

STOP_GRACE_SECONDS = 5

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
        self.root.geometry('900x680')
        self.root.minsize(760, 520)

        self.procs = {'sim': None, 'rviz': None}
        self.proc_widgets = {}
        self.log_queue = queue.Queue()

        self._build_widgets()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.after(100, self._poll_log_queue)

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
        self.z_var = tk.StringVar(value='0.4')

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

        rviz_row = ttk.Frame(self.root, padding=(16, 4))
        rviz_row.pack(fill='x')
        ttk.Label(rviz_row, text='RViz', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            rviz_row, key='rviz', start_text='▶  Open RViz', stop_text='■  Close RViz')

        util_row = ttk.Frame(self.root, padding=(16, 4))
        util_row.pack(fill='x')
        ttk.Label(util_row, text='', width=6).pack(side='left')
        self.kill_button = ttk.Button(
            util_row, text='Kill gz/ros traces', style='Kill.TButton', command=self.kill_traces)
        self.kill_button.pack(side='left')

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
                elif kind == 'kill_done':
                    self.kill_button.configure(state='normal')
        except queue.Empty:
            pass
        self.root.after(100, self._poll_log_queue)

    def stop_process(self, key):
        proc = self.procs[key]
        if proc is None:
            return
        self._set_badge(self.proc_widgets[key]['status'], 'Stopping...', 'stopping')
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

    def _on_close(self):
        for proc in self.procs.values():
            if proc is not None:
                self._send_signal_to_group(proc.pid, signal.SIGINT)
        self.root.destroy()


def main():
    root = tk.Tk()
    SimControlGui(root)
    root.mainloop()


if __name__ == '__main__':
    main()
