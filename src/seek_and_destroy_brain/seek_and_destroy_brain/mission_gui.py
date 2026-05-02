#!/usr/bin/env python3
"""
mission_gui.py - Seek & Destroy Mission Control GUI
Tkinter-based GUI that runs alongside Joyce's xterm state machine.
Run: ros2 run seek_and_destroy_brain mission_gui
"""


import tkinter as tk
from tkinter import ttk, font
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import os


BG        = '#07090f'
PANEL     = '#0d1120'
BORDER    = '#1c2640'
CYAN      = '#00e5ff'
PINK      = '#ff2d78'
GREEN     = '#00ff88'
YELLOW    = '#ffd700'
ORANGE    = '#ff6b35'
DIM       = '#3a5070'
TEXT      = '#b8cce0'
WHITE     = '#ffffff'




class MissionGUI(Node):
    def __init__(self):
        super().__init__('mission_gui')


        self.state           = 'IDLE'
        self.last_detection  = 'None'
        self.last_confidence = 0.0
        self.log_lines       = []


        self.create_subscription(String, '/detection/result',  self._on_detection, 10)
        self.create_subscription(String, '/mission/status',    self._on_status,    10)
        self.create_subscription(String, '/mission/state',     self._on_state,     10)


        self.cmd_pub    = self.create_publisher(String, '/mission/command',    10)
        self.resume_pub = self.create_publisher(Bool,   '/explore/resume',     10)
        self.target_pub = self.create_publisher(String, '/mission/set_target', 10)


        self.get_logger().info('Mission GUI node started')


    def _on_detection(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) >= 2:
            try:
                name = parts[0].strip()
                conf = float(parts[1])
                if name == 'none' or conf <= 0.01:
                    self.last_detection  = None
                    self.last_confidence = 0.0
                else:
                    self.last_detection  = name
                    self.last_confidence = conf
            except ValueError:
                pass


    def _on_status(self, msg: String):
        self._log(f'STATUS: {msg.data}')


    def _on_state(self, msg: String):
        old = self.state
        self.state = msg.data
        if old != self.state:
            self._log(f'STATE -> {self.state}')
            # Clear stale detection when transitioning states
            if self.state == 'EXPLORING':
                self.last_detection  = 'None'
                self.last_confidence = 0.0


    def _log(self, text: str):
        ts = time.strftime('%H:%M:%S')
        self.log_lines.append(f'[{ts}] {text}')
        if len(self.log_lines) > 100:
            self.log_lines.pop(0)


    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self._log(f'CMD -> {cmd}')


    def send_target(self, name: str):
        msg = String()
        msg.data = name
        self.target_pub.publish(msg)
        self._log(f'TARGET -> {name}')


    def set_explore(self, resume: bool):
        msg = Bool()
        msg.data = resume
        self.resume_pub.publish(msg)
        self._log(f'EXPLORE {"RESUMED" if resume else "PAUSED"}')




class App:
    def __init__(self, root: tk.Tk, node: MissionGUI):
        self.root    = root
        self.node    = node
        self.running = True
        self._start  = time.time()


        root.title('SEEK & DESTROY — MISSION CONTROL')
        root.configure(bg=BG)
        root.geometry('900x680')
        root.resizable(True, True)


        self.f_title  = font.Font(family='Courier', size=16, weight='bold')
        self.f_header = font.Font(family='Courier', size=12, weight='bold')
        self.f_mono   = font.Font(family='Courier', size=11)
        self.f_big    = font.Font(family='Courier', size=24, weight='bold')
        self.f_small  = font.Font(family='Courier', size=10)


        self._build_ui()
        self._update_loop()


    def _build_ui(self):
        # Header
        hdr = tk.Frame(self.root, bg=PANEL, pady=12)
        hdr.pack(fill='x', padx=8, pady=(8, 4))
        tk.Label(hdr, text='SEEK', bg=PANEL, fg=CYAN,  font=self.f_title).pack(side='left', padx=(12, 0))
        tk.Label(hdr, text=' & ', bg=PANEL, fg=PINK,   font=self.f_title).pack(side='left')
        tk.Label(hdr, text='DESTROY', bg=PANEL, fg=GREEN, font=self.f_title).pack(side='left')
        tk.Label(hdr, text='  //  MISSION CONTROL', bg=PANEL, fg=DIM, font=self.f_mono).pack(side='left')
        self.state_label = tk.Label(hdr, text='● IDLE', bg=PANEL, fg=YELLOW, font=self.f_header)
        self.state_label.pack(side='right', padx=12)
        self.clock_label = tk.Label(hdr, text='--:--:--', bg=PANEL, fg=DIM, font=self.f_mono)
        self.clock_label.pack(side='right', padx=12)


        # Body
        cols = tk.Frame(self.root, bg=BG)
        cols.pack(fill='both', expand=True, padx=8, pady=4)
        left  = tk.Frame(cols, bg=BG, width=300)
        right = tk.Frame(cols, bg=BG)
        left.pack(side='left', fill='y', padx=(0, 4))
        right.pack(side='left', fill='both', expand=True)
        left.pack_propagate(False)


        self._panel(left,  'TARGET SELECTION',  self._build_target_panel)
        self._panel(left,  'LIVE DETECTION',     self._build_detection_panel)
        self._panel(left,  'MISSION STATS',      self._build_stats_panel)
        self._panel(right, 'MISSION CONTROLS',   self._build_controls_panel)
        self._panel(right, 'MISSION LOG',         self._build_log_panel, expand=True)


    def _panel(self, parent, title, builder, expand=False):
        frame = tk.Frame(parent, bg=PANEL, bd=1, relief='flat')
        if expand:
            frame.pack(fill='both', expand=True, pady=(0, 4))
        else:
            frame.pack(fill='x', pady=(0, 4))
        tk.Frame(frame, bg=CYAN, height=1).pack(fill='x')
        tk.Label(frame, text=f'  {title}', bg=PANEL, fg=DIM,
                 font=self.f_small, anchor='w').pack(fill='x', padx=8, pady=(4, 0))
        inner = tk.Frame(frame, bg=PANEL)
        inner.pack(fill='both', expand=True, padx=8, pady=(2, 8))
        builder(inner)


    def _build_target_panel(self, parent):
        tk.Label(parent, text='Click to start mission:', bg=PANEL, fg=DIM,
                 font=self.f_small).pack(anchor='w', pady=(0, 4))
        self.target_buttons = {}
        for label, key, color in [
            ('🔴  Red Cylinder',   'redcylinder',   '#cc2200'),
            ('🔵  Blue Box',       'bluebox',        '#0055cc'),
            ('🟢  Green Cylinder', 'greencylinder',  '#00aa33'),
        ]:
            btn = tk.Button(
                parent, text=label, bg=PANEL, fg=color,
                font=self.f_mono, relief='flat', bd=1,
                highlightbackground=BORDER, highlightthickness=1,
                cursor='hand2', anchor='w', padx=8, pady=6,
                command=lambda k=key, l=label: self._select_target(k, l)
            )
            btn.pack(fill='x', pady=2)
            self.target_buttons[key] = btn


        self.selected_target_label = tk.Label(
            parent, text='Selected: none', bg=PANEL, fg=DIM, font=self.f_small)
        self.selected_target_label.pack(anchor='w', pady=(4, 0))


        tk.Frame(parent, bg=BORDER, height=1).pack(fill='x', pady=6)
        tk.Button(parent, text='🔄  SEARCH AGAIN', bg=PANEL, fg=YELLOW,
                  font=self.f_mono, relief='flat',
                  highlightbackground=YELLOW, highlightthickness=1,
                  cursor='hand2', padx=8, pady=6,
                  command=lambda: self.node.send_target('SEARCH_AGAIN')
                  ).pack(fill='x')


    def _build_detection_panel(self, parent):
        self.det_name_label = tk.Label(
            parent, text='Detecting: ---', bg=PANEL, fg=CYAN, font=self.f_mono)
        self.det_name_label.pack(anchor='w')
        tk.Label(parent, text='Confidence:', bg=PANEL, fg=DIM,
                 font=self.f_small).pack(anchor='w', pady=(4, 0))
        bar_frame = tk.Frame(parent, bg=BORDER, height=14)
        bar_frame.pack(fill='x', pady=2)
        bar_frame.pack_propagate(False)
        self.conf_bar = tk.Frame(bar_frame, bg=GREEN, width=0)
        self.conf_bar.pack(side='left', fill='y')
        self.conf_label = tk.Label(
            parent, text='0.000  (threshold: 0.100)', bg=PANEL, fg=GREEN, font=self.f_mono)
        self.conf_label.pack(anchor='w')


    def _build_stats_panel(self, parent):
        stats = tk.Frame(parent, bg=PANEL)
        stats.pack(fill='x')
        stats.columnconfigure(0, weight=1)
        stats.columnconfigure(1, weight=1)
        self.stat_state = self._stat_box(stats, 'IDLE',  'STATE', YELLOW, 0, 0)
        self.stat_time  = self._stat_box(stats, '00:00', 'TIME',  CYAN,   0, 1)


    def _stat_box(self, parent, val, label, color, row, col):
        frame = tk.Frame(parent, bg=BORDER, padx=8, pady=8)
        frame.grid(row=row, column=col, padx=2, pady=2, sticky='ew')
        v = tk.Label(frame, text=val, bg=BORDER, fg=color, font=self.f_big)
        v.pack()
        tk.Label(frame, text=label, bg=BORDER, fg=DIM, font=self.f_small).pack()
        return v


    def _build_controls_panel(self, parent):
        r1 = tk.Frame(parent, bg=PANEL)
        r1.pack(fill='x', pady=2)
        self._btn(r1, '▶  START EXPLORATION', GREEN,
                  lambda: self.node.set_explore(True)).pack(side='left', padx=2, fill='x', expand=True)
        self._btn(r1, '⏸  PAUSE EXPLORATION', YELLOW,
                  lambda: self.node.set_explore(False)).pack(side='left', padx=2, fill='x', expand=True)


        r2 = tk.Frame(parent, bg=PANEL)
        r2.pack(fill='x', pady=2)
        self._btn(r2, '🏠  SEND HOME', CYAN,
                  lambda: self.node.send_command('GO_HOME')).pack(side='left', padx=2, fill='x', expand=True)
        self._btn(r2, '⊘  EMERGENCY STOP', PINK,
                  self._emergency_stop).pack(side='left', padx=2, fill='x', expand=True)


        tk.Label(parent, text='SPAWN OBSTACLE:', bg=PANEL, fg=DIM,
                 font=self.f_small).pack(anchor='w', pady=(8, 2))
        r3 = tk.Frame(parent, bg=PANEL)
        r3.pack(fill='x', pady=2)
        self._btn(r3, '+ Small Box',  ORANGE,
                  lambda: self._spawn_obstacle(0.3, 0.5)).pack(side='left', padx=2, fill='x', expand=True)
        self._btn(r3, '+ Large Box',  ORANGE,
                  lambda: self._spawn_obstacle(0.6, 0.8)).pack(side='left', padx=2, fill='x', expand=True)
        self._btn(r3, '+ Cylinder',   ORANGE,
                  lambda: self._spawn_obstacle(0.25, 0.6, 'cylinder')).pack(side='left', padx=2, fill='x', expand=True)


        tk.Frame(parent, bg=BORDER, height=1).pack(fill='x', pady=8)
        self._btn(parent, '✕  QUIT MISSION', PINK, self._quit).pack(fill='x', padx=2)


    def _build_log_panel(self, parent):
        self.log_text = tk.Text(
            parent, bg='#030508', fg=TEXT, font=self.f_mono,
            relief='flat', bd=0, wrap='word', state='disabled', height=10)
        scroll = tk.Scrollbar(parent, command=self.log_text.yview, bg=BORDER)
        self.log_text.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.log_text.pack(fill='both', expand=True)
        self.log_text.tag_config('ok',   foreground=GREEN)
        self.log_text.tag_config('warn', foreground=YELLOW)
        self.log_text.tag_config('err',  foreground=PINK)
        self.log_text.tag_config('info', foreground=TEXT)
        self.log_text.tag_config('dim',  foreground=DIM)
        self._log_write('[SYSTEM] Mission Control ready', 'ok')
        self._log_write('[SYSTEM] Click target button or wait for xterm', 'dim')


    def _btn(self, parent, text, color, command):
        return tk.Button(
            parent, text=text, bg=PANEL, fg=color,
            font=self.f_mono, relief='flat', bd=1,
            highlightbackground=color, highlightthickness=1,
            cursor='hand2', padx=6, pady=8,
            activebackground=color, activeforeground=BG,
            command=command
        )


    def _select_target(self, key, label):
        for k, btn in self.target_buttons.items():
            btn.configure(highlightthickness=1)
        self.target_buttons[key].configure(highlightthickness=2)
        self.selected_target_label.configure(text=f'Selected: {label}', fg=CYAN)
        self.node.send_target(key)
        self._log_write(f'[TARGET] {label}', 'ok')


    def _emergency_stop(self):
        self.node.set_explore(False)
        self.node.send_command('GO_HOME')
        self._log_write('[STOP] Emergency stop!', 'err')


    def _quit(self):
        self.node.set_explore(False)
        self.node.send_target('QUIT')
        self._log_write('[QUIT] Shutting down...', 'err')
        self.root.after(1500, self._do_quit)


    def _do_quit(self):
        self.running = False
        self.node.destroy_node()
        rclpy.try_shutdown()
        self.root.destroy()


    def _spawn_obstacle(self, size, height, shape='box'):
        import random
        x = round(random.uniform(-2.5, 2.5), 1)
        y = round(random.uniform(-2.5, 2.5), 1)
        name = f'obs_{int(time.time())}'
        if shape == 'cylinder':
            geo = f'<cylinder><radius>{size}</radius><length>{height}</length></cylinder>'
        else:
            geo = f'<box><size>{size} {size} {height}</size></box>'
        sdf = (f"<sdf version='1.7'><model name='{name}'><static>true</static>"
               f"<link name='link'><visual name='v'><geometry>{geo}</geometry>"
               f"<material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material>"
               f"</visual><collision name='c'><geometry>{geo}</geometry></collision>"
               f"</link></model></sdf>")
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
               '-name', name, '-x', str(x), '-y', str(y), '-z', '0.0', '-string', sdf]
        threading.Thread(target=lambda: subprocess.run(cmd), daemon=True).start()
        self._log_write(f'[OBS] Spawned {shape} at ({x}, {y})', 'warn')


    def _log_write(self, text: str, tag='info'):
        self.log_text.configure(state='normal')
        self.log_text.insert('end', text + '\n', tag)
        self.log_text.see('end')
        self.log_text.configure(state='disabled')


    def _update_loop(self):
        if not self.running:
            return
        try:
            self._update_ui()
        except Exception:
            pass
        self.root.after(200, self._update_loop)


    def _update_ui(self):
        self.clock_label.configure(text=time.strftime('%H:%M:%S'))


        elapsed = int(time.time() - self._start)
        m, s = divmod(elapsed, 60)
        self.stat_time.configure(text=f'{m:02d}:{s:02d}')


        state = self.node.state
        colors = {
            'IDLE': YELLOW, 'EXPLORING': CYAN, 'HOME': GREEN,
            'SAVING_MAP': ORANGE, 'END': GREEN,
            'HOME_FAILED': PINK, 'END_FAILED': PINK,
        }
        col = colors.get(state, TEXT)
        self.state_label.configure(text=f'● {state}', fg=col)
        self.stat_state.configure(text=state[:6], fg=col)


        det  = self.node.last_detection
        conf = self.node.last_confidence
        if det and det != 'None':
            det_col = GREEN if conf >= 0.10 else ORANGE
            self.det_name_label.configure(text=f'Detecting: {det}', fg=det_col)
            self.conf_label.configure(text=f'{conf:.3f}  (threshold: 0.100)', fg=det_col)
            bar_w = min(int((conf / 0.30) * 220), 220)
            self.conf_bar.configure(width=bar_w, bg=det_col)
        else:
            # ← add this branch
            self.det_name_label.configure(text='Detecting: ---', fg=DIM)
            self.conf_label.configure(text='0.000  (threshold: 0.100)', fg=DIM)
            self.conf_bar.configure(width=0)


        new_logs = self.node.log_lines[:]
        self.node.log_lines.clear()
        for line in new_logs:
            tag = ('ok'   if any(x in line for x in ['TARGET', 'FOUND', 'ready']) else
                   'err'  if any(x in line for x in ['STOP', 'FAILED', 'QUIT']) else
                   'warn' if any(x in line for x in ['TIMEOUT', 'OBS', '⚠']) else
                   'info')
            self._log_write(line, tag)




def main(args=None):
    rclpy.init(args=args)
    node = MissionGUI()


    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()


    root = tk.Tk()
    app = App(root, node)


    def on_close():
        app.running = False
        node.destroy_node()
        rclpy.try_shutdown()
        root.destroy()


    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()

if __name__ == '__main__':
    main()