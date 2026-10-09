#!/usr/bin/env python3
"""ehand6_gui: single-shot command GUI for ehand6_ctrl (tkinter + rclpy).

Per joint: sliders for position / speed / torque (all 0..1) and a "send" checkbox.
The [Send] button publishes ONE sensor_msgs/JointState on `joint_command`
(position, velocity = speed, effort = torque). Nothing is published periodically.
Also: Reset / Emergency-stop buttons (std_srvs/Trigger) and a live view of `hand_state`.
"""
import queue
import threading
import tkinter as tk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from ehand6_msgs.msg import HandState

DEFAULT_JOINTS = ['thumb_h', 'thumb_v', 'index', 'middle', 'ring', 'little']
COLUMNS = ['position', 'speed', 'torque']
COLUMN_DEFAULTS = {'position': 0.5, 'speed': 0.5, 'torque': 0.8}

STATE_NAMES = {0: 'init', 1: 'standby', 2: 'calibrating', 3: 'position', 4: 'reserved4',
               5: 'aging', 6: 'FAULT', 7: 'waiting'}
FAULT_NAMES = {0: 'none', 1: 'overcurrent', 2: 'overvoltage', 3: 'undervoltage',
               4: 'overheat', 5: 'stall', 6: 'comm timeout', 7: 'hardware'}


class GuiNode(Node):
    """ROS side. Callbacks run in the executor thread and only post events to a queue."""

    def __init__(self, events):
        super().__init__('ehand6_gui')
        self.events = events
        self.joint_names = list(self.declare_parameter('joint_names', DEFAULT_JOINTS).value)
        cmd_topic = self.declare_parameter('command_topic', 'joint_command').value
        state_topic = self.declare_parameter('state_topic', 'hand_state').value
        reset_srv = self.declare_parameter('reset_service', '/ehand6_ctrl/reset').value
        estop_srv = self.declare_parameter('estop_service', '/ehand6_ctrl/emergency_stop').value

        self.pub = self.create_publisher(JointState, cmd_topic, 10)
        self.create_subscription(HandState, state_topic, self._on_state, 10)
        self.cli_s = {
            'reset': self.create_client(Trigger, reset_srv),
            'emergency stop': self.create_client(Trigger, estop_srv),
        }

    def _on_state(self, msg):
        self.events.put(('state', msg))

    def publish_command(self, names, position, velocity, effort):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(v) for v in position]
        msg.velocity = [float(v) for v in velocity]
        msg.effort = [float(v) for v in effort]
        self.pub.publish(msg)

    def call_trigger(self, label):
        """Returns False if the service is not available."""
        cli = self.cli_s[label]
        if not cli.service_is_ready():
            return False
        future = cli.call_async(Trigger.Request())
        future.add_done_callback(lambda f, lb=label: self._on_trigger_done(lb, f))
        return True

    def _on_trigger_done(self, label, future):
        try:
            res = future.result()
            self.events.put(('trigger_done', label, res.success, res.message))
        except Exception as e:  # noqa: BLE001
            self.events.put(('trigger_done', label, False, str(e)))


class App:
    """Tk side (main thread only)."""

    def __init__(self, root, node, events):
        self.root, self.node, self.events = root, node, events
        root.title('eHand-6 command')
        names = node.joint_names

        self.send_vars = [tk.BooleanVar(value=True) for _ in names]
        self.vars = {c: [tk.DoubleVar(value=COLUMN_DEFAULTS[c]) for _ in names] for c in COLUMNS}

        grid = tk.Frame(root)
        grid.pack(padx=8, pady=8)
        tk.Label(grid, text='joint').grid(row=0, column=0)
        tk.Label(grid, text='send').grid(row=0, column=1)
        for ci, c in enumerate(COLUMNS):
            tk.Label(grid, text=f'{c} (0..1)').grid(row=0, column=2 + ci)

        # joint rows (grid rows 2..), "all" row is row 1 and is created afterwards
        for i, name in enumerate(names):
            r = 2 + i
            tk.Label(grid, text=name, anchor='w', width=9).grid(row=r, column=0, sticky='w')
            tk.Checkbutton(grid, variable=self.send_vars[i]).grid(row=r, column=1)
            for ci, c in enumerate(COLUMNS):
                self._scale(grid, variable=self.vars[c][i]).grid(row=r, column=2 + ci, padx=4)

        tk.Label(grid, text='all', anchor='w', width=9, fg='#555').grid(row=1, column=0, sticky='w')
        self.all_send = tk.BooleanVar(value=True)
        tk.Checkbutton(grid, variable=self.all_send, command=self._toggle_all_send).grid(row=1, column=1)
        for ci, c in enumerate(COLUMNS):
            s = self._scale(grid, command=lambda v, col=c: self._set_all(col, float(v)))
            s.set(COLUMN_DEFAULTS[c])
            s.grid(row=1, column=2 + ci, padx=4)

        bar = tk.Frame(root)
        bar.pack(padx=8, pady=(0, 4), fill='x')
        tk.Button(bar, text='Send (position mode)', width=22, command=self.on_send,
                  bg='#2e7d32', fg='white').pack(side='left')
        self.btn_reset = tk.Button(bar, text='Reset', width=10,
                                   command=lambda: self.on_trigger('reset'))
        self.btn_reset.pack(side='left', padx=8)
        tk.Button(bar, text='EMERGENCY STOP', width=16, bg='#c62828', fg='white',
                  command=lambda: self.on_trigger('emergency stop')).pack(side='right')

        self.status = tk.StringVar(value='ready')
        tk.Label(root, textvariable=self.status, anchor='w', fg='#1565c0').pack(fill='x', padx=8)

        self.state_text = tk.StringVar(value='(no hand_state received yet)')
        tk.Label(root, textvariable=self.state_text, justify='left', anchor='w',
                 font='TkFixedFont').pack(fill='x', padx=8, pady=8)

        root.after(50, self._poll)

    @staticmethod
    def _scale(parent, **kw):
        return tk.Scale(parent, from_=0.0, to=1.0, resolution=0.01, orient=tk.HORIZONTAL,
                        length=200, **kw)

    def _set_all(self, col, value):
        for v in self.vars[col]:
            v.set(value)

    def _toggle_all_send(self):
        for v in self.send_vars:
            v.set(self.all_send.get())

    # ---- actions
    def on_send(self):
        names, pos, vel, eff = [], [], [], []
        for i, name in enumerate(self.node.joint_names):
            if not self.send_vars[i].get():
                continue
            names.append(name)
            pos.append(round(self.vars['position'][i].get(), 2))
            vel.append(round(self.vars['speed'][i].get(), 2))
            eff.append(round(self.vars['torque'][i].get(), 2))
        if not names:
            self.status.set('no joint selected')
            return
        self.node.publish_command(names, pos, vel, eff)
        self.status.set('sent: ' + ', '.join(
            f'{n} p={p:.2f} s={s:.2f} t={t:.2f}' for n, p, s, t in zip(names, pos, vel, eff)))

    def on_trigger(self, label):
        if not self.node.call_trigger(label):
            self.status.set(f'{label}: service not available (is ehand6_ctrl running?)')
            return
        self.status.set(f'{label}: requested...')
        if label == 'reset':
            self.btn_reset.config(state='disabled')  # re-enabled when the response arrives

    # ---- events from the ROS thread
    def _poll(self):
        try:
            while True:
                ev = self.events.get_nowait()
                if ev[0] == 'state':
                    self.state_text.set(self._format_state(ev[1]))
                elif ev[0] == 'trigger_done':
                    _, label, ok, message = ev
                    self.status.set(f'{label}: {"OK" if ok else "FAILED"} - {message}')
                    if label == 'reset':
                        self.btn_reset.config(state='normal')
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    @staticmethod
    def _format_state(m):
        lines = [f'source={m.source}  all_standby={m.all_standby}  '
                 f'system={STATE_NAMES.get(m.system_state, "?")}/'
                 f'{FAULT_NAMES.get(m.system_fault, "?")}',
                 f'{"joint":<9}{"state":<13}{"fault":<14}{"pos":>6}{"vel":>6}']
        for j in m.joints:
            lines.append(f'{j.name:<9}{STATE_NAMES.get(j.state, "?"):<13}'
                         f'{FAULT_NAMES.get(j.fault, "?"):<14}{j.position:>6.2f}{j.velocity:>6.2f}')
        return '\n'.join(lines)


def main(args=None):
    rclpy.init(args=args)
    events = queue.Queue()
    node = GuiNode(events)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        root = tk.Tk()
        App(root, node, events)
        root.mainloop()
    except tk.TclError as e:
        print(f'cannot open GUI (no DISPLAY?): {e}')
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
