"""
Vectored Thrust PID Tuner  (Serial edition)
-------------------------------------------
PyQt5 UI to tune 2 independent PID controllers over Serial
communicating with an Arduino Nano BLE Sense Rev2.

This is the USB-serial (COM port) version. For the Bluetooth Low Energy
version, see ui_withdebug.py.

Serial protocol (Python -> Arduino):
    'p <motor> <val>\\n'     set Kp       motor: 1=Roll, 2=Pitch
    'i <motor> <val>\\n'     set Ki
    'd <motor> <val>\\n'     set Kd
    't <motor> <val>\\n'     set timestep (ms)
    'v <motor> <val>\\n'     set brushless throttle (0..100 %)  motor: 3=Brushless
    '+ <motor>\\n'           activate controller
    '- <motor>\\n'           deactivate controller
    '? <motor>\\n'           query current gains (device replies with 'Kp=.. Ki=.. Kd=.. T=..')

Gains are NOT sent live as sliders move. Use each panel's "Send" button to push
the slider values to the device, and "Query" to pull the device's real values
back into the sliders.

Serial protocol (Arduino -> Python):
    'ACC <ax>,<ay>,<az>\\n'  raw accelerometer (g)
    'GYR <gx>,<gy>,<gz>\\n'  raw gyroscope (deg/s)
    'ROL <roll>\\n'          estimated roll angle (deg)
    'PIT <pitch>\\n'         estimated pitch angle (deg)
    'SRO <rollCommand>\\n'   servo roll command
    'SPI <pitchCommand>\\n'  servo pitch command
    'THR <throttle>\\n'      brushless throttle actually running (0..100 %)
    'DA1 <value>\\n'         extra debug data channel 1 (single float)
    'DA2 <value>\\n'         extra debug data channel 2 (single float)

Install dependencies:
    pip install pyserial PyQt5 pyqtgraph
"""

import sys
import time
import threading
from collections import deque

import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QComboBox, QPushButton, QGroupBox,
    QDoubleSpinBox, QStatusBar, QSplitter, QFrame, QTextEdit, QCheckBox,
    QProgressBar
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette

try:
    import pyqtgraph as pg
    HAS_PYQTGRAPH = True
except ImportError:
    HAS_PYQTGRAPH = False

# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────

BAUD_RATE   = 115200
PLOT_WINDOW = 200
UPDATE_MS   = 50
RAW_LOG_MAX_LINES = 300   # cap the debug console so it doesn't grow forever

KP_RANGE = (0.0, 10.0, 0.01)
KI_RANGE = (0.0,  5.0, 0.001)
KD_RANGE = (0.0,  5.0, 0.001)

# motor index matches Arduino switch(motor): 1=Roll, 2=Pitch, 3=Brushless
AXES = [
    {"name": "Roll",  "motor": 1, "color": "#FFB74D"},
    {"name": "Pitch", "motor": 2, "color": "#4FC3F7"},
]

# Brushless throttle: motor index 3, 'v' command, 0..100 %, sent live.
THROTTLE_MOTOR = 3
THROTTLE_COLOR = "#81C784"

IMU_COLORS = ["#ef5350", "#81C784", "#CE93D8",   # ACC x/y/z
              "#FF8A65", "#4DD0E1", "#FFF176"]    # GYR x/y/z

# Attitude plot channels: (serial key, legend label, color, dashed?)
# Estimated angles are drawn solid; the servo commands share the axis colour
# but are drawn dashed so measured-vs-command is easy to read.
ATT_CHANNELS = [
    ("ROL", "roll",     "#FFB74D", False),   # estimated roll angle
    ("SRO", "roll cmd", "#FFB74D", True),    # servo roll command
    ("PIT", "pitch",    "#4FC3F7", False),   # estimated pitch angle
    ("SPI", "pitch cmd", "#4FC3F7", True),   # servo pitch command
]

# Extra debug data channels: (serial key, legend label, color)
# Single float per line ('DA1 <value>' / 'DA2 <value>'), meant for ad-hoc
# testing/telemetry. Both are drawn on the same plot.
DAT_CHANNELS = [
    ("DA1", "data 1", "#F06292"),
    ("DA2", "data 2", "#BA68C8"),
]

# ─────────────────────────────────────────────
#  SERIAL WORKER
# ─────────────────────────────────────────────

class SerialWorker(QObject):
    telemetry_received = pyqtSignal(str)   # every raw line, parsed or not
    connection_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._ser       = None
        self._running   = False
        self._cmd_queue = []
        self._lock      = threading.Lock()

    def connect(self, port: str):
        self.disconnect()
        try:
            self._ser     = serial.Serial(port, BAUD_RATE, timeout=0.1)
            self._running = True
            threading.Thread(target=self._loop, daemon=True).start()
            time.sleep(2)   # allow Arduino reset
            self.connection_changed.emit(True)
        except serial.SerialException as e:
            self.connection_changed.emit(False)
            raise e

    def disconnect(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None
        self.connection_changed.emit(False)

    def send_gain(self, cmd: str, motor: int, value: float):
        with self._lock:
            self._cmd_queue.append(f"{cmd} {motor} {value:.4f}\n")

    def send_action(self, cmd: str, motor: int):
        with self._lock:
            self._cmd_queue.append(f"{cmd} {motor}\n")

    def _loop(self):
        while self._running and self._ser and self._ser.is_open:
            # flush outgoing commands first
            with self._lock:
                cmds, self._cmd_queue = self._cmd_queue, []
            for c in cmds:
                try:
                    self._ser.write(c.encode())
                except serial.SerialException:
                    self._running = False
                    self.connection_changed.emit(False)
                    return

            # read every line currently buffered, not just one —
            # prevents falling behind if the Arduino sends fast bursts
            try:
                while self._ser.in_waiting:
                    raw = self._ser.readline()
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if line:
                        self.telemetry_received.emit(line)
            except serial.SerialException:
                self._running = False
                self.connection_changed.emit(False)
                return

# ─────────────────────────────────────────────
#  GAIN SLIDER
# ─────────────────────────────────────────────

class GainSlider(QWidget):
    value_changed = pyqtSignal(float)

    def __init__(self, label: str, vmin: float, vmax: float, step: float,
                 color: str, parent=None):
        super().__init__(parent)
        self._min   = vmin
        self._step  = step
        self._steps = int(round((vmax - vmin) / step))

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)

        lbl = QLabel(label)
        lbl.setFixedWidth(28)
        lbl.setFont(QFont("Consolas", 9, QFont.Bold))
        lbl.setStyleSheet(f"color: {color};")
        layout.addWidget(lbl)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self._steps)
        self.slider.setFixedHeight(18)
        self.slider.setStyleSheet(f"""
            QSlider::groove:horizontal {{
                height: 4px; background: #2a2a3a; border-radius: 2px;
            }}
            QSlider::handle:horizontal {{
                background: {color}; width: 14px; height: 14px;
                margin: -5px 0; border-radius: 7px;
            }}
            QSlider::sub-page:horizontal {{
                background: {color}; border-radius: 2px;
            }}
        """)
        layout.addWidget(self.slider, stretch=1)

        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(vmin, vmax)
        self.spinbox.setSingleStep(step)
        self.spinbox.setDecimals(3)
        self.spinbox.setFixedWidth(76)
        self.spinbox.setStyleSheet("""
            QDoubleSpinBox {
                background: #1e1e2e; color: #e0e0f0;
                border: 1px solid #3a3a5a; border-radius: 3px; padding: 1px 4px;
            }
        """)
        layout.addWidget(self.spinbox)

        self.slider.valueChanged.connect(self._slider_moved)
        self.spinbox.valueChanged.connect(self._spin_moved)

    def _slider_moved(self, tick):
        val = self._min + tick * self._step
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(val)
        self.spinbox.blockSignals(False)
        self.value_changed.emit(val)

    def _spin_moved(self, val):
        tick = int(round((val - self._min) / self._step))
        self.slider.blockSignals(True)
        self.slider.setValue(tick)
        self.slider.blockSignals(False)
        self.value_changed.emit(val)

    def set_value(self, val):
        """Programmatically move the slider (e.g. from a '?' query reply).
        Routes through the spinbox so the slider + readout stay in sync."""
        self.spinbox.setValue(val)

    def get_value(self):
        return self.spinbox.value()

# ─────────────────────────────────────────────
#  PID PANEL
# ─────────────────────────────────────────────

class PIDPanel(QGroupBox):
    """One panel per axis/servo. Emits (arduino_cmd, motor_index, value)."""

    cmd_ready    = pyqtSignal(str, int, float)   # gain commands  e.g. ('p', 1, 1.5)
    action_ready = pyqtSignal(str, int)          # action commands e.g. ('+', 1)

    def __init__(self, axis: dict, parent=None):
        super().__init__(parent)
        self._motor = axis["motor"]
        color       = axis["color"]

        self.setTitle(f'{axis["name"]}  (motor {axis["motor"]})')
        self.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.setStyleSheet(f"""
            QGroupBox {{
                border: 1px solid {color}55; border-radius: 6px;
                margin-top: 10px; padding: 8px; background: #13131f;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 12px;
                color: {color}; font-size: 11px; font-weight: bold;
                letter-spacing: 2px;
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        self.kp = GainSlider("KP", *KP_RANGE, color)
        self.ki = GainSlider("KI", *KI_RANGE, color)
        self.kd = GainSlider("KD", *KD_RANGE, color)

        # (slider, arduino-cmd-char, readout-key, format-string)
        gain_defs = [
            (self.kp, 'p', "Kp", "{:.2f}"),
            (self.ki, 'i', "Ki", "{:.3f}"),
            (self.kd, 'd', "Kd", "{:.3f}"),
        ]

        # Manual workflow: moving a slider only updates the on-screen readout.
        # Nothing is sent to the device until the user clicks "Send"; the
        # device's real values are pulled back in via "Query".
        for slider, cmd, key, fmt in gain_defs:
            layout.addWidget(slider)
            _key, _fmt = key, fmt
            slider.value_changed.connect(
                lambda v, k=_key, f=_fmt: self._readouts[k].setText(f.format(v))
            )

        def make_btn(label):
            btn = QPushButton(label)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background: {color}22; color: {color};
                    border: 1px solid {color}66; border-radius: 4px; padding: 3px 8px;
                }}
                QPushButton:hover {{ background: {color}44; }}
            """)
            return btn

        # Send: push the slider values to the device.
        # Query: ask the device for its actual values and load them into the sliders.
        btn_layout = QHBoxLayout()
        send_btn = make_btn("Send →")
        send_btn.setToolTip("Send the slider Kp/Ki/Kd to the device")
        send_btn.clicked.connect(self._emit_send)
        btn_layout.addWidget(send_btn)

        query_btn = make_btn("Query")
        query_btn.setToolTip("Read the device's real Kp/Ki/Kd back into the sliders")
        query_btn.clicked.connect(lambda: self.action_ready.emit("?", self._motor))
        btn_layout.addWidget(query_btn)
        layout.addLayout(btn_layout)

        action_layout = QHBoxLayout()
        for label, cmd in [("Activate", "+"), ("Deactivate", "-")]:
            btn = make_btn(label)
            _cmd = cmd
            btn.clicked.connect(lambda _, c=_cmd: self.action_ready.emit(c, self._motor))
            action_layout.addWidget(btn)
        layout.addLayout(action_layout)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {color}33;")
        layout.addWidget(sep)

        readout_row = QHBoxLayout()
        self._readouts = {}
        for key in ["Kp", "Ki", "Kd"]:
            box = QWidget()
            bl  = QVBoxLayout(box)
            bl.setContentsMargins(2, 2, 2, 2)
            bl.setSpacing(1)
            t = QLabel(key)
            t.setFont(QFont("Consolas", 7))
            t.setStyleSheet("color: #555580;")
            t.setAlignment(Qt.AlignCenter)
            v = QLabel("–")
            v.setFont(QFont("Consolas", 11, QFont.Bold))
            v.setStyleSheet(f"color: {color};")
            v.setAlignment(Qt.AlignCenter)
            bl.addWidget(t)
            bl.addWidget(v)
            readout_row.addWidget(box)
            self._readouts[key] = v
        layout.addLayout(readout_row)

        # Now that self._readouts exists, push the sliders' initial
        # values into the labels so they don't start on "–".
        for slider, _cmd, key, fmt in gain_defs:
            self._readouts[key].setText(fmt.format(slider.get_value()))

    def _emit_send(self):
        """Push the current slider values to the device (p/i/d for this motor)."""
        self.cmd_ready.emit("p", self._motor, self.kp.get_value())
        self.cmd_ready.emit("i", self._motor, self.ki.get_value())
        self.cmd_ready.emit("d", self._motor, self.kd.get_value())

    def apply_gains(self, kp, ki, kd):
        """Load device values (from a '?' query reply) into the sliders.
        The sliders cascade to the spinboxes and readouts automatically."""
        self.kp.set_value(kp)
        self.ki.set_value(ki)
        self.kd.set_value(kd)

# ─────────────────────────────────────────────
#  THROTTLE PANEL
# ─────────────────────────────────────────────

class ThrottlePanel(QGroupBox):
    """Brushless throttle, 0..100 %. Emits ('v', 3, percent) live as the
    slider moves — no Send button, every move goes straight to the device."""

    cmd_ready    = pyqtSignal(str, int, float)   # ('v', 3, 42.0)
    action_ready = pyqtSignal(str, int)          # ('+', 3) / ('-', 3)

    def __init__(self, parent=None):
        super().__init__(parent)
        color = THROTTLE_COLOR

        self.setTitle(f"Throttle  (motor {THROTTLE_MOTOR})")
        self.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.setStyleSheet(f"""
            QGroupBox {{
                border: 1px solid {color}55; border-radius: 6px;
                margin-top: 10px; padding: 8px; background: #13131f;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 12px;
                color: {color}; font-size: 11px; font-weight: bold;
                letter-spacing: 2px;
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        row = QHBoxLayout()
        lbl = QLabel("%")
        lbl.setFixedWidth(28)
        lbl.setFont(QFont("Consolas", 9, QFont.Bold))
        lbl.setStyleSheet(f"color: {color};")
        row.addWidget(lbl)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setFixedHeight(18)
        self.slider.setStyleSheet(f"""
            QSlider::groove:horizontal {{
                height: 4px; background: #2a2a3a; border-radius: 2px;
            }}
            QSlider::handle:horizontal {{
                background: {color}; width: 14px; height: 14px;
                margin: -5px 0; border-radius: 7px;
            }}
            QSlider::sub-page:horizontal {{
                background: {color}; border-radius: 2px;
            }}
        """)
        row.addWidget(self.slider, stretch=1)

        self._readout = QLabel("0 %")
        self._readout.setFont(QFont("Consolas", 11, QFont.Bold))
        self._readout.setStyleSheet(f"color: {color};")
        self._readout.setFixedWidth(56)
        self._readout.setAlignment(Qt.AlignCenter)
        row.addWidget(self._readout)
        layout.addLayout(row)

        # Throttle reported back by the board ('THR <value>', 1..100).
        # This is the *actual* value the Nano is running, which may differ
        # from the commanded slider position.
        meter_row = QHBoxLayout()
        meter_lbl = QLabel("THR")
        meter_lbl.setFixedWidth(28)
        meter_lbl.setFont(QFont("Consolas", 9, QFont.Bold))
        meter_lbl.setStyleSheet(f"color: {color};")
        meter_row.addWidget(meter_lbl)

        self._meter = QProgressBar()
        self._meter.setRange(1, 100)
        self._meter.setValue(1)
        self._meter.setTextVisible(False)
        self._meter.setFixedHeight(14)
        self._meter.setStyleSheet(f"""
            QProgressBar {{
                background: #2a2a3a; border: 1px solid {color}44;
                border-radius: 3px;
            }}
            QProgressBar::chunk {{
                background: {color}; border-radius: 2px;
            }}
        """)
        meter_row.addWidget(self._meter, stretch=1)

        self._meter_readout = QLabel("– %")
        self._meter_readout.setFont(QFont("Consolas", 11, QFont.Bold))
        self._meter_readout.setStyleSheet(f"color: {color};")
        self._meter_readout.setFixedWidth(56)
        self._meter_readout.setAlignment(Qt.AlignCenter)
        meter_row.addWidget(self._meter_readout)
        layout.addLayout(meter_row)

        # Activate / deactivate the brushless — '+ 3' / '- 3'.
        action_layout = QHBoxLayout()
        for label, cmd in [("Activate", "+"), ("Deactivate", "-")]:
            btn = QPushButton(label)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background: {color}22; color: {color};
                    border: 1px solid {color}66; border-radius: 4px; padding: 3px 8px;
                }}
                QPushButton:hover {{ background: {color}44; }}
            """)
            _cmd = cmd
            btn.clicked.connect(
                lambda _, c=_cmd: self.action_ready.emit(c, THROTTLE_MOTOR)
            )
            action_layout.addWidget(btn)
        layout.addLayout(action_layout)

        self.slider.valueChanged.connect(self._on_moved)

    def _on_moved(self, percent):
        self._readout.setText(f"{percent} %")
        self.cmd_ready.emit("v", THROTTLE_MOTOR, float(percent))

    def set_reported(self, percent: float):
        """Show the throttle value reported by the board ('THR <value>')."""
        clamped = max(1, min(100, int(round(percent))))
        self._meter.setValue(clamped)
        self._meter_readout.setText(f"{percent:.0f} %")


# ─────────────────────────────────────────────
#  IMU PLOT
# ─────────────────────────────────────────────

def make_imu_plot():
    pg.setConfigOptions(antialias=True, background="#0d0d1a", foreground="#555580")

    layout = pg.GraphicsLayoutWidget()
    # a little breathing room between the four plots so they don't read as
    # one cramped stack
    layout.ci.layout.setSpacing(14)
    layout.ci.setContentsMargins(6, 6, 6, 6)

    def add_plot(row, col, title, y_unit, bottom):
        """Create one styled plot. Only the bottom row of the grid shows the
        'samples' x-axis label so the middle gridlines stay clean."""
        p = layout.addPlot(row=row, col=col, title=title)
        p.showGrid(x=True, y=True, alpha=0.15)
        # legend anchored top-left, inside the view, semi-transparent so it
        # never hides the traces
        legend = p.addLegend(offset=(8, 8), labelTextSize="7pt")
        legend.setBrush(pg.mkBrush(13, 13, 26, 180))
        p.setLabel("left", y_unit)
        if bottom:
            p.setLabel("bottom", "samples")
        p.getAxis("left").setWidth(42)   # align y-axes across the grid
        return p

    # 2×2 grid: raw sensors on the left column, derived signals on the right.
    #   ┌ Accelerometer ┬ Attitude ┐
    #   └ Gyroscope     ┴ Extra    ┘
    acc_plot = add_plot(0, 0, "Accelerometer (g)",              "g",     False)
    att_plot = add_plot(0, 1, "Attitude — angle vs command (°)", "°",     False)
    gyr_plot = add_plot(1, 0, "Gyroscope (°/s)",                "°/s",   True)
    dat_plot = add_plot(1, 1, "Extra Data (DA1 / DA2)",         "value", True)

    # all plots share the same x scale (sample index) — link them so panning
    # or zooming one scrolls them all together
    for p in (att_plot, gyr_plot, dat_plot):
        p.setXLink(acc_plot)

    curves = {}
    labels = ["x", "y", "z"]
    for i, lbl in enumerate(labels):
        curves[f"ACC_{lbl}"] = acc_plot.plot([], [], name=lbl,
            pen=pg.mkPen(color=IMU_COLORS[i], width=2))
        curves[f"GYR_{lbl}"] = gyr_plot.plot([], [], name=lbl,
            pen=pg.mkPen(color=IMU_COLORS[3 + i], width=2))

    for key, label, color, dashed in ATT_CHANNELS:
        style = Qt.DashLine if dashed else Qt.SolidLine
        curves[key] = att_plot.plot([], [], name=label,
            pen=pg.mkPen(color=color, width=2, style=style))

    for key, label, color in DAT_CHANNELS:
        curves[key] = dat_plot.plot([], [], name=label,
            pen=pg.mkPen(color=color, width=2))

    return layout, curves

# ─────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Vectored Thrust — PID Tuner")
        self.setMinimumSize(1280, 900)
        # start with a generous default size so the plots get plenty of
        # vertical room; the y-axes read much more clearly when taller
        self.resize(1600, 1000)

        self._worker = SerialWorker()
        self._worker.telemetry_received.connect(self._on_telemetry)
        self._worker.connection_changed.connect(self._on_connection)

        self._imu_buffers = {
            f"{src}_{ax}": deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)
            for src in ["ACC", "GYR"] for ax in ["x", "y", "z"]
        }
        # single-value attitude channels (roll/pitch angle + servo commands)
        # plus the extra 'DAT' debug channel — all single float per line.
        self._imu_buffers.update({
            key: deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)
            for key, *_ in ATT_CHANNELS + DAT_CHANNELS
        })

        # ── diagnostics state ───────────────────────────────────────────
        self._rx_count       = 0
        self._rx_parsed      = 0
        self._rx_unparsed    = 0
        self._last_rx_time   = None

        # The Arduino's '?' reply ("Kp=.. Ki=.. Kd=.. T=..") does NOT include
        # the motor index, so we remember which motor each Query was fired for
        # (FIFO) and route the next gain reply to that panel.
        self._pending_query  = deque()

        self._build_ui()
        self._apply_theme()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_plots)
        self._timer.start(UPDATE_MS)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_rx_status)
        self._status_timer.start(500)

    # ── UI ───────────────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(10, 10, 10, 6)

        root.addWidget(self._build_connection_bar())

        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(2)
        splitter.setStyleSheet("QSplitter::handle { background: #2a2a3a; }")

        # left: PID panels
        panels_widget = QWidget()
        panels_layout = QVBoxLayout(panels_widget)
        panels_layout.setSpacing(8)
        panels_layout.setContentsMargins(0, 0, 4, 0)

        self._pid_panels = {}
        for ax in AXES:
            panel = PIDPanel(ax)
            panel.cmd_ready.connect(self._on_gain_changed)
            panel.action_ready.connect(self._on_action)
            panels_layout.addWidget(panel)
            self._pid_panels[ax["motor"]] = panel

        # Brushless throttle — sends 'v 3 <percent>' live as the slider moves.
        self._throttle_panel = ThrottlePanel()
        self._throttle_panel.cmd_ready.connect(self._on_gain_changed)
        self._throttle_panel.action_ready.connect(self._on_action)
        panels_layout.addWidget(self._throttle_panel)

        panels_layout.addStretch()
        splitter.addWidget(panels_widget)

        # right: IMU plot + raw debug console stacked vertically
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(6)
        right_layout.setContentsMargins(4, 0, 0, 0)

        if HAS_PYQTGRAPH:
            self._plot_widget, self._curves = make_imu_plot()
            right_layout.addWidget(self._plot_widget, stretch=5)
        else:
            no_plot = QLabel("Install pyqtgraph for live IMU plot\n\npip install pyqtgraph")
            no_plot.setAlignment(Qt.AlignCenter)
            no_plot.setStyleSheet("color: #555580; font-size: 13px;")
            right_layout.addWidget(no_plot, stretch=3)
            self._curves = {}

        right_layout.addWidget(self._build_debug_console(), stretch=1)

        splitter.addWidget(right_widget)
        splitter.setSizes([380, 620])
        root.addWidget(splitter, stretch=1)

        self._status = QStatusBar()
        self._status.setStyleSheet("color: #555580; font-size: 11px;")
        self.setStatusBar(self._status)
        self._status.showMessage("Not connected")

    def _build_debug_console(self):
        box = QGroupBox("Raw Serial  (debug)")
        box.setFont(QFont("Segoe UI", 9, QFont.Bold))
        box.setStyleSheet("""
            QGroupBox {
                border: 1px solid #3a3a5a; border-radius: 6px;
                margin-top: 8px; padding: 6px; background: #0a0a14;
            }
            QGroupBox::title {
                subcontrol-origin: margin; left: 10px;
                color: #888; font-size: 10px; letter-spacing: 1px;
            }
        """)
        layout = QVBoxLayout(box)
        layout.setSpacing(4)

        top_row = QHBoxLayout()
        self._rx_label = QLabel("RX: 0  |  parsed: 0  |  unparsed: 0  |  last: –")
        self._rx_label.setFont(QFont("Consolas", 9))
        self._rx_label.setStyleSheet("color: #888;")
        top_row.addWidget(self._rx_label)
        top_row.addStretch()

        self._pause_chk = QCheckBox("Pause log")
        self._pause_chk.setStyleSheet("color: #888;")
        top_row.addWidget(self._pause_chk)

        clear_btn = QPushButton("Clear")
        clear_btn.setFixedWidth(60)
        clear_btn.setStyleSheet(self._btn_style("#3a3a5a"))
        clear_btn.clicked.connect(lambda: self._console.clear())
        top_row.addWidget(clear_btn)
        layout.addLayout(top_row)

        self._console = QTextEdit()
        self._console.setReadOnly(True)
        self._console.setFont(QFont("Consolas", 8))
        self._console.setStyleSheet("""
            QTextEdit {
                background: #050508; color: #6fcf97;
                border: 1px solid #2a2a3a; border-radius: 4px;
            }
        """)
        layout.addWidget(self._console)

        return box

    def _build_connection_bar(self):
        bar = QWidget()
        bar.setFixedHeight(42)
        bar.setStyleSheet("background: #13131f; border-radius: 6px;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 4, 10, 4)
        layout.setSpacing(8)

        lbl = QLabel("PORT")
        lbl.setStyleSheet("color: #555580; font-size: 9px; letter-spacing: 2px;")
        lbl.setFont(QFont("Consolas", 9))
        layout.addWidget(lbl)

        self._port_combo = QComboBox()
        self._port_combo.setFixedWidth(200)
        self._port_combo.setStyleSheet("""
            QComboBox {
                background: #1e1e2e; color: #e0e0f0;
                border: 1px solid #3a3a5a; border-radius: 4px; padding: 2px 8px;
            }
            QComboBox QAbstractItemView {
                background: #1e1e2e; color: #e0e0f0;
                selection-background-color: #3a3a6a;
            }
        """)
        self._refresh_ports()
        layout.addWidget(self._port_combo)

        refresh_btn = QPushButton("↻")
        refresh_btn.setFixedSize(30, 28)
        refresh_btn.setToolTip("Refresh port list")
        refresh_btn.clicked.connect(self._refresh_ports)
        refresh_btn.setStyleSheet(self._btn_style("#3a3a5a"))
        layout.addWidget(refresh_btn)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setFixedWidth(90)
        self._connect_btn.clicked.connect(self._toggle_connection)
        self._connect_btn.setStyleSheet(self._btn_style("#4FC3F7"))
        layout.addWidget(self._connect_btn)

        layout.addStretch()

        self._dot = QLabel("●")
        self._dot.setFont(QFont("Arial", 14))
        self._dot.setStyleSheet("color: #333355;")
        layout.addWidget(self._dot)

        return bar

    @staticmethod
    def _btn_style(color):
        return f"""
            QPushButton {{
                background: {color}22; color: {color};
                border: 1px solid {color}66; border-radius: 4px;
                padding: 3px 10px; font-size: 12px;
            }}
            QPushButton:hover  {{ background: {color}44; }}
            QPushButton:pressed {{ background: {color}66; }}
        """

    def _apply_theme(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #0d0d1a; color: #e0e0f0;
                font-family: "Segoe UI", sans-serif;
            }
        """)

    # ── port management ──────────────────────────────────────────────────

    def _refresh_ports(self):
        self._port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_combo.addItems(ports if ports else ["No ports found"])

    def _toggle_connection(self):
        if self._connect_btn.text() == "Connect":
            port = self._port_combo.currentText()
            if not port or port == "No ports found":
                self._status.showMessage("No serial port selected.")
                return
            try:
                self._worker.connect(port)
                self._connect_btn.setText("Disconnect")
                self._connect_btn.setStyleSheet(self._btn_style("#ef5350"))
                self._rx_count = self._rx_parsed = self._rx_unparsed = 0
            except serial.SerialException as e:
                self._status.showMessage(f"Connection failed: {e}")
        else:
            self._worker.disconnect()
            self._connect_btn.setText("Connect")
            self._connect_btn.setStyleSheet(self._btn_style("#4FC3F7"))

    def _on_connection(self, connected):
        if connected:
            self._dot.setStyleSheet("color: #81C784;")
            self._status.showMessage(f"Connected — {self._port_combo.currentText()}")
        else:
            self._dot.setStyleSheet("color: #ef5350;")
            self._status.showMessage("Disconnected")

    # ── serial events ────────────────────────────────────────────────────

    def _on_gain_changed(self, cmd: str, motor: int, value: float):
        self._worker.send_gain(cmd, motor, value)

    def _on_action(self, cmd: str, motor: int):
        if cmd == "?":
            self._pending_query.append(motor)
        self._worker.send_action(cmd, motor)

    def _on_telemetry(self, line: str):
        """
        Every raw line from the Arduino lands here first — including
        things that AREN'T telemetry (e.g. "Kp=1.50 Ki=..." replies to '?',
        or "unknown cmd: x"). Everything gets logged to the debug console;
        only ACC/GYR lines get parsed into the plot buffers.
        """
        self._rx_count += 1
        self._last_rx_time = time.strftime("%H:%M:%S")

        ok = (self._try_parse_imu(line)
              or self._try_parse_throttle(line)
              or self._try_parse_gains(line))
        if ok:
            self._rx_parsed += 1
        else:
            self._rx_unparsed += 1

        if not self._pause_chk.isChecked():
            color = "#6fcf97" if ok else "#aaa"
            self._console.append(f'<span style="color:{color}">{line}</span>')
            # keep the console capped so it doesn't grow forever
            doc = self._console.document()
            if doc.blockCount() > RAW_LOG_MAX_LINES:
                cursor = self._console.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()

    def _try_parse_imu(self, line: str) -> bool:
        """
        Parses:
            ACC <ax>,<ay>,<az>     3-axis accelerometer
            GYR <gx>,<gy>,<gz>     3-axis gyroscope
            ROL/PIT/SRO/SPI <v>    single-value attitude channels
        Returns True if the line matched and was applied to the plot buffers.
        """
        if ' ' not in line:
            return False

        sep = line.index(' ')
        kind   = line[:sep]
        values = line[sep + 1:].split(',')

        # single-value attitude channels (roll/pitch angle + servo commands)
        if kind in self._imu_buffers and kind not in ("ACC", "GYR"):
            if len(values) != 1:
                return False
            try:
                v = float(values[0])
            except ValueError:
                return False
            self._imu_buffers[kind].append(v)
            return True

        if kind not in ("ACC", "GYR") or len(values) != 3:
            return False

        try:
            x, y, z = (float(v) for v in values)
        except ValueError:
            return False

        self._imu_buffers[f"{kind}_x"].append(x)
        self._imu_buffers[f"{kind}_y"].append(y)
        self._imu_buffers[f"{kind}_z"].append(z)
        return True

    def _try_parse_throttle(self, line: str) -> bool:
        """
        Parses the board's throttle report:  'THR <value>'  (1..100).
        Feeds the ThrottlePanel meter so the actual value the Nano is
        running is visible alongside the commanded slider.
        """
        if not line.startswith("THR "):
            return False
        try:
            value = float(line[4:].strip())
        except ValueError:
            return False
        self._throttle_panel.set_reported(value)
        return True

    def _try_parse_gains(self, line: str) -> bool:
        """
        Parses the '?' query reply:  'Kp=1.50 Ki=0.000 Kd=0.000 T=50'
        The reply carries no motor index, so it's routed to the panel whose
        Query was fired earliest and hasn't been answered yet (FIFO).
        Returns True if the line matched and was applied to a panel.
        """
        if not line.startswith("Kp="):
            return False

        parts = {}
        for token in line.split():
            key, sep, val = token.partition("=")
            if sep:
                parts[key] = val
        try:
            kp = float(parts["Kp"])
            ki = float(parts["Ki"])
            kd = float(parts["Kd"])
        except (KeyError, ValueError):
            return False

        if self._pending_query:
            motor = self._pending_query.popleft()
            panel = self._pid_panels.get(motor)
            if panel:
                panel.apply_gains(kp, ki, kd)
        return True

    def _refresh_rx_status(self):
        last = self._last_rx_time or "–"
        self._rx_label.setText(
            f"RX: {self._rx_count}  |  parsed: {self._rx_parsed}  |  "
            f"unparsed: {self._rx_unparsed}  |  last: {last}"
        )

    # ── plot refresh ──────────────────────────────────────────────────────

    def _refresh_plots(self):
        if not HAS_PYQTGRAPH:
            return
        for key, curve in self._curves.items():
            curve.setData(list(self._imu_buffers[key]))

    # ── cleanup ───────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self._worker.disconnect()
        super().closeEvent(event)


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window,          QColor("#0d0d1a"))
    palette.setColor(QPalette.WindowText,      QColor("#e0e0f0"))
    palette.setColor(QPalette.Base,            QColor("#1e1e2e"))
    palette.setColor(QPalette.AlternateBase,   QColor("#13131f"))
    palette.setColor(QPalette.Text,            QColor("#e0e0f0"))
    palette.setColor(QPalette.Button,          QColor("#1e1e2e"))
    palette.setColor(QPalette.ButtonText,      QColor("#e0e0f0"))
    palette.setColor(QPalette.Highlight,       QColor("#4FC3F7"))
    palette.setColor(QPalette.HighlightedText, QColor("#0d0d1a"))
    app.setPalette(palette)

    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec_())
