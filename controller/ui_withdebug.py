"""
Vectored Thrust PID Tuner
--------------------------
PyQt5 UI to tune 2 independent PID controllers over Serial
communicating with an Arduino Nano BLE Sense Rev2.

Serial protocol (Python -> Arduino):
    'p <motor> <val>\\n'     set Kp       motor: 1=Roll, 2=Pitch
    'i <motor> <val>\\n'     set Ki
    'd <motor> <val>\\n'     set Kd
    't <motor> <val>\\n'     set timestep (ms)
    '+ <motor>\\n'           activate controller
    '- <motor>\\n'           deactivate controller

Serial protocol (Arduino -> Python):
    'ACC <ax>,<ay>,<az>\\n'  raw accelerometer (g)
    'GYR <gx>,<gy>,<gz>\\n'  raw gyroscope (deg/s)
    'ROL <roll>\\n'          estimated roll angle (deg)
    'PIT <pitch>\\n'         estimated pitch angle (deg)
    'SRO <rollCommand>\\n'   servo roll command
    'SPI <pitchCommand>\\n'  servo pitch command

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
    QDoubleSpinBox, QStatusBar, QSplitter, QFrame, QTextEdit, QCheckBox
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

# motor index matches Arduino switch(motor): 1=Roll, 2=Pitch
AXES = [
    {"name": "Roll",  "motor": 1, "color": "#FFB74D"},
    {"name": "Pitch", "motor": 2, "color": "#4FC3F7"},
]

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

        for slider, cmd, key, fmt in gain_defs:
            layout.addWidget(slider)
            _cmd, _key, _fmt = cmd, key, fmt
            # send the new gain over serial
            slider.value_changed.connect(
                lambda v, c=_cmd: self.cmd_ready.emit(c, self._motor, v)
            )
            # AND reflect it immediately in the on-screen readout —
            # this connection was missing before, which is why the
            # Kp/Ki/Kd labels never updated when dragging a slider.
            slider.value_changed.connect(
                lambda v, k=_key, f=_fmt: self._readouts[k].setText(f.format(v))
            )

        btn_layout = QHBoxLayout()
        for label, cmd in [("Activate", "+"), ("Deactivate", "-"), ("Query ?", "?")]:
            btn = QPushButton(label)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background: {color}22; color: {color};
                    border: 1px solid {color}66; border-radius: 4px; padding: 3px 8px;
                }}
                QPushButton:hover {{ background: {color}44; }}
            """)
            _cmd = cmd
            btn.clicked.connect(lambda _, c=_cmd: self.action_ready.emit(c, self._motor))
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)

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

    def update_readouts(self, kp, ki, kd):
        """External setter, e.g. for populating values from a '?' query reply."""
        self._readouts["Kp"].setText(f"{kp:.2f}")
        self._readouts["Ki"].setText(f"{ki:.3f}")
        self._readouts["Kd"].setText(f"{kd:.3f}")

# ─────────────────────────────────────────────
#  IMU PLOT
# ─────────────────────────────────────────────

def make_imu_plot():
    pg.setConfigOptions(antialias=True, background="#0d0d1a", foreground="#555580")

    layout = pg.GraphicsLayoutWidget()

    acc_plot = layout.addPlot(row=0, col=0, title="Accelerometer (g)")
    acc_plot.showGrid(x=True, y=True, alpha=0.15)
    acc_plot.addLegend(offset=(10, 10))
    acc_plot.setLabel("left", "g")

    gyr_plot = layout.addPlot(row=1, col=0, title="Gyroscope (°/s)")
    gyr_plot.showGrid(x=True, y=True, alpha=0.15)
    gyr_plot.addLegend(offset=(10, 10))
    gyr_plot.setLabel("left", "°/s")

    att_plot = layout.addPlot(row=2, col=0, title="Attitude — angle vs command (°)")
    att_plot.showGrid(x=True, y=True, alpha=0.15)
    att_plot.addLegend(offset=(10, 10))
    att_plot.setLabel("left", "°")

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

    return layout, curves

# ─────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Vectored Thrust — PID Tuner")
        self.setMinimumSize(1040, 720)

        self._worker = SerialWorker()
        self._worker.telemetry_received.connect(self._on_telemetry)
        self._worker.connection_changed.connect(self._on_connection)

        self._imu_buffers = {
            f"{src}_{ax}": deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)
            for src in ["ACC", "GYR"] for ax in ["x", "y", "z"]
        }
        # single-value attitude channels (roll/pitch angle + servo commands)
        self._imu_buffers.update({
            key: deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)
            for key, *_ in ATT_CHANNELS
        })

        # ── diagnostics state ───────────────────────────────────────────
        self._rx_count       = 0
        self._rx_parsed      = 0
        self._rx_unparsed    = 0
        self._last_rx_time   = None

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

        panels_layout.addStretch()
        splitter.addWidget(panels_widget)

        # right: IMU plot + raw debug console stacked vertically
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(6)
        right_layout.setContentsMargins(4, 0, 0, 0)

        if HAS_PYQTGRAPH:
            self._plot_widget, self._curves = make_imu_plot()
            right_layout.addWidget(self._plot_widget, stretch=3)
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

        ok = self._try_parse_imu(line)
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
    window.show()
    sys.exit(app.exec_())