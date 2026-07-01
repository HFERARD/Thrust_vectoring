"""
Vectored Thrust PID Tuner
--------------------------
PyQt5 UI to tune 2 independent PID controllers over Serial
communicating with an Arduino Nano BLE Sense Rev2.

Serial protocol (Python -> Arduino):
    'p <motor> <val>\\n'          set Kp       motor: 1=Roll, 2=Pitch
    'i <motor> <val>\\n'          set Ki
    'd <motor> <val>\\n'          set Kd
    's <motor> <val>\\n'          set setpoint
    't <motor> <val>\\n'          set timestep (ms)
    '+ <motor>\\n'         activate controller
    '- <motor>\\n'       deactivate controller

Serial protocol (Arduino -> Python):
    'ACC <ax>,<ay>,<az>\\n'       raw accelerometer (g)
    'GYR <gx>,<gy>,<gz>\\n'       raw gyroscope (deg/s)

Install dependencies:
    pip install pyserial PyQt5 pyqtgraph

    
CODED USING CLAUDE SONNET 4.6
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
    QDoubleSpinBox, QStatusBar, QSplitter, QFrame
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
PLOT_WINDOW = 200    # number of data points in live plot
UPDATE_MS   = 50     # UI refresh interval (ms)

# PID gain ranges (min, max, step)
KP_RANGE = (0.0, 10.0, 0.01)
KI_RANGE = (0.0,  5.0, 0.001)
KD_RANGE = (0.0,  5.0, 0.001)
SP_RANGE = (-90.0, 90.0, 0.5)

# motor index matches Arduino switch(motor): 1=Roll, 2=Pitch
AXES = [
    {"name": "Roll",  "motor": 1, "color": "#FFB74D"},  # amber
    {"name": "Pitch", "motor": 2, "color": "#4FC3F7"},  # sky blue
]

# IMU channels shown in the plot
IMU_CHANNELS = {
    "ACC": ["ax", "ay", "az"],
    "GYR": ["gx", "gy", "gz"],
}
IMU_COLORS = ["#ef5350", "#81C784", "#CE93D8",   # ACC x/y/z
              "#FF8A65", "#4DD0E1", "#FFF176"]    # GYR x/y/z

# ─────────────────────────────────────────────
#  SERIAL WORKER
# ─────────────────────────────────────────────

class SerialWorker(QObject):
    telemetry_received = pyqtSignal(str)
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
        """e.g. send_gain('p', 1, 1.5)  →  'p 1 1.5000\\n'"""
        with self._lock:
            self._cmd_queue.append(f"{cmd} {motor} {value:.4f}\n")

    def send_action(self, cmd: str, motor: int):
        """e.g. send_action('activate', 1)  →  'activate 1\\n'"""
        with self._lock:
            self._cmd_queue.append(f"{cmd} {motor}\n")

    def _loop(self):
        while self._running and self._ser and self._ser.is_open:
            with self._lock:
                cmds, self._cmd_queue = self._cmd_queue, []
            for c in cmds:
                try:
                    self._ser.write(c.encode())
                except serial.SerialException:
                    self._running = False
                    self.connection_changed.emit(False)
                    return
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
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
    """One panel per axis. Emits (arduino_cmd, motor_index, value)."""

    cmd_ready = pyqtSignal(str, int, float)    # gain commands  e.g. ('p', 1, 1.5)
    action_ready = pyqtSignal(str, int)         # action commands e.g. ('+', 1)

    def __init__(self, axis: dict, parent=None):
        super().__init__(parent)
        self._motor = axis["motor"]
        color       = axis["color"]

        self.setTitle(axis["name"])
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

        # ── gain sliders ────────────────────────────────────────────────
        self.kp = GainSlider("KP", *KP_RANGE, color)
        self.ki = GainSlider("KI", *KI_RANGE, color)
        self.kd = GainSlider("KD", *KD_RANGE, color)
        self.sp = GainSlider("SP", *SP_RANGE, color)

        for slider, cmd in [(self.kp, 'p'), (self.ki, 'i'),
                            (self.kd, 'd'), (self.sp, 's')]:
            layout.addWidget(slider)
            _cmd = cmd
            slider.value_changed.connect(
                lambda v, c=_cmd: self.cmd_ready.emit(c, self._motor, v)
            )

        # ── activate / deactivate buttons ────────────────────────────────
        btn_layout = QHBoxLayout()
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
            btn.clicked.connect(lambda _, c=_cmd: self.action_ready.emit(c, self._motor))
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)

        # ── separator + readouts ─────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {color}33;")
        layout.addWidget(sep)

        readout_row = QHBoxLayout()
        self._readouts = {}
        for key in ["Kp", "Ki", "Kd", "SP"]:
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

    def update_readouts(self, kp, ki, kd, sp):
        self._readouts["Kp"].setText(f"{kp:.2f}")
        self._readouts["Ki"].setText(f"{ki:.3f}")
        self._readouts["Kd"].setText(f"{kd:.3f}")
        self._readouts["SP"].setText(f"{sp:+.1f}°")

# ─────────────────────────────────────────────
#  IMU PLOT
# ─────────────────────────────────────────────

def make_imu_plot():
    """Two stacked plots: accelerometer on top, gyroscope below."""
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

    curves = {}
    labels = ["x", "y", "z"]
    for i, lbl in enumerate(labels):
        curves[f"ACC_{lbl}"] = acc_plot.plot([], [], name=lbl,
            pen=pg.mkPen(color=IMU_COLORS[i], width=2))
        curves[f"GYR_{lbl}"] = gyr_plot.plot([], [], name=lbl,
            pen=pg.mkPen(color=IMU_COLORS[3 + i], width=2))

    return layout, curves

# ─────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Vectored Thrust — PID Tuner")
        self.setMinimumSize(980, 640)

        self._worker = SerialWorker()
        self._worker.telemetry_received.connect(self._on_telemetry)
        self._worker.connection_changed.connect(self._on_connection)

        # IMU data buffers: ACC_x, ACC_y, ACC_z, GYR_x, GYR_y, GYR_z
        self._imu_buffers = {
            f"{src}_{ax}": deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)
            for src in ["ACC", "GYR"] for ax in ["x", "y", "z"]
        }

        self._build_ui()
        self._apply_theme()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_plots)
        self._timer.start(UPDATE_MS)

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

        splitter.addWidget(panels_widget)

        # right: IMU plot
        if HAS_PYQTGRAPH:
            self._plot_widget, self._curves = make_imu_plot()
            splitter.addWidget(self._plot_widget)
        else:
            no_plot = QLabel("Install pyqtgraph for live IMU plot\n\npip install pyqtgraph")
            no_plot.setAlignment(Qt.AlignCenter)
            no_plot.setStyleSheet("color: #555580; font-size: 13px;")
            splitter.addWidget(no_plot)
            self._curves = {}

        splitter.setSizes([380, 580])
        root.addWidget(splitter, stretch=1)

        self._status = QStatusBar()
        self._status.setStyleSheet("color: #555580; font-size: 11px;")
        self.setStatusBar(self._status)
        self._status.showMessage("Not connected")

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
        Parses:
            ACC <ax>,<ay>,<az>
            GYR <gx>,<gy>,<gz>
        """
        try:
            sep = line.index(' ')
            kind   = line[:sep]           # 'ACC' or 'GYR'
            values = line[sep + 1:].split(',')

            if kind not in ("ACC", "GYR") or len(values) != 3:
                return

            x, y, z = float(values[0]), float(values[1]), float(values[2])
            self._imu_buffers[f"{kind}_x"].append(x)
            self._imu_buffers[f"{kind}_y"].append(y)
            self._imu_buffers[f"{kind}_z"].append(z)

        except (ValueError, KeyError):
            pass

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