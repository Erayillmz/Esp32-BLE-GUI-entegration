import sys
import asyncio
import time
from dataclasses import dataclass
from typing import Optional, List, Deque
from collections import deque

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QCheckBox, QSpinBox, QDoubleSpinBox,
    QFileDialog, QMessageBox
)

from bleak import BleakClient
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import pandas as pd

TARGET_ADDRESS = "24:DC:C3:AF:A9:92"
CHARACTERISTIC_UUID_TX = "abcd1234-5678-90ab-cdef-1234567890ac"
CHARACTERISTIC_UUID_RX = "abcd1234-5678-90ab-cdef-1234567890ab"

# --- Kalman filtresi sınıfı ---
class Kalman1D:
    def __init__(self, Q=0.01, R=1.0):
        self.Q = Q
        self.R = R
        self.x = 0.0
        self.P = 1.0
        self.initialized = False
    def update(self, measurement):
        if measurement is None or pd.isna(measurement):
            return float('nan')
        if not self.initialized:
            self.x = measurement
            self.initialized = True
        else:
            self.P += self.Q
            K = self.P / (self.P + self.R)
            self.x += K * (measurement - self.x)
            self.P *= (1 - K)
        return self.x

@dataclass
class Sample:
    t: float
    ax: Optional[float] = None
    ay: Optional[float] = None
    az: Optional[float] = None
    gx: Optional[float] = None
    gy: Optional[float] = None
    gz: Optional[float] = None

class BLEReader(QThread):
    sample_ready = pyqtSignal(object)
    status = pyqtSignal(str)

    def __init__(self, address: str, uuid_notify: str, uuid_write: str):
        super().__init__()
        self.address = address
        self.uuid_notify = uuid_notify
        self.uuid_write = uuid_write
        self._running = False
        self._t0 = None
        self._client: Optional[BleakClient] = None

    async def _write_cmd(self, cmd: str):
        try:
            if self._client and self._client.is_connected:
                await self._client.write_gatt_char(self.uuid_write, cmd.encode('utf-8'))
                self.status.emit(f"Command sent: {cmd}")
            else:
                self.status.emit("Cannot write: not connected.")
        except Exception as e:
            self.status.emit(f"Cannot write the command: {e}")

    def send_start(self):
        asyncio.run(self._write_cmd("start"))

    def send_stop(self):
        asyncio.run(self._write_cmd("stop"))

    def stop(self):
        self._running = False

    def run(self):
        try:
            asyncio.run(self._runner())
        except Exception as e:
            self.status.emit(f"BLE thread error: {e}")

    async def _runner(self):
        self._running = True
        self._t0 = time.perf_counter()
        try:
            async with BleakClient(self.address) as client:
                self._client = client
                if not client.is_connected:
                    self.status.emit("Cannot connect.")
                    return
                self.status.emit("Connected. Notify starting…")

                def _cb(_sender, data: bytearray):
                    try:
                        line = data.decode(errors='ignore').strip()
                        if not line:
                            return
                        s = self.parse_line(line)
                        if s:
                            self.sample_ready.emit(s)
                    except Exception:
                        pass

                await client.start_notify(self.uuid_notify, _cb)
                self.status.emit("Notify active. Data awaiting…")

                while self._running and client.is_connected:
                    await asyncio.sleep(0.05)

                try:
                    await client.stop_notify(self.uuid_notify)
                except Exception:
                    pass
                self.status.emit("Notify has been stopped.")
        except Exception as e:
            self.status.emit(f"BLE error: {e}")
        finally:
            self._client = None

    def parse_line(self, line: str) -> Optional[Sample]:
        t = time.perf_counter() - (self._t0 or time.perf_counter())
        try:
            parts = [p for p in line.replace(',', ' ').split() if p]
            vals = {}
            for p in parts:
                if ':' in p:
                    k, v = p.split(':', 1)
                    vals[k.strip().lower()] = v.strip()
            ax = self._to_float(vals.get('ax'))
            ay = self._to_float(vals.get('ay'))
            az = self._to_float(vals.get('az'))
            gx = self._to_float(vals.get('gx'))
            gy = self._to_float(vals.get('gy'))
            gz = self._to_float(vals.get('gz'))
            if any(x is not None for x in (ax, ay, az, gx, gy, gz)):
                return Sample(t=t, ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz)
        except Exception:
            return None
        return None

    @staticmethod
    def _to_float(x: Optional[str]) -> Optional[float]:
        if x is None or x == '':
            return None
        try:
            return float(x)
        except Exception:
            return None

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(8.5, 4.8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)

class BLEIMUGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 BLE IMU – live graph + Kalman")
        self.resize(1250, 720)

        self.reader: Optional[BLEReader] = None
        self.logging_enabled = False
        self.log_file = None

        self.maxlen = 5000
        # Ham veri buffer
        self.t_buf = deque(maxlen=self.maxlen)
        self.ax_buf = deque(maxlen=self.maxlen)
        self.ay_buf = deque(maxlen=self.maxlen)
        self.az_buf = deque(maxlen=self.maxlen)
        self.gx_buf = deque(maxlen=self.maxlen)
        self.gy_buf = deque(maxlen=self.maxlen)
        self.gz_buf = deque(maxlen=self.maxlen)
        # Kalman veri buffer
        self.ax_kf_buf = deque(maxlen=self.maxlen)
        self.ay_kf_buf = deque(maxlen=self.maxlen)
        self.az_kf_buf = deque(maxlen=self.maxlen)
        self.gx_kf_buf = deque(maxlen=self.maxlen)
        self.gy_kf_buf = deque(maxlen=self.maxlen)
        self.gz_kf_buf = deque(maxlen=self.maxlen)
        # Kalman filtre nesneleri
        self.kf = {k: Kalman1D() for k in ["ax","ay","az","gx","gy","gz"]}

        central = QWidget(self); self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Üst bar: bağlantı
        top = QHBoxLayout()
        self.edt_addr = QLineEdit(TARGET_ADDRESS)
        self.edt_tx = QLineEdit(CHARACTERISTIC_UUID_TX)
        self.edt_rx = QLineEdit(CHARACTERISTIC_UUID_RX)
        self.btn_connect = QPushButton("Connect")
        self.btn_start = QPushButton("start")
        self.btn_stop = QPushButton("stop")
        self.btn_disconnect = QPushButton("Cut")
        self.lbl_status = QLabel("Ready.")
        self.btn_connect.clicked.connect(self.connect_ble)
        self.btn_start.clicked.connect(self.send_start)
        self.btn_stop.clicked.connect(self.send_stop)
        self.btn_disconnect.clicked.connect(self.disconnect_ble)
        for w in (QLabel("Adress:"), self.edt_addr, QLabel("TX UUID:"), self.edt_tx,
                  QLabel("RX UUID:"), self.edt_rx, self.btn_connect, self.btn_start,
                  self.btn_stop, self.btn_disconnect, self.lbl_status):
            top.addWidget(w)
        root.addLayout(top)

        # Orta bar: kanal seçim, smoothing, kalman
        mid = QHBoxLayout()
        self.cb_ax = QCheckBox("AX"); self.cb_ax.setChecked(True)
        self.cb_ay = QCheckBox("AY")
        self.cb_az = QCheckBox("AZ"); self.cb_az.setChecked(True)
        self.cb_gx = QCheckBox("GX")
        self.cb_gy = QCheckBox("GY")
        self.cb_gz = QCheckBox("GZ")
        for w in (self.cb_ax, self.cb_ay, self.cb_az, self.cb_gx, self.cb_gy, self.cb_gz):
            mid.addWidget(w)
        mid.addSpacing(16)
        self.cb_smooth = QCheckBox("Smoothing")
        self.spn_win = QSpinBox(); self.spn_win.setRange(1, 9999); self.spn_win.setValue(5)
        mid.addWidget(self.cb_smooth); mid.addWidget(QLabel("Window:")); mid.addWidget(self.spn_win)
        mid.addSpacing(16)
        self.cb_kalman = QCheckBox("Kalman Filter")
        self.spn_q = QDoubleSpinBox(); self.spn_q.setDecimals(4); self.spn_q.setValue(0.01)
        self.spn_r = QDoubleSpinBox(); self.spn_r.setDecimals(4); self.spn_r.setValue(1.0)
        mid.addWidget(self.cb_kalman)
        mid.addWidget(QLabel("Q:")); mid.addWidget(self.spn_q)
        mid.addWidget(QLabel("R:")); mid.addWidget(self.spn_r)
        mid.addSpacing(16)
        self.spn_maxlen = QSpinBox(); self.spn_maxlen.setRange(100, 200000); self.spn_maxlen.setValue(self.maxlen)
        self.spn_maxlen.valueChanged.connect(self.update_maxlen)
        mid.addWidget(QLabel("Last N sample:")); mid.addWidget(self.spn_maxlen)
        mid.addSpacing(16)
        self.btn_log = QPushButton("Log Start")
        self.btn_log.clicked.connect(self.toggle_logging)
        mid.addWidget(self.btn_log)
        root.addLayout(mid)

        self.canvas = MplCanvas(self)
        root.addWidget(self.canvas, 1)

        self.timer = QTimer(self); self.timer.setInterval(50)
        self.timer.timeout.connect(self.redraw)
        self.timer.start()

    def connect_ble(self):
        if self.reader is not None:
            QMessageBox.information(self, "Info", "Already connected."); return
        addr = self.edt_addr.text().strip()
        uuid_tx = self.edt_tx.text().strip()
        uuid_rx = self.edt_rx.text().strip()
        if not addr or not uuid_tx or not uuid_rx:
            QMessageBox.warning(self, "Warning", "Adress, TX and RX UUID cannot be empty."); return
        self.reader = BLEReader(addr, uuid_tx, uuid_rx)
        self.reader.sample_ready.connect(self.on_sample)
        self.reader.status.connect(self.on_status)
        self.reader.start()
        self.on_status("Trying connection…")

    def send_start(self):
        if self.reader: self.reader.send_start()
        else: self.on_status("Not connected.")

    def send_stop(self):
        if self.reader: self.reader.send_stop()
        else: self.on_status("Not connected.")

    def disconnect_ble(self):
        try:
            if self.reader:
                try: self.reader.send_stop()
                except Exception: pass
                self.reader.stop(); self.reader.wait(1000)
                self.reader = None
        finally:
            self.on_status("Connection lost.")

    def on_status(self, msg: str):
        self.lbl_status.setText(msg)

    def toggle_logging(self):
        if not self.logging_enabled:
            path, _ = QFileDialog.getSaveFileName(self, "Choose log file", "ble_live_log.csv", "CSV (*.csv)")
            if not path: return
            try:
                self.log_file = open(path, 'w', encoding='utf-8')
                self.log_file.write("time,AX,AY,AZ,GX,GY,GZ\n")
                self.logging_enabled = True
                self.btn_log.setText("Log Stop")
                self.on_status(f"Log started: {path}")
            except Exception as e:
                QMessageBox.critical(self, "Warning", f"Log could not open: {e}")
        else:
            try:
                if self.log_file: self.log_file.close()
            finally:
                self.log_file = None; self.logging_enabled = False
                self.btn_log.setText("Log Başlat")
                self.on_status("Log durduruldu.")

    def update_maxlen(self, n: int):
        self.maxlen = int(n)
        def _resize(dq: Deque[float]) -> Deque[float]:
            new = deque(maxlen=self.maxlen)
            for v in list(dq)[-self.maxlen:]: new.append(v)
            return new
        # Tüm bufferları yeniden boyutlandır
        for attr in ["t_buf","ax_buf","ay_buf","az_buf","gx_buf","gy_buf","gz_buf",
                     "ax_kf_buf","ay_kf_buf","az_kf_buf","gx_kf_buf","gy_kf_buf","gz_kf_buf"]:
            setattr(self, attr, _resize(getattr(self, attr)))

    def on_sample(self, s: Sample):
        self.t_buf.append(s.t)
        ax = self._nan_if_none(s.ax); ay = self._nan_if_none(s.ay); az = self._nan_if_none(s.az)
        gx = self._nan_if_none(s.gx); gy = self._nan_if_none(s.gy); gz = self._nan_if_none(s.gz)
        self.ax_buf.append(ax); self.ay_buf.append(ay); self.az_buf.append(az)
        self.gx_buf.append(gx); self.gy_buf.append(gy); self.gz_buf.append(gz)
        if self.cb_kalman.isChecked():
            for key, val in zip(["ax","ay","az","gx","gy","gz"], [ax,ay,az,gx,gy,gz]):
                self.kf[key].Q = self.spn_q.value(); self.kf[key].R = self.spn_r.value()
            self.ax_kf_buf.append(self.kf["ax"].update(ax))
            self.ay_kf_buf.append(self.kf["ay"].update(ay))
            self.az_kf_buf.append(self.kf["az"].update(az))
            self.gx_kf_buf.append(self.kf["gx"].update(gx))
            self.gy_kf_buf.append(self.kf["gy"].update(gy))
            self.gz_kf_buf.append(self.kf["gz"].update(gz))
        else:
            self.ax_kf_buf.append(float('nan'))
            self.ay_kf_buf.append(float('nan'))
            self.az_kf_buf.append(float('nan'))
            self.gx_kf_buf.append(float('nan'))
            self.gy_kf_buf.append(float('nan'))
            self.gz_kf_buf.append(float('nan'))
        if self.logging_enabled and self.log_file:
            row = [f"{s.t:.6f}", self._fmt(ax), self._fmt(ay), self._fmt(az),
                   self._fmt(gx), self._fmt(gy), self._fmt(gz)]
            try: self.log_file.write(','.join(row) + "\n")
            except Exception: pass

    @staticmethod
    def _nan_if_none(x: Optional[float]) -> float:
        return float('nan') if x is None or pd.isna(x) else x
    @staticmethod
    def _fmt(x: Optional[float]) -> str:
        return "" if (x is None or pd.isna(x)) else f"{x:.6f}"

    def _maybe_smooth(self, y: List[float]) -> List[float]:
        if not self.cb_smooth.isChecked(): return y
        w = max(1, int(self.spn_win.value()))
        if w <= 1 or len(y) < 2: return y
        out: List[float] = []
        for i in range(len(y)):
            start = max(0, i - w + 1)
            window = [yy for yy in y[start:i+1] if not pd.isna(yy)]
            out.append(sum(window)/len(window) if window else float('nan'))
        return out

    def redraw(self):
        axp = self.canvas.ax
        axp.clear()
        t = list(self.t_buf)
        if not t:
            self.canvas.draw(); return
        def plot_pair(cb, buf_ham, buf_kf, label):
            if cb.isChecked():
                axp.plot(t, self._maybe_smooth(list(buf_ham)), label=label)
                if self.cb_kalman.isChecked():
                    axp.plot(t, list(buf_kf), '--', label=label+"_KF")
        plot_pair(self.cb_ax, self.ax_buf, self.ax_kf_buf, 'AX')
        plot_pair(self.cb_ay, self.ay_buf, self.ay_kf_buf, 'AY')
        plot_pair(self.cb_az, self.az_buf, self.az_kf_buf, 'AZ')
        plot_pair(self.cb_gx, self.gx_buf, self.gx_kf_buf, 'GX')
        plot_pair(self.cb_gy, self.gy_buf, self.gy_kf_buf, 'GY')
        plot_pair(self.cb_gz, self.gz_buf, self.gz_kf_buf, 'GZ')
        axp.set_xlabel("Zaman (s)"); axp.set_ylabel("Ham / Kalman")
        axp.set_title("ESP32 BLE IMU – Canlı")
        axp.grid(True, linestyle=":", linewidth=0.6)
        axp.legend(loc='upper right', ncol=3, framealpha=0.85)
        self.canvas.fig.tight_layout(); self.canvas.draw()

    def closeEvent(self, event):
        try: self.disconnect_ble()
        except Exception: pass
        try:
            if self.logging_enabled and self.log_file:
                self.log_file.close()
        except Exception: pass
        event.accept()

def main():
    app = QApplication(sys.argv)
    w = BLEIMUGui()
    w.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()


