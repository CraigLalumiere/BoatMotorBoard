import math

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt


class AnalogGauge(QtWidgets.QWidget):
    def __init__(self, title, unit, minimum, maximum, redline=None, parent=None):
        super().__init__(parent)
        self.title = title
        self.unit = unit
        self.minimum = minimum
        self.maximum = maximum
        self.redline = redline
        self.value = minimum
        self.setMinimumSize(180, 170)

    def set_value(self, value):
        self.value = max(self.minimum, min(self.maximum, value))
        self.update()

    def paintEvent(self, event):
        _ = event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        rect = self.rect().adjusted(12, 12, -12, -12)
        size = min(rect.width(), rect.height())
        center = QtCore.QPointF(rect.center().x(), rect.top() + size * 0.58)
        radius = size * 0.42

        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QColor("#121a24"))
        painter.drawRoundedRect(self.rect().adjusted(1, 1, -1, -1), 8, 8)

        arc_rect = QtCore.QRectF(
            center.x() - radius,
            center.y() - radius,
            radius * 2,
            radius * 2,
        )
        painter.setPen(QtGui.QPen(QtGui.QColor("#293849"), 10, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(arc_rect, 210 * 16, -240 * 16)

        redline_span = self._normalized_value(self.redline)
        if redline_span is not None:
            painter.setPen(QtGui.QPen(QtGui.QColor("#7f1d1d"), 10, Qt.SolidLine, Qt.RoundCap))
            painter.drawArc(
                arc_rect,
                int((210 - redline_span * 240) * 16),
                int(-(1.0 - redline_span) * 240 * 16),
            )

        span = 0 if self.maximum == self.minimum else (self.value - self.minimum) / (self.maximum - self.minimum)
        color = QtGui.QColor("#28c76f")
        if self.redline is not None and self.value >= self.redline:
            color = QtGui.QColor("#ef4444")
        elif span > 0.82:
            color = QtGui.QColor("#f59e0b")

        painter.setPen(QtGui.QPen(color, 10, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(arc_rect, 210 * 16, int(-240 * span * 16))

        painter.setPen(QtGui.QPen(QtGui.QColor("#60758c"), 2))
        for tick in range(0, 11):
            angle = math.radians(210 - tick * 24)
            inner = QtCore.QPointF(
                center.x() + math.cos(angle) * radius * 0.78,
                center.y() - math.sin(angle) * radius * 0.78,
            )
            outer = QtCore.QPointF(
                center.x() + math.cos(angle) * radius * 0.93,
                center.y() - math.sin(angle) * radius * 0.93,
            )
            painter.drawLine(inner, outer)

        needle_angle = math.radians(210 - span * 240)
        needle_end = QtCore.QPointF(
            center.x() + math.cos(needle_angle) * radius * 0.72,
            center.y() - math.sin(needle_angle) * radius * 0.72,
        )
        painter.setPen(QtGui.QPen(QtGui.QColor("#e5eef8"), 4, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(center, needle_end)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QColor("#e5eef8"))
        painter.drawEllipse(center, 5, 5)

        painter.setPen(QtGui.QColor("#dbe7f3"))
        title_font = QtGui.QFont("Segoe UI", 10, QtGui.QFont.DemiBold)
        painter.setFont(title_font)
        painter.drawText(rect, Qt.AlignTop | Qt.AlignHCenter, self.title.upper())

        value_font = QtGui.QFont("Consolas", 17, QtGui.QFont.Bold)
        painter.setFont(value_font)
        value_text = f"{self.value:.0f}" if self.maximum > 500 else f"{self.value:.1f}"
        painter.drawText(rect.adjusted(0, int(size * 0.62), 0, 0), Qt.AlignHCenter, value_text)

        unit_font = QtGui.QFont("Segoe UI", 9)
        painter.setFont(unit_font)
        painter.setPen(QtGui.QColor("#8fa3b8"))
        painter.drawText(rect.adjusted(0, int(size * 0.78), 0, 0), Qt.AlignHCenter, self.unit)

    def _normalized_value(self, value):
        if value is None or self.maximum == self.minimum:
            return None

        return max(0.0, min(1.0, (value - self.minimum) / (self.maximum - self.minimum)))


class DigitalReadout(QtWidgets.QFrame):
    def __init__(self, label, unit="", parent=None):
        super().__init__(parent)
        self.unit = unit
        self.setObjectName("dashboardTile")

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(2)

        self.label_widget = QtWidgets.QLabel(label.upper())
        self.label_widget.setObjectName("tileLabel")
        self.value_widget = QtWidgets.QLabel("--")
        self.value_widget.setObjectName("tileValue")
        self.value_widget.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(self.label_widget)
        layout.addWidget(self.value_widget)

    def set_value(self, value):
        self.value_widget.setText(f"{value} {self.unit}".strip())


class LedIndicator(QtWidgets.QFrame):
    def __init__(
        self,
        label,
        good_when=True,
        true_color="#22c55e",
        false_color="#ef4444",
        parent=None,
    ):
        super().__init__(parent)
        self.good_when = good_when
        self.true_color = true_color
        self.false_color = false_color
        self.setObjectName("dashboardTile")

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(10)

        self.led = QtWidgets.QLabel()
        self.led.setFixedSize(18, 18)

        self.label_widget = QtWidgets.QLabel(label.upper())
        self.label_widget.setObjectName("tileLabel")

        layout.addWidget(self.led)
        layout.addWidget(self.label_widget, 1)
        self.set_unknown()

    def set_unknown(self):
        self.led.setStyleSheet(
            "border-radius: 9px;"
            "background: #475569;"
            "border: 1px solid #64748b;"
        )

    def set_value(self, value):
        ok = value == self.good_when
        color = self.true_color if ok else self.false_color
        self.led.setStyleSheet(
            "border-radius: 9px;"
            f"background: {color};"
            f"border: 1px solid {color};"
        )
        self.setStyleSheet(
            "#dashboardTile {"
            "background: #121a24;"
            "border: 1px solid #223042;"
            "border-radius: 8px;"
            "}"
        )


class MotorDashboard(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("motorDashboard")

        self.temp_gauge = AnalogGauge("Temperature", "deg C", 0, 140, redline=80)
        self.rpm_gauge = AnalogGauge("RPM", "rev/min", 0, 7000, redline=6000)

        self.pressure = DigitalReadout("Pressure", "psi")
        self.vbat = DigitalReadout("Battery", "V")
        self.engine_time = DigitalReadout("Engine Time", "h")
        self.tick = DigitalReadout("Tick", "ms")

        self.temp_good = LedIndicator("Temp Good")
        self.pressure_good = LedIndicator("Pressure Good")
        self.neutral = LedIndicator("Neutral", false_color="#475569")
        self.start = LedIndicator("Start", false_color="#475569")
        self.buzzer = LedIndicator(
            "Buzzer",
            good_when=True,
            true_color="#ef4444",
            false_color="#475569",
        )

        self._build_layout()
        self.setStyleSheet("""
            #motorDashboard {
                background: #0b1118;
            }
            #dashboardTile {
                background: #121a24;
                border: 1px solid #223042;
                border-radius: 8px;
            }
            #tileLabel {
                color: #8fa3b8;
                font: 700 10pt "Segoe UI";
            }
            #tileValue {
                color: #e5eef8;
                font: 700 18pt "Consolas";
            }
        """)

    def _build_layout(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        title = QtWidgets.QLabel("MOTOR DASHBOARD")
        title.setStyleSheet("color: #dbe7f3; font: 700 15pt 'Segoe UI';")
        root.addWidget(title)

        gauge_row = QtWidgets.QHBoxLayout()
        gauge_row.setSpacing(12)
        gauge_row.addWidget(self.temp_gauge, 1)
        gauge_row.addWidget(self.rpm_gauge, 1)
        root.addLayout(gauge_row)

        values = QtWidgets.QGridLayout()
        values.setHorizontalSpacing(10)
        values.setVerticalSpacing(10)
        values.addWidget(self.pressure, 0, 0)
        values.addWidget(self.vbat, 0, 1)
        values.addWidget(self.engine_time, 1, 0)
        values.addWidget(self.tick, 1, 1)
        root.addLayout(values)

        indicators = QtWidgets.QGridLayout()
        indicators.setHorizontalSpacing(10)
        indicators.setVerticalSpacing(10)
        indicators.addWidget(self.temp_good, 0, 0)
        indicators.addWidget(self.pressure_good, 0, 1)
        indicators.addWidget(self.neutral, 1, 0)
        indicators.addWidget(self.start, 1, 1)
        indicators.addWidget(self.buzzer, 2, 0, 1, 2)
        root.addLayout(indicators)
        root.addStretch(1)

    def update_motor_data(self, data):
        self.temp_gauge.set_value(data.temperature)
        self.rpm_gauge.set_value(data.tachometer)
        self.pressure.set_value(f"{data.pressure:.1f}")
        self.vbat.set_value(f"{data.vbat:.2f}")
        self.engine_time.set_value(f"{data.engine_minutes / 60.0:.1f}")
        self.tick.set_value(data.milliseconds_tick)
        self.temp_good.set_value(data.temp_good)
        self.pressure_good.set_value(data.pres_good)
        self.neutral.set_value(data.neutral)
        self.start.set_value(data.start)
        self.buzzer.set_value(data.buzzer)
