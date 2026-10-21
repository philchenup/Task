#!/usr/bin/env python3
"""功能区演示：QTabBar 切换 QStackedWidget，Data Panel 保持不动。

选择「文件」只显示打开 / 保存 / 关闭 / 关闭所有，
颜色、法向量等留在各自的页里。

运行:
    python3 demo/ribbon_demo.py
截图:
    python3 demo/ribbon_demo.py --shots /tmp/ribbon-shots
"""

from __future__ import annotations

import random
import sys
from pathlib import Path

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt
from PyQt6.QtGui import (
    QAction,
    QColor,
    QFont,
    QIcon,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
)
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QStackedWidget,
    QTabBar,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

ICON = "#3aa0dc"
RIBBON_BG = "#3a4250"


def make_icon(draw, size: int = 32) -> QIcon:
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    pen = QPen(QColor(ICON))
    pen.setWidthF(1.7)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    painter.setPen(pen)
    painter.setBrush(Qt.BrushStyle.NoBrush)
    draw(painter, size)
    painter.end()
    return QIcon(pixmap)


def draw_folder(p: QPainter, _s: int) -> None:
    path = QPainterPath()
    path.moveTo(5, 12)
    path.lineTo(5, 9)
    path.lineTo(13, 9)
    path.lineTo(15, 12)
    path.lineTo(27, 12)
    path.lineTo(27, 24)
    path.lineTo(5, 24)
    path.closeSubpath()
    p.drawPath(path)


def draw_save(p: QPainter, _s: int) -> None:
    p.drawRoundedRect(QRectF(7, 4, 18, 24), 1.5, 1.5)
    p.drawRect(QRectF(11, 4, 10, 7))
    p.drawRect(QRectF(10, 17, 12, 8))
    p.drawLine(QPointF(12, 20), QPointF(20, 20))


def draw_close(p: QPainter, _s: int) -> None:
    p.drawLine(QPointF(9, 9), QPointF(23, 23))
    p.drawLine(QPointF(23, 9), QPointF(9, 23))


def draw_close_all(p: QPainter, _s: int) -> None:
    p.drawRoundedRect(QRectF(5, 5, 22, 22), 2, 2)
    p.drawLine(QPointF(11, 11), QPointF(21, 21))
    p.drawLine(QPointF(21, 11), QPointF(11, 21))


def draw_color(p: QPainter, _s: int) -> None:
    colors = ["#e85d4c", "#f0c040", "#3dbe6e", "#3aa0dc", "#8e6adf", "#f08a3c"]
    rect = QRectF(4, 4, 24, 24)
    p.setPen(Qt.PenStyle.NoPen)
    for i, color in enumerate(colors):
        p.setBrush(QColor(color))
        p.drawPie(rect, int(i * 60 * 16), int(60 * 16))
    p.setBrush(QColor(RIBBON_BG))
    p.drawEllipse(QRectF(11, 11, 10, 10))


def draw_normal(p: QPainter, _s: int) -> None:
    p.drawLine(QPointF(7, 25), QPointF(24, 8))
    path = QPainterPath()
    path.moveTo(24, 8)
    path.lineTo(14, 8)
    path.moveTo(24, 8)
    path.lineTo(24, 18)
    p.drawPath(path)


def draw_filter(p: QPainter, _s: int) -> None:
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QColor(ICON))
    heights = [12, 18, 8, 16, 10, 14]
    for i, height in enumerate(heights):
        p.drawRoundedRect(QRectF(4 + i * 4.2, 26 - height, 2.6, height), 1, 1)


def draw_downsample(p: QPainter, _s: int) -> None:
    p.drawEllipse(QRectF(6, 6, 20, 20))
    p.drawLine(QPointF(16, 4), QPointF(16, 10))
    p.drawLine(QPointF(16, 22), QPointF(16, 28))
    p.drawLine(QPointF(4, 16), QPointF(10, 16))
    p.drawLine(QPointF(22, 16), QPointF(28, 16))
    p.setBrush(QColor(ICON))
    p.drawEllipse(QRectF(14, 14, 4, 4))


def draw_crop(p: QPainter, _s: int) -> None:
    p.drawEllipse(QRectF(3, 3, 8, 8))
    p.drawEllipse(QRectF(3, 21, 8, 8))
    p.drawLine(QPointF(9, 9), QPointF(26, 24))
    p.drawLine(QPointF(9, 23), QPointF(26, 8))


def draw_tcp(p: QPainter, _s: int) -> None:
    points = [(5, 22), (11, 16), (15, 19), (21, 9), (27, 12)]
    for start, end in zip(points, points[1:]):
        p.drawLine(QPointF(*start), QPointF(*end))
    p.setBrush(QColor(ICON))
    p.drawEllipse(QRectF(23.5, 7.5, 5, 5))
    p.setBrush(Qt.BrushStyle.NoBrush)
    p.drawLine(QPointF(5, 27), QPointF(5, 24))
    p.drawLine(QPointF(5, 27), QPointF(8, 27))


def draw_hand_eye(p: QPainter, _s: int) -> None:
    p.drawArc(QRectF(3, 10, 26, 14), 20 * 16, 140 * 16)
    p.drawArc(QRectF(3, 8, 26, 14), 200 * 16, 140 * 16)
    p.setBrush(QColor(ICON))
    p.drawEllipse(QRectF(13, 12, 6, 6))


def draw_library(p: QPainter, _s: int) -> None:
    for top in (7, 14, 21):
        p.drawLine(QPointF(6, top + 3), QPointF(16, top))
        p.drawLine(QPointF(16, top), QPointF(26, top + 3))
        p.drawLine(QPointF(6, top + 3), QPointF(16, top + 6))
        p.drawLine(QPointF(16, top + 6), QPointF(26, top + 3))


def draw_gear(p: QPainter, _s: int) -> None:
    p.save()
    p.translate(16, 16)
    p.setBrush(Qt.BrushStyle.NoBrush)
    for _ in range(8):
        p.drawRoundedRect(QRectF(-2.2, -14, 4.4, 5.2), 1, 1)
        p.rotate(45)
    p.drawEllipse(QRectF(-8, -8, 16, 16))
    p.setBrush(QColor(RIBBON_BG))
    p.drawEllipse(QRectF(-3.2, -3.2, 6.4, 6.4))
    p.restore()


def draw_camera(p: QPainter, _s: int) -> None:
    p.drawRoundedRect(QRectF(4, 10, 24, 16), 2, 2)
    p.drawLine(QPointF(11, 10), QPointF(13, 6))
    p.drawLine(QPointF(13, 6), QPointF(20, 6))
    p.drawLine(QPointF(20, 6), QPointF(22, 10))
    p.drawEllipse(QRectF(12, 13, 9, 9))


ICON_DRAW = {
    "folder": draw_folder,
    "save": draw_save,
    "close": draw_close,
    "close-all": draw_close_all,
    "color": draw_color,
    "normal": draw_normal,
    "filter": draw_filter,
    "downsample": draw_downsample,
    "crop": draw_crop,
    "tcp": draw_tcp,
    "hand-eye": draw_hand_eye,
    "library": draw_library,
    "gear": draw_gear,
    "camera": draw_camera,
}


PAGES = [
    {
        "title": "文件",
        "groups": [
            {
                "name": "文件",
                "items": [
                    ("打开", "folder"),
                    ("保存", "save"),
                    ("关闭", "close"),
                    ("关闭所有", "close-all"),
                ],
            }
        ],
    },
    {
        "title": "点云",
        "groups": [
            {
                "name": "显示",
                "items": [("颜色", "color"), ("法向量", "normal")],
            },
            {
                "name": "处理",
                "items": [
                    ("滤波", "filter"),
                    ("降采样", "downsample"),
                    ("点云裁剪", "crop"),
                ],
            },
        ],
    },
    {
        "title": "标定",
        "groups": [
            {
                "name": "标定",
                "items": [("TCP标定", "tcp"), ("手眼标定", "hand-eye")],
            }
        ],
    },
    {
        "title": "配置",
        "groups": [
            {
                "name": "配置",
                "items": [
                    ("工件库", "library"),
                    ("视觉配置", "gear"),
                    ("截图", "camera"),
                ],
            }
        ],
    },
]


def all_labels() -> list[str]:
    labels = []
    for page in PAGES:
        for group in page["groups"]:
            for label, _icon in group["items"]:
                labels.append(label)
    return labels


class DataPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("dataPanel")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.status = "Data Panel 不随标签切换"
        rng = random.Random(11)
        self.points: list[tuple[float, float, float]] = []
        for cx, cy, spread, count in (
            (0.46, 0.48, 0.11, 1100),
            (0.60, 0.40, 0.045, 500),
            (0.38, 0.58, 0.03, 280),
        ):
            for _ in range(count):
                x = rng.gauss(cx, spread)
                y = rng.gauss(cy, spread * 0.85)
                z = rng.random()
                if 0.04 < x < 0.96 and 0.08 < y < 0.92:
                    self.points.append((x, y, z))

    def set_status(self, text: str) -> None:
        self.status = text
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#14171c"))
        painter.setPen(QColor("#8b939e"))
        font = painter.font()
        font.setPointSize(11)
        painter.setFont(font)
        painter.drawText(16, 28, "Data Panel")
        for x, y, z in self.points:
            px = int(x * self.width())
            py = int(y * self.height())
            color = QColor(90 + int(z * 80), 176 + int(z * 50), 230)
            color.setAlpha(150 + int(z * 80))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(QRectF(px, py, 2.2, 2.2))
        painter.setPen(QColor("#9aa3ad"))
        painter.drawText(16, self.height() - 16, self.status)
        painter.end()


class RibbonBar(QWidget):
    def __init__(self, on_trigger, on_page) -> None:
        super().__init__()
        self.on_page = on_page
        self.setObjectName("ribbon")
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        tab_host = QWidget()
        tab_host.setObjectName("tabHost")
        tab_row = QHBoxLayout(tab_host)
        tab_row.setContentsMargins(10, 0, 10, 0)
        tab_row.setSpacing(0)
        self.tabs = QTabBar()
        self.tabs.setObjectName("ribbonTabs")
        self.tabs.setExpanding(False)
        self.tabs.setDrawBase(False)
        self.tabs.setDocumentMode(True)
        tab_row.addWidget(self.tabs)
        tab_row.addStretch(1)
        root.addWidget(tab_host)

        self.stack = QStackedWidget()
        self.stack.setObjectName("ribbonStack")
        self.stack.setFixedHeight(108)
        root.addWidget(self.stack)

        self.page_labels: list[list[str]] = []
        for page in PAGES:
            self._add_page(page, on_trigger)

        self.tabs.currentChanged.connect(self.stack.setCurrentIndex)
        self.tabs.currentChanged.connect(self._page_changed)
        self._page_changed(0)

    def _add_page(self, page: dict, on_trigger) -> None:
        self.tabs.addTab(page["title"])
        host = QWidget()
        host.setObjectName("ribbonPage")
        layout = QHBoxLayout(host)
        layout.setContentsMargins(12, 6, 12, 4)
        layout.setSpacing(0)
        labels: list[str] = []
        for index, group in enumerate(page["groups"]):
            if index:
                layout.addWidget(self._separator())
            layout.addWidget(self._group(group, on_trigger, labels))
        layout.addStretch(1)
        self.page_labels.append(labels)
        self.stack.addWidget(host)

    def _separator(self) -> QFrame:
        line = QFrame()
        line.setFrameShape(QFrame.Shape.VLine)
        line.setObjectName("groupSep")
        line.setFixedWidth(1)
        return line

    def _group(self, group: dict, on_trigger, labels: list[str]) -> QWidget:
        widget = QWidget()
        column = QVBoxLayout(widget)
        column.setContentsMargins(8, 0, 8, 0)
        column.setSpacing(2)
        row = QHBoxLayout()
        row.setSpacing(2)
        for label, icon_name in group["items"]:
            labels.append(label)
            action = QAction(make_icon(ICON_DRAW[icon_name]), label, self)
            action.triggered.connect(lambda _checked=False, text=label: on_trigger(text))
            button = QToolButton()
            button.setDefaultAction(action)
            button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
            button.setAutoRaise(True)
            button.setIconSize(QSize(28, 28))
            button.setMinimumWidth(76)
            button.setObjectName("ribbonButton")
            row.addWidget(button)
        column.addLayout(row)
        caption = QLabel(group["name"])
        caption.setObjectName("groupLabel")
        caption.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        column.addWidget(caption)
        return widget

    def _page_changed(self, index: int) -> None:
        if index < 0:
            return
        shown = self.page_labels[index]
        hidden = [label for label in all_labels() if label not in shown]
        self.on_page(PAGES[index]["title"], shown, hidden)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("功能区演示 — 文件 / 点云 / 标定 / 配置")
        central = QWidget()
        central.setObjectName("central")
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.data_panel = DataPanel()

        self.result = QWidget()
        self.result.setObjectName("result")
        result_layout = QGridLayout(self.result)
        result_layout.setContentsMargins(18, 12, 18, 12)
        result_layout.setHorizontalSpacing(18)
        result_layout.setVerticalSpacing(3)
        title = QLabel("演示结果")
        title.setObjectName("resultTitle")
        result_layout.addWidget(title, 0, 0, 1, 2)
        self.current_value = QLabel()
        self.shown_value = QLabel()
        self.hidden_value = QLabel()
        self.shown_value.setWordWrap(True)
        self.hidden_value.setWordWrap(True)
        self.current_value.setObjectName("currentValue")
        self.hidden_value.setObjectName("hiddenLine")
        for row, key in enumerate(("当前标签", "显示", "隐藏"), start=1):
            key_label = QLabel(key)
            key_label.setObjectName("resultKey")
            result_layout.addWidget(key_label, row, 0)
        result_layout.addWidget(self.current_value, 1, 1)
        result_layout.addWidget(self.shown_value, 2, 1)
        result_layout.addWidget(self.hidden_value, 3, 1)
        result_layout.setColumnStretch(1, 1)

        self.ribbon = RibbonBar(self._on_trigger, self._on_page)
        layout.addWidget(self.ribbon)
        layout.addWidget(self.result)
        layout.addWidget(self.data_panel, 1)
        self.resize(1180, 720)

    def _on_page(self, title: str, shown: list[str], hidden: list[str]) -> None:
        self.current_value.setText(title)
        self.shown_value.setText("  ·  ".join(shown))
        self.hidden_value.setText("  ·  ".join(hidden))

    def _on_trigger(self, text: str) -> None:
        self.data_panel.set_status(f"已执行：{text}    Data Panel 保持不变")


STYLE = """
QWidget#central { background: #1b1f25; color: #e8eaed; }
QWidget#tabHost { background: #2a3038; }
QTabBar#ribbonTabs::tab {
    background: #2a3038;
    color: #c5ccd4;
    padding: 8px 22px 7px 22px;
    margin: 0;
    border: none;
    border-bottom: 2px solid #2a3038;
    font-size: 14px;
}
QTabBar#ribbonTabs::tab:selected {
    background: #3a4250;
    color: #ffffff;
    border-bottom: 2px solid #3aa0dc;
}
QTabBar#ribbonTabs::tab:hover:!selected {
    color: #ffffff;
    background: #343b46;
}
QWidget#ribbonPage, QStackedWidget#ribbonStack { background: #3a4250; }
QToolButton#ribbonButton {
    color: #e8eaed;
    background: transparent;
    border: 1px solid transparent;
    border-radius: 4px;
    padding: 4px 6px 2px 6px;
    font-size: 12px;
}
QToolButton#ribbonButton:hover {
    background: #4c5666;
    border: 1px solid #667386;
}
QToolButton#ribbonButton:pressed {
    background: #2f3844;
}
QLabel#groupLabel { color: #9aa3ad; font-size: 11px; }
QFrame#groupSep { color: #5c6778; background: #5c6778; margin: 8px 6px; }
QWidget#result { background: #22262d; border-top: 1px solid #1a1d22; }
QLabel { color: #e8eaed; font-size: 14px; }
QLabel#resultTitle { color: #8b939e; font-size: 12px; }
QLabel#resultKey { color: #8b939e; min-width: 72px; }
QLabel#currentValue { color: #8ecfff; font-size: 15px; }
QLabel#hiddenLine { color: #8e98a2; }
"""


def grab_shots(window: MainWindow, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    window.show()
    app = QApplication.instance()
    assert app is not None
    for index, page in enumerate(PAGES):
        window.ribbon.tabs.setCurrentIndex(index)
        app.processEvents()
        window.grab().save(str(out_dir / f"{index}_{page['title']}.png"))
        print(out_dir / f"{index}_{page['title']}.png")


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("WenQuanYi Micro Hei", 10))
    window = MainWindow()
    window.setStyleSheet(STYLE)
    if "--shots" in sys.argv:
        out = Path(sys.argv[sys.argv.index("--shots") + 1])
        grab_shots(window, out)
        return
    window.resize(1480, 900)
    window.move(40, 32)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
