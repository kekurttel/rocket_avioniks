#!/usr/bin/env python3
"""Uygulamayi Linux'ta baslatir.

EXE'nin icindeki main.py ile ayni isi yapar: stil ayari, AppController'i QML
kontekstine baglar ve qml/main.qml dosyasini yukler. Uygulama koduna mudahale etmez.
"""

import os
import sys
from pathlib import Path

APP = Path(__file__).resolve().parent / "app"
sys.path.insert(0, str(APP))
# Uygulama, kaynak agaci disinda calisirken kaynaklari sys._MEIPASS uzerinden arar.
sys.frozen = True
sys._MEIPASS = str(APP)

from PySide6.QtGui import QIcon
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuickControls2 import QQuickStyle
from PySide6.QtWidgets import QApplication

from ukb_tester_gui.backend.app_controller import AppController

os.environ.setdefault("QT_SCALE_FACTOR", "1")

QQuickStyle.setStyle("Basic")
app = QApplication(sys.argv)
app.setApplicationName("ukb-tester-gui")
app.setApplicationVersion("0.1.0")
app.setOrganizationName("ukb")
icon = APP / "assets" / "icons" / "app_icon.png"
if icon.exists():
    app.setWindowIcon(QIcon(str(icon)))

engine = QQmlApplicationEngine()
engine.rootContext().setContextProperty("appController", AppController())
engine.load(str(APP / "qml" / "main.qml"))
if not engine.rootObjects():
    sys.exit("main.qml yuklenemedi")
sys.exit(app.exec())
