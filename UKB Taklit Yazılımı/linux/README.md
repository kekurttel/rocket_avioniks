# Ö-UKB Taklit Yazılımı — Linux

EXE, PyInstaller ile paketlenmiş saf Python 3.11 + PySide6/QML uygulaması. İçinde
`ctypes.windll`, `winreg`, `msvcrt` gibi Windows API çağrısı yok; port yolları zaten
`/dev/ttyS*` biçiminde. Bu yüzden kaynak kod, QML ve CSV dosyalarına dokunmadan çalışıyor.

    ./install.sh   # EXE'yi app/ klasörüne açar, uv/venv kurar, PySide6 + pyserial yükler
    ./run.sh       # uygulamayı başlatır

Kurulum internet ister (~350 MB tekerlek). `uv` varsa Python 3.11'i kendisi indirir;
`uv` yoksa sistemde `python3.11` gerekir. Kod, QML, CSV ve ikonlar byte-byte aynıdır.

## Dosyalar

- extract.py — PyInstaller paketini açar, pyc başlıklarını ve kaynak ağacını yazar.
- launcher.py — EXE'nin main.py'siyle aynı 20 satır: QML motorunu kurar.
- install.sh — sanal ortam + bağımlılıklar (PySide6-Addons, QML'in kullandığı QtCharts için).
- run.sh — başlatır.

Uygulama ağacı (app/) silinip yeniden üretilebilir: python3 extract.py
EXE güncellenirse tekrar kurulum scriptini çalıştır.

## Kart bağlı değilken test

    socat -d -d pty,raw,echo=0 pty,raw,echo=0

Çıkan iki /dev/pts/N değerinden birini Ayarlar → Bağlantı Ayarı → Port listesinden seç.
Diğerini socat -d -d pty,raw,echo=0 /dev/ttyUSB0 ile kartın ucuna bağla.
