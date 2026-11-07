#!/usr/bin/env python3
"""Mic arm PC agent: deploys the arm when any app opens the microphone, stows it when they stop.

Windows already tracks which apps are using the microphone (it's what drives the mic icon
in the taskbar) under
    HKCU\\Software\\Microsoft\\Windows\\CurrentVersion\\CapabilityAccessManager\\ConsentStore\\microphone
An app is using the mic right now when its LastUsedTimeStart is set and LastUsedTimeStop is 0.
Reading those keys twice a second costs effectively nothing.

    python mic_agent.py                     # tray icon
    python mic_agent.py --console           # no tray, log to the console
    python mic_agent.py --dry-run --console # no stand needed: just log DEPLOY / STOW
"""

import argparse
import json
import logging
import os
import sys
import threading
import time
import winreg
from logging.handlers import RotatingFileHandler
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # only needed when talking to the stand
    serial = None

APP_DIR = Path(sys.executable if getattr(sys, "frozen", False) else __file__).resolve().parent
CONFIG_PATH = APP_DIR / "config.json"
LOG_PATH = APP_DIR / "mic_agent.log"
CONSENT_KEY = r"Software\Microsoft\Windows\CurrentVersion\CapabilityAccessManager\ConsentStore\microphone"
PICO_VID = 0x2E8A

DEFAULT_CONFIG = {
    "stow_delay_s": 45,   # wait this long after the last app closes the mic before stowing
    "poll_s": 0.5,
    "com_port": None,     # e.g. "COM5"; None = find the Pico automatically
    "ignore_apps": [],    # app names (as shown in the log) that should never deploy the arm
}

log = logging.getLogger("mic_agent")


def load_config():
    cfg = dict(DEFAULT_CONFIG)
    if CONFIG_PATH.exists():
        cfg.update(json.loads(CONFIG_PATH.read_text()))
    else:
        CONFIG_PATH.write_text(json.dumps(DEFAULT_CONFIG, indent=2) + "\n")
    return cfg


# ---------------------------------------------------------------- mic detection

def _in_use(key):
    try:
        start, _ = winreg.QueryValueEx(key, "LastUsedTimeStart")
        stop, _ = winreg.QueryValueEx(key, "LastUsedTimeStop")
    except OSError:
        return False
    return start > 0 and stop == 0


def _subkeys(key):
    i = 0
    while True:
        try:
            yield winreg.EnumKey(key, i)
        except OSError:
            return
        i += 1


def app_name(subkey, packaged):
    """Readable name: 'Discord.exe' for desktop apps, 'WindowsSoundRecorder' for Store apps."""
    if not packaged:
        return subkey.replace("#", "\\").rsplit("\\", 1)[-1]
    return subkey.split("_")[0].split(".")[-1]


def mic_users():
    """Names of the apps that have the microphone open right now."""
    users = []
    with winreg.OpenKey(winreg.HKEY_CURRENT_USER, CONSENT_KEY) as root:
        for name in _subkeys(root):
            with winreg.OpenKey(root, name) as k:
                if name == "NonPackaged":
                    for exe in _subkeys(k):
                        with winreg.OpenKey(k, exe) as ek:
                            if _in_use(ek):
                                users.append(app_name(exe, packaged=False))
                elif _in_use(k):
                    users.append(app_name(name, packaged=True))
    return users


# ---------------------------------------------------------------- serial link to the stand

class Stand:
    """Keeps a serial connection to the Pico open, reconnecting when it's unplugged."""

    def __init__(self, port, dry_run, on_state):
        self.port, self.dry_run, self.on_state = port, dry_run, on_state
        self.ser = None
        self.lock = threading.Lock()
        self.connected_at = 0.0

    def find_port(self):
        if self.port:
            return self.port
        for p in list_ports.comports():
            if p.vid == PICO_VID:
                return p.device
        return None

    def ensure_connected(self):
        """Returns True if connected. Call regularly; it retries quietly."""
        if self.dry_run or self.ser:
            return True
        port = self.find_port()
        if not port:
            return False
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.5)
        except serial.SerialException as e:
            log.debug("can't open %s: %s", port, e)
            return False
        self.connected_at = time.time()
        log.info("connected to stand on %s", port)
        threading.Thread(target=self._reader, args=(self.ser,), daemon=True).start()
        self.send("STATUS?")
        return True

    def _reader(self, ser):
        while ser is self.ser:
            try:
                line = ser.readline().decode(errors="replace").strip()
            except (serial.SerialException, OSError):
                self._drop()
                return
            if not line:
                continue
            log.info("stand: %s", line)
            if line.startswith("STATE "):
                self.on_state(line)

    def _drop(self):
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        log.warning("lost connection to stand")
        self.on_state(None)

    def send(self, cmd):
        log.info("-> %s%s", cmd, " (dry run)" if self.dry_run else "")
        if self.dry_run:
            return True
        with self.lock:
            if not self.ser:
                return False
            try:
                self.ser.write((cmd + "\n").encode())
                return True
            except (serial.SerialException, OSError):
                pass
        self._drop()
        return False


# ---------------------------------------------------------------- deploy / stow logic

class Agent:
    def __init__(self, cfg, dry_run):
        self.cfg = cfg
        self.paused = False
        self.state_line = None if not dry_run else "STATE (dry run)"
        self.listeners = []
        self.stand = Stand(cfg["com_port"], dry_run, self._on_state)
        self.deployed = False   # we asked for DEPLOY on the current connection
        self.idle_since = None
        self.last_users = []
        self.stop = threading.Event()
        self.was_connected = False

    def _on_state(self, line):
        self.state_line = line
        for fn in self.listeners:
            fn()

    @property
    def state(self):
        if not self.state_line:
            return "DISCONNECTED"
        parts = self.state_line.split()
        return parts[1] if len(parts) > 1 else "?"

    def command(self, cmd):
        self.stand.send(cmd)

    def tick(self):
        connected = self.stand.ensure_connected()
        if connected and not self.was_connected:
            self.deployed = False  # after a (re)connect, re-send DEPLOY if the mic is in use
        self.was_connected = connected

        try:
            users = [u for u in mic_users() if u not in self.cfg["ignore_apps"]]
        except OSError as e:
            log.error("can't read mic usage: %s", e)
            return
        if users != self.last_users:
            log.info("mic in use by: %s", ", ".join(users) if users else "nobody")
            self.last_users = users
        if self.paused or not connected:
            return

        now = time.time()
        if users:
            self.idle_since = None
            if not self.deployed and self.stand.send("DEPLOY"):
                self.deployed = True
        elif self.deployed:
            self.idle_since = self.idle_since or now
            if now - self.idle_since >= self.cfg["stow_delay_s"] and self.stand.send("STOW"):
                self.deployed = False
                self.idle_since = None

    def run(self):
        log.info("watching the microphone (stow delay %ss)", self.cfg["stow_delay_s"])
        while not self.stop.is_set():
            self.tick()
            self.stop.wait(self.cfg["poll_s"])


# ---------------------------------------------------------------- tray

STATE_COLOURS = {
    "STOWED": (110, 110, 120), "DEPLOYING": (230, 150, 20), "STOWING": (230, 150, 20),
    "LIVE": (40, 180, 70), "TEACH": (160, 60, 200), "FAULT": (220, 40, 40),
}


def make_icon_image(state, paused):
    from PIL import Image, ImageDraw

    img = Image.new("RGBA", (64, 64), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    colour = STATE_COLOURS.get(state, (60, 60, 60))
    d.rounded_rectangle((22, 6, 42, 40), radius=10, fill=colour)        # mic capsule
    d.arc((14, 20, 50, 50), 0, 180, fill=colour, width=4)               # holder
    d.line((32, 50, 32, 58), fill=colour, width=4)
    if paused:
        d.rectangle((44, 44, 50, 62), fill=(250, 250, 250))
        d.rectangle((54, 44, 60, 62), fill=(250, 250, 250))
    return img


def run_tray(agent):
    import pystray

    def refresh():
        icon.icon = make_icon_image(agent.state, agent.paused)
        icon.title = f"Mic arm: {agent.state}" + (" (paused)" if agent.paused else "")

    def toggle_pause(_icon, _item):
        agent.paused = not agent.paused
        log.info("auto mode %s", "paused" if agent.paused else "resumed")
        refresh()

    def quit_(_icon, _item):
        agent.stop.set()
        icon.stop()

    menu = pystray.Menu(
        pystray.MenuItem("Deploy", lambda: agent.command("DEPLOY")),
        pystray.MenuItem("Stow", lambda: agent.command("STOW")),
        pystray.MenuItem("Teach", lambda: agent.command("TEACH")),
        pystray.MenuItem("Pause auto mode", toggle_pause, checked=lambda _: agent.paused),
        pystray.Menu.SEPARATOR,
        pystray.MenuItem("Open log", lambda: os.startfile(LOG_PATH)),
        pystray.MenuItem("Quit", quit_),
    )
    icon = pystray.Icon("mic_arm", make_icon_image(agent.state, False), "Mic arm", menu)
    agent.listeners.append(refresh)
    threading.Thread(target=agent.run, daemon=True).start()
    icon.run()


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Deploy the mic arm whenever an app uses the microphone.")
    ap.add_argument("--console", action="store_true", help="no tray icon; log to the console")
    ap.add_argument("--dry-run", action="store_true", help="don't talk to the stand, just log what would be sent")
    ap.add_argument("--list", action="store_true", help="print which apps are using the mic right now and exit")
    args = ap.parse_args()

    if args.list:
        print("\n".join(mic_users()) or "nobody is using the mic")
        return

    handlers = [RotatingFileHandler(LOG_PATH, maxBytes=500_000, backupCount=2, encoding="utf-8")]
    if args.console or not getattr(sys, "frozen", False):
        handlers.append(logging.StreamHandler())
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S", handlers=handlers)

    if serial is None and not args.dry_run:
        sys.exit("pyserial is missing: pip install -r requirements.txt")

    agent = Agent(load_config(), args.dry_run)
    if args.console:
        try:
            agent.run()
        except KeyboardInterrupt:
            pass
    else:
        run_tray(agent)


if __name__ == "__main__":
    main()
