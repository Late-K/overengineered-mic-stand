# Mic arm PC agent (Windows)

A tiny tray app. When **any** app opens the microphone (Discord, OBS, Zoom, Teams, Audacity, a browser tab…), it tells the stand to `DEPLOY`. It sends `STOW` once the mic has been unused for 45 seconds, so a quick rejoin doesn't make the arm fold and unfold.

It reads the same Windows data that powers the microphone icon in the taskbar:
```
HKCU\Software\Microsoft\Windows\CurrentVersion\CapabilityAccessManager\ConsentStore\microphone
```
It checks twice a second, which costs effectively no CPU. It never listens to any audio.

> **Behringer C-1 note:** the C-1 plugs into your audio interface, so Windows sees the *interface* as the microphone. Make the interface your default recording device (Sound settings → Input) so apps open it.

## Run it

```bash
pip install -r requirements.txt
```
```bash
python mic_agent.py
```

A mic icon appears in the tray. Its colour shows the stand's state:

| Colour | State |
|---|---|
| Grey | Stowed |
| Amber | Moving |
| Green | Live |
| Purple | Teach |
| Red | Fault |
| Dark | Stand not connected |

**Right-click the icon for:**
- Deploy, Stow and Teach
- **Pause auto mode**, which stops it reacting to the mic
- Open log
- Quit

**Other ways to run it:**

| Command | What it does |
|---|---|
| `python mic_agent.py --console` | No tray icon; the log goes to the terminal |
| `python mic_agent.py --dry-run --console` | Doesn't need the stand; only logs what it would send |
| `python mic_agent.py --list` | Shows which apps are using the mic right now |

## Settings

`config.json` is created next to the script (or the .exe) the first time it runs:

| Key | Default | Meaning |
|---|---|---|
| `stow_delay_s` | 45 | Seconds of no mic use before stowing |
| `poll_s` | 0.5 | How often to check |
| `com_port` | `null` | e.g. `"COM5"`. `null` = find the Pico automatically (USB vendor ID 2E8A) |
| `ignore_apps` | `[]` | App names exactly as the log prints them, e.g. `["chrome.exe"]`, that should never deploy the arm |

## Make it an .exe and start it with Windows

1. Run `build_exe.bat`. It produces `dist\mic-agent.exe`.
2. Press **Win+R**, type `shell:startup`, and press Enter.
3. Put a shortcut to `mic-agent.exe` in the folder that opens.

The log (`mic_agent.log`) and `config.json` live next to the .exe.

## Test it without the stand

1. Run:
   ```bash
   python mic_agent.py --dry-run --console
   ```
2. Open **Voice Recorder** (or join a Discord voice channel). The log shows `mic in use by: …` and then `-> DEPLOY`.
3. Close it. About 45 s later the log shows `-> STOW`.

## Troubleshooting

- **Nothing happens when an app uses the mic.**
  - Check Windows Settings → Privacy → Microphone. Both "Allow apps to access your microphone" and "Allow desktop apps…" must be on, or Windows doesn't record usage.
  - Run `--list` while the app is using the mic.
- **An app keeps the mic open all the time** (some chat apps, or a browser tab you've left open). Add it to `ignore_apps`.
- **Hum in your recordings after plugging in the stand.** That's a ground loop through USB. Fit a USB isolator (ADuM3160 module) between the PC and the stand.
