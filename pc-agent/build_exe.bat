@echo off
rem Builds dist\mic-agent.exe (a single file, no console window).
cd /d "%~dp0"
python -m pip install -r requirements.txt pyinstaller || exit /b 1
python -m PyInstaller --onefile --windowed --name mic-agent mic_agent.py || exit /b 1
echo.
echo Built dist\mic-agent.exe - see README.md to start it automatically at login.
