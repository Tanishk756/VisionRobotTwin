@echo off
if not exist .venv (
    echo [ERROR] Virtual environment not found. Running setup.bat first ...
    call setup.bat
)

echo [INFO] Activating VisionRobotTwin virtual environment ...
call .venv\Scripts\activate.bat

echo [INFO] Starting VisionRobotTwin Application ...
python main.py %*
