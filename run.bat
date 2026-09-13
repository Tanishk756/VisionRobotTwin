@echo off
setlocal

if not exist .venv (
    echo [INFO] Virtual environment not found. Running setup.bat first ...
    call setup.bat
    if errorlevel 1 (
        echo [ERROR] setup.bat failed. Cannot launch application.
        exit /b 1
    )
)

echo [INFO] Activating VisionRobotTwin virtual environment ...
call .venv\Scripts\activate.bat
if errorlevel 1 (
    echo [ERROR] Failed to activate virtual environment.
    exit /b 1
)

echo [INFO] Starting VisionRobotTwin Application ...
python main.py %*
if errorlevel 1 (
    exit /b 1
)
exit /b 0
