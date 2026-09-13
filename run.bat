@echo off
setlocal

if not exist .venv (
    echo [INFO] Virtual environment not found. Running setup.bat first ...
    call setup.bat
    if %ERRORLEVEL% NEQ 0 (
        echo [ERROR] setup.bat failed. Cannot launch application.
        exit /b %ERRORLEVEL%
    )
)

echo [INFO] Activating VisionRobotTwin virtual environment ...
call .venv\Scripts\activate.bat
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Failed to activate virtual environment.
    exit /b %ERRORLEVEL%
)

echo [INFO] Starting VisionRobotTwin Application ...
python main.py %*
exit /b %ERRORLEVEL%
