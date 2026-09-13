@echo off
echo ======================================================================
echo  VisionRobotTwin: Environment Setup
echo ======================================================================

:: Check if Python is available
python --version >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Python is not installed or not found in system PATH.
    echo Please install Python 3.10, 3.11, or 3.12 and ensure 'Add to PATH' is checked.
    pause
    exit /b 1
)

:: Create Virtual Environment if not exists
if not exist .venv (
    echo [INFO] Creating Python virtual environment in .venv ...
    python -m venv .venv
    if %ERRORLEVEL% NEQ 0 (
        echo [ERROR] Failed to create virtual environment.
        pause
        exit /b 1
    )
)

echo [INFO] Activating virtual environment ...
call .venv\Scripts\activate.bat

echo [INFO] Upgrading pip ...
python -m pip install --upgrade pip

echo [INFO] Installing project dependencies ...
python -m pip install -r requirements-dev.txt
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Dependency installation encountered an issue.
    pause
    exit /b 1
)

echo [INFO] Generating ArUco marker printable assets ...
python tools\generate_aruco_markers.py

echo [INFO] Running test suite ...
pytest -v

echo ======================================================================
echo [SUCCESS] VisionRobotTwin is ready to run!
echo Launch application with:  run.bat  or  python main.py
echo ======================================================================
pause
