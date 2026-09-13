@echo off
setlocal

echo ======================================================================
echo  VisionRobotTwin: Environment Setup (v1.1)
echo ======================================================================

:: Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is not installed or not found in system PATH.
    echo Please install Python 3.10, 3.11, or 3.12 and ensure 'Add to PATH' is checked.
    exit /b 1
)

:: Create Virtual Environment if not exists
if not exist .venv (
    echo [INFO] Creating Python virtual environment in .venv ...
    python -m venv .venv
    if errorlevel 1 (
        echo [ERROR] Failed to create virtual environment.
        exit /b 1
    )
)

echo [INFO] Activating virtual environment ...
call .venv\Scripts\activate.bat
if errorlevel 1 (
    echo [ERROR] Failed to activate virtual environment.
    exit /b 1
)

echo [INFO] Upgrading pip ...
python -m pip install --upgrade pip
if errorlevel 1 (
    echo [ERROR] Failed to upgrade pip.
    exit /b 1
)

echo [INFO] Installing project dependencies ...
python -m pip install -r requirements-dev.txt
if errorlevel 1 (
    echo [ERROR] Dependency installation encountered an issue.
    exit /b 1
)

echo [INFO] Generating ArUco marker printable assets ...
python tools\generate_aruco_markers.py
if errorlevel 1 (
    echo [ERROR] Marker generation failed.
    exit /b 1
)

echo [INFO] Running test suite ...
pytest -v
if errorlevel 1 (
    echo [ERROR] Automated test suite failed.
    exit /b 1
)

echo ======================================================================
echo [SUCCESS] VisionRobotTwin setup and validation passed!
echo Launch application with:  run.bat  or  python main.py
echo ======================================================================
exit /b 0
