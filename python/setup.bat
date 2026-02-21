@echo off
REM Edge-AI Multi-Sport Tracker Setup Script for Windows
REM Compatible with Windows 10/11

echo ==========================================
echo Edge-AI Multi-Sport Tracker Setup
echo ==========================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: Python is not installed or not in PATH
    echo Please install Python 3.8 or higher from python.org
    pause
    exit /b 1
)

for /f "tokens=*" %%i in ('python --version') do set PYTHON_VERSION=%%i
echo Found: %PYTHON_VERSION%
echo.

REM Create virtual environment
echo Creating virtual environment...
if exist .venv (
    echo Warning: Virtual environment already exists at .venv\
    set /p RECREATE="Do you want to recreate it? (y/n): "
    if /i "%RECREATE%"=="y" (
        rmdir /s /q .venv
        python -m venv .venv
        echo Virtual environment recreated
    ) else (
        echo Using existing virtual environment
    )
) else (
    python -m venv .venv
    echo Virtual environment created at .venv\
)
echo.

REM Activate virtual environment
echo Activating virtual environment...
call .venv\Scripts\activate.bat
if %errorlevel% neq 0 (
    echo Error: Failed to activate virtual environment
    pause
    exit /b 1
)
echo Virtual environment activated
echo.

REM Upgrade pip
echo Upgrading pip...
python -m pip install --upgrade pip
echo.

REM Install dependencies
echo Installing Python packages from requirements.txt...
pip install -r requirements.txt
if %errorlevel% neq 0 (
    echo Error: Failed to install packages
    pause
    exit /b 1
)
echo Python packages installed successfully
echo.

REM Check YOLOv8 weights
echo Checking YOLOv8 weights...
if not exist training\yolov8n.pt (
    echo YOLOv8n weights not found. They will be downloaded automatically on first run.
) else (
    echo YOLOv8n weights found: training\yolov8n.pt
)
echo.

REM Verify installation
echo Verifying installation...
python -c "import cv2, numpy, imutils, ultralytics, serial; print('All core packages imported successfully'); print(f'  - OpenCV: {cv2.__version__}'); print(f'  - NumPy: {numpy.__version__}'); print(f'  - Ultralytics: {ultralytics.__version__}')"
if %errorlevel% neq 0 (
    echo Error: Package verification failed
    pause
    exit /b 1
)
echo.

echo ==========================================
echo Setup completed successfully!
echo ==========================================
echo.
echo To activate the virtual environment:
echo   .venv\Scripts\activate
echo.
echo To run the tracker:
echo   python compute-vision\cv-kinematic-tracker.py
echo.
echo To run with YOLOv8:
echo   python compute-vision\cv-kinematic-prose-tracker-v3v1.py
echo.
pause
