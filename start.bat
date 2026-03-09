@echo off
echo Starting CaddyMate Server...
echo.
echo Installing dependencies if needed...
pip install -q -r requirements.txt

echo.
echo Starting Flask server...
echo The application will be available at http://localhost:5000
echo.
echo Press Ctrl+C to stop the server
echo.

python server.py
