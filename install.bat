@echo off
rem RoboField setup for Windows: double-click this file.
rem Makes a private Python environment in .venv, installs every library RoboField uses,
rem checks the result, and puts a RoboField icon on the Desktop. Safe to run again.
setlocal
cd /d "%~dp0"
echo.
echo  RoboField setup
echo  ===============
echo.

set "PY="
where py >nul 2>nul && set "PY=py -3"
if not defined PY (
  where python >nul 2>nul && set "PY=python"
)
if not defined PY goto :nopython
%PY% -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" || goto :nopython

if not exist ".venv\Scripts\python.exe" (
  echo Creating the Python environment in .venv ...
  %PY% -m venv .venv || goto :fail
)
echo Installing the libraries (the first time downloads about 1 GB; it takes a few minutes) ...
".venv\Scripts\python.exe" -m pip install --upgrade pip || goto :fail
".venv\Scripts\python.exe" -m pip install -e ".[all]" || goto :fail

echo.
".venv\Scripts\python.exe" tools\doctor.py
echo.
if not defined ROBOFIELD_NO_SHORTCUT ".venv\Scripts\python.exe" tools\robofield_app.py --shortcut

echo.
echo  Done. Start RoboField from the Desktop icon, or with:
echo      .venv\Scripts\python tools\robofield_app.py
echo.
if not defined ROBOFIELD_NO_PAUSE pause
exit /b 0

:nopython
echo Python 3.10 or newer was not found.
echo Install it from https://www.python.org/downloads/ (tick "Add python.exe to PATH"), then run this again.
if not defined ROBOFIELD_NO_PAUSE pause
exit /b 1

:fail
echo.
echo Setup stopped with an error: see the messages above. docs/install.md has fixes for common ones.
if not defined ROBOFIELD_NO_PAUSE pause
exit /b 1
