@echo off
REM Lanza el simulador dentro del entorno virtual del proyecto.
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo El entorno virtual no existe. Creandolo...
    py -m venv .venv || goto :error
    ".venv\Scripts\python.exe" -m pip install --upgrade pip
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt || goto :error
)

".venv\Scripts\python.exe" simulador.py
if errorlevel 1 goto :error
endlocal
exit /b 0

:error
echo.
echo Ocurrio un error al ejecutar el simulador.
pause
exit /b 1
