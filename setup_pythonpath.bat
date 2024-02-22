REM get current project path
set PROJECT_PATH=%~dp0
echo %PROJECT_PATH%

REM carla version
set CARLA_VERSION=0.9.14

REM set up paths
set CARLA_ROOT=C:\Users\yf3831\CARLA_%CARLA_VERSION%\WindowsNoEditor
set PYTHONPATH=%LEADERBOARD_ROOT%;%SCENARIO_RUNNER_ROOT%;%CARLA_ROOT%\PythonAPI\carla;%CARLA_ROOT%\PythonAPI\carla\dist\carla-%CARLA_VERSION%-py3.7-win-amd64.egg;%PYTHONPATH%
