@echo off
setlocal

set "SDK_ROOT=D:\ti\mspm0_sdk_2_04_00_06"
set "SYSCFG_PATH=D:\ti\SYSCONFIG\sysconfig_cli.bat"
set "PROJECT_ROOT=%~dp0.."
set "SYSCFG_FILE=%PROJECT_ROOT%\empty.syscfg"

if not exist "%SYSCFG_PATH%" (
    echo Couldn't find SysConfig Tool: %SYSCFG_PATH%
    exit /b 1
)

if not exist "%SDK_ROOT%\.metadata\product.json" (
    echo Couldn't find MSPM0 SDK product file: %SDK_ROOT%\.metadata\product.json
    exit /b 1
)

if not exist "%SYSCFG_FILE%" (
    echo Couldn't find SysConfig file: %SYSCFG_FILE%
    exit /b 1
)

echo Using MSPM0 SDK: %SDK_ROOT%
echo Using SysConfig: %SYSCFG_PATH%
"%SYSCFG_PATH%" -o "%PROJECT_ROOT%" -s "%SDK_ROOT%\.metadata\product.json" --compiler keil "%SYSCFG_FILE%"
exit /b %ERRORLEVEL%
