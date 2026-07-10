@echo off
setlocal enabledelayedexpansion

:: Read sdk_config.ini from project root (two levels up from tools/keil/)
set CONFIG_FILE=%~dp0..\..\sdk_config.ini

:: Validate config file exists
if not exist "%CONFIG_FILE%" (
    echo.
    echo Couldn't find sdk_config.ini at %CONFIG_FILE%
    echo.
    exit /b 1
)

:: Read SYSCFG path from [SYSCFG] section
set SYSCFG_PATH=
for /f "delims=" %%a in ('type "%CONFIG_FILE%"') do (
    set "line=%%a"
    if "!line!"=="[SYSCFG]" set "READ_PATH=1"
    if "!line!"=="[DEVICE]" set "READ_PATH=0"
    if "!READ_PATH!"=="1" (
        for /f "tokens=1,* delims==" %%x in ("!line!") do (
            if "%%x"=="path" set "SYSCFG_PATH=%%y"
        )
    )
)

set "SYSCFG_CLI=%SYSCFG_PATH%\sysconfig_cli.bat"

:: Read SDK path from [MSPM0_SDK] section
set SDK_PATH=
set READ_PATH=0
for /f "delims=" %%a in ('type "%CONFIG_FILE%"') do (
    set "line=%%a"
    if "!line!"=="[MSPM0_SDK]" set "READ_PATH=1"
    if "!line!"=="[SYSCFG]" set "READ_PATH=0"
    if "!READ_PATH!"=="1" (
        for /f "tokens=1,* delims==" %%x in ("!line!") do (
            if "%%x"=="path" set "SDK_PATH=%%y"
        )
    )
)
:: SDK_PATH from ini is already clean (no spaces around =)

:: Validate SYSCFG tool
if not exist "%SYSCFG_CLI%" (
    echo.
    echo Couldn't find Sysconfig Tool at %SYSCFG_CLI%
    echo Please update the path in sdk_config.ini [SYSCFG] section.
    echo.
    exit /b 1
)

:: Validate SDK root
if not exist "%SDK_PATH%\.metadata\product.json" (
    echo.
    echo Couldn't find SDK at %SDK_PATH%
    echo Please update the path in sdk_config.ini [MSPM0_SDK] section.
    echo.
    exit /b 1
)

echo Using Sysconfig Tool from %SYSCFG_CLI%
echo Using SDK from %SDK_PATH%

set PROJ_DIR=%~1
set PROJ_DIR=%PROJ_DIR:'=%

set SYSCFG_FILE=%~2
set SYSCFG_FILE=%SYSCFG_FILE:'=%

:: Search for the syscfg file — try direct path first, then recursive search
set "FULL_SYSCFG_PATH=%PROJ_DIR%\%SYSCFG_FILE%"
if exist "%FULL_SYSCFG_PATH%" goto syscfg_found

:: Fallback: recursively search from PROJ_DIR downward
set FULL_SYSCFG_PATH=
for /r "%PROJ_DIR%" %%f in ("%SYSCFG_FILE%") do (
    if not defined FULL_SYSCFG_PATH set "FULL_SYSCFG_PATH=%%f"
)
if not defined FULL_SYSCFG_PATH (
    echo Couldn't find %SYSCFG_FILE% under %PROJ_DIR%
    exit /b 1
)

:syscfg_found
for %%i in ("%FULL_SYSCFG_PATH%") do set "SYSCFG_DIR=%%~dpi"

%SYSCFG_CLI% -o "%SYSCFG_DIR%" -s "%SDK_PATH%\.metadata\product.json" --compiler keil "%FULL_SYSCFG_PATH%"
