@ECHO off
SETLOCAL

ECHO.
ECHO Generating Windows packages
ECHO ===========================

SET PWD=%~dp0
CD %PWD%
CALL:READARGUMENTS %*
CALL:INIT

MKDIR bin\%TARGET_SYSTEM% 2> nul

MKDIR build\%TARGET_SYSTEM% 2> nul
CD build\%TARGET_SYSTEM%



goto:eof

:READARGUMENTS
IF "%1"=="Win64_x64" SET TARGET_SYSTEM=Win64_x64
IF "%1"=="Win32_x86" SET TARGET_SYSTEM=Win32_x86
goto:eof

:INIT
IF "%TARGET_SYSTEM%" == "" SET TARGET_SYSTEM=Win64_x64
IF "%BUILD_SYSTEM%" == "" SET BUILD_SYSTEM=VS2015
IF "%BUILD_TYPE%" == "" SET BUILD_TYPE=Release
goto:eof

:ERROR
ECHO ERROR: %~f0
EXIT /B 1