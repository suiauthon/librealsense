@ECHO off
SETLOCAL

ECHO.
ECHO Generating Windows packages
ECHO ===========================

SET PWD=%~dp0
CD %PWD%
CALL:READARGUMENTS %*
CALL:INIT

MKDIR build\%TARGET_SYSTEM% 2> nul
CD build\%TARGET_SYSTEM%

cmake -G "Visual Studio 14 2015 Win64" -DCMAKE_BUILD_TYPE=Release -DCPACK_SYSTEM_NAME=Win64_x64 ../../

exit

CALL "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86

msbuild PACKAGE.vcxproj /t:Build /p:Configuration=Release

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