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

cmake -G "Visual Studio 14 2015 Win64" -DINSTALL_DYNAMIC_CALIBRATOR=ON -DCMAKE_BUILD_TYPE=Release -DCPACK_SYSTEM_NAME=Win64_x64 ../../
if ERRORLEVEL 1 goto ERROR

CALL "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86

msbuild PACKAGE.vcxproj /t:Build /p:Configuration=Release
if ERRORLEVEL 1 goto ERROR

rem TODO sing the binaries inside the package
if defined SIGN_BINARIES (
    signtool.exe sign /q /ac "%ROOT_CA_PATH%" /f "%CODESIGN_CERT_PATH%\GlobalSign_FRAMOS_eToken.cer" /csp "eToken Base Cryptographic Provider" /kc "[{{%FRAMOS_TOKEN_PASSWORD%}}]=te-ee307152-4ddb-460e-bbc9-c87e75365a17" /tr http://rfc3161timestamp.globalsign.com/advanced /sha1 634217D4B35321AD8863A38BFF93C24594FA7C60 /td SHA256 FRAMOS-librealsense2-*-Win64_x64.exe
)

goto:eof

:READARGUMENTS
IF "%1"=="Win64_x64" SET TARGET_SYSTEM=Win64_x64
IF "%1"=="Win32_x86" SET TARGET_SYSTEM=Win32_x86
IF "%2"=="sign" SET "SIGN_BINARIES=true"
goto:eof

:INIT
IF "%TARGET_SYSTEM%" == "" SET TARGET_SYSTEM=Win64_x64
IF "%BUILD_SYSTEM%" == "" SET BUILD_SYSTEM=VS2015
IF "%BUILD_TYPE%" == "" SET BUILD_TYPE=Release
IF "%SMARTEK_VISION_PATH%"=="" SET SMARTEK_VISION_PATH=C:\SmartekVision
IF "%CODESIGN_CERT_PATH%"=="" SET CODESIGN_CERT_PATH=%SMARTEK_VISION_PATH%\Projects\Misc\CodeSignCert
IF "%ROOT_CA_PATH%"=="" SET ROOT_CA_PATH=%CODESIGN_CERT_PATH%\GlobalSign_Root_CA.crt
goto:eof

:ERROR
ECHO ERROR: %~f0
EXIT /B 1