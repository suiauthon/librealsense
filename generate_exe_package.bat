@ECHO off
SETLOCAL

ECHO.
ECHO Generating Windows packages
ECHO ===========================

SET PWD=%~dp0
CD %PWD%
CALL:READARGUMENTS %*
CALL:INIT
CALL:BUILD_LIBREALSENSE
if "%TARGET_SYSTEM%" == "Win64_x64" (
    CALL:BUILD_DYNAMIC_CALIBRATOR
)
CALL:PACKAGE
if defined SIGN_BINARIES (
    CALL:SIGN
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
IF "%TARGET_SYSTEM%" == "Win64_x64" (
    SET GENERATOR="Visual Studio 14 2015 Win64"
    SET CMAKE_FLAGS=-DDYNAMIC_CALIBRATOR_PATH=%DYNAMIC_CALIBRATOR_PATH%
) ELSE IF "%TARGET_SYSTEM%" == "Win32_x86" (
    SET GENERATOR="Visual Studio 14 2015"
    SET CAMERA_SUITE_TARGET_SYSTEM=%TARGET_SYSTEM%
)
CALL "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86
if ERRORLEVEL 1 goto ERROR
goto:eof

:BUILD_LIBREALSENSE
MKDIR build\%TARGET_SYSTEM% 2> nul
CD build\%TARGET_SYSTEM%
cmake -G %GENERATOR% %CMAKE_FLAGS% -DCMAKE_BUILD_TYPE=Release -DCPACK_SYSTEM_NAME=%TARGET_SYSTEM% ../../
if ERRORLEVEL 1 goto ERROR
msbuild librealsense2.sln /t:Build /p:Configuration=Release
if ERRORLEVEL 1 goto ERROR
CD %PWD%
goto:eof

:BUILD_DYNAMIC_CALIBRATOR
CD "%DYNAMIC_CALIBRATOR_PATH%"
MKDIR build 2> nul
CD build
cmake -G %GENERATOR% -DLIBRS_INCLUDE_DIR=%PWD%\include -DLIBRS_LIBRARY_DIR=%PWD%\build\%TARGET_SYSTEM%\Release ..
if ERRORLEVEL 1 goto ERROR
msbuild examples\DynamicCalibrator\DynamicCalibrator.vcxproj /t:Build /p:Configuration=Release
msbuild examples\CalibrationTables\CalibrationTables.vcxproj /t:Build /p:Configuration=Release
if ERRORLEVEL 1 goto ERROR
cd %PWD%
goto:eof

:PACKAGE
msbuild build\%TARGET_SYSTEM%\PACKAGE.vcxproj /t:Build /p:Configuration=Release
if ERRORLEVEL 1 goto ERROR
goto:eof

:SIGN
rem TODO sign the binaries inside the package
signtool.exe sign /q /ac "%ROOT_CA_PATH%" /f "%CODESIGN_CERT_PATH%\GlobalSign_FRAMOS_eToken.cer" /csp "eToken Base Cryptographic Provider" /kc "[{{%FRAMOS_TOKEN_PASSWORD%}}]=te-ee307152-4ddb-460e-bbc9-c87e75365a17" /tr http://rfc3161timestamp.globalsign.com/advanced /sha1 634217D4B35321AD8863A38BFF93C24594FA7C60 /td SHA256 build\%TARGET_SYSTEM%\FRAMOS-librealsense2-*-%TARGET_SYSTEM%.exe
if ERRORLEVEL 1 goto ERROR
goto:eof

:ERROR
ECHO ERROR: %~f0
EXIT /B 1