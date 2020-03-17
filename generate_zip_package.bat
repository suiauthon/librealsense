@ECHO off
SETLOCAL

ECHO.
ECHO Generating Windows archive
ECHO ===========================

SET PWD=%~dp0
CD %PWD%
CALL:INIT
IF ERRORLEVEL 1 GOTO ERROR

RD /S /Q FRAMOS_D400e_Software_Package
MKDIR FRAMOS_D400e_Software_Package
CD FRAMOS_D400e_Software_Package
COPY "%CAMERA_SUITE_PACKAGE_PATH%\*_x64.exe" . > nul
COPY ..\build\Win64_x64\*.exe . > nul
COPY ..\FRAMOS_D400e_Software_Package_Changelog.txt . > nul
COPY ..\FRAMOS_LibRealSense_Changelog.txt . > nul
COPY "%CAMERA_SUITE_CHANGELOG%" FRAMOS_CameraSuite_Changelog.txt > nul
SET /p VERSION_AND_DATE=< FRAMOS_D400e_Software_Package_Changelog.txt
FOR /F %%a in ("%VERSION_AND_DATE%") do SET VERSION=%%a
SET VERSION=%VERSION:.=_%
CD ..
"C:\Program Files\7-Zip\7z" a FRAMOS_D400e_Software_Package_%VERSION%_Win64_x64.zip FRAMOS_D400e_Software_Package\

goto:eof

:INIT
IF NOT EXIST "%CAMERA_SUITE_PACKAGE_PATH%" (
	ECHO Variable CAMERA_SUITE_PACKAGE_PATH does not point to a valid folder
	EXIT /B 1
)
IF NOT EXIST "%CAMERA_SUITE_CHANGELOG%" (
	ECHO Variable CAMERA_SUITE_CHANGELOG does not point to a valid file
	EXIT /B 1
)
IF NOT EXIST "C:\Program Files\7-Zip\7z.exe" (
	ECHO 7Zip not installed
	EXIT /B 1
)
goto:eof

:ERROR
ECHO ERROR: %~f0
EXIT /B 1