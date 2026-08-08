@echo off
rem Build 3DScan (Debug) with qmake + nmake into build\vscode
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
set PATH=C:\Qt\6.8.3\msvc2022_64\bin;%PATH%
if not exist "%~dp0..\build\vscode" mkdir "%~dp0..\build\vscode"
cd /d "%~dp0..\build\vscode"
C:\Qt\6.8.3\msvc2022_64\bin\qmake.exe "%~dp0..\3DScan.pro"
nmake -f Makefile.Debug
