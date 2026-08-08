@echo off
set PATH=C:\Qt\6.8.3\msvc2022_64\bin;C:\Program Files (x86)\VTK\bin;%PATH%
start "" "%~dp0..\build\vscode\release\simulation.exe"
