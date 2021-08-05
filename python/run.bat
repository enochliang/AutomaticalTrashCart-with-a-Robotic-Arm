@echo off
set /p workspace=please key in driver_letter:
cd %workspace%:\Project\Devtools
%windir%\system32\cmd.exe /k %workspace%:\Project\Devtools\setup.bat
