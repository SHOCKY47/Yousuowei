@echo off
for /r build\bin %%i in (*.hex) do (
    probe-rs-cli.exe download --chip MM32F3277G9P --format hex %%i
    probe-rs-cli.exe reset --chip MM32F3277G9P
    exit /b
)