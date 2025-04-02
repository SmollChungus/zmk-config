@echo off
echo Building serene65 firmware...

cd C:\Users\mpamu\code\zmk\app
west build -p -b serene65 -S zmk-usb-logging -- -DZMK_EXTRA_MODULES="C:/Users/mpamu/zmk-config" -DZMK_CONFIG="C:/Users/mpamu/zmk-config/config" -DKCONFIG_WARN_UNUSED=y

if %ERRORLEVEL% NEQ 0 (
  echo Build failed with error code %ERRORLEVEL%
  
  exit /b %ERRORLEVEL%
)

echo Build completed successfully!
echo Firmware is located at: C:\Users\mpamu\code\zmk\app\build\zephyr\zmk.bin

echo Preparing to flash firmware...
echo Please put your device in bootloader mode now!

:: Add dfu-util to PATH for west flash
set PATH=%PATH%;C:\QMK_MSYS\mingw64\bin

echo Flashing firmware...
west flash