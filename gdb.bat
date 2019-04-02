echo "Please run 'st-util' in another terminal window"
echo ""
arm-none-eabi-gdb -iex "add-auto-load-safe-path ." -ex "tar ext :3333" -ex "load-reset" %1
