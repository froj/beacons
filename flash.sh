#!/usr/bin/sh
{
    echo "reset halt"
    sleep 1
    echo "flash write_image erase $1"
    sleep 1
    echo "!"
} | telnet localhost 4444
