#!/bin/bash
openocd -f /usr/share/openocd/scripts/interface/stlink-v2-1.cfg \
        -f /usr/share/openocd/scripts/target/stm32f1x.cfg \
        -c init \
        -c halt \
        -c "flash erase_address 0x08000000 0x20000" \
        -c "program build/stm32_GPRMC_sync.bin verify reset 0x08000000" \
        -c shutdown
