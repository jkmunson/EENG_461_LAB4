#!/bin/bash
sudo echo ""
sudo openocd -f ti_ek-tm4c123gxl.cfg &> /dev/null &
openocd_pid=$!

gdb-multiarch -q -ex "file objs/lab4binary.axf" -ex "target extended-remote localhost:3333" -ex "b main" -ex "tui new-layout def src 1 {-horizontal asm 1 regs 1} 1 status 0 {-horizontal cmd 1} 1" -ex "layout def" -ex "mon reset halt" -ex "ni" -ex "focus cmd" -tui

kill $openocd_pid
