nrfjprog -f nrf52 --eraseall
nrfjprog -f nrf52 --program s132_nrf52_6.1.1_softdevice.hex 
nrfjprog -f nrf52 --program argos_dfu.hex
nrfjprog -f nrf52 --reset
cat ./commit_hash
