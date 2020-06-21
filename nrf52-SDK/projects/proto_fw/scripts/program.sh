nrfjprog -f nrf52 --eraseall
nrfjprog -f nrf52 --program s132_nrf52_6.1.1_softdevice.hex --sectorerase
nrfjprog -f nrf52 --program proto_fw.hex --sectorerase
nrfjprog -f nrf52 --reset
