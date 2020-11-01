# qpc port 
armfly stm32f103ze-ek v2 board
apollo stm32f767ig board

## current status
not all toolchain ported, but have verified follows:

qv ok (armfly-stm32f103ze-ek|gnu)

qk ok (armfly-stm32f103ze-ek|gnu)

qf and ucosii ok (armfly-stm32f103ze-ek|gnu)

qf and freertos ok (armfly-stm32f103ze-ek|gnu)

qs ok (armfly-stm32f103ze-ek|gnu,mdk)(apollo-stm32f767ig|mdk)

## qspy

qspy -c COM3 -b 115200

qspy -c /dev/ttyUSB0 -b 115200
![](doc/qspy.png)

python3 qview
![](doc/qview.png)

## to be done
1.figure out a better way to avoid f103 qspy mode QF_poolInit() failure in main()

  for now I just simply commente qspy segment in QF_poolInit()

  Because Rxbuf didn't initialized when used in here, and F103 always throw HardFault_Handler() when executing QS_INSERT_BYTE() or somelike statments.

2.gnu version of apollo stm32f767ig (startup.s)

3.mdk version of armfly stm32f103ze-ek (startup.c)

4.UML QMtool usage




