/*
 * fault.cpp
 *
 *  Created on: Apr 11, 2014
 *      Author: walmis
 */

#include <xpcc/debug.hpp>
#include <xpcc/architecture.hpp>

enum { r0, r1, r2, r3, r12, lr, pc, psr};

void boot_jump( uint32_t address ){
   __asm("LDR SP, [R0]\n"
   "LDR PC, [R0, #4]");
}


extern "C" void HardFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}
extern "C" void UsageFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}
extern "C" void BusFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}

extern "C"
void Hard_Fault_Handler(uint32_t stack[]) {

	//register uint32_t* stack = (uint32_t*)__get_MSP();

	XPCC_LOG_DEBUG .printf("Hard Fault\n");

	XPCC_LOG_DEBUG .printf("r0  = 0x%08x\n", stack[r0]);
	XPCC_LOG_DEBUG .printf("r1  = 0x%08x\n", stack[r1]);
	XPCC_LOG_DEBUG .printf("r2  = 0x%08x\n", stack[r2]);
	XPCC_LOG_DEBUG .printf("r3  = 0x%08x\n", stack[r3]);
	XPCC_LOG_DEBUG .printf("r12 = 0x%08x\n", stack[r12]);
	XPCC_LOG_DEBUG .printf("lr  = 0x%08x\n", stack[lr]);
	XPCC_LOG_DEBUG .printf("pc  = 0x%08x\n", stack[pc]);
	XPCC_LOG_DEBUG .printf("psr = 0x%08x\n", stack[psr]);

	while(1);

	//for(int i = 0; i < 10000; i++) {}
	//NVIC_SystemReset();

}


