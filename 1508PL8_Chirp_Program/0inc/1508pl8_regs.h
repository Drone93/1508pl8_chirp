//Registers for 1508pl8

#ifndef uint32_t
#include "stdint.h"		//so the compiler knows what's the uintxx_t mean 
#endif

/*
Struct templates for reg addresses (register map)
Each reg is assigned to its corresponding address in 1508pl8_regs.c
*/


typedef __packed struct 
{
	uint8_t prf_num;
	uint32_t   dPhx_L;
	uint32_t   dPhx_M;
	uint32_t   dPhx_H;
	uint32_t   Px;
	uint32_t   Mulx;
	uint32_t   Offsetx;
}_1508pl8_CH1_prof_regs;		//1st channel harmonic synthesys profile regs

//_1508pl8_CH1_prof_regs pl8_ch1_profile[64];			//array for each profile's regs

typedef __packed struct
{
	uint32_t   LS_CTR;
	uint32_t   LS_CRFMIN;
	uint32_t   TSW;
	uint32_t   LS_TPH1_L;
	uint32_t   LS_TPH1_M;
	uint32_t   LS_TPH1_H;
	uint32_t   LS_TPH2_L;
	uint32_t   LS_TPH2_M;
	uint32_t   LS_TPH2_H;
	uint32_t   LS_TPH3_L;
	uint32_t   LS_TPH3_M;
	uint32_t   LS_TPH3_H;
	uint32_t   LS_TPH4_L;
	uint32_t   LS_TPH4_M;
	uint32_t   LS_TPH4_H;
	uint32_t   LS_F1_L;
	uint32_t   LS_F1_M;
	uint32_t   LS_F1_H;
	uint32_t   LS_F2_L;
	uint32_t   LS_F2_M;
	uint32_t   LS_F2_H;
	uint32_t   LS_Ph1;
	uint32_t   LS_Ph2;
	uint32_t   LS_dF1_L;
	uint32_t   LS_dF1_M;
	uint32_t   LS_dF1_H;
	uint32_t   LS_dF2_L;
	uint32_t   LS_dF2_M;
	uint32_t   LS_dF2_H;
	uint32_t   dPh_all_L;
	uint32_t   dPh_all_M;
	uint32_t   dPh_all_H;
	uint32_t   P_all;
	uint32_t   Mul_all;
	uint32_t   Offset_all;
	
	_1508pl8_CH1_prof_regs __packed profile[64];			//array for each profile's regs
	
}_1508pl8_CH1_regs;				//1st channel regs

typedef __packed struct
{
	uint32_t   SWRST;
	uint32_t   DEVID;
	uint32_t   SEL_REG;
	uint32_t   CTR;
	uint32_t   SYNC;
	uint32_t   CLR;
	uint32_t   LINK;
	uint32_t   ROUTE;
	uint32_t   TC_L;
	uint32_t   TC_H;
	uint32_t   T_CAPTURE;
	uint32_t   T_SEL_STATE;
	uint32_t   T_E_SEL;
}_1508pl8_com_regs;			//common regs

//prototypes

extern _1508pl8_com_regs pl8_com_regs;
extern _1508pl8_CH1_regs pl8_ch1_regs;
extern uint32_t __packed *pl8_reg_ptr;

void _1508pl8_reg_init(void);
