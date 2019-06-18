
#include "1508pl8_regs.h"

_1508pl8_com_regs pl8_com_regs;
_1508pl8_CH1_regs pl8_ch1_regs;
uint32_t __packed *pl8_reg_ptr;

void _1508pl8_reg_init(void)
{
	pl8_reg_ptr = &pl8_com_regs.SWRST;		//point to the first element of struct
for(uint8_t i=0;i<10;i++)
	{
		*pl8_reg_ptr++ = i;		//assign common reg addresses (0000 - 0009)
	}
	
	pl8_com_regs.T_CAPTURE = 0x00E0;
	pl8_com_regs.T_SEL_STATE = 0x00E1;
	pl8_com_regs.T_E_SEL = 0x00E2;
	
	pl8_ch1_regs.LS_CTR = 0x1000;
	pl8_ch1_regs.LS_CRFMIN = 0x1001;
	pl8_ch1_regs.TSW = 0x1002;
	pl8_ch1_regs.LS_TPH1_L = 0x1010;
	pl8_ch1_regs.LS_TPH1_M = 0x1011;
	pl8_ch1_regs.LS_TPH1_H = 0x1012;
	
	pl8_ch1_regs.LS_TPH2_L = 0x1014;
	pl8_ch1_regs.LS_TPH2_M = 0x1015;
	pl8_ch1_regs.LS_TPH2_H = 0x1016;
	
	pl8_ch1_regs.LS_TPH3_L = 0x1018;
	pl8_ch1_regs.LS_TPH3_M = 0x1019;
	pl8_ch1_regs.LS_TPH3_H = 0x101A;
	
	pl8_ch1_regs.LS_TPH4_L = 0x101C;
	pl8_ch1_regs.LS_TPH4_M = 0x101D;
	pl8_ch1_regs.LS_TPH4_H = 0x101E;
	
	pl8_ch1_regs.LS_F1_L = 0x1020;
	pl8_ch1_regs.LS_F1_M = 0x1021;
	pl8_ch1_regs.LS_F1_H = 0x1022;
	
	pl8_ch1_regs.LS_F2_L = 0x1024;
	pl8_ch1_regs.LS_F2_M = 0x1025;
	pl8_ch1_regs.LS_F2_H = 0x1026;
	
	pl8_ch1_regs.LS_Ph1 = 0x1030;
	pl8_ch1_regs.LS_Ph2 = 0x1031;
	
	pl8_ch1_regs.LS_dF1_L = 0x1040;
	pl8_ch1_regs.LS_dF1_M = 0x1041;
	pl8_ch1_regs.LS_dF1_H = 0x1042;
	
	pl8_ch1_regs.LS_dF2_L = 0x1044;
	pl8_ch1_regs.LS_dF2_M = 0x1045;
	pl8_ch1_regs.LS_dF2_H = 0x1046;
	
	pl8_ch1_regs.dPh_all_L = 0x1300;
	pl8_ch1_regs.dPh_all_M = 0x1301;
	pl8_ch1_regs.dPh_all_H = 0x1302;
	
	pl8_ch1_regs.P_all = 0x1304;
	pl8_ch1_regs.Mul_all = 0x1305;
	pl8_ch1_regs.Offset_all = 0x1306;
	
for(uint8_t i=0;i<64;i++)
{
	pl8_ch1_regs.profile[i].prf_num = i;		//assign profile numbers into structure
																									//fill structs with reg addresses
	pl8_ch1_regs.profile[i].dPhx_L = 0x1400 + 0x10 * i;
	pl8_ch1_regs.profile[i].dPhx_M = 0x1401 + 0x10 * i;
	pl8_ch1_regs.profile[i].dPhx_H = 0x1402 + 0x10 * i;
	pl8_ch1_regs.profile[i].Px = 0x1404 + 0x10 * i;
	pl8_ch1_regs.profile[i].Mulx = 0x1405 + 0x10 * i;
	pl8_ch1_regs.profile[i].Offsetx = 0x1406 + 0x10 * i;
	
}

}
