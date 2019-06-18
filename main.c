
/*
A program for chirp modulation (LFM) based on 1508PL8
Features SPI1:

PA7 - MOSI
PA6 - MISO
PA5 - SCK
PA4 - NSS

Setup:
CMSIS->core
Device->Startup
StdPeriph->framework,SPI,GPIO,RCC

Opt. for Target-> C/C++->Include paths->0inc,0src
*/

#define SYSCLK_FREQ_72MHZ		//system clock to 72 mhz
#define _1508PL8_CLK_MHZ	800	//define 1508pl8's clock speed (for calculating timings, durations, etc.)
#define STAGE1_DUR_US			1000000
#define STAGE2_DUR_US			0
#define STAGE3_DUR_US			1000000	//stage 3 duration for freq_inc calculation (in us)
#define STAGE4_DUR_US			0
#define CENTRAL_FREQ_MHZ	10			//central frequency for LFM
#include "stm32_top_header.h"	//various functions
#include "1508pl8_regs.h"			//1508pl8 regs

uint16_t chirp_buf[3]={0,0,0};			//48-bit buffer for chirp duration, freqs and other

void spi1_init(void);
void spi1_write_reg(uint16_t addr, uint16_t data);	//write to 1508pl8
uint16_t spi1_read_reg(uint16_t addr);							//read reg
void chirp_stage_duration_calc_ns(uint64_t _ns, uint16_t* buffer);	//calculate duration for chirp stages in ns
void chirp_stage_duration_calc_us(uint64_t _us, uint16_t* buffer);
void chirp_stage_duration_calc_ms(uint32_t _ms, uint16_t* buffer);
void chirp_stage_duration_calc_s(uint32_t _s, uint16_t* buffer);

void chirp_freq_calc_hz(double _hz, uint16_t* buffer);
void chirp_freq_calc_khz(double _khz, uint16_t* buffer);	//calculate initial freqs 
void chirp_freq_calc_Mhz(double _Mhz, uint16_t* buffer);

																												//freq increment
void chirp_freq_inc_hz(double _hz, uint16_t* buffer, uint64_t stage_duration_us);
void chirp_freq_inc_khz(double _khz, uint16_t* buffer, uint64_t stage_duration_us);
void chirp_freq_inc_Mhz(double _Mhz, uint16_t* buffer, uint64_t stage_duration_us);

void chirp_input_code_converter(uint32_t init_freq_input, uint16_t freq_inc_input, uint16_t* init_freq_buf_out, uint16_t* freq_inc_buf_out);

int main(void)
{

	SystemInit();			//init system clock
	delay_init();			//init delay in stm32_delay_asm.c
	tft_init_defaults();		//init TFT display with defaults(see TFT_SPI.c)
	/*
	TFT uses SPI2 (B12-B15), 
	and this project features an old version of TFT_SPI.c
	*/
	tft_dec_value("SYSCLK:",SystemCoreClock); tft_newl();
	
	
	spi1_init();

	_1508pl8_reg_init();	//init pl8 regs

	
	spi1_write_reg(pl8_com_regs.SWRST,0x0078);			//reset
	//_delay_ms(1000);
	spi1_write_reg(pl8_ch1_regs.dPh_all_L,0);
	spi1_write_reg(pl8_ch1_regs.dPh_all_M,0);
	spi1_write_reg(pl8_ch1_regs.dPh_all_H,0);
	spi1_write_reg(pl8_ch1_regs.Mul_all,0x7FF8);		//write Max amplitude to all profiles
	spi1_write_reg(pl8_ch1_regs.Offset_all,0);
	
	spi1_write_reg(pl8_ch1_regs.LS_Ph1,0x0);
	spi1_write_reg(pl8_ch1_regs.LS_Ph2,0x0);
	
	chirp_stage_duration_calc_us(STAGE1_DUR_US,chirp_buf);	//1,3 stage
	
	spi1_write_reg(pl8_ch1_regs.LS_TPH1_L,chirp_buf[0]);
	spi1_write_reg(pl8_ch1_regs.LS_TPH1_M,chirp_buf[1]);	//configure LFM stage 1 duration
	spi1_write_reg(pl8_ch1_regs.LS_TPH1_H,chirp_buf[2]);
	
	spi1_write_reg(pl8_ch1_regs.LS_TPH3_L,chirp_buf[0]);
	spi1_write_reg(pl8_ch1_regs.LS_TPH3_M,chirp_buf[1]);	//configure LFM stage 3 duration
	spi1_write_reg(pl8_ch1_regs.LS_TPH3_H,chirp_buf[2]);
	
	//tft_hex_value("LS_TPH1_L:",spi1_read_reg(pl8_ch1_regs.LS_TPH1_L));tft_newl();
	
	tft_hex_value("Dur_US:",chirp_buf[0]); tft_newl();
	tft_hex_value("Dur_US:",chirp_buf[1]); tft_newl();
	tft_hex_value("Dur_US:",chirp_buf[2]); tft_newl();
	
	chirp_stage_duration_calc_us(STAGE2_DUR_US,chirp_buf);						//2,4 stage
	
	spi1_write_reg(pl8_ch1_regs.LS_TPH2_L,chirp_buf[0]);
	spi1_write_reg(pl8_ch1_regs.LS_TPH2_M,chirp_buf[1]);	//configure LFM stage 2 duration
	spi1_write_reg(pl8_ch1_regs.LS_TPH2_H,chirp_buf[2]);
	
	spi1_write_reg(pl8_ch1_regs.LS_TPH4_L,chirp_buf[0]);
	spi1_write_reg(pl8_ch1_regs.LS_TPH4_M,chirp_buf[1]);	//configure LFM stage 4 duration
	spi1_write_reg(pl8_ch1_regs.LS_TPH4_H,chirp_buf[2]);
	
	chirp_freq_calc_Mhz(CENTRAL_FREQ_MHZ-0.5,chirp_buf);					//calc initial freq 1
	
	spi1_write_reg(pl8_ch1_regs.LS_F1_L,chirp_buf[0]);
	spi1_write_reg(pl8_ch1_regs.LS_F1_M,chirp_buf[1]);
	spi1_write_reg(pl8_ch1_regs.LS_F1_H,chirp_buf[2]);	//config init freq 1
	
	//tft_hex_value("LS_F1_L:",spi1_read_reg(pl8_ch1_regs.LS_F1_L));tft_newl();
	
	tft_hex_value("Dur_F1:",chirp_buf[0]); tft_newl();
	tft_hex_value("Dur_F1:",chirp_buf[1]); tft_newl();
	tft_hex_value("Dur_F1:",chirp_buf[2]); tft_newl();
	
	chirp_freq_inc_khz(1000,chirp_buf,STAGE1_DUR_US);
	
	spi1_write_reg(pl8_ch1_regs.LS_dF1_L,chirp_buf[0]);
	spi1_write_reg(pl8_ch1_regs.LS_dF1_M,chirp_buf[1]);
	spi1_write_reg(pl8_ch1_regs.LS_dF1_H,chirp_buf[2]);	//config freq 1 inc
	
	//tft_hex_value("LS_dF1_L:",spi1_read_reg(pl8_ch1_regs.LS_dF1_L));tft_newl();
	
	tft_hex_value("Dur_F1I:",chirp_buf[0]); tft_newl();
	tft_hex_value("Dur_F1I:",chirp_buf[1]); tft_newl();
	tft_hex_value("Dur_F1I:",chirp_buf[2]); tft_newl();
	
	chirp_freq_calc_Mhz(CENTRAL_FREQ_MHZ+0.5,chirp_buf);			//calc initial freq 2
	
	spi1_write_reg(pl8_ch1_regs.LS_F2_L,chirp_buf[0]);
	spi1_write_reg(pl8_ch1_regs.LS_F2_M,chirp_buf[1]);
	spi1_write_reg(pl8_ch1_regs.LS_F2_H,chirp_buf[2]);	//config init freq 2
	
	//tft_hex_value("LS_F2_L:",spi1_read_reg(pl8_ch1_regs.LS_F2_L));tft_newl();
	
	tft_hex_value("Dur_F2:",chirp_buf[0]); tft_newl();
	tft_hex_value("Dur_F2:",chirp_buf[1]); tft_newl();
	tft_hex_value("Dur_F2:",chirp_buf[2]); tft_newl();
	
	chirp_freq_inc_khz(-1000,chirp_buf,STAGE3_DUR_US);
	
	spi1_write_reg(pl8_ch1_regs.LS_dF2_L,chirp_buf[0]);
	spi1_write_reg(pl8_ch1_regs.LS_dF2_M,chirp_buf[1]);
	spi1_write_reg(pl8_ch1_regs.LS_dF2_H,chirp_buf[2]);	//config freq 2 inc
	
	//tft_hex_value("LS_DF2_L:",spi1_read_reg(pl8_ch1_regs.LS_dF2_L));tft_newl();
	/*
	tft_hex_value("Dur_F2I:",chirp_buf[0]); tft_newl();
	tft_hex_value("Dur_F2I:",chirp_buf[1]); tft_newl();
	tft_hex_value("Dur_F2I:",chirp_buf[2]); tft_newl();
	*/
	//spi1_write_reg(pl8_ch1_regs.LS_Ph1,0xC405);		//?????????
	//spi1_write_reg(pl8_ch1_regs.LS_Ph2,0x1033);
	
	spi1_write_reg(pl8_com_regs.CTR,0x1000);		//DAC1 enable
	//spi1_write_reg(pl8_ch1_regs.TSW,0x0003);
	spi1_write_reg(pl8_ch1_regs.LS_CTR, 0xB370);				//LFM on, auto mode on, etc B310
	spi1_write_reg(pl8_com_regs.CLR, 0x0010);					//run 1 stage of LFM
	
	//tft_hex_value("CTR:",spi1_read_reg(pl8_com_regs.CTR));tft_newl();
	//tft_hex_value("LS_CTR:",spi1_read_reg(pl8_ch1_regs.LS_CTR));tft_newl();
	//_delay_ms(5000);
	//spi1_write_reg(pl8_com_regs.CLR, 0x0040);		//stop LFM
	
	chirp_input_code_converter(0x28F5C2,10000,chirp_buf,chirp_buf);
}

void spi1_write_reg(uint16_t addr, uint16_t data)
{
		clr(GPIOA->ODR,4);			//NSS \__	
	while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = 0x10;			//set addr 
	while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = ((addr & 0xFF00)>>8);	//send high byte
		while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = (addr & 0x00FF);			//send low byte
	while(!(SPI1->SR & SPI_SR_RXNE)){}	//wait until RXNE = 1
	(void)SPI1->DR;	//CLEAR RXNE flag
	while((SPI1->SR & SPI_SR_BSY)){}	//wait until BSY = 0 
		_delay_us(1);
		set(GPIOA->ODR,4);			//NSS __/
	
	_delay_us(20);
		
		clr(GPIOA->ODR,4);			//NSS \__
	while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = 0x20;			//wr data 
	while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = ((data & 0xFF00)>>8);	//send high byte
		while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = (data & 0x00FF);
	while(!(SPI1->SR & SPI_SR_RXNE)){}	//wait until RXNE = 1
	(void)SPI1->DR;
	while((SPI1->SR & SPI_SR_BSY)){}	//wait until BSY = 0 
		_delay_us(1);
		set(GPIOA->ODR,4);			//NSS __/
}
//read 16-bit reg from 1508pl8
uint16_t spi1_read_reg(uint16_t addr)
{
	uint8_t read_data8;
	uint16_t read_data16;
	
	clr(GPIOA->ODR,4);			//NSS \__	
	while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = 0xB0;			//SETAFT 
	while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = ((addr & 0xFF00)>>8);	//send high byte
		while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = (addr & 0x00FF);			//send low byte
	while(!(SPI1->SR & SPI_SR_RXNE)){}	//wait until RXNE = 1
	(void)SPI1->DR;	//CLEAR RXNE flag
	while((SPI1->SR & SPI_SR_BSY)){}	//wait until BSY = 0 
		_delay_us(1);
	set(GPIOA->ODR,4);			//NSS __/
	
	_delay_us(20);
		
	clr(GPIOA->ODR,4);			//NSS \__	
	while(!(SPI1->SR & SPI_SR_TXE)){}	//wait until TXE = 1
	SPI1->DR = 0x00;			//NOP		
	while(!(SPI1->SR & SPI_SR_RXNE)){}	//wait until RXNE = 1
	(void)SPI1->DR;	//CLEAR RXNE flag
		
	SPI1->DR = 0xFF;	//8 Dummy clocks	
	while(!(SPI1->SR & SPI_SR_RXNE)){}	//wait until RXNE = 1
	(void)SPI1->DR;	//CLEAR RXNE flag				

	SPI1->DR = 0xFF;			//send stuff 1	
	while(!(SPI1->SR & SPI_SR_RXNE)){}	//wait until RXNE = 1
	read_data16 = SPI1->DR;	// read data1	
  SPI1->DR = 0xFF;			//send stuff 2
	while(!(SPI1->SR & SPI_SR_RXNE)){}	//wait until RXNE = 1
	read_data8 = SPI1->DR;	// read data2
		
	while((SPI1->SR & SPI_SR_BSY)){}	//wait until BSY = 0 
		
	read_data16 <<= 8;				//shift left
	read_data16 |= read_data8;		
		_delay_us(1);
	set(GPIOA->ODR,4);			//NSS __/
		
	return read_data16;
}

void spi1_init(void)
{
	
	//set(AFIO->MAPR,AFIO_MAPR_SPI1_REMAP);	//remap spi1 to A15-B5
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			//ENABLE clock to all ports
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
		//SPI1 init:
	GPIO_InitTypeDef gpio;
	SPI_InitTypeDef spi;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);		//Enable clock to SPI 1
	
	gpio.GPIO_Pin = GPIO_Pin_6;				//Choose 4 pin for MISO
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;			//Input floating
	//gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);			//Initialize port B with this settings
	
	gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;	//Choose 3,5 pins for SCK, MOSI
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;			//Alternate function PP
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);			//Initialize port B with this settings
	
	gpio.GPIO_Pin = GPIO_Pin_4;				//Choose 15 pin for NSS
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;			//PP
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);			//Initialize port B with this settings
	
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;		//Direction of SPI - full duplex (both MISO, MOSI are used)
	spi.SPI_Mode = SPI_Mode_Master;												//master mode
	spi.SPI_DataSize = SPI_DataSize_8b;								//8-bit data length
	spi.SPI_CPOL = SPI_CPOL_Low;											//CPOL = 0
	spi.SPI_CPHA = SPI_CPHA_1Edge;									//CPHA = 0
	spi.SPI_NSS = SPI_NSS_Soft;										//NSS by soft (GPIOA15)
	spi.SPI_FirstBit = SPI_FirstBit_MSB;					//MSB first
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//Baud rate 
	SPI_Init(SPI1, &spi);			//Init SPI with this settings
	
	SPI1->CR2 |=SPI_CR2_SSOE;	//SSOE = 1 (enable NSS)
	
	SPI_Cmd(SPI1, ENABLE);		//enable SPI2
}

//Chirp modulation - specific funcs

//calculate chirp stage duration in ns
void chirp_stage_duration_calc_ns(uint64_t _ns, uint16_t* buffer)
{
	uint8_t _ns_base = (4*1000)/_1508PL8_CLK_MHZ;			//calculate time base in ns (Tclk*4)
	uint64_t base_ticks = (_ns/_ns_base);		//calculate ticks
	
	for(uint8_t i=0;i<3;i++)
	{
	*buffer++ = (uint16_t)(base_ticks >> 16*i);		//stack 48-bit code into buffer, buffer[0] is low, buffer[2] is high
	}
}

void chirp_stage_duration_calc_us(uint64_t _us, uint16_t* buffer)
{
	chirp_stage_duration_calc_ns(_us * 1000, buffer);
}

void chirp_stage_duration_calc_ms(uint32_t _ms, uint16_t* buffer)
{
	chirp_stage_duration_calc_ns(_ms * 1000000, buffer);
}

void chirp_stage_duration_calc_s(uint32_t _s, uint16_t* buffer)
{
	chirp_stage_duration_calc_ns(_s * 1000000000, buffer);
}
//calculate frequencies

void chirp_freq_calc_hz(double _hz, uint16_t* buffer)
{
	double freq_sum;	//Fout(Mhz)*2^48/Fclk = Fh*2^32+Fm*2^16+Fl
	
	uint8_t pl8_clk_div_32 = (_1508PL8_CLK_MHZ)/32;
	
	freq_sum = (double)0x2000000000/(double)pl8_clk_div_32;						//2^37 / clk/32
	freq_sum /= (double)15625; 
	freq_sum *= (double)_hz;
	
	double div_rem = (freq_sum/0x100000000);								//division remainder = freq_um/2^32
	
	uint16_t Fh = (uint16_t)(freq_sum/0x100000000);						//Fh = (whole)freq_sum/2^32
	div_rem -=Fh;																				//div_rem = div_rem - Fh
	freq_sum = div_rem * 0x100000000;															//0,xxxx * 2^32
	
	div_rem = ((freq_sum)/0x10000);
	uint16_t Fm = (uint16_t)((freq_sum)/0x10000);				//Fm = (whole)freq_sum / 2^16
	div_rem -=Fm;	
	freq_sum = div_rem * 0x10000;											//0.xxxx * 2^16
	
	uint16_t Fl = (uint16_t)(freq_sum);							//Fl = (whole)freq_sum-Fh-Fm
	
	*buffer++ = Fl;		//stack 48-bit code into buffer, buffer[0] is low, buffer[2] is high
	*buffer++ = Fm;
	*buffer = Fh;
}

void chirp_freq_calc_khz(double _khz, uint16_t* buffer)
{
	chirp_freq_calc_hz(_khz*1000,buffer);
}

void chirp_freq_calc_Mhz(double _Mhz, uint16_t* buffer)
{
	chirp_freq_calc_hz(_Mhz*1000000,buffer);
}

//calculate freq increment
/*
void chirp_freq_inc_hz(signed long _hz, uint16_t* buffer)		//freq increment is signed value
{
	uint64_t _div_base = _1508PL8_CLK_MHZ*9765625000000000;	//pl8_clk*1000000*10000/2^10 * 1000000000
	uint64_t div_base = _div_base/0x2000000000;	/// 2^37
	
	tft_dec_value("DBS:",div_base); tft_newl();
	
	uint64_t __hz;
	
	if(_hz < 0) {__hz = ~_hz+1;  __hz &= 0xFFFFFFFFFFFF;}		//if signed, then translate into unsigned
	else {__hz = _hz & 0xFFFFFFFFFFFF;}																	//else left as it is
	__hz *= 1000000000;
	__hz = __hz/div_base;
	if(_hz < 0) {__hz = ~__hz+1;  __hz &= 0xFFFFFFFFFFFF;}	//translate back
	
	for(uint8_t i=0;i<3;i++)
	{
	*buffer++ = (uint16_t)((__hz) >> 16*i);		//stack 48-bit code into buffer, buffer[0] is low, buffer[2] is high
	}
}
*/

void chirp_freq_inc_hz(double _hz, uint16_t* buffer, uint64_t stage_duration_us)
{
	double __hz;
	if(_hz<0) {__hz = -_hz;}
	else {__hz = _hz;}
	double _div_base = (double)_1508PL8_CLK_MHZ*(double)9765625;	//pl8_clk*1000000/2^10
	_div_base = _div_base/(double)0x2000000000;	/// 2^37
	_div_base /= (double)100; 									//don't know where it came from, but it kinda works
	
	_div_base *=(double)(stage_duration_us); 	//the final value is calculated as /(stage_duration)
	uint64_t _ticks;

	_ticks = (uint64_t)(__hz/_div_base);				//whole part of the number
	//_ticks /=(st_dur_us*_1508PL8_CLK_MHZ/4); 	//the final value is calculated as _ticks/(stage_duration/4Tclk)
	
	if(_hz < 0) {_ticks = ~_ticks+1;  _ticks &= 0xFFFFFFFFFFFF;}		//if signed, then translate into unsigned
	else {_ticks = _ticks & 0xFFFFFFFFFFFF;}																	//else left as it is
	
	for(uint8_t i=0;i<3;i++)
	{
	*buffer++ = (uint16_t)((_ticks) >> 16*i);		//stack 48-bit code into buffer, buffer[0] is low, buffer[2] is high
	}
}

void chirp_freq_inc_khz(double _khz, uint16_t* buffer, uint64_t stage_duration_us)
{
	chirp_freq_inc_hz(_khz*1000,buffer, stage_duration_us);
}

void chirp_freq_inc_Mhz(double _Mhz, uint16_t* buffer, uint64_t stage_duration_us)
{
	chirp_freq_inc_hz(_Mhz*1000000,buffer, stage_duration_us);
}
//
//calculate initial freq and freq inc values from input 4-byte and 2-byte 2-complement words
//Input:uint32_t init_freq_input, uint16_t freq_inc_input
//Output: 2 buffers of 48 bit (three 16-bit words) each
void chirp_input_code_converter(uint32_t init_freq_input, uint16_t freq_inc_input, uint16_t* init_freq_buf_out, uint16_t* freq_inc_buf_out)
{
	/*
	first 4-byte word represents relative initial frequency specified in 2's - complement code
	that means it represents deviation relative to the central frequency
	i.e. if the central freq is 10MHz and init_freq_input is -1MHz, then the freq value at the start of the next frame will be 9 MHz
	
	the 2-byte freq_inc_input specifies frequency increment (decrement) in 2's - complement code at 1,3 LFM stages 
	*/
	uint64_t _freq_in;
	double _init_freq;
	if(init_freq_input & 0x80000000)	{_freq_in = ~init_freq_input+1;}	//if MSB = 1, convert to direct code
	else {_freq_in = init_freq_input;}							//otherwise no conversion
	
	tft_hex_value("FR_IN:",_freq_in); tft_newl();
	
	_init_freq = (double)(_freq_in*(_1508PL8_CLK_MHZ*1000000))/(double)0x80000000;					//output value = init_freq*Fclk/2^31 (Hz)
	tft_float_dec_value("FREQ_INIT:",_init_freq,3); tft_newl();
	if(init_freq_input & 0x80000000){_init_freq -= CENTRAL_FREQ_MHZ*1000000;}
	else {_init_freq += CENTRAL_FREQ_MHZ*1000000;}																				//add/substract from central freq
	
	chirp_freq_calc_hz(_init_freq,init_freq_buf_out);		//calculate initial frequency
	
	
	
}
