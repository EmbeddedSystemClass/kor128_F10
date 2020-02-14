/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mLibrary
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mLibrary.c
*********************************************************************/

#include "mLibrary.h"

extern SPI_HandleTypeDef hspi2;

void print_float(uint32_t ch, float temp) {
    int32_t i, j;
    float f;
    
    f = 100.00f * temp;
    
    j = (int32_t)f;
    
    if((int32_t)temp > 0)
    {
        i = (j - ((int32_t)temp*100));
    }
    else
    {
        i = (((int32_t)temp*100) - j);
    }
    
    j = j / 100;
    
    printf("AFEC0 Ch%d: %d.%d[mv]\r\n", (int)ch, (int)j, (int)i);
}

_Bool getAbit(uint8_t data, uint8_t bit) {
  return (data & (1 << bit)) >> bit;
}

uint32_t get_diff_tick(uint32_t curr_tick, uint32_t start_tick)
{
	uint32_t overflow_previous_tick=0, overflow_after_tick=0;
	
    if(curr_tick < start_tick)
    {
        /*
         * 현재 tick이 Opening 시작 때 저장한 tick보다 작은 경우(overflow)
         * 
         * 0xFFFFFFFF(4294967295)에서 0으로 overflow되는데 1 tick이 소요 됨
         * 
         * 
         * opening 시작 시 tick이 4294967195이고, 200 tick만큼 지난 경우
         * curr_tick(99) < start_tick(4294967195)						// overflow 시 1 tick이 소요되므로 99까지만 count 됨
         * 
         * overflow_tick = 4294967295 - 4294967195 = 100
         * 
         * start_tick = 4294965295
         * curr_tick = 4999
         */
    	overflow_previous_tick = 0xFFFFFFFF - start_tick;
    	overflow_after_tick = curr_tick + 1;
        return overflow_previous_tick + overflow_after_tick;
    }
    else
    {
    	/*
    	 * 현재 tick이 Opening 시작 때 저장한 tick보다 큰 경우 (정상)
    	 */
        return curr_tick - start_tick;
    }
}

void mram_write_enable()
{
	uint8_t aSPITxBuffer = CMD_WREN;
	HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, &aSPITxBuffer, 1, 5000);
	HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_SET);
}

void mram_write_diable()
{
	uint8_t aSPITxBuffer = CMD_WRDI;
	HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, &aSPITxBuffer, 1, 5000);
	HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_SET);
}

void mram_byte_write(uint32_t addr, uint8_t data)
{
	uint8_t wrbuf[5]={0};
	
	if(addr&0xFF800000)
	{
		printf("## MRAM Address Can't exceed 0x40.0000(4,194,304=4MByte)\r\n");
		return;
	}
	else
	{
		wrbuf[0] = CMD_WRITE;
		wrbuf[1] = ((addr >> 16) & 0xff);		// 0x00.FF.00.00
		wrbuf[2] = ((addr >> 8) & 0xff);		// 0x00.00.FF.00
		wrbuf[3] = (addr & 0xff);				// 0x00.00.00.FF
		wrbuf[4] = data;
		
		HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, (uint8_t*)wrbuf, 5, 5000);
		HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_SET);	
	}
}

void mram_block_write(void)
{
	uint32_t i=0;
	
	printf("## Start MRAM Block Write\r\n");
	for(i=0; i<256; i++)
	{
		mram_byte_write(i, i);
		printf(".");
	}
	printf("\r\n");
	
	printf("## Start MRAM Block Read\r\n");
	for(i=0; i<256; i++)
		printf("%02X ", mram_byte_read(i));
	printf("\r\n");
}

uint8_t mram_byte_read(uint32_t addr)
{
	uint8_t rdbuf[5]={0};
	
	rdbuf[0] = CMD_READ;
	rdbuf[1] = ((addr >> 16) & 0xff);
	rdbuf[2] = ((addr >> 8) & 0xff);
	rdbuf[3] = (addr & 0xff);
	rdbuf[4] = 0;
	
	HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi2, (uint8_t*)rdbuf, 5, 5000);
	HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_SET);
	return rdbuf[4];
}

void mram_erase(uint32_t addr, uint32_t size)
{
	uint32_t i, err_cnt=0;

	printf("## Start MRAM Erase\r\n");
	for(i=0; i<size; i++)				// 256 == 0x100
	{
		mram_byte_write(i, 0);
	}
	
	for(i=0; i<size; i++)
	{
		if(mram_byte_read(i) != 0)	err_cnt++;
	}
	
	if(err_cnt==0)	printf("## MRAM Erase OK\r\n");
	else			printf("## MRAM Erase %d Cnt Fail\r\n", (int)err_cnt);
}

/*
 *  고장 정보들을 모두 삭제 한다.
 */
void mram_erase_Fault_data(void)
{
	uint8_t Trace_count = 0;
	int i,j =0;
	Trace_count = mram_byte_read(MRAM_FAULT_ADDR);

	for(i=0; i<Trace_count; i++)
	{
		for(j=1; j<61; j++)
		{
			mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+j,0);
		}
	}
	mram_byte_write(MRAM_FAULT_ADDR,0);
}

void Erase_fault(uint8_t error_code)
{
	uint8_t lamp_count = 0;
	if(error_list_flag[error_code] == true)
	{
		error_list_flag[error_code] = false;
		for(_ErrorCodeList i =DCU_HARD_FAULT; i<DCU_OBSTACLE; i++)
		{
			if(error_list_flag[i] ==false)
			{
				lamp_count = 0;
			}
			else
			{
				lamp_count++;
			}
		}
		if(lamp_count == 0) mip_EmergencyLamp(0);// 고장이 없는 상태이므로 램프를 소거한다.
	}
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mLibrary.c
*********************************************************************/
