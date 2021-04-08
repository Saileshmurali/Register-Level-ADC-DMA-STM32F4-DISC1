#include "stm32f407xx.h"
void configureLED(void);
void ADC_Config(void);
void dma_config(void);
uint8_t value[8200];
uint32_t add=(uint32_t)value,reg,length=8000;
void ADC_Config(void)
{
	RCC->AHB1ENR |= (1UL<<0);// Enable clock for Port A
	RCC->APB2ENR |= (1UL<<8);  // Enable ADC1 clock
	ADC->CCR |= (1UL<<16);  //Pre-scale clock to ADC by 4
	ADC1->CR1 |= (2UL<<24); //8 bit ADC
	ADC1->CR2 |= (1UL<<1);  //Enable continuous conversion
	ADC1->CR2 |= (1UL<<10); //Enable EOC after every conversion
	ADC1->SMPR2 |= (7UL<<3);//Select sampling time as 480 cycles for channel 1
	GPIOA->MODER |= (3UL<<2); //Configure PA1 in analog mode
	ADC1->SQR3 |= (1<<0);//Select channel 1 in ADC (Corresponds to PA1)
}
void dma_config(void)
{
	RCC->AHB1ENR |=(1UL<<22);//Enable DMA2 clock
	ADC1->CR2 |= (1UL<<8);  //Enable DMA
	ADC1->CR2 |= (1UL<<9);
	DMA2->LIFCR |= 0x7DUL;//Clear all interrupt flags
	DMA2_Stream0->CR &=~(7UL<<25);//Select channel 0 of DMA
	DMA2_Stream0->CR &=~(3UL<<13);//Set memory of DMA to 8bit per data
	DMA2_Stream0->CR &=~(3UL<<11);//Peripheral size to 8 bit per data
	DMA2_Stream0->CR |=(1UL<<10);//Memory increment mode
	DMA2_Stream0->CR &=~(3UL<<6);//Data transfer from peripheral to memory
	DMA2_Stream0->CR |=(3UL<<3);//Transfer full and half complete interrupt Enable
	DMA2_Stream0->CR |= (1UL << 8);//Enable circular mode
	DMA2_Stream0->NDTR=length;//Set DMA length to 8000
	DMA2_Stream0->M0AR=add;//Select destination memory address as buffer declared
	reg=(uint32_t)&ADC1->DR;
	DMA2_Stream0->PAR=reg;//Select peripheral address as ADC1 data register
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);//Enable DMA Interrupt

}
void configureLED(void)
{
	RCC->AHB1ENR |=(1UL<<3);  //Enable GPIOD clock
	GPIOD->MODER &= ~(0xFFUL<<12*2);
	GPIOD->MODER |= (0x55UL<<12*2);//Set Pins PD12-15 as output
}

void DMA2_Stream0_IRQHandler(void)
{
	GPIOD->ODR ^=(1UL<<12);//Toggle LED
	DMA2->LIFCR |= 0x7DUL;//Clear all interrupt flags
}
int main()
{
	ADC_Config();
	configureLED();
	dma_config();
	DMA2_Stream0->CR |=(1UL<<0);//Enable DMA
	ADC1->CR2 |= (1UL<<0); // Start the ADC
	ADC1->CR2 |= (1UL<<30);// Start the conversion
	while(1)
	{
	}
}
