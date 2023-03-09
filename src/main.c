/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "Utility.h"

/* Descrição:
 Implementação de uma comunicação serial simples controlada por software:
 - O pino PA0 opera em dreno aberto como pino de comunicação entre duas placas
 - Os pinos PA0 de duas placas devem ser interligados juntamente com o GND
 - Os botões K0 e K1 acionam, respectivamente, os LEDs D2 e D3 da outra placa (operação remota)
 - Há um buzzer no pino PA1 para sinalizar o envio de um dado
*/

//Protótipos de funções
void envia_cmd(uint8_t);	//função para enviar um comando no barramento
uint8_t recebe_cmd(void);	//função para receber um comando
void buzzer(void);			//função de ativação do buzzer

int main(void)
{
	Configure_Clock();			//configura o sistema de clock
	Delay_Start();				//inicializa funções de Delay

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN;	//habilita o clock do GPIOA e GPIOE

	GPIOA->ODR |= (1<<7) | (1<<6) | (1 << 2);		//inicia com leds e buzzer desligados e linha COM em idle
	GPIOA->OTYPER |= (1 << 2);						//saída open-drain em PA0
	GPIOA->PUPDR |= (0b01 << 4);					//habilita pull-up em PA0
	GPIOA->MODER |= (0b01 << 14) | (0b01 << 12) | (0b01 << 4) | (0b01 << 2)  ; 	//pinos PA0, PA1, PA6 e PA7 no modo saída
	GPIOE->PUPDR |= (0b01 << 8) | (0b01 << 6);								//habilita pull-up em PE4 e PE3

	GPIOA->MODER &=~(0b11)  ;
	GPIOA->PUPDR |= (0b10);



	Delay_ms(100);	//aguarda sinais estabilizarem
	while(1)
	{
		if(!(GPIOE->IDR & (1 << 4)))			//verifica se PE4 foi pressionado
		{
			envia_cmd(0b01);
			Delay_ms(50);
			if( !(GPIOA->IDR & (1 << 2)) ) //verifica se  há retorno
			{

			}
			else //se não há retorno
			{
				GPIOA->ODR ^= 1 << 6;			//alterna o estado do LED em PA6
			}
			buzzer();							//sinaliza o fim do envio
			Delay_ms(75);					//filtro de bouncing
			while(!(GPIOE->IDR & (1 << 4)));	//aguarda o botão ser solto
		}

		if(!(GPIOE->IDR & (1 << 3)))			//verifica se PE3 foi pressionado
		{
			envia_cmd(0b10);
			Delay_ms(50);
			if( GPIOA->IDR & (1 << 2) ) //verifica se NÃO há retorno
			{
				GPIOA->ODR ^= 1 << 7;			//alterna o estado do LED em PA7
			}
			buzzer();							//sinaliza o fim do envio
			Delay_ms(75);					//filtro de bouncing
			while(!(GPIOE->IDR & (1 << 3)));	//aguarda o botão ser solto
		}

		if(!(GPIOA->IDR & (1 << 2)))	//verifica se há comunicação
		{
			uint8_t recebido = recebe_cmd();	//recebe o comando
			if(recebido == 0b01)
			{
				GPIOA->ODR &= ~ (1 << 2);	//start bit
				//envia_retorno();
				Delay_ms(200);
				GPIOA->ODR |= (1<< 2);	//stop bit
				//envia_cmd(0b00); //indica que ao microcontrolador 2 que o comando foi recebido
				GPIOA->ODR ^= 1 << 6;			//alterna o estado do LED em PA6
			}
			if(recebido == 0b10)
			{
				GPIOA->ODR &= ~ (1 << 2);	//start bit
				//envia_retorno();
				Delay_ms(200);
				GPIOA->ODR |= (1 << 2);	//stop bit
				//envia_cmd(0b00); //indica que ao microcontrolador 2 que o comando foi recebido
				GPIOA->ODR ^= 1 << 7;			//alterna o estado do LED em PA7
			}
		}
	}
}



//Função para envio de um comando
void envia_cmd(uint8_t dado)
{
	GPIOA->ODR &= ~(1 << 2);	//start bit
	Delay_us(50);		//aguarda tempo do start bit
	for(uint8_t counter=0; counter<2; ++counter) //envia os bits do comando
	{
		if(dado & 1)			//envia o próximo bit
			GPIOA->ODR |= ( 1 << 2);
		else
			GPIOA->ODR &= ~ (1 << 2);
		dado >>= 1;				//desloca os bits para envio posterior
		Delay_us(50);			//aguarda o tempo do bit
	}
	GPIOA->ODR |= (1 << 2);	//stop bit
	Delay_us(50);		//tempo do stop bit
}

//Função para recebimento de um comando
uint8_t recebe_cmd(void)
{
	uint8_t dado_recebido = 0;
	uint8_t dado_retornado = 0;
	Delay_us(25);			//aguarda metade do start bit
	if(!(GPIOA->IDR & (1 << 2)))	//confirma que houve um start bit
	{
		for(uint8_t counter=0; counter<2; ++counter)	//leitura dos bits
		{
			Delay_us(50);				//aguarda o tempo do bit
			if(GPIOA->IDR & (1 << 2))
				dado_recebido |= 1;
			else
				dado_recebido &= ~1;
			dado_recebido <<= 1;
		}

		Delay_us(50);			//aguarda para fazer leitura do stop bit
		if((GPIOA->IDR & (1 << 2)))	//confirma que houve um stop bit
		{
			dado_recebido >>= 1;
			Delay_us(25);		//aguarda o fim do tempo do stop bit
			return dado_retornado = ((dado_recebido & 0b10) >> 1) | ((dado_recebido & 0b01) << 1); //retorna o dado recebido
		}
		else
			return 0;	//não houve recepção do stop bit, aborta recepção
	}
	else
		return 0;		//não houve recepção do start bit, aborta recepção
}

//Função de sinalização de fim de envio de dado
void buzzer(void)
{
	GPIOA->ODR |= 1 << 1;			//liga o buzzer
	Delay_ms(10);					//aguarda
	GPIOA->ODR &= ~(1 << 1);		//desliga o buzzer
	Delay_ms(80);					//aguarda
	GPIOA->ODR |= 1 << 1;			//liga o buzzer
	Delay_ms(10);					//aguarda
	GPIOA->ODR &= ~(1 << 1);		//desliga o buzzer
}
