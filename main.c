
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#define F_CPUX 8000000L

//dioda do debagowania
#define LED_deb PA7
 //// dioda do symulacji losowoœci
#define LED_sim PA6
#define DDR_sim DDRA
#define PORT_sim PORTA
#define PIN_sim PINA
/*
//przycisk funkcyjny
#define key_fun PC5
#define PORT_key PORTC
#define PIN_key PINC
#define DDR_key DDRC
*/
#define ilosc_losowan 128U
#define los_liczb ilosc_losowan/8U

#define FIFO_L 128 //dlugosc kolejek FIFO


struct key_st
{
	volatile unsigned char :5;
	volatile unsigned char button :1;
};
// pole bitowe
#define keyh ((struct key_st*)&PINC)->button
#define keyh_dir ((struct key_st*)&DDRC)->button
#define keyh_pullup ((struct key_st*)&PORTC)->button
//------------
// losowanie po kliknieciu
//debounce na innym timerze ??

//OGARNIJ sie RAKU

/*
 losowanie z powietrza
 // ADC INIT !!!!!
  *
  losuj 100 razy wrzucaj ostatni bit (LSB) ADC do
  tabeli.
  uint8_t tabela_los[] !!!

  tabela_los[i]= ADC &0x01;  // 0b0000 0001 <- LSB
  tabela_los[1]=tabela_los[i] << 1;
  co ósme/szesnaste losowanie inkrement i , (i++)
  bajt 8^2 losowych , 16^2 na word.
  sprawdz czy lepiej skalowac na mniejszej ilosci czyli
  0-255 ale wiekse interwaly
  czy 0 - 16^2 ale mniejsze interwaly
  PORTD na diodzie :C
  ////////////
  lets do levels and counting
  case <==> odbior
  every ISR timer1 count up
  if its moar than xxx
  lvl up down
  rthen go for losowanie
  */
//-------------------------------
/*INTERWA£ CZASU NA TIMERZE 0
- moze FSM
	SLEEP 0
	LOSOWANIE 1
	NADAWANIE 2
	INTERAKCJA 3

	0->1->2->3 .... 3->0.. wait for ext interrupt

*/
//volatile uint8_t n=0;

enum tryb {energy_save, losowanie, nadawanie, interakcja };

volatile uint8_t rnd_tab[los_liczb];
volatile uint8_t  stan=nadawanie;

volatile uint8_t debug_val=0;

void PutToSerial(unsigned char data);
void StrToSerial(char *msg);
void ADC_init (void);
void PutIntToSerial (uint8_t integer);
void TIMER0_init(void);
void TIMER2_init(void);



ISR(ADC_vect)
{
	//ADC=n;
	static uint8_t rnd_cnt = 0; // licznik losowan
	static uint8_t tab_cnt = 0;
	static uint8_t inc = 0;

	rnd_tab[inc]|= ADC & 0x01;

	if(tab_cnt >= 7) // jesli przesunieto wynik 7 razy od 0 do 6 to kolejny bajt
	{
		PutIntToSerial(rnd_tab[inc]);
		StrToSerial("\r");
		tab_cnt = 0;
		inc++;
	}
	else // w przeciwnym razie przesun bitowo o jeden 0000 000x w 0000 00x0
	{
		rnd_tab[inc]<<=1;
	}

	if(inc > los_liczb)
	{
		inc = 0 ;
	}

	if(rnd_cnt >= 127)
	{
		rnd_cnt = 0;
		ADCSRA&=~(1<<ADEN); // wylacz przetwornik po 128 znakach
	}
	rnd_cnt++;
	tab_cnt++;
}


ISR(TIMER2_COMP_vect)
{
static uint16_t keycnt=0;
static uint8_t keylev=0;
static uint8_t keyr=0;

	if(keycnt==0)
	{
		switch(keylev)
		{
		case 0:     // waiting for press
			if(keyh!=1)
			{
				keyr=keyh;
				keylev=1;
				keycnt=200;
			}
			break;
		case 1:	 // button was pressed, debouncing start
			if(keyh == keyr)
			{
				//PORTA^=(1<<PA7);
				debug_val ++;
				PutIntToSerial(debug_val);
				keylev=2;
			}
			break;
		case 2:
			if(keyh==1)
				{
				keylev=0;
				}
			break;
		}
	}


if(keycnt>0)
	keycnt--;

}


ISR(TIMER0_COMP_vect)
{

	static uint8_t cnt = 0;
	static uint16_t rnd_cnt = 0;
	static uint8_t wibr = 0;
	static uint8_t tab_ind = 0; // indeks tablicy


	switch(stan)
	{
	case nadawanie:
		if((tab_ind==4 )||( tab_ind==5 )||( tab_ind==6 )|| (tab_ind==10) ||( tab_ind==11 )||( tab_ind==12))
		{
			wibr = 0;
		}
		else
		{
			wibr = 1;
		}
		cnt++;
		if(cnt>=100)
		{
			cnt=0;
		}

		if(rnd_cnt>= rnd_tab[tab_ind])
		{
			StrToSerial("[");
			PutIntToSerial(rnd_tab[tab_ind]);
			StrToSerial("]");
			tab_ind++;
			rnd_cnt=0;
		}
		rnd_cnt++;
		if(tab_ind > 15)
		{
			tab_ind = 0;
			PORT_sim^=(1<<LED_deb);
			StrToSerial("\r");

		}

		if(wibr)
		{
			PORT_sim |= (1<<LED_sim);
					}
		else
		{
			PORT_sim &=~(1<<LED_sim);
		}
		break;
		//start  counting
	case interakcja:

		break;
	}
	// 16 random bytes   wibracje 4B / przerwa 2B / wibracje 4B / przerwa 2B wibracje 4B || 4B + 2B + 4B + 2B + 4B = 16 B
	// zastanów sie czy nie lepiej losowac te¿ sekwencje w sensie iloœc przerw i wibracji.
	// 10 ms podstawa 4B 1B + 1B... = 255 *4 = 0-1020 * 10 ms / najkrócej 0 najd³u¿ej 10 sekund -.-' za dlugo ! :) przemianuj mniej bajtow ?
	// losuj mniej bajtow  albo interwa³ czasu zmniejsz 2ms ?? 0 do 2s thats fine.
	// na razie proba na akutalnym
}


void PutIntToSerial (uint8_t integer) // zamiana INT'a na chara
{
	if(integer >=100)
	{
		PutToSerial((((integer%1000)-(integer%100))/100 + 0x30)); // setki
	}
	if(integer >=10)
	{
		PutToSerial((((integer%100)-(integer%10))/10) + 0x30); // dziesiatki
	}
	PutToSerial(integer%10 + 0x30);
}

//czêstotliwoœæ zegara mikrokotrolera
#define BAUDRATE 9600L//115200L //prêdkoœæ transmisji w bodach
#define BAUD_REG ((F_CPU/(16*BAUDRATE))-1) //dzielnika cz. UBRR
//---------------------------------------------------------------
void uart_init(void) //Inicjowanie portu UART
{
	UBRRH=(BAUD_REG>>8); //dzielnk czestotliwosci transmisji
	UBRRL=BAUD_REG;
	//Format ramki danych: 8data,1stopbit
	UCSRC |=((1<<URSEL)| (1<<UCSZ0)|(1<<UCSZ1));
	UCSRB |=((1<<RXEN)| (1<<TXEN)); //zezwolenie na odbior i nadawanie
	UCSRB |=(1<<RXCIE); //zezwolenie na przerwanie odb.
}


struct {
	unsigned char wi; //indeks odczytu
	unsigned char ri; //indeks zapisu
	char buff[FIFO_L];
}
	InputFifo ={0,0}, //kolejeka wejœciowa FIFO
	OutputFifo={0,0}; //kolejeka wyjœciowa FIFO

ISR(USART_RXC_vect) /*VECTOR(11), USART, RxComplete*/
{
	InputFifo.buff[InputFifo.wi++]=UDR; //umieszczenie danej w kolejce
	if (InputFifo.wi == FIFO_L) InputFifo.wi = 0;
}
//----------------------------------------------------
// zwraca: 0 -gdy bufor odbiornika pusty
// 1 -gdy pobrany znak umieszczony w '*p_dada'
unsigned char GetFromSerial(unsigned char *p_dada)
{
	if (InputFifo.ri == InputFifo.wi) return 0;

	else {
		*p_dada = InputFifo.buff[InputFifo.ri++];
		if (InputFifo.ri == FIFO_L) InputFifo.ri=0;
		return 1;
	}
}

ISR(USART_UDRE_vect) /*VECTOR(12), USART Data Register Empty*/
{
	if (OutputFifo.wi == OutputFifo.ri)
	{
		UCSRB &= ~(1<<UDRIE); //bufor FIFO pusty - wylaczenie przerwania
	}
	else {
		UDR = OutputFifo.buff[OutputFifo.ri++]; //dana z bufora do rejestru UDR
		if (OutputFifo.ri == FIFO_L) OutputFifo.ri = 0;
	}
}
//----------------------------------------------------------------
void PutToSerial(unsigned char data)
{
	OutputFifo.buff[OutputFifo.wi++] = data;

	if (OutputFifo.wi == FIFO_L ) OutputFifo.wi = 0;
	UCSRB |=(1<<UDRIE); // wlaczenie przerwan
}

void StrToSerial(char *msg)
{
	while(*msg!=0) PutToSerial(*msg++);
}

int main(void)
{
	DDR_sim|=((1<<LED_deb)|(1<<LED_sim));
	keyh_dir=0;
	keyh_pullup=1;

	sei();

	ADC_init();
	uart_init();
	TIMER0_init();
	TIMER2_init();

	while(1)
	{
	}
	return 0;
}

void ADC_init (void)
{
	ADMUX|=(1<<REFS0);  // avcc[niepod³¹czony kondek na avcc a gnd-celowo]
	ADCSRA|=(1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS1)|(1<<ADPS2);
	// wl. ADC/start konwersji/autotriger EN/interrupt execute EN/ presk 64  f_adc=8MHz/64=125kHz
}
void TIMER2_init(void)
{
	OCR2=199; // F_CPU 8Mhz/16MHz przerwanie co 0,4ms/0,2ms
	TCCR2|=(1<<WGM21|1<<CS21);  //CTC pres 8
	TIMSK|=(1<<OCIE2);  //Odblokowanie przerwan
}

void TIMER0_init(void)
{
	// Interwa³ czasu 10 ms / interrupt /  CTC   // f CTC = fio/(2*presc*(1+OCR) -> t = 10 ms 1/t = fCTC -> 1/10ms -> 100 Hz
	// OCR ~= 77 5ms , OCR ~=155 10ms // presc 256 8 000 000 / 256 = 31250
	TCCR0 |= (1<<WGM01)|(1<<CS02); // // preskaler 256  | CTC
	TIMSK |=(1<<OCIE0); // flaga przerwania | wykonanie przerwana
	OCR0 = 155;
}

