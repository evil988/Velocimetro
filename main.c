#define F_CPU 16000000UL    //define a frequência do microcontrolador - 16MHz
#include <util/delay.h>     //biblioteca para o uso das rotinas de atraso
#include <avr/interrupt.h>  //para o uso de interrupções
#include <avr/io.h>         //definições do componente especificado
#include <avr/pgmspace.h>   //para o uso do PROGMEM, gravação de dados na flash

//Definições de macros para o trabalho com bits
#define set_bit(y,bit_x)(y|=(1<<bit_x))
#define clr_bit(y,bit_x)(y&=~(1<<bit_x))
#define cpl_bit(y,bit_x)(y^=(1<<bit_x))
#define tst_bit(y,bit_x)(y&(1<<bit_x))

//Definicoes para LCD
#define DADOS_LCD       PORTD //4 bits de dados do LCD no PORTD
#define CONTROLE_LCD    PORTD //PORT com os pinos de controle do LCD (pino R/W em 0).
#define E               PD3   //pino de habilitação do LCD (enable)
#define RS              PD2   //pino para informar se o dado é uma instrução ou caractere
#define tamanho_vetor   3     //número de digitos individuais para a conversão por ident_num()
#define conv_ascii      48    //48 se ident_num() deve retornar um número no formato ASCII (0 para formato normal)

//sinal de habilitação para o LCD
#define pulso_enable() _delay_us(1); set_bit(CONTROLE_LCD,E); _delay_us(1); clr_bit(CONTROLE_LCD,E); _delay_us(45)

//prototipo das funcoes
void sensorUltrassonico();
void velocidade();
void cmd_LCD(unsigned char c, char cd);
void inicia_LCD_4bits();
void escreve_LCD();
//void escreve_LCD(char *c);
void escreve_LCD_Flash(const char *c, unsigned char cursor_position);
void separa_numeros(unsigned short valor, unsigned char *digitos);
void dados_iniciais();

//constantes
PROGMEM const char mensagem1[] = "Distancia:    cm\0";
PROGMEM const char mensagem2[] = "xxx\0";
PROGMEM const char mensagem3[] = "Vel.:       cm/s\0";

//variaveis
unsigned short inicio_sinal,distancia,posicao;
short velocidade_media;
unsigned char digitos[tamanho_vetor]; //declaração da variável para armazenagem dos digitos

//interrupção por captura do valor do TCNT1
ISR(TIMER1_CAPT_vect)
{
	cpl_bit(TCCR1B,ICES1);                    //troca a borda de captura do sinal
	if(!tst_bit(TCCR1B,ICES1))                //lê o valor de contagem do TC1 na borda de subida do sinal
	inicio_sinal = ICR1;                    //salva a primeira contagem para determinar a largura do pulso
	else                                      //lê o valor de contagem do TC1 na borda de descida do sinal
	distancia = (ICR1 - inicio_sinal)/58/2; //agora ICR1 tem o valor do TC1 na borda de descida do sinal, então calcula a distância
}

void setup()
{
	ADCSRA = 0b10000111;  //ADC habilitado, frequencia = F_CPU/128;
	ADCSRB = 0x00;        //conversão contínua;
	
	//configuracao ultrassom;
	DDRB  = 0b00000010;  //somente pino de disparo como saída (PB1), captura no PB0 (ICP1)
	PORTB = 0b11111101;
	
	TCCR1B = (1<<ICES1)|(1<<CS11);  //TC1 com prescaler = 8, captura na borda de subida
	TIMSK1 = (1<<ICIE1);            //habilita a interrupção por captura
	sei();                          //habilita a chave de interrupções globais

	//configuracao LCD
	DDRD = 0xFF;  //todos os pinos do PORTD como saída para escrita no LCD
	inicia_LCD_4bits();
	escreve_LCD_Flash(mensagem1,0x80);
	escreve_LCD_Flash(mensagem3,0xC0);
}

int main()
{
	setup();
	
	dados_iniciais();
	
	while(1)
	{
		sensorUltrassonico();
		posicao = distancia - posicao;
		velocidade();
		posicao = distancia;
	}
}

void dados_iniciais()
{
	sensorUltrassonico();
	posicao = distancia;
}

void velocidade()
{
	velocidade_media = posicao/0.5;
	cmd_LCD(0xC6,0);

	if((velocidade_media>(431/0.5))||(velocidade_media<(-431/0.5)))
	{
		cmd_LCD(' ',1);
		escreve_LCD_Flash(mensagem2,0xC7);
		return;
	}
	
	if (velocidade_media < 0)
	{
		velocidade_media = velocidade_media * (-1);
		cmd_LCD('-',1);
	}
	else
	cmd_LCD(' ',1);
	
	separa_numeros(velocidade_media, digitos);
	escreve_LCD();
}

void sensorUltrassonico()
{
	//pulso de 10us para iniciar as funcoes do sensor
	set_bit(PORTB,PB1);
	_delay_us(10);
	clr_bit(PORTB,PB1);

	//desloca o cursor para a posição
	cmd_LCD(0x8B,0);
	if(distancia<431) //se o pulso for menor que 25 ms mostra o valor da distancia
	{
		separa_numeros(distancia, digitos);
		escreve_LCD();
	}
	else //senão escreve xxx no valor
	escreve_LCD_Flash(mensagem2,0x8B);
	
	_delay_ms(500);  //tempo minimo para uma nova medida de distancia;
}

void cmd_LCD(unsigned char c, char cd)
{
	if(cd) //caractere
	set_bit(CONTROLE_LCD,RS);
	else   //instrução
	clr_bit(CONTROLE_LCD,RS);
	
	//primeiro nibble de dados - 4 MSB
	//compila o código para os pinos de dados do LCD nos 4 MSB do PORT
	DADOS_LCD = (DADOS_LCD & 0x0F)|(0xF0 & c);
	
	pulso_enable();
	//segundo nibble de dados - 4 LSB
	//compila o código para os pinos de dados do LCD nos 4 MSB do PORT
	DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (c<<4));
	
	pulso_enable();
	if((cd==0) && (c<4)) //se for instrução de retorno ou limpeza espera LCD estar pronto
	_delay_ms(2);
}

void inicia_LCD_4bits() //sequência ditada pelo fabricando do circuito integrado HD44780
{
	//o LCD será só escrito. Então, R/W é sempre zero.
	clr_bit(CONTROLE_LCD,RS);  //RS em zero indicando que o dado para o LCD será uma instrução
	clr_bit(CONTROLE_LCD,E);   //pino de habilitação em zero
	_delay_ms(20);          //tempo para estabilizar a tensão do LCD, após VCC ultrapassar 4.5 V (na prática pode ser maior)
	
	//interface de 4 bits
	DADOS_LCD = (DADOS_LCD & 0x0F) | 0x30;
	
	pulso_enable(); //habilitação respeitando os tempos de resposta do LCD
	_delay_ms(5);
	pulso_enable();
	_delay_us(200);
	pulso_enable();
	
	//interface de 4 bits, deve ser enviado duas vezes (a outra está abaixo)
	DADOS_LCD = (DADOS_LCD & 0x0F) | 0x20;
	
	pulso_enable();
	
	cmd_LCD(0x28,0); //interface de 4 bits 2 linhas (aqui se habilita as 2 linhas)
	//são enviados os 2 nibbles (0x2 e 0x8)
	cmd_LCD(0x08,0); //desliga o display
	cmd_LCD(0x01,0); //limpa todo o display
	cmd_LCD(0x0C,0); //mensagem aparente cursor inativo não piscando
	cmd_LCD(0x80,0); //inicializa cursor na primeira posição a esquerda - 1a linha
}

void escreve_LCD()
{
	unsigned char n = tamanho_vetor - 1;
	for(n;n!=255;n--){
		cmd_LCD(digitos[n],1);
	}
}

void escreve_LCD_Flash(const char *c, unsigned char cursor_position)
{
	cmd_LCD(cursor_position,0);
	for (;pgm_read_byte(&(*c))!=0;c++) cmd_LCD(pgm_read_byte(&(*c)),1);
}

void separa_numeros(unsigned short valor, unsigned char *digitos)
{
	unsigned char n;
	for(n=0; n<tamanho_vetor; n++)
	digitos[n] = 0 + conv_ascii; //limpa vetor para armazenagem dos digitos
	
	while (valor!=0)
	{
		*digitos = (valor%10) + conv_ascii; //pega o resto da divisão por 10
		valor /=10;                         //pega o inteiro da divisão por 10
		digitos++;
	}
}