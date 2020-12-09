/******************************************************************************
 ******************************************************************************
 ***    UNIVERSIDADE DE BRAS�LIA - UNB                                      ***
 ***    DISCIPLINA: FGA0096 - ELETR�NICA EMBARCADA        TURMA: A          ***
 ***    PROFESSOR: Guillermo Alvarez Bestard, Dr. Eng. Mecatr�nica          ***
 ***    ALUNO: Lucas dos Santos Barros de Sousa   MATR�CULA:180022555       ***
 ***    ALUNO: Matheus Oliveira Dias              MATR�CULA:18/0025104      ***
 ***    ALUNO: Victor Hugo Ciurlini               MATR�CULA:11/0021223      ***
 ***               TRABALHO FINAL ELETR�NICA EMBARCADA                      ***
 ******************************************************************************
 ******************************************************************************/

/* DESCRI��O DE FUNCIONAMENTO*/


//DECLARA��O DE BIBLIOTECAS:
#include "mcc_generated_files/mcc.h"

//DEFINI��ES
#define conv_I  0.1                         //O coeficiente de convers�o da corrente � de 0.5, visto que o valor m�x de cprrente � de 1000 mA, no entanto como teremos que transmitir o valor de corrente/5 para evitar mais operacoes pelo uC j� multiplicamos o valor por 1/5=0.2, logo 0.5*0.2=0.1          
#define conv_temp  0.3                      //O coeficiente de convers�o da temperatura � de 0.1, visto que o valor m�x de temperatura assumido � de 100�C, no entanto como teremos que transmitir o valor de temperatura*3 para evitar mais operacoes pelo uC j� multiplicamos aqui logo 0.1*3=0.3
#define conv_second  0.0000005
#define distance_1_pulse_mult2  1.5        // 15 mm/20 pulsos = 0.75 mm, no entanto usaremos: 0.75*2 = 1.5 para evitar mais uma opera��o a fim de cumprir o preparo de envio
#define distance_1_pulse_mult5  3.75        // 15 mm/20 pulsos = 0.75 mm,no entanto usaremos: 0.75*5 = 3.75 para evitar mais uma opera��o a fim de cumprir o preparo de envio

//DECLARA��O DOS PROT�TIPOS DAS FUN��ES: 
void comunicacao ();
void controle();
void gerenciamento();

//DECLARA��O DAS VARI�VEIS:
int and_dst;                    //Vari�vel para armazenas o andar de destino
int and_origem;                 //Vari�vel para armazenas o andar de origem
int pulses;
float I_m;
float temp_mt;
int aux_tempo = 0;
uint8_t state_motor;
uint8_t and_ating;
uint16_t ccp_value;
uint8_t byte[5];

//FUN��ES DE INTERRUP��O

//INTERRUP��O PARA ENVIO DOS DADOS A CADA 100mS (5 BYTES)
void send_data()
	{
        aux_tempo++;
        if(aux_tempo == 14)
        {
            int i=0;
            while(i<5)
            {
                if(EUSART_is_tx_ready())
                    {   
                        EUSART_Write(byte[i]);
                        i++;
                    }        
            }
        
            aux_tempo =0;
        }
	}

void sensor1()
    {
        and_ating=0;
    }

void sensor2()
    {
        and_ating=1;
    }

void sensor3()
    {
        and_ating=2;
    }

void sensor4()
    {
        and_ating=3;
    }

void get_pulse(uint16_t capturedValue)
    {
        TMR1_WriteTimer(0);
        ccp_value = capturedValue;
    }

// APLICA��O PROPRIAMENTE DITA
void main(void)
{
    
  //FUN��ES DE INICIALIZA��O
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
   //PASSAGEM DE PAR�METROS PARA FUN��ES DE INTERRUP��O 
    TMR4_SetInterruptHandler(send_data);
    IOCBF0_SetInterruptHandler(sensor1);
    IOCBF3_SetInterruptHandler(sensor2);
    IOCBF4_SetInterruptHandler(sensor3);
    IOCBF5_SetInterruptHandler(sensor4);
    CCP4_SetCallBack(get_pulse);
    
    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();
    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    // LOOP PRINCIPAL
    while (1)
    {
       comunicacao ();    // Chama a fun��o resons�vel por tratar a comunica��o
       controle();        // Chama a fun��o resons�vel por tratar o controle do motor
       gerenciamento();   // Chama a fun��o resons�vel por gerenciar as solicita��es
    }
}



void comunicacao ()
{
 //   uint8_t byte_0,byte_1,byte_2,byte_3,byte_4;
 //   uint8_t byte[5];
    uint8_t aux;// and_dst,and_ating;       
    float speed = distance_1_pulse_mult5/(((float)ccp_value)*conv_second);
    float position = distance_1_pulse_mult2*pulses;
    if(EUSART_is_rx_ready())
        {
            aux= EUSART_Read();
            and_origem = (int)((aux & 0x0C)>>2);    //Pega os bits 2,3 e os transforma em inteiros
            and_dst = (int)(aux & 0x02);            //Pega os bits 0,1 e os transforma em inteiros 
        }
// OBS PARA LUCAS: TALVEZ N�O SEJA NECESS�RIO O DESLOCAMENTO >>1 ANALISAR ROTEIRO!!!
//VERIFICAR SE O VETOR FUNCIONA! SE N�O FUNCIONAR MUDE A L�GICA DO ENVIO E DESCOMENTE ABAIXO
//    byte_0 = ((state_motor<<4)& 0x20)|(and_ating & 0x02);
//    byte_1 = 0x80 |((((uint8_t)position)>>1)& 0x7F);  
//    byte_2 = 0x80 |((((uint8_t)speed)>>1)& 0x7F);
//    byte_3 = 0x80 |((((uint8_t)I_m)>>1)& 0x7F);
//    byte_4 = 0x80 |((((uint8_t)temp_mt)>>1)& 0x7F);
    
    byte[0] = ((state_motor<<4)& 0x20)|(and_ating & 0x02);
    byte[1] = 0x80 |((((uint8_t)position)>>1)& 0x7F);  
    byte[2] = 0x80 |((((uint8_t)speed)>>1)& 0x7F);
    byte[3] = 0x80 |((((uint8_t)I_m)>>1)& 0x7F);
    byte[4] = 0x80 |((((uint8_t)temp_mt)>>1)& 0x7F);

}

void controle()
{
   I_m = conv_I*ADC_GetConversion(0);         //Realiza a leitura da corrente, converte para mA usando *conv_I, j� esta preparado para envio, veja as defini��es! (Resolu��o da medi��o 0.5 mA )
   temp_mt = conv_temp*ADC_GetConversion(1);   //Realiza a leitura da temperatura, converte para C usando *conv_temp, j� esta preparado para envio, veja as defini��es! (Resolu��o da medi��o 0.1 �C))
}

void gerenciamento()
{

}