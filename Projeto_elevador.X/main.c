/******************************************************************************
 ******************************************************************************
 ***    UNIVERSIDADE DE BRASÍLIA - UNB                                      ***
 ***    DISCIPLINA: FGA0096 - ELETRÔNICA EMBARCADA        TURMA: A          ***
 ***    PROFESSOR: Guillermo Alvarez Bestard, Dr. Eng. Mecatrônica          ***
 ***    ALUNO: Lucas dos Santos Barros de Sousa   MATRÍCULA:180022555       ***
 ***    ALUNO: Matheus Oliveira Dias              MATRÍCULA:18/0025104      ***
 ***    ALUNO: Victor Hugo Ciurlini               MATRÍCULA:11/0021223      ***
 ***               TRABALHO FINAL ELETRÔNICA EMBARCADA                      ***
 ******************************************************************************
 ******************************************************************************/

/* DESCRIÇÃO DE FUNCIONAMENTO*/


//DECLARAÇÃO DE BIBLIOTECAS:
#include "mcc_generated_files/mcc.h"

//DEFINIÇÕES
#define conv_I  0.1                         //O coeficiente de conversão da corrente é de 0.5, visto que o valor máx de cprrente é de 1000 mA, no entanto como teremos que transmitir o valor de corrente/5 para evitar mais operacoes pelo uC já multiplicamos o valor por 1/5=0.2, logo 0.5*0.2=0.1          
#define conv_temp  0.3                      //O coeficiente de conversão da temperatura é de 0.1, visto que o valor máx de temperatura assumido é de 100ºC, no entanto como teremos que transmitir o valor de temperatura*3 para evitar mais operacoes pelo uC já multiplicamos aqui logo 0.1*3=0.3
#define conv_second  0.0000005
#define distance_1_pulse_mult2  1.5        // 15 mm/20 pulsos = 0.75 mm, no entanto usaremos: 0.75*2 = 1.5 para evitar mais uma operação a fim de cumprir o preparo de envio
#define distance_1_pulse_mult5  3.75        // 15 mm/20 pulsos = 0.75 mm,no entanto usaremos: 0.75*5 = 3.75 para evitar mais uma operação a fim de cumprir o preparo de envio

//DECLARAÇÃO DOS PROTÓTIPOS DAS FUNÇÕES: 
void comunicacao ();
void controle();
void gerenciamento();

//DECLARAÇÃO DAS VARIÁVEIS:
int and_dst;                    //Variável para armazenas o andar de destino
int and_origem;                 //Variável para armazenas o andar de origem
int pulses;
float I_m;
float temp_mt;
int aux_tempo = 0;
uint8_t state_motor;
uint8_t and_ating;
uint16_t ccp_value;
uint8_t byte[5];

//FUNÇÕES DE INTERRUPÇÃO

//INTERRUPÇÃO PARA ENVIO DOS DADOS A CADA 100mS (5 BYTES)
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

// APLICAÇÃO PROPRIAMENTE DITA
void main(void)
{
    
  //FUNÇÕES DE INICIALIZAÇÃO
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
   //PASSAGEM DE PARÂMETROS PARA FUNÇÕES DE INTERRUPÇÃO 
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
       comunicacao ();    // Chama a função responsável por tratar a comunicação
       controle();        // Chama a função responsável por tratar o controle do motor
       gerenciamento();   // Chama a função responsável por gerenciar as solicitações
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
// OBS PARA LUCAS: TALVEZ NÃO SEJA NECESSÁRIO O DESLOCAMENTO >>1 ANALISAR ROTEIRO!!!
//VERIFICAR SE O VETOR FUNCIONA! SE NÃO FUNCIONAR MUDE A LÓGICA DO ENVIO E DESCOMENTE ABAIXO
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
    I_m = conv_I*ADC_GetConversion(0);          // Realiza a leitura da corrente, converte para mA usando *conv_I, já esta preparado para envio, veja as definições! (Resolução da medição 0.5 mA )
    temp_mt = conv_temp*ADC_GetConversion(1);   // Realiza a leitura da temperatura, converte para C usando *conv_temp, já esta preparado para envio, veja as definições! (Resolução da medição 0.1 ºC))
    int count = 0;                              // Contador responsável pelos loops
    int a = 52;                                 // Valor da aceleração boa para o motor (cálculado por torricelli)
    int max_dutyValue = 1023;                   // Valor máximo aceito pelo PWM (v = 20 mm/s))
    int min_dutyValue = 256;                    // Valor do PWM para v = 5 mm/s
    int destiny = 240;                          // Variável para receber o andar de destino
    int route = destiny - 40;                   // Controla o número de pulsos com v = 20 mm/s
    int dutyValue = 0;                          // Inicia dutyValue em zero
    int pulse = 0;                              // Inicia número de pulsos em zero

    for(count = 0; count < 20; count++){        // Loop responsável pela aceleração do motor
        pulse+=1;                               // Contador para número de pulsos
        dutyValue+=a;                           // Adiciona valor de aceleração no dutyValue
        if(dutyValue > max_dutyValue){          // Comparação se o valor de dutyValue não ultrapassa o máximo permitido
            PWM3_LoadDutyValue(max_dutyValue);  // Caso sim, esse valor é substituído pelo valor máximo
        }else{                                  // Else
            PWM3_LoadDutyValue(dutyValue);      // Envia dutyValue com o valor acrescido de aceleração do motor
        }
        
    }
    count = 0;                                  // Reinicia o contador para o próximo loop
    for(count = 0; count < route ; count++){    // Loop responsável pela velocidade máxima constante
        pulse+=1;                               // Contador de pulsos
        PWM3_LoadDutyValue(max_dutyValue);      // Envia max_dutyValue para o PWM
    }
    count = 0;                                  // Reinicia o contador para o próximo loop
    for(count = 0; count < 20; count++){        // Loop responsável pela desaceleração do motor
        pulse+=1;                               // Contador de pulsos
        dutyValue-=a;                           // Subtrai valor da aceleração do dutyValue
        if(dutyValue < min_dutyValue){          // Comparação se o valor de dutyValue não ultrapassa o mínimo permitido na desaceleração
            PWM3_LoadDutyValue(min_dutyValue);  // Caso sim, esse valor é substituído pelo valor mínimo para v = 5 mm/s
        }else{                                  // Else
        PWM3_LoadDutyValue(dutyValue);          // Envia dutyValue com o valor decrescido de aceleração do motor
        }
    }
    PWM3_LoadDutyValue(0);                      // Para o motor

}

void gerenciamento()
{

}