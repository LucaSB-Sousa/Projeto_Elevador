/******************************************************************************
 ******************************************************************************
 ***    UNIVERSIDADE DE BRASï¿½LIA - UNB                                      ***
 ***    DISCIPLINA: FGA0096 - ELETRï¿½NICA EMBARCADA        TURMA: A          ***
 ***    PROFESSOR: Guillermo Alvarez Bestard, Dr. Eng. Mecatrï¿½nica          ***
 ***    ALUNO: Lucas dos Santos Barros de Sousa   MATRï¿½CULA:180022555       ***
 ***    ALUNO: Matheus Oliveira Dias              MATRï¿½CULA:18/0025104      ***
 ***    ALUNO: Victor Hugo Ciurlini               MATRï¿½CULA:11/0021223      ***
 ***               TRABALHO FINAL ELETRï¿½NICA EMBARCADA                      ***
 ******************************************************************************
 ******************************************************************************/

/* DESCRIï¿½ï¿½O DE FUNCIONAMENTO*/


//DECLARAï¿½ï¿½O DE BIBLIOTECAS:
#include "mcc_generated_files/mcc.h"

//DEFINIï¿½ï¿½ES
#define conv_I  0.1                         //O coeficiente de conversï¿½o da corrente ï¿½ de 0.5, visto que o valor mï¿½x de cprrente ï¿½ de 1000 mA, no entanto como teremos que transmitir o valor de corrente/5 para evitar mais operacoes pelo uC jï¿½ multiplicamos o valor por 1/5=0.2, logo 0.5*0.2=0.1          
#define conv_temp  0.3                      //O coeficiente de conversï¿½o da temperatura ï¿½ de 0.1, visto que o valor mï¿½x de temperatura assumido ï¿½ de 100ï¿½C, no entanto como teremos que transmitir o valor de temperatura*3 para evitar mais operacoes pelo uC jï¿½ multiplicamos aqui logo 0.1*3=0.3
#define conv_second  0.0000005
#define distance_1_pulse_mult2  1.5        // 15 mm/20 pulsos = 0.75 mm, no entanto usaremos: 0.75*2 = 1.5 para evitar mais uma operaï¿½ï¿½o a fim de cumprir o preparo de envio
#define distance_1_pulse_mult5  3.75        // 15 mm/20 pulsos = 0.75 mm,no entanto usaremos: 0.75*5 = 3.75 para evitar mais uma operaï¿½ï¿½o a fim de cumprir o preparo de envio

//DECLARAï¿½ï¿½O DOS PROTï¿½TIPOS DAS FUNï¿½ï¿½ES: 
void comunicacao ();
void controle();
void movimento();


//DECLARAï¿½ï¿½O DAS VARIï¿½VEIS:
int and_dst = 0;                    //Variï¿½vel para armazenas o andar de destino
int pulses = 0;
int sentido = 0;
float I_m = 0;
float temp_mt = 0;
int aux_tempo = 0;
int aux_tempo_d2s = 0;
int estado = 0;
int acabou_delay =0;

//------------------------------------------------------------------------------

uint8_t state_motor = 0;
uint8_t and_ating = 0;
uint16_t ccp_value = 0;
uint8_t byte[5];

//FUNï¿½ï¿½ES DE INTERRUPï¿½ï¿½O

//INTERRUPï¿½ï¿½O PARA ENVIO DOS DADOS A CADA 100mS (5 BYTES)
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

//INTERRUPï¿½ï¿½O PARA ENVIO DOS DADOS A CADA 100mS (5 BYTES)
void delay_2s()
	{
        aux_tempo_d2s++;
        if(aux_tempo_d2s == 280)
        {
            acabou_delay=1;
            aux_tempo_d2s =0;
        }
	}

void sensor1()                          //trata interrupção do primeiro sensor
    {
        and_ating=0;                    //atualiza variavel usada no gerenciamento (de 1 a 4)
    }

void sensor2()                          //trata interrupção do segundo sensor
    {
        and_ating=1;                    //atualiza variavel usada no gerenciamento (de 1 a 4)
    }

void sensor3()                          //trata interrupção do terceiro sensor
    {
        and_ating=2;                    //atualiza variavel usada no gerenciamento (de 1 a 4)
    }

void sensor4()                          //trata interrupção do quarto sensor
    {
        and_ating=3;                    //atualiza variavel usada no gerenciamento (de 1 a 4)
    }

void get_pulse(uint16_t capturedValue)
    {
        TMR1_WriteTimer(0);
        ccp_value = capturedValue;
        if(sentido == 1){
            pulses++;
        }else if(sentido == 0 && pulses >= 0){
            pulses--;
        }
    }

// APLICAï¿½ï¿½O PROPRIAMENTE DITA
void main(void)
{
    
  //FUNï¿½ï¿½ES DE INICIALIZAï¿½ï¿½O
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
   //PASSAGEM DE PARï¿½METROS PARA FUNï¿½ï¿½ES DE INTERRUPï¿½ï¿½O 
    TMR4_SetInterruptHandler(send_data);
    TMR6_SetInterruptHandler(delay_2s);
    IOCBF0_SetInterruptHandler(sensor1);
    IOCBF3_SetInterruptHandler(sensor2);
    IOCBF4_SetInterruptHandler(sensor3);
    IOCBF5_SetInterruptHandler(sensor4);
    CCP4_SetCallBack(get_pulse);
    PIE3bits.TMR6IE=0; //desativa timer 6 visto que só é usado no delay
    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();
    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    // LOOP PRINCIPAL
    while (1)
    {
       comunicacao ();    // Chama a funï¿½ï¿½o responsï¿½vel por tratar a comunicaï¿½ï¿½o
       controle();   // Chama a funï¿½ï¿½o responsï¿½vel pelo controle do motor
    }
}

void comunicacao ()
{
 //   uint8_t byte_0,byte_1,byte_2,byte_3,byte_4;
 //   uint8_t byte[5];
    uint8_t aux;// and_dst,and_ating;       
    float speed = distance_1_pulse_mult5/(((float)ccp_value)*conv_second);
    float position = distance_1_pulse_mult2*pulses;
    if(EUSART_is_rx_ready())                                //lê novas solicitações             
        {
            aux= EUSART_Read();
            and_dst = (int)(aux & 0x02);                    //Pega os bits 0,1 e os transforma em inteiros 
        }

// OBS PARA LUCAS: TALVEZ Nï¿½O SEJA NECESSï¿½RIO O DESLOCAMENTO >>1 ANALISAR ROTEIRO!!!
//VERIFICAR SE O VETOR FUNCIONA! SE Nï¿½O FUNCIONAR MUDE A Lï¿½GICA DO ENVIO E DESCOMENTE ABAIXO
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

void controle()                                 //rotina responsável por controlar o motor
{
    I_m = conv_I*ADC_GetConversion(0);          // Realiza a leitura da corrente, converte para mA usando *conv_I, jï¿½ esta preparado para envio, veja as definiï¿½ï¿½es! (Resoluï¿½ï¿½o da mediï¿½ï¿½o 0.5 mA )
    temp_mt = conv_temp*ADC_GetConversion(1);   // Realiza a leitura da temperatura, converte para C usando *conv_temp, jï¿½ esta preparado para envio, veja as definiï¿½ï¿½es! (Resoluï¿½ï¿½o da mediï¿½ï¿½o 0.1 ï¿½C))
    int count = 0;                              // Contador responsï¿½vel pelos loops
    int tempo_espera = 400000000;//4000;

    
    Dir_SetHigh();
    estado = 1;
    movimento();
   // LedG_SetHigh();
  //  LedR_SetLow();


    Dir_SetLow();
    estado = 2;
    movimento();
    and_dst=0;
  //  LedG_SetHigh();
  //  LedR_SetLow();
    
}

void movimento()
{
    
    int a = 26;                                 // Valor da aceleraï¿½ï¿½o boa para o motor (cï¿½lculado por torricelli)
    int max_dutyValue = 612;                    // Valor mï¿½ximo aceito pelo PWM (v = 20 mm/s))
    int min_dutyValue = 153;                    // Valor do PWM para v = 5 mm/s
    int destiny = and_dst*60;                 // Variï¿½vel para receber o andar de destino [OBS: Validar com o Matheus essa variï¿½vel]
    int route = destiny - 40;                   // Controla o nï¿½mero de pulsos com v = 20 mm/s
    int dutyValue = 0;                          // Inicia dutyValue em zero
    int count=0;
    int ilum = 1023;
    
   // LedR_SetHigh();
  //  LedG_SetLow();
    
    for(count = 0; count < 20; count++){        // Loop responsï¿½vel pela aceleraï¿½ï¿½o do motor
        dutyValue+=a;                           // Adiciona valor de aceleraï¿½ï¿½o no dutyValue

        EPWM1_LoadDutyValue(ilum=-2);
        if(dutyValue > max_dutyValue){          // Comparaï¿½ï¿½o se o valor de dutyValue nï¿½o ultrapassa o mï¿½ximo permitido
            PWM3_LoadDutyValue(max_dutyValue);  // Caso sim, esse valor ï¿½ substituï¿½do pelo valor mï¿½ximo
        }else{                                  // Else
            PWM3_LoadDutyValue(dutyValue);      // Envia dutyValue com o valor acrescido de aceleraï¿½ï¿½o do motor
        }
        
    }
    count = 0;                                  // Reinicia o contador para o prï¿½ximo loop
    for(count = 0; count < route ; count++){    // Loop responsï¿½vel pela velocidade mï¿½xima constante
        PWM3_LoadDutyValue(max_dutyValue);      // Envia max_dutyValue para o PWM
    }
    
    count = 0;                                  // Reinicia o contador para o prï¿½ximo loop
    for(count = 0; count < 20; count++){        // Loop responsï¿½vel pela desaceleraï¿½ï¿½o do motor
        dutyValue-=a;                           // Subtrai valor da aceleraï¿½ï¿½o do dutyValue
        EPWM1_LoadDutyValue(ilum=-2);
        if(dutyValue < min_dutyValue){          // Comparação se o valor de dutyValue não ultrapassa o mínimo permitido na desaceleraï¿½ï¿½o
            PWM3_LoadDutyValue(min_dutyValue);  // Caso sim, esse valor ï¿½ substituï¿½do pelo valor mï¿½nimo para v = 5 mm/s
        }else{                                  // Else
        PWM3_LoadDutyValue(dutyValue);          // Envia dutyValue com o valor decrescido de aceleraï¿½ï¿½o do motor
        }
    }
    
    estado = 0;
    //Aguarda 2 segundos após parada
    PIE3bits.TMR6IE=1;    // Ativa interrupção do timer utilizado para contagem da espera
    acabou_delay=0;
    TMR6_WriteTimer(0);
    while (1)
        {
        if(acabou_delay == 1)
            {
                acabou_delay=0;
                break;
            }
        }
    PIE3bits.TMR6IE=0;   // Desativa interrupção do timer utilizado para contagem da espera
}