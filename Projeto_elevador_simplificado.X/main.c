/******************************************************************************
 ******************************************************************************
 ***    UNIVERSIDADE DE BRASÍLIA - UNB                                      ***
 ***    DISCIPLINA: FGA0096 - ELETRTÔNICA EMBARCADA        TURMA: A         ***
 ***    PROFESSOR: Guillermo Alvarez Bestard, Dr. Eng. Mecatrônicanica      ***
 ***    ALUNO: Lucas dos Santos Barros de Sousa   MATRÍCULA:180022555       ***
 ***    ALUNO: Matheus Oliveira Dias              MATRÍCULA:18/0025104      ***
 ***    ALUNO: Victor Hugo Ciurlini               MATRÍCULA:11/0021223      ***
 ***               TRABALHO FINAL ELETRÔNICA EMBARCADA                      ***
 ******************************************************************************
 ******************************************************************************/




//DECLARAÇÃO DE BIBLIOTECAS:
#include "mcc_generated_files/mcc.h"

//DEFINIÇÕES
#define conv_I  0.449                         // coeficiente de convessão para corrente.
#define conv_temp  0.11                       //O coeficiente de converssão da temperatura 0.11 graus C p/ mV 
#define conv_second  0.000004                //Tempo de 1 incremento do Timer 4
#define distance_1_pulse  0.837              // 180 mm/215 pulsos = 0.837 mm

//DECLARAÇÃO DOS PROTOTIPOS DAS FUN~ÇÕES: 
void comunicacao ();
void controle();
void movimento();
void update();


//DECLARAÇÃO DAS VARIÁVEIS:
int and_dst = 0;                    
int pulses = 0;
int sentido = 0;
float I_m = 0;
float temp_mt = 0;
int aux_tempo = 0;
int estado = 0;
int destiny;
int and_atual = 0;
int luminosidade = 0;
int luminos;
int distancia;
float speed = 0.0;
float position = 0.0;

//------------------------------------------------------------------------------

uint8_t state_motor = 0;
uint8_t and_ating = 0;
uint16_t ccp_value = 0;
uint8_t byte[5];
uint8_t IM_B = 0;
uint8_t TM_B = 0;
//uint8_t aux = 0;

//FUNCÕES DE INTERRUPÇÃO

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

void sensor1()                          //trata interrupção do primeiro sensor
    {
        and_ating=0;                    //atualiza variável andar atual e byte do andar atual para comunicação(de 0 a 3)
        and_atual = 0;
        pulses = 0;
    }

void sensor2()                          //trata interrupção do segundo sensor
    {
        and_ating=1;                   //atualiza variável andar atual e byte do andar atual para comunicação(de 0 a 3)
        and_atual = 1;
    }

void sensor3()                          //trata interrupção do terceiro sensor
    {
        and_ating=2;                    //atualiza variável andar atual e byte do andar atual para comunicação (de 0 a 3)
        and_atual = 2;
    }

void sensor4()                          //trata interrupção do quarto sensor
    {
        and_ating=3;                    //atualiza variável andar atual e byte do andar atual para comunicação (de 0 a 3)
        and_atual = 3;                  
    }
//Função para captura de tempo entre pulsos (1 pulso borda de subida), além da utilização para cálculo de posição e controle dos leds)
void get_pulse(uint16_t capturedValue)
    {
        TMR1_WriteTimer(0);
        ccp_value = capturedValue;
     //   Incrementa pulsos se estiver subindo
        if(Dir_GetValue()==1)
            {
                pulses++;
            }
          //   Decrementa pulsos se estiver descendo
        else if(Dir_GetValue() == 0 && pulses >= 0)
            {
                pulses--;
            }
        //Controle dos leds em função da distância
        if (luminos == 1)
            {
                luminosidade-=(1023/destiny);
                if(luminosidade< 0)
                {luminosidade = 15;}
                EPWM1_LoadDutyValue(luminosidade);
            
            }
         
    }

// APLICAÇÃO PROPRIAMENTE DITA
void main(void)
{
    
  //FUNÇÕES DE INICIALIZAÇÃO
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
   //PASSAGEM DE PARAMETROS PARA FUNÇÕES DE INTERRUPÇÃO 
    TMR4_SetInterruptHandler(send_data);
    IOCBF0_SetInterruptHandler(sensor1);
    IOCBF3_SetInterruptHandler(sensor2);
    IOCBF4_SetInterruptHandler(sensor3);
    IOCBF5_SetInterruptHandler(sensor4);
    CCP4_SetCallBack(get_pulse);

    luminos=1;
    luminosidade = 1023;
    //VERIFICAÇÃO DE POSIÇÃO, SE ESTIVER EM UM ANDAR DIFERENTE DE 1 RETORNA A POSIÇÃO INICIAL
    while(S1_GetValue()!=0)
        {
            Dir_SetLow();
            PWM3_LoadDutyValue(512);
        }
    luminos=0;
    EPWM1_LoadDutyValue(0);
    PWM3_LoadDutyValue(0);
    
    // LOOP PRINCIPAL
    while (1)
    {
        //LEITURA E TRATAMENTO
        I_m = ADC_GetConversion(0);          // Realiza a leitura da corrente, converte para mA usando *conv_I, jï¿½ esta preparado para envio, veja as definiï¿½ï¿½es! (Resoluï¿½ï¿½o da mediï¿½ï¿½o 0.5 mA )
        IM_B = (uint8_t)((conv_I*I_m)/4);
        temp_mt = ADC_GetConversion(1);   // Realiza a leitura da temperatura, converte para C usando *conv_temp, jï¿½ esta preparado para envio, veja as definiï¿½ï¿½es! (Resoluï¿½ï¿½o da mediï¿½ï¿½o 0.1 ï¿½C))
        TM_B = (uint8_t)(conv_temp*temp_mt*2);
        
        //FUNÇÕES DE FLUXO
        comunicacao ();    // Chama a funï¿½ï¿½o responsï¿½vel por tratar a comunicaï¿½ï¿½o
        controle();   // Chama a funï¿½ï¿½o responsï¿½vel pelo controle do motor
    }
}

//REALIZA O RECEBIMENTO E TRATAMENTO DE FLUXO DE DADOS
void comunicacao ()
{
    uint8_t recebe = 0;
    if(EUSART_is_rx_ready())                                            
        {
            recebe= EUSART_Read();
            and_dst = (int)((recebe) & 0x03);
        }
    update();        
}

//REALIZA O CONTROLE DO FUNCIONAMENTO DAS SOLICITAÇÕES
void controle()                                 
{
    // Subida
    if(and_dst>and_atual)
        {
        // Se estiver descendo aguarda 500ms e muda de sentido
        if(Dir_GetValue()==0)
            {    
                PWM3_LoadDutyValue(0);
                __delay_ms(500);
            }
            Dir_SetHigh();
            estado = 1;
            movimento();
          } 
    // Descida
    else if(and_dst<and_atual)
        {
            // Se estiver subindo aguarda 500ms e muda de sentido
            if(Dir_GetValue()==1)
                {
                    PWM3_LoadDutyValue(0);
                    __delay_ms(500);
                }
                Dir_SetLow();
                estado = 2;
                movimento();
        }
    //Se estiver no andar de destino - andar de destino = 0
    if(and_dst==and_atual)
        {
            estado = 0;
            and_dst=0;
        }
}

//FUNÇÃO DE CONTROLE DA MOVIMENTAÇÃO DO MOTOR
void movimento()
{
    distancia = abs(and_atual-and_dst);         //Diferença de andar solicitado e atual
    int mean_dutyValue = 450;
    destiny = distancia*68;                     //Posição em pulsos do andar de destino                
    luminos=1;
    luminosidade = 1023;
   
    // Loop de controle do motor (ativa PWM enquanto não atingir andar de destino) 
    while(and_atual!=and_dst)
        {        
            PWM3_LoadDutyValue(mean_dutyValue);
            update();                           //Atualiza dados de saída
        }
    //Controle dos leds
    EPWM1_LoadDutyValue(0);
    luminos = 0;
    //Para elevador e aguarda 2 segundos
    PWM3_LoadDutyValue(0);
    estado = 0;
    __delay_ms(2000);
}

//FUNÇÃO DE PREPARO E TRATAMENTO DOS DADOS
void update(){
    I_m = ADC_GetConversion(0);          // Realiza a leitura da corrente, converte para mA usando *conv_I, jï¿½ esta preparado para envio, veja as definiï¿½ï¿½es! (Resoluï¿½ï¿½o da mediï¿½ï¿½o 0.5 mA )
    IM_B = (uint8_t)((conv_I*I_m)/4);
    temp_mt = (float)ADC_GetConversion(1);   // Realiza a leitura da temperatura, converte para C usando *conv_temp, jï¿½ esta preparado para envio, veja as definiï¿½ï¿½es! (Resoluï¿½ï¿½o da mediï¿½ï¿½o 0.1 ï¿½C))
    TM_B = (uint8_t)(conv_temp*temp_mt*2);
    speed = (distance_1_pulse/(((float)ccp_value)*conv_second))*4;
    if(estado==0)
    {
        speed = 0;
    }
    position = distance_1_pulse*pulses;
    
    //PREPARO DOS DADOS PARA TRANSMISSÃO
    byte[0] = ((state_motor<<4)& 0x30)|(and_ating & 0x03); 
    byte[1] = 0x80 |((((uint8_t)position)>>1)& 0x7F); 
    byte[2] = 0x80 |((((uint8_t)speed))& 0x7F); 
    byte[3] = 0x80 |(((IM_B))& 0x7F); 
    byte[4] = 0x80 |(((TM_B))& 0x7F); 
}