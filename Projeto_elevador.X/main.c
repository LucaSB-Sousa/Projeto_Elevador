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
int and_dst = 0;                    //Vari�vel para armazenas o andar de destino
int and_origem = 0;                 //Vari�vel para armazenas o andar de origem
int pulses = 0;
int sentido = 0;
float I_m = 0;
float temp_mt = 0;
int aux_tempo = 0;
uint8_t state_motor = 0;
uint8_t and_ating = 0;
uint16_t ccp_value = 0;
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
        if(sentido == 1){
            pulses++;
        }else if(sentido == 0 && pulses >= 0){
            pulses--;
        }
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
       comunicacao ();    // Chama a fun��o respons�vel por tratar a comunica��o
       controle();        // Chama a fun��o respons�vel por tratar o controle do motor
       gerenciamento();   // Chama a fun��o respons�vel por gerenciar as solicita��es
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
    I_m = conv_I*ADC_GetConversion(0);          // Realiza a leitura da corrente, converte para mA usando *conv_I, j� esta preparado para envio, veja as defini��es! (Resolu��o da medi��o 0.5 mA )
    temp_mt = conv_temp*ADC_GetConversion(1);   // Realiza a leitura da temperatura, converte para C usando *conv_temp, j� esta preparado para envio, veja as defini��es! (Resolu��o da medi��o 0.1 �C))
    int count = 0;                              // Contador respons�vel pelos loops
    int a = 52;                                 // Valor da acelera��o boa para o motor (c�lculado por torricelli)
    int max_dutyValue = 1023;                   // Valor m�ximo aceito pelo PWM (v = 20 mm/s))
    int min_dutyValue = 256;                    // Valor do PWM para v = 5 mm/s
    int destiny = 240;                          // Vari�vel para receber o andar de destino [OBS: Validar com o Matheus essa vari�vel]
    int route = destiny - 40;                   // Controla o n�mero de pulsos com v = 20 mm/s
    int dutyValue = 0;                          // Inicia dutyValue em zero
    //int pulse = 0;                            // Inicia n�mero de pulsos em zero

    for(count = 0; count < 20; count++){        // Loop respons�vel pela acelera��o do motor
        //pulse+=1;                               // Contador para n�mero de pulsos
        dutyValue+=a;                           // Adiciona valor de acelera��o no dutyValue
        if(dutyValue > max_dutyValue){          // Compara��o se o valor de dutyValue n�o ultrapassa o m�ximo permitido
            PWM3_LoadDutyValue(max_dutyValue);  // Caso sim, esse valor � substitu�do pelo valor m�ximo
        }else{                                  // Else
            PWM3_LoadDutyValue(dutyValue);      // Envia dutyValue com o valor acrescido de acelera��o do motor
        }
        
    }
    count = 0;                                  // Reinicia o contador para o pr�ximo loop
    for(count = 0; count < route ; count++){    // Loop respons�vel pela velocidade m�xima constante
        //pulse+=1;                               // Contador de pulsos
        PWM3_LoadDutyValue(max_dutyValue);      // Envia max_dutyValue para o PWM
    }
    
    count = 0;                                  // Reinicia o contador para o pr�ximo loop
    for(count = 0; count < 20; count++){        // Loop respons�vel pela desacelera��o do motor
        //pulse+=1;                               // Contador de pulsos
        dutyValue-=a;                           // Subtrai valor da acelera��o do dutyValue
        if(dutyValue < min_dutyValue){          // Compara��o se o valor de dutyValue n�o ultrapassa o m�nimo permitido na desacelera��o
            PWM3_LoadDutyValue(min_dutyValue);  // Caso sim, esse valor � substitu�do pelo valor m�nimo para v = 5 mm/s
        }else{                                  // Else
        PWM3_LoadDutyValue(dutyValue);          // Envia dutyValue com o valor decrescido de acelera��o do motor
        }
    }
    PWM3_LoadDutyValue(0);                      // Para o motor

}

void gerenciamento()
{
    
    int fila_up[10];
    int fila_down[10];
    int and_sol[10];
    int fila_origem[10];
    
    //Confere se houve uma nova solicita��o
    if(and_sol[0]!=and_dst || fila_origem[0]!=and_origem)
        {
            if(and_ating<and_sol[0])
                { //subir 
                    sentido = 1; 
                    if(and_origem>and_ating && and_dst>and_origem)
                        {
                            fila_up = fila_inicio(and_origem,fila_up)
                            fila_up = fila_inicio(and_dst,fila_up);
                        }
                }
            if(and_ating>and_sol[0])
                { //descer 
                    sentido = 0; 
                    if(and_origem<and_ating && and_dst<and_origem)
                        {
                            fila_down = fila_inicio(and_origem,fila_down);
                            fila_down = fila_inicio(and_dst,fila_down);
                        }
                }

            //Organizar a fila

            //Parar o elevador no primeiro valor da fila

            if(sentido==1 && and_ating==fila_up)
                {
                    //parar no and_ating
                }
            if(sentido==0 && and_ating==fila_down)
                {
                    //parar no and_ating
                }

//            if(fila_up[0]==max)
//                {
//                    fila_up = [0,0,0,0,0,0,0,0,0,0];
//                    sentido = 0;
//                }
//            if(fila_down[0]==min)
//                {
//                    fila_down = [0,0,0,0,0,0,0,0,0,0];
//                }

            //Retroceder fila
            //Retira o andar de destino solicitado na fila
            fila_up = fila_retr(fila_up);
            //Retira o andar de origem solicitado na fila
            fila_down = fila_retr(fila_down);  
        }
    //Coloca o andar de destino solicitado na fila
    //and_sol = fila_inicio(and_dst,and_sol);
    //Coloca o andar de origem solicitado na fila
    //fila_origem = fila_inicio(and_origem,fila_origem);
    
    
        
        for(int j=0;j<=9;j++){ 
                if(and_ating<and_sol[0]){ //subir 
                    sentido = 1; 
                    comunicacao();
                    if(and_origem>and_ating && and_dst>and_ating){
                    
                    }
                }
            }
        
        for(int i=0;i<=10;i++){
            if(and_origem_up==and_atingido){
                //parar elevador
                //colocar andar solicitado nessa origem na fila de subida
        }  
            
    }
        // colocar em ordem decrescente a fila [2,3,4]
    }
    
    if(and_ating>and_dst){ //descer 
        sentido = 0;
        for(int j=0;j<=9;j++){ 
            if(j==and_sol[j]){
                break;
            }
            if(and_sol[j]<and_ating){
                fila_down[j] = and_sol[j];
            }
        }
        
        for(int i=0;i<=10;i++){
            if(and_origem_down==and_atingido){
                //parar elevador
                //colocar andar solicitado nessa origemna fila de descida
        }
            
    }
        
    }
    
    for(int i=0;i<=10;i++){
        if(and_origem_down==and_atingido){
            //parar elevador
        }
    }
    
    switch(and_final){
        case 1:
            if(and_ating==1){
                //parar
            }
        case 2:
            if(and_ating==1){
                //parar
            }
        case 3:
            if(and_ating==1){
                //parar
            }
        case 4:
            if(and_ating==1){
                //parar
            }
        default:
            if(and_ating==1){
                //parar
            }
            
    }
    
    
    
}

int fila_inicio(int andar_dst, int and_s[10]){
    for(int i=0;i<=9;i++){
        if(i==0){
            and_s[1] = and_s[0];
            and_s[0] = andar_dst
        }
        if(i<0){
            and_s[i+1] = and_s[i];
        }
        if(i==9){
            break;
        }
    }
    return and_s;
}
 
int fila_retr(int and_s[10]){
    for(int i=0;i<=9;i++){
        if(i==0){
            and_s[0] = and_s[1];
        }
        if(i<0){
            and_s[i] = and_s[i+1];
        }
        if(i==9){
            and_s[9] = 0;
        }
    }
    return and_s;
}