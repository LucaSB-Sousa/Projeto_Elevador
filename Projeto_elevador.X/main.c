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
void gerenciamento();
void col_up();
void col_down();
void subir();
void descer();
void parar();
void max_down();
void min_up();
void update();


//DECLARAï¿½ï¿½O DAS VARIï¿½VEIS:
int and_dst = 0;                    //Variï¿½vel para armazenas o andar de destino
int and_origem = 0;                 //Variï¿½vel para armazenas o andar de origem
int pulses = 0;
int sentido = 0;
float I_m = 0;
float temp_mt = 0;
int aux_tempo = 0;
int sentido2;
int andar;
int distancia;

//Variaveis - gerenciamento
int fila_up[10] = {0,0,0,0,0,0,0,0,0,0};
int fila_down[10] = {0,0,0,0,0,0,0,0,0,0};
int origem_down[10] = {0,0,0,0,0,0,0,0,0,0};
int origem_up[10] = {0,0,0,0,0,0,0,0,0,0};
int max_fila_up,max_origem_down,min_origem_up,min_fila_down;
int out_value=0;
int and_ating2;
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

void sensor1()
    {
        and_ating=1;
        and_ating2=0;
    }

void sensor2()
    {
        and_ating=2;
        and_ating2=1;
    }

void sensor3()
    {
        and_ating=3;
        and_ating2=2;
    }

void sensor4()
    {
        and_ating=4;
        and_ating2=3;
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
       comunicacao ();    // Chama a funï¿½ï¿½o responsï¿½vel por tratar a comunicaï¿½ï¿½o
       controle();        // Chama a funï¿½ï¿½o responsï¿½vel por tratar o controle do motor
       gerenciamento();   // Chama a funï¿½ï¿½o responsï¿½vel por gerenciar as solicitaï¿½ï¿½es
    }
}

void comunicacao ()
{
 //   uint8_t byte_0,byte_1,byte_2,byte_3,byte_4;
 //   uint8_t byte[5];
//    uint8_t aux;// and_dst,and_ating;       
    float speed = distance_1_pulse_mult5/(((float)ccp_value)*conv_second);
    float position = distance_1_pulse_mult2*pulses;
//    if(EUSART_is_rx_ready())
//        {
//            aux= EUSART_Read();
//            and_origem = (int)((aux & 0x0C)>>2);    //Pega os bits 2,3 e os transforma em inteiros
//            and_dst = (int)(aux & 0x02);            //Pega os bits 0,1 e os transforma em inteiros 
//        }
// OBS PARA LUCAS: TALVEZ Nï¿½O SEJA NECESSï¿½RIO O DESLOCAMENTO >>1 ANALISAR ROTEIRO!!!
//VERIFICAR SE O VETOR FUNCIONA! SE Nï¿½O FUNCIONAR MUDE A Lï¿½GICA DO ENVIO E DESCOMENTE ABAIXO
//    byte_0 = ((state_motor<<4)& 0x20)|(and_ating & 0x02);
//    byte_1 = 0x80 |((((uint8_t)position)>>1)& 0x7F);  
//    byte_2 = 0x80 |((((uint8_t)speed)>>1)& 0x7F);
//    byte_3 = 0x80 |((((uint8_t)I_m)>>1)& 0x7F);
//    byte_4 = 0x80 |((((uint8_t)temp_mt)>>1)& 0x7F);
    
    byte[0] = ((state_motor<<4)& 0x20)|(and_ating2 & 0x02);
    byte[1] = 0x80 |((((uint8_t)position)>>1)& 0x7F);  
    byte[2] = 0x80 |((((uint8_t)speed)>>1)& 0x7F);
    byte[3] = 0x80 |((((uint8_t)I_m)>>1)& 0x7F);
    byte[4] = 0x80 |((((uint8_t)temp_mt)>>1)& 0x7F);

}

void controle()
{
    I_m = conv_I*ADC_GetConversion(0);          // Realiza a leitura da corrente, converte para mA usando *conv_I, jï¿½ esta preparado para envio, veja as definiï¿½ï¿½es! (Resoluï¿½ï¿½o da mediï¿½ï¿½o 0.5 mA )
    temp_mt = conv_temp*ADC_GetConversion(1);   // Realiza a leitura da temperatura, converte para C usando *conv_temp, jï¿½ esta preparado para envio, veja as definiï¿½ï¿½es! (Resoluï¿½ï¿½o da mediï¿½ï¿½o 0.1 ï¿½C))
    int count = 0;                              // Contador responsï¿½vel pelos loops
    int a = 52;                                 // Valor da aceleraï¿½ï¿½o boa para o motor (cï¿½lculado por torricelli)
    int max_dutyValue = 1023;                   // Valor mï¿½ximo aceito pelo PWM (v = 20 mm/s))
    int min_dutyValue = 256;                    // Valor do PWM para v = 5 mm/s
    int destiny = distancia*60;                          // Variï¿½vel para receber o andar de destino [OBS: Validar com o Matheus essa variï¿½vel]
    int route = destiny - 40;                   // Controla o nï¿½mero de pulsos com v = 20 mm/s
    int dutyValue = 0;                          // Inicia dutyValue em zero
    //int pulse = 0;                            // Inicia nï¿½mero de pulsos em zero

    for(count = 0; count < 20; count++){        // Loop responsï¿½vel pela aceleraï¿½ï¿½o do motor
        //pulse+=1;                               // Contador para nï¿½mero de pulsos
        dutyValue+=a;                           // Adiciona valor de aceleraï¿½ï¿½o no dutyValue
        if(dutyValue > max_dutyValue){          // Comparaï¿½ï¿½o se o valor de dutyValue nï¿½o ultrapassa o mï¿½ximo permitido
            PWM3_LoadDutyValue(max_dutyValue);  // Caso sim, esse valor ï¿½ substituï¿½do pelo valor mï¿½ximo
        }else{                                  // Else
            PWM3_LoadDutyValue(dutyValue);      // Envia dutyValue com o valor acrescido de aceleraï¿½ï¿½o do motor
        }
        
    }
    count = 0;                                  // Reinicia o contador para o prï¿½ximo loop
    for(count = 0; count < route ; count++){    // Loop responsï¿½vel pela velocidade mï¿½xima constante
        //pulse+=1;                               // Contador de pulsos
        PWM3_LoadDutyValue(max_dutyValue);      // Envia max_dutyValue para o PWM
    }
    
    count = 0;                                  // Reinicia o contador para o prï¿½ximo loop
    for(count = 0; count < 20; count++){        // Loop responsï¿½vel pela desaceleraï¿½ï¿½o do motor
        //pulse+=1;                               // Contador de pulsos
        dutyValue-=a;                           // Subtrai valor da aceleraï¿½ï¿½o do dutyValue
        if(dutyValue < min_dutyValue){          // Comparaï¿½ï¿½o se o valor de dutyValue nï¿½o ultrapassa o mï¿½nimo permitido na desaceleraï¿½ï¿½o
            PWM3_LoadDutyValue(min_dutyValue);  // Caso sim, esse valor ï¿½ substituï¿½do pelo valor mï¿½nimo para v = 5 mm/s
        }else{                                  // Else
        PWM3_LoadDutyValue(dutyValue);          // Envia dutyValue com o valor decrescido de aceleraï¿½ï¿½o do motor
        }
    }
    update();

    PWM3_LoadDutyValue(0);                      // Para o motor

}

void gerenciamento() {
    
    update();
//
//
//  //Ler porta serial
//
//                                        if(and_dst>and_origem) /*se a solicitação for de subida*/
//                                        {
//                                          int i=0;
//                                          for(i=8;i>=0;i--) /*organiza a fila de subida*/
//                                          {
//                                              if(i==0){
//                                                  fila_up[1] = fila_up[0];
//                                                  fila_up[0] = and_dst;
//
//                                              }
//                                              if(i>0){
//                                                  fila_up[i+1] = fila_up[i];
//                                              }
//                                          }
//
//                                          for(i=8;i>=0;i--) /*organiza a fila de origem de subida*/
//                                          {
//                                              if(i==0){
//                                                  origem_up[1] = origem_up[0];
//                                                  origem_up[0] = and_origem;
//
//                                              }
//                                              if(i>0){
//                                                  origem_up[i+1] = origem_up[i];
//                                              }
//                                          }
//                                        }
//
//                                        int i;
//                                        max_fila_up = fila_up[0];
//                                        for(i=1;i<=9;i++)   /*define o andar máximo da fila de subida*/
//                                        {
//                                                if(fila_up[i]>max_fila_up)
//                                                {
//                                                    max_fila_up = fila_up[i];
//                                                }
//                                        }
//                                        if(max_fila_up==0)
//                                        {
//                                            max_fila_up=1;
//                                        }
//
//                                        if(and_dst<and_origem)  /*se a solicitação for de descida*/
//                                        {
//                                            int i=0;
//                                          for(i=8;i>=0;i--) /*organiza a fila de descida*/
//                                          {
//                                              if(i==0){
//                                                  fila_down[1] = fila_down[0];
//                                                  fila_down[0] = and_dst;
//
//                                              }
//                                              if(i>0){
//                                                  fila_down[i+1] = fila_down[i];
//                                              }
//                                          }
//
//                                          for(i=8;i>=0;i--) /*organiza a fila de origem para descida*/
//                                          {
//                                              if(i==0){
//                                                  origem_down[1] = origem_down[0];
//                                                  origem_down[0] = and_origem;
//                                              }
//                                              if(i>0){
//                                                  origem_down[i+1] = origem_down[i];
//                                              }
//                                          }
//                                        }
//
//
//                                        min_fila_down = 4;
//                                        for(i=0;i<=9;i++)   /*define o menor andar da fila de descida*/
//                                        {
//                                            if(fila_down[i]<min_fila_down&&fila_down[i]>0)
//                                            {
//                                                min_fila_down = fila_down[i];
//                                            }
//                                        }
//
//
//
           /*se houver solicitação de subida muda o sentido para '1': subida*/
          if(fila_up[0]!=out_value||fila_up[1]!=out_value||fila_up[2]!=out_value||fila_up[3]!=out_value||fila_up[4]!=out_value||fila_up[5]!=out_value||fila_up[6]!=out_value||fila_up[7]!=out_value||fila_up[8]!=out_value||fila_up[9]!=out_value)
          {
            sentido2=1;
            col_up();
          }

          /*se houver solicitação de descida muda o sentido para '0': descida*/
          else if(origem_down[0]!=out_value||origem_down[1]!=out_value||origem_down[2]!=out_value||origem_down[3]!=out_value||origem_down[4]!=out_value||origem_down[5]!=out_value||origem_down[6]!=out_value||origem_down[7]!=out_value||origem_down[8]!=out_value||origem_down[9]!=out_value)
          {
            sentido2=0;
            col_down();
          }

          /*se não houver nova solicitação muda o sentido para '2': parado*/
          else
          {
            sentido2=2;
            distancia=abs(and_ating-1);
             if(and_ating > 1)
                {
                 sentido = 0;
                  LATAbits.LATA2 = 0;
                }

          }

}
//
void col_up() /*rotina de subida coletiva*/
    {
        while(and_ating!=min_origem_up)
        {   
            distancia=abs(and_ating-min_origem_up);
            if(and_ating>min_origem_up)  /*se estiver acima do andar maximo de origem o elevador desce*/
            {
                LATAbits.LATA2 = 0;
            }
            if(and_ating<min_origem_up)  /*se estiver abaixo do andar maximo de origem o elevador sobe*/
            {
                //Dir_SetHigh();
                LATAbits.LATA2 = 1;
            }  
        }
        LATAbits.LATA2 = 1;
        do
        {   
            if(and_ating==min_origem_up) /*continua enquanto não atingir o maior andar solicitado*/
            {
                min_up();
                distancia=abs(and_ating-min_origem_up);
            }
        }while (and_ating!=max_fila_up);
    }

void col_down()  /*rotina de descida coletiva*/
    {
        while(and_ating!=max_origem_down)  /*continua enquanto não atingir o maior andar de origem para descer uma vez só*/
        {   
            distancia = abs(and_ating-max_origem_down);
            if(and_ating>max_origem_down)  /*se estiver acima do andar maximo de origem o elevador desce*/
            {
                LATAbits.LATA2 = 0;
            }
            if(and_ating<max_origem_down)  /*se estiver abaixo do andar maximo de origem o elevador sobe*/
            {
                LATAbits.LATA2 = 1;
            }
        }
        LATAbits.LATA2 = 0;
        do
        {   
            if(and_ating==max_origem_down)  /*continua enquanto não atingir o menor andar solicitado*/
            {
                max_down();
                distancia=abs(and_ating-max_origem_down);           
            }
        }while(and_ating!=min_fila_down);
    }

//void subir(int andar, int distancia)
//{
//    Dir_SetHigh();
//}
//
//void descer(int andar, int distancia)
//{
//    Dir_SetLow();
//}

void min_up()
{
    
    min_origem_up = 4;
    int i;
    for(i=0;i<=9;i++)   /*define o menor andar de origem para subir*/
    {
        if(origem_up[i]<min_origem_up&&origem_up[i]>0)
        {
            min_origem_up = origem_up[i];
        }
        if(fila_up[i]<min_origem_up&&fila_up[i]>0)
        {
            min_origem_up = fila_up[i];
        }
    }
    
    for(i=0;i<=9;i++)   /*define o menor andar de origem para subir*/
    {
        if(origem_up[i]==min_origem_up)
        {
            origem_up[i]=out_value;
        }
        if(fila_up[i]==min_origem_up)
        {
            fila_up[i]=out_value;
        }
    }
    
  
}

void max_down(){
    
    max_origem_down = origem_down[0];
    int i;
    for(i=1;i<=9;i++)   /*define maior andar solicitando descida*/
    {
        if(origem_down[i]>max_origem_down)
        {
            max_origem_down = origem_down[i];
        }
        if(fila_down[i]>max_origem_down)
        {
            max_origem_down = fila_down[i];
        }
    }
    
    for(i=0;i<=9;i++)   /*define o menor andar de origem para subir*/
    {
        if(origem_down[i]==max_origem_down)
        {
            origem_down[i]=out_value;
        }
        if(fila_up[i]==max_origem_down)
        {
            fila_down[i]=out_value;
        }
    }
    
    if(max_origem_down==0)
    {
        max_origem_down=1;
    }
    
    
}

void update()
{
    uint8_t aux;// and_dst,and_ating;
    if(EUSART_is_rx_ready())
        {
            aux= EUSART_Read();
            and_origem = (int)((aux & 0x0C)>>2);    //Pega os bits 2,3 e os transforma em inteiros
            and_dst = (int)(aux & 0x02);            //Pega os bits 0,1 e os transforma em inteiros 
        }
    
    if(and_dst>and_origem) /*se a solicitação for de subida*/
    {
      int i=0;
      for(i=8;i>=0;i--) /*organiza a fila de subida*/
      {
          if(i==0){
              fila_up[1] = fila_up[0];
              fila_up[0] = and_dst;

          }
          if(i>0){
              fila_up[i+1] = fila_up[i];
          }
      }

      for(i=8;i>=0;i--) /*organiza a fila de origem de subida*/
      {
          if(i==0){
              origem_up[1] = origem_up[0];
              origem_up[0] = and_origem;

          }
          if(i>0){
              origem_up[i+1] = origem_up[i];
          }
      }
    }

    int i;
    max_fila_up = fila_up[0];
    for(i=1;i<=9;i++)   /*define o andar máximo da fila de subida*/
    {
            if(fila_up[i]>max_fila_up)
            {
                max_fila_up = fila_up[i];
            }
    }
    if(max_fila_up==0)
    {
        max_fila_up=1;
    }

    if(and_dst<and_origem)  /*se a solicitação for de descida*/
    {
        int i=0;
      for(i=8;i>=0;i--) /*organiza a fila de descida*/
      {
          if(i==0){
              fila_down[1] = fila_down[0];
              fila_down[0] = and_dst;

          }
          if(i>0){
              fila_down[i+1] = fila_down[i];
          }
      }

      for(i=8;i>=0;i--) /*organiza a fila de origem para descida*/
      {
          if(i==0){
              origem_down[1] = origem_down[0];
              origem_down[0] = and_origem;
          }
          if(i>0){
              origem_down[i+1] = origem_down[i];
          }
      }
    }


    min_fila_down = 4;
    for(i=0;i<=9;i++)   /*define o menor andar da fila de descida*/
    {
        if(fila_down[i]<min_fila_down&&fila_down[i]>0)
        {
            min_fila_down = fila_down[i];
        }
    }
}