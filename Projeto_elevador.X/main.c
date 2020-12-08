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



#include "mcc_generated_files/mcc.h"

/*
                         Main application
 */



void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    while (1)
    {
        // Add your application code
    }
}
