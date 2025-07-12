//
// Arquivos de Inclus�o
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"

//
// Defini��es de Constantes
//
#define F_PWM                  10000.0f     // Frequ�ncia de chaveamento (Hz)
#define T_PWM                  (1.0f / F_PWM) // Per�odo de chaveamento (s)
#define DT_SIM                 0.000005f    // Passo de simula��o (5 �s)
#define N_STEPS_PER_CYCLE      (uint32_t)(T_PWM / DT_SIM) // Passos por ciclo PWM

// Par�metros do Conversor Buck
#define VIN                    12.0f       // Tens�o de entrada (V)
#define L                      0.001f      // Indut�ncia (H)
#define C                      0.00001f    // Capacit�ncia (F)
#define R_LOAD                 10.0f       // Carga resistiva (Ohm)

// Constantes auxiliares (evita divis�es repetidas no loop)
#define INV_L                  (DT_SIM / L)
#define INV_C                  (DT_SIM / C)
#define INV_R_LOAD             (1.0f / R_LOAD)

//
// Vari�veis Globais da Simula��o
//
volatile float32_t g_vout_sim = 0.0f;        // Tens�o de sa�da simulada
volatile float32_t g_il_sim = 0.0f;          // Corrente no indutor simulada
volatile uint32_t g_step_counter = 0;        // Contador de passos dentro do ciclo PWM
volatile bool g_switch_on = false;           // Estado da chave (true = ligada)
volatile bool g_new_step_ready = false;      // Flag para novo passo de simula��o
volatile float g_duty_cycle = 0.5f;          // Raz�o c�clica (entre 0 e 1)

//
// Fun��o Principal
//
void main(void)
{
    float32_t v_l, i_c;

    // Inicializa��es do dispositivo
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();

    // Habilita interrup��es globais
    EINT;
    ERTM;

    // Loop principal
    while (1)
    {
        // Executa apenas se a ISR indicar que � hora de simular
        if (g_new_step_ready)
        {
            g_new_step_ready = false;

            // Tens�o no indutor
            v_l = g_switch_on ? (VIN - g_vout_sim) : (-g_vout_sim);

            // Corrente do capacitor
            i_c = g_il_sim - (g_vout_sim * INV_R_LOAD);

            // Atualiza��o via m�todo de Euler
            g_il_sim += INV_L * v_l;
            g_vout_sim += INV_C * i_c;
        }
    }
}

//
// Interrup��o do Timer (gera novo passo de simula��o HIL)
//
__interrupt void INT_myCPUTIMER0_ISR(void)
{
    // Define estado da chave com base na raz�o c�clica
    g_switch_on = (g_step_counter < (uint32_t)(g_duty_cycle * N_STEPS_PER_CYCLE));

    // Atualiza contador
    g_step_counter++;

    // Reinicia no fim do ciclo PWM
    if (g_step_counter >= N_STEPS_PER_CYCLE)
        g_step_counter = 0;

    // Sinaliza para o loop principal que deve simular o pr�ximo passo
    g_new_step_ready = true;

    // Libera nova interrup��o
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}
