//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"

//
// Defines
//
#define F_SWITCH            10000.0F   // Frequência de chaveamento do Buck (100 kHz)
#define TS                  (1.0F / F_SWITCH) // Período de chaveamento
#define SIMULATION_STEP_SIZE 0.000005F // Passo de tempo para a simulação (5 us)
#define NUM_SIMULATION_STEPS_PER_PWM_CYCLE (uint32_t)(TS / SIMULATION_STEP_SIZE)

//
// Buck Converter Parameters
//
#define VIN                 12.0F       // Tensão de entrada (V)
#define L_BUCK              0.001F     // Indutância (H) - 100 uH
#define C_BUCK              0.00001F   // Capacitância (F) - 1 uF
#define R_LOAD              10.0F       // Resistência de carga (Ohm)

//
// Global Variables for Buck Converter State and Simulation Control
//
volatile float32_t Vout_sim = 0.0F;  // Tensão de saída simulada
volatile float32_t IL_sim = 0.0F;    // Corrente do indutor simulada
volatile uint32_t pwm_step_counter = 0; // Contador de passos de simulação dentro de um ciclo PWM
volatile bool switch_on_state = false; // true se a chave estiver ligada, false se desligada
volatile bool new_simulation_step_ready = false; // Flag para indicar que um novo passo de simulação está pronto
volatile float duty_cycle=0.5f;

void main(void)
{
    // Device Initialization
    Device_init();


    //
    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();
    //
    // Initializes the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    Board_init();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;
    float32_t V_inductor;
    float32_t I_capacitor;

    while(1)
    {

        // Only execute the simulation step if the ISR has flagged it
        if (new_simulation_step_ready)
        {
            // Clear the flag immediately
            new_simulation_step_ready = false;

            // Solve the Buck converter equations for the current simulation step


                // Solve difference equations based on the current state determined by the ISR
                if (switch_on_state) // Chave ligada
                {
                    V_inductor = VIN - Vout_sim;
                    I_capacitor = IL_sim - (Vout_sim / R_LOAD);
                }
                else // Chave desligada
                {
                    V_inductor = -Vout_sim;
                    I_capacitor = IL_sim - (Vout_sim / R_LOAD);
                }

                // Update state variables using Euler's method
                IL_sim += (SIMULATION_STEP_SIZE / L_BUCK) * V_inductor;
                Vout_sim += (SIMULATION_STEP_SIZE / C_BUCK) * I_capacitor;

            // At this point, Vout_sim and IL_sim hold the updated values
            // These values can be used for debugging, plotting, or control algorithms
        }
    }

}


__interrupt void INT_myCPUTIMER0_ISR(void)
{
    // Determine the switch state for the *upcoming* simulation step
    if (pwm_step_counter < (uint32_t)(duty_cycle * NUM_SIMULATION_STEPS_PER_PWM_CYCLE))
    {
        switch_on_state = true; // Chave ligada
    }
    else
    {
        switch_on_state = false; // Chave desligada
    }

    // Increment simulation step counter
    pwm_step_counter++;

    // Reset counter if a full PWM cycle has passed
    if (pwm_step_counter >= NUM_SIMULATION_STEPS_PER_PWM_CYCLE)
    {
        pwm_step_counter = 0;
    }

    // Set flag to indicate that a new simulation step needs to be processed by the main loop
    new_simulation_step_ready = true;

    // Acknowledge this interrupt to receive more interrupts from this group
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}
