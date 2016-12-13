#include "stm32f4xx.h"
// Keil::Device:STM32Cube HAL:Common

#define LED0 (1<<12)
#define LED1 (1<<13)
#define LED2 (1<14)
#define LED3 (1<<15)

/* Macros for word accesses */
#define HW32_REG(ADDRESS) (*((volatile unsigned long *)(ADDRESS)))
/* Use Breakpoint to stop when error is detected
(KEIL MDK specific intrinsic) */
/* it can be changed to while(1) XXif needed */
#define stop_cpu ;
void LED_initialize(void); // Initialize LED
void task0(void); // Toggle LED0
void task1(void); // Toggle LED1
void task2(void); // Toggle LED2
void task3(void); // Toggle LED3
// Event to tasks
volatile uint32_t systick_count=0;
// Stack for each task (8Kbytes each - 1024 x 8 bytes)
long long task0_stack[1024], task1_stack[1024],
task2_stack[1024], task3_stack[1024];
// Data use by OS
uint32_t curr_task = 0; // Current task
uint32_t* _curr_task = &curr_task;
uint32_t next_task = 1; // Next task
uint32_t* _next_task = &next_task;
uint32_t PSP_array[4];
uint32_t* _PSP_array = (uint32_t*)&PSP_array; // Process Stack Pointer for each task
// -------------------------------------------------------------
// Start of main program
int main(void)
{
  SCB->CCR |= SCB_CCR_STKALIGN_Msk; // Enable double word stack alignment
  //(recommended in Cortex-M3 r1p1, default in Cortex-M3 r2px and Cortex-M4)
  LED_initialize();
  // Starting the task scheduler
  // Create stack frame for task0
  PSP_array[0] = ((unsigned int) task0_stack)
  + (sizeof task0_stack) - 16*4;
  HW32_REG((PSP_array[0] + (14<<2))) = (unsigned long) task0;
  // initial Program Counter
  HW32_REG((PSP_array[0] + (15<<2))) = 0x01000000; // initial xPSR
  // Create stack frame for task1
  PSP_array[1] = ((unsigned int) task1_stack) + (sizeof task1_stack) - 16*4;
  HW32_REG((PSP_array[1] + (14<<2))) = (unsigned long) task1;
  // initial Program Counter
  HW32_REG((PSP_array[1] + (15<<2))) = 0x01000000; // initial xPSR
  // Create stack frame for task2
  PSP_array[2] = ((unsigned int) task2_stack) + (sizeof task2_stack) - 16*4;
  HW32_REG((PSP_array[2] + (14<<2))) = (unsigned long) task2;
  // initial Program Counter
  HW32_REG((PSP_array[2] + (15<<2))) = 0x01000000; // initial xPSR
  // Create stack frame for task3
  PSP_array[3] = ((unsigned int) task3_stack) + (sizeof task3_stack) - 16*4;
  HW32_REG((PSP_array[3] + (14<<2))) = (unsigned long) task3;
  // initial Program Counter
  HW32_REG((PSP_array[3] + (15<<2))) = 0x01000000; // initial xPSR
  curr_task = 0; // Switch to task #0 (Current task)
  __set_PSP((PSP_array[curr_task] + 16*4)); // Set PSP to topof task 0 stack
  NVIC_SetPriority(PendSV_IRQn, 0xFF); // Set PendSV to lowest possible priority
  SysTick_Config(168); // 1000 Hz SysTick interrupton 168MHz core clock
  __set_CONTROL(0x3); // Switch to use Process Stack, unprivilegedstate
  __ISB(); // Execute ISB after changing CONTROL (architecturalrecommendation)
  //GPIOD->BSRR |= LED1;
  task0(); // Start task 0
  while(1) {

    stop_cpu;// Should not be here
  }
}
    // ------------------------------------------------------------
void task0(void) // Toggle LED #0
{
  while (1) {
    if (systick_count & 0x80) {
      GPIOD->BSRR = LED0;
    } else {
      GPIOD->BSRR = LED0 << 16;
    } // Clear LED 0
  }
}

void task1(void) // Toggle LED #0
{
  GPIOD->BSRR = LED1;
  while (1) {
    if (systick_count & 0x100) {
      //GPIOD->BSRR = LED1;
    } else {
      //GPIOD->BSRR = LED1;
    } // Clear LED 0
  }
}
void task2(void) // Toggle LED #0
{
  GPIOD->BSRR |= LED2;
  while (1) {
    if (systick_count & 0x200) {
      //GPIOD->BSRR = LED2;
    } else {
      //GPIOD->BSRR = LED2;
    } // Clear LED 0
  }
}
void task3(void) // Toggle LED #0
{
  while (1) {
    if (systick_count & 0x400) {
      //GPIOD->BSRR = LED3;
    } else {
      //GPIOD->BSRR = LED3;
    } // Clear LED 0
  }
}

  // ------------------------------------------------------------
void PendSV_Handler(void)
{ // Context switching code
  // Simple version - assume No floating point support
  // -------------------------
  // Save current context
  asm("MRS R0, PSP\n" // Get current process stack pointer value
      "STMDB R0!,{R4-R11}\n" // Save R4 to R11 in task stack (8 regs)
      "LDR R1,=_curr_task\n"
      "LDR R2,[R1]\n" // Get current task ID
      "LDR R3,=_PSP_array\n"
      "STR R0,[R3, R2, LSL #2]\n" // Save PSP value into PSP_array
      // -------------------------
      // Load next context
      "LDR R4,=_next_task\n"
      "LDR R4,[R4]\n" // Get next task ID
      "STR R4,[R1]\n" // Set curr_task = next_task
      "LDR R0,[R3, R4, LSL #2]\n" // Load PSP value from PSP_array
      "LDMIA R0!,{R4-R11}\n" // Load R4 to R11 from task   stack (8 regs)
      "MSR PSP, R0\n" // Set PSP to next task
      "BX LR\n");
}
    // ------------------------------------------------------------
void SysTick_Handler(void) // 1KHz
{ // Increment systick counter for LED blinking
  systick_count++;
  GPIOD->BSRR |= LED3;
  // Simple task round robin scheduler
  switch(curr_task) {
  case(0): next_task=1; break;
  case(1): next_task=2; break;
  case(2): next_task=3; break;
  case(3): next_task=0; break;
  default: next_task=0;
  stop_cpu;
  break; // Should not be here
  }
  if (curr_task!=next_task){ // Context switching needed
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk; // Set PendSV to pending
  }
  return;
}
    // ------------------------------------------------------------
void LED_initialize(void)
{
  // Configure LED outputs
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enable Port D clock
  // Set pin 12, 13, 14, 15 as general purpose output mode(pull-push)
  GPIOD->MODER |= (GPIO_MODER_MODER12_0 |
  GPIO_MODER_MODER13_0 |
  GPIO_MODER_MODER14_0 |
  GPIO_MODER_MODER15_0 ) ;
  // GPIOE->OTYPER |= 0; // No need to change - use pull-push output
  GPIOD->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR12 | // 100MHz operations
  GPIO_OSPEEDER_OSPEEDR13 |
  GPIO_OSPEEDER_OSPEEDR14 |
  GPIO_OSPEEDER_OSPEEDR15 );
  GPIOD->PUPDR = 0; // No pull up , no pull down
  //GPIOD->BSRR |= LED0;
  return;
}
