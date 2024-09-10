/**
  ******************************************************************************
  * @file    main.c
  * @author  Kirellos, Loren
  * @date    Sep 5 2024
  * @brief   ECE 362 Lab 1 
  ******************************************************************************
*/


/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "osalama01";

/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <stdint.h>

void initb();
void initc();
void setn(int32_t pin_num, int32_t val);
int32_t readpin(int32_t pin_num);
void buttons(void);
void keypad(void);
void autotest(void);
extern void internal_clock(void);
extern void nano_wait(unsigned int n);

int main(void) {
    internal_clock(); // do not comment!
    // Comment until most things have been implemented
    autotest();
    initb();
    initc();

    // uncomment one of the loops, below, when ready
    // while(1) {
    //   buttons();
    // }

    while(1) {
      

    for(;;);
    
    return 0;
    }
}

/**
 * @brief Init GPIO port B
 *        Pin 0: input
 *        Pin 4: input
 *        Pin 8-11: output
 *
 */
void initb() {
   // Enable RCC clock for GPIO Port B
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure pins PB8, PB9, PB10, PB11 as outputs
    
    GPIOB->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk |
                      GPIO_MODER_MODER10_Msk | GPIO_MODER_MODER11_Msk);
    
    GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 |
                     GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);

    
    GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER4_Msk);

}

/**
 * @brief Init GPIO port C
 *        Pin 0-3: inputs with internal pull down resistors
 *        Pin 4-7: outputs
 *
 */
void initc() {
   // Enable RCC clock for GPIO Port C
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Configure pins PC4, PC5, PC6, PC7 as outputs
    // Clear the mode bits for PC4 to PC7
    GPIOC->MODER &= ~(GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER5_Msk |
                      GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);
    // Set PC4, PC5, PC6, PC7 as output (01 mode)
    GPIOC->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
                     GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);

    // Configure pins PC0, PC1, PC2, PC3 as inputs with pull-down resistors
    // Clear the mode bits for PC0 to PC3
    GPIOC->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk |
                      GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
    // Set PC0, PC1, PC2, PC3 to pull-down resistors
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk | GPIO_PUPDR_PUPDR1_Msk |
                      GPIO_PUPDR_PUPDR2_Msk | GPIO_PUPDR_PUPDR3_Msk);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1 |
                     GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR3_1);
}
  

/**
 * @brief Set GPIO port B pin to some value
 *
 * @param pin_num: Pin number in GPIO B
 * @param val    : Pin value, if 0 then the
 *                 pin is set low, else set high
 */
// void setn(int32_t pin_num, int32_t val) {
//   if (val == 0) {
//     GPIOB -> BRR = (1<< pin_num);
//   }
//   else {
//     GPIOB -> BSRR = (1 << pin_num);
//   }
// }

void setn(int32_t pin_num, int32_t val) {
    if (val == 0) {
        GPIOB->BRR = (1 << pin_num);
    } else {
        GPIOB->BSRR = (1 << pin_num);
    }
}


/**
 * @brief Read GPIO port B pin values
 *
 * @param pin_num   : Pin number in GPIO B to be read
 * @return int32_t  : 1: the pin is high; 0: the pin is low
 */
// int32_t readpin(int32_t pin_num) {
//   if (pin_num != 0) {
//     return 0x1;
//   }
//   else {
//     return 0x0;
//   }
    
// }
int32_t readpin(int32_t pin_num) {
    return (GPIOB->IDR & (1 << pin_num)) ? 1 : 0;
}


/**
 * @brief Control LEDs with buttons
 *        Use PB0 value for PB8
 *        Use PB4 value for PB9
 *
 */
// void buttons(void) {
//       // Read button input at PB0
//     uint32_t pb0_state = GPIOB->IDR & GPIO_IDR_0;
//     // Put the PB0 value as output for PB8
//     if (pb0_state) {
//         GPIOB->ODR |= GPIO_ODR_8;  // Turn on LED at PB8
//     } else {
//         GPIOB->ODR &= ~GPIO_ODR_8;  // Turn off LED at PB8
//     }

//     // Read button input at PB4
//     uint32_t pb4_state = GPIOB->IDR & GPIO_IDR_4;
    
//     if (pb4_state) {
//         GPIOB->ODR |= GPIO_ODR_9;  // Turn on LED at PB9
//     } else {
//         GPIOB->ODR &= ~GPIO_ODR_9;  // Turn off LED at PB9
//     }

  
// }

void buttons(void) {
    // Read button input at PB0
    int32_t pb0_state = readpin(0); // Use readpin function
    // Set PB8 based on PB0 value
    setn(8, pb0_state);

    // Read button input at PB4
    int32_t pb4_state = readpin(4); // Use readpin function
    // Set PB9 based on PB4 value
    setn(9, pb4_state);
}


/**
 * @brief Control LEDs with keypad
 * 
 */
// void keypad(void) {
//   for (int i = 0; i < 4; i++) {
//         // Set the ith column to be 1 using GPIOC->ODR
//         GPIOC->ODR = (1 << (i + 4));  // PC4-PC7 are columns

        
//         //nano_wait(1000000);

        
//         uint32_t row_input = GPIOC->IDR & 0xF;

        
//         if (row_input & (1 << i)) {
//             GPIOB->ODR |= (1 << (i + 8));  // Turn on corresponding LED
//         } else {
//             GPIOB->ODR &= ~(1 << (i + 8));  // Turn off corresponding LED
//         }
//     }
//     return;
// }

void keypad(void) {
    for (int i = 0; i < 4; i++) {
        // Set the ith column to high
        GPIOC->ODR = (1 << (i + 4)); // Set PC4-PC7 to high

        nano_wait(1000000); // Wait to allow the signal to stabilize

        uint32_t row_input = GPIOC->IDR & 0xF; // Read row inputs

        // Check the ith row and control LEDs accordingly
        if (row_input & (1 << i)) {
            setn(i + 8, 1); // Turn on corresponding LED
        } else {
            setn(i + 8, 0); // Turn off corresponding LED
        }
    }
}
