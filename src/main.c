/**
 * @file    main.c
 * @brief   SPI Master Demo
 * @details Shows Master loopback demo for QSPI1
 *          Read the printf() for instructions
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// #include "dma.h"
#include "led.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "spi.h"
#include "tmr.h"
#include "uart.h"
#include <stdbool.h>

#include "gpio.h"

/***** Globals *****/
const mxc_gpio_cfg_t hall_pin = {
    .port = MXC_GPIO0,
    .mask = MXC_GPIO_PIN_0,
    .pad = MXC_GPIO_PAD_NONE,
    .func = MXC_GPIO_FUNC_OUT,
    .vssel = MXC_GPIO_VSSEL_VDDIO,
    .drvstr = MXC_GPIO_DRVSTR_0};
const mxc_gpio_cfg_t interrupt_out_pin = {
    .port = MXC_GPIO0,
    .mask = MXC_GPIO_PIN_9,
    .pad = MXC_GPIO_PAD_NONE,
    .func = MXC_GPIO_FUNC_OUT,
    .vssel = MXC_GPIO_VSSEL_VDDIO,
    .drvstr = MXC_GPIO_DRVSTR_0};
const mxc_gpio_cfg_t rck_pin = {
    .port = MXC_GPIO0,
    .mask = MXC_GPIO_PIN_3,
    .pad = MXC_GPIO_PAD_NONE,
    .func = MXC_GPIO_FUNC_OUT,
    .vssel = MXC_GPIO_VSSEL_VDDIO,
    .drvstr = MXC_GPIO_DRVSTR_0};
const mxc_gpio_cfg_t clr_pin = {
    .port = MXC_GPIO0,
    .mask = MXC_GPIO_PIN_2,
    .pad = MXC_GPIO_PAD_NONE,
    .func = MXC_GPIO_FUNC_OUT,
    .vssel = MXC_GPIO_VSSEL_VDDIO,
    .drvstr = MXC_GPIO_DRVSTR_0};

#define SPI MXC_SPI0
#define DATA_LEN 3       // Words
#define SPI_SPEED 1600000 // Bit Rate
#define WRITE_BIT 0x80
volatile int SPI_FLAG;
int arr[8] = {0b00000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000};
uint32_t a[24] = {0x010000, 0x020000, 0x040000, 0x080000, 0x100000, 0x200000, 0x400000, 0x800000, 0x000100, 0x000200, 0x000400, 0x000800, 0x001000, 0x002000, 0x004000, 0x008000, 0x000001, 0x000002, 0x000004, 0x000008, 0x000010, 0x000020, 0x000040, 0x000080};
int A[9][24] = {
    {0b000000000000111110000000},
    {0b000000000010000001000000},
    {0b000000001000000000100000},
    {0b000000010000000000010000},
    {0b000001111111111111111000},
    {0b000010000000000000001000},
    {0b000100000000000000000100},
    {0b001000000000000000000010},
    {0b010000000000000000000001}};
int line_index = 0;
int speed = 2000;
int count = 0;
bool sens_flag = FALSE;
bool timer_flag = FALSE;
volatile uint32_t systick_counter = 0;
int time_rev = 1;
int led_flash_delay = 0;

/***** Functions *****/
void gpio_isr(void *cbdata)
{
    printf("\n\n\nHELLORONAN\n\n\n");
    speed = systick_counter;
    printf("change in time between hall pulses: %d", speed);
    // sens_flag=TRUE;
    systick_counter = 0;
    count = 0;

    // time_rev = systick_counter - time_rev;
    // led_flash_delay=0;
}

void SPI_Callback(mxc_spi_req_t *req, int error)
{
    SPI_FLAG = error;
}

void Delay_ISR(void)
{
    MXC_DelayHandler();
}
int SPI_ShiftReg_Init()
{

    int retVal;
    retVal = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED); // Changed SS polarity
    MXC_SPI_SetMode(SPI, SPI_MODE_0);
    MXC_SPI_SetDataSize(SPI, 8);
    MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);
    // RCLk pin
    MXC_GPIO_Config(&rck_pin);

    // clr PIN
    MXC_GPIO_Config(&clr_pin);
    // retVal=SPI_MAX31723_Read_Reg(0x02); // Continuous measurement, 9-bit resolution

    return retVal;
}

void SPI_Write_ShiftReg(uint32_t w_data)
{
    uint32_t retVal;
    
    uint32_t tx_data[2] = {w_data, 0x00000000};
    // uint8_t rx_data[2] = {0x00,0x00};
    uint32_t rx_data[2] = {0x00000000, 0x00000000};
    // printf("ola");
    mxc_spi_req_t req;
    // SPI Request
    req.spi = SPI;
    req.txData = (uint32_t *)tx_data;
    req.rxData = (uint32_t *)rx_data;
    req.txLen = DATA_LEN;
    req.rxLen = DATA_LEN;
    req.ssIdx = 0;
    req.ssDeassert = 1;
    req.txCnt = 0;
    req.rxCnt = 0;
    req.completeCB = (spi_complete_cb_t)SPI_Callback;
    // req.completeCB = NULL;
    SPI_FLAG = 1;

    retVal = MXC_SPI_MasterTransaction(&req);
    // MXC_Delay(MXC_DELAY_MSEC(20));
    if (retVal != E_NO_ERROR)
    {
        printf("Issue reading SPI peripheral: error %d", retVal);
    }
    return;
}

//*************UNUSED FOR (HALL SESNOR)******************
void Delay_Complete_Callback(int result)
{
    // Handle the completion of the delay here
    if (result == E_NO_ERROR)
    {
        // Delay completed successfully
        // systick_counter++;
        // there is an issue with MXC_DelayAbort() function
    }
    else if (result == E_ABORT)
    {
        // Delay was aborted
    }
    // timer_flag=TRUE;
    // count++;
    // MXC_DelayAsync(MXC_DELAY_MSEC(1), Delay_Complete_Callback);
    systick_counter++;
}
int main(void)
{
    int retVal;
    int j = 0;
    int multiplicand=1;
    uint32_t spi_message = 0x000000;
    printf("\n**************************** RONAN'S POV (PERSISTENCE OF VISION) ***********************\n");
    // Configure interrupt
    MXC_GPIO_Config(&hall_pin);
    MXC_GPIO_Config(&interrupt_out_pin);
    MXC_GPIO_RegisterCallback(&hall_pin, gpio_isr, &interrupt_out_pin);
    MXC_GPIO_IntConfig(&hall_pin, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(hall_pin.port, hall_pin.mask);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(hall_pin.port)));
    // Initialise the asynchronouse delay of 1 millisecond

    // Configure the peripheral

    // retVal = TPIC6C595_Init();
    retVal = SPI_ShiftReg_Init();
    if (retVal != E_NO_ERROR)
    {
        printf("\nSPI INITIALIZATION ERROR\n");
        return retVal;
    }

    // MXC_DelayAsync(MXC_DELAY_MSEC(1), Delay_Complete_Callback);

    // Enabling SysTick interrupt
    // NVIC_EnableIRQ(SysTick_IRQn);
    // MXC_NVIC_SetVector(SysTick_IRQn, Delay_ISR);

    while (1)
    {
        for (int i = 0; (i < 24); i++)
            {
                // turn off clr pin
                MXC_GPIO_OutToggle(clr_pin.port, clr_pin.mask);
                // send data via spi
                SPI_Write_ShiftReg(a[i]);
                // SPI_Write_ShiftReg((0x00));
                // set latch high
                MXC_GPIO_OutToggle(rck_pin.port, rck_pin.mask);
                MXC_Delay((MXC_DELAY_USEC(800)));
                // set latch low
                MXC_GPIO_OutClr(rck_pin.port, rck_pin.mask);
                // set clear high clear output
                MXC_GPIO_OutClr(clr_pin.port, clr_pin.mask);
                //printf("\nHELLO %d", i);
            }
        for (int i = 24; (i > 0); i--)
            {
                // turn off clr pin
                MXC_GPIO_OutToggle(clr_pin.port, clr_pin.mask);
                // send data via spi
                SPI_Write_ShiftReg(a[i]);
                // SPI_Write_ShiftReg((0x00));
                // set latch high
                MXC_GPIO_OutToggle(rck_pin.port, rck_pin.mask);
                MXC_Delay((MXC_DELAY_USEC(800)));
                // set latch low
                MXC_GPIO_OutClr(rck_pin.port, rck_pin.mask);
                // set clear high clear output
                MXC_GPIO_OutClr(clr_pin.port, clr_pin.mask);
                //printf("\nHELLO %d", i);
            }
        
        
        
    }
}
