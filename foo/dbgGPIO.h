// ----------------------------------------------------------------------------
// --- Written by Dmitri Kelbas
// --- Modified by Dmitri Kelbas [14-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __DBG_GPIO_H_
#define __DBG_GPIO_H_

/*
 *                              -- READ ME FIRST --
 *
 * This file has macros and functions to use Xilinx Zynq-702 board GPIOs to debug/profile the code.
 *
 * To toggle GPIO one need to:
 * a)  include this header file into source code *.cpp file. i.e.
 *          #include "platform/dbgGPIO.h"
 * b)  call macro SET_GPIO_ON before the event and macro SET_GPIO_OFF after the event, i.e.
 *          SET_GPIO_ON( GPIO_BIT_9 );
 *          processImplObjects_b();
 *          SET_GPIO_OFF( GPIO_BIT_9 );
 *
 * To disable functionality and exclude code from compilation one can comment out
 *  #define PROFILING_BY_LED_GPIO
 *
 * GPIO[0..7] are connected to LEDs on Zynq board, and have poor transitions.
 * GPIO[8..11] are connected/mapped by FPGA designer, require modified BOOT.bin and have excellent transitions.
 */

// ----------------------------------------------------------------------------
#include "mecl/core/MeclTypes.h"
// ----------------------------------------------------------------------------
// Suppress the QACPP MISRA warning regarding use of macros
// PRQA S 1020, 1025 EOF
#define FPGA_REG_VALUE32(addr, offset)  (*((volatile uint32_t*) ((char*)(addr) + (offset))))

#define GPIO_BIT_0  0x01
#define GPIO_BIT_1  0x02
#define GPIO_BIT_2  0x04
#define GPIO_BIT_3  0x08
#define GPIO_BIT_4  0x10
#define GPIO_BIT_5  0x20
#define GPIO_BIT_6  0x40
#define GPIO_BIT_7  0x80
#define GPIO_BIT_8  0x100
#define GPIO_BIT_9  0x200
#define GPIO_BIT_10 0x400
#define GPIO_BIT_11 0x800

// -----------------------------------------------------------------------------------------------------------
// -!!- NOTE: -!!-
// The default Zynq-702 fabric in boot.bin file supports writing to the 8 bit port
// connected to 8 LESDs DS15-DS22 which are hooked up to J63 connector.
// The issue with those pins is their power is not enough to provide 'sharp' edges of
// transition between '1' and '0', leading to ringing.
// The on-site made fabric and boot.bin file was created, which connects upper 4 bits to connector J62.
// -----------------------------------------------------------------------------------------------------------
// Be aware, running code with enabled PROFILING_BY_LED_GPIO on original boot.bin fabric may cause kernel hang
// To use it on default kernel and boot.bin, please use only GPIO_BIT_0 through GPIO_BIT_7
// -----------------------------------------------------------------------------------------------------------

#ifdef PROFILING_BY_LED_GPIO
// ----------------------------------------------------------------------------
static void *virt_User_LED_addr = NULL;
// ----------------------------------------------------------------------------

/* map and get virtual address range according to given physical address */
static void* map_and_get_virtual_address(off_t phys_mem_address, int memfd, unsigned map_size){
    unsigned page_size = getpagesize();

    /* determine total offset (from 0) */
    unsigned offset = phys_mem_address & ~(off_t) (page_size - 1);
    unsigned offset_in_page = phys_mem_address & (page_size - 1);

    /* Map page(s) of memory into user space (it may not be at the start of the page) */
    void* map_base_reg = mmap(0, map_size, PROT_READ | PROT_WRITE, MAP_SHARED,
            memfd, offset);
    if (map_base_reg == MAP_FAILED) {
        printf("Can't map memory from physical address 0x%08lX to user space.\n", phys_mem_address);
    }

    /* get the virtual address of the device in user space which will be an offset from the base
     that was mapped as memory is mapped at the start of a page */
    virt_User_LED_addr = (void*)((char*) map_base_reg + offset_in_page);
    return virt_User_LED_addr;
}

static void* initUserGPIOVirtualAddress( void ){
    static sint32_t memfd = 0;
    static uint32_t hwinit_once = 1;

    if( virt_User_LED_addr == NULL )
    {
        if(hwinit_once)
        {
            memfd = open("/dev/mem", O_RDWR | O_SYNC); //  | O_DIRECT
            if (memfd == -1) {
                printf("Can't open /dev/mem.\n");
            }
            hwinit_once=0;
        }
        virt_User_LED_addr = map_and_get_virtual_address(0x40050030, memfd, getpagesize());
    }
    return virt_User_LED_addr;
};

static void setGpio( uint32_t GPIObit )
{
    if( virt_User_LED_addr == NULL )
        initUserGPIOVirtualAddress();

    if( virt_User_LED_addr == NULL )
        return;
    else
        FPGA_REG_VALUE32(virt_User_LED_addr, 0 ) |= (GPIObit); // GPIO set to 1, LED is on
}

static void clearGpio( uint32_t GPIObit )
{
    if( virt_User_LED_addr == NULL )
        initUserGPIOVirtualAddress();

    if( virt_User_LED_addr == NULL )
        return;
    else
        FPGA_REG_VALUE32(virt_User_LED_addr, 0 ) &= ~(GPIObit); // GPIO is 0, LED is off
}

// ----------------------------------------------------------------------------
#endif

#ifdef PROFILING_BY_LED_GPIO
#define SET_GPIO_ON( b ) { clearGpio( b ); setGpio( b ); }
#define SET_GPIO_OFF( b ) { clearGpio( b ); }
#else
#define SET_GPIO_ON( b )
#define SET_GPIO_OFF( b )
#endif

#endif
