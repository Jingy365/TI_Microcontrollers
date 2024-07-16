/*******************************************************************************
 * DMA module 20-Bit register library
 *
 *
 * DMA_SA_DA.h
 *
 * This library provides the "DMA_INIT" macro. This macro may be used to write
 * an address into the DMA-registers (DMAxSA and DMAxDA).
 *
 * The macro allows to write 20-Bit address values into the DMA-registers if the
 * DMA-registers of the controller allow the use of 20-Bit address values (if the
 * type SFR_20BIT is defined). The macro is used to avoid a write access to the
 * high-nibble of the high-word of the 20-Bit DMA-registers. Avoiding such an
 * access is important, as such an access would clear the contents of the accessed
 * DMA-register.
 *
 * In those chases where the DMA-registers of the used controller only support
 * 16-Bit address values, the macro only allows to write 16-Bit address values
 * into the DMA-registers.
 *
 *
 * The examples below illustrate the use of the macro.
 *
 * int values[10];
 * DMA_INIT(array, DMA0SA);   //the start address of array values is written
 *                             //into the DMA-register DMA0SA
 *
 * DMA_INIT(0x5FFFF, DMA1DA); //the address 0x5FFFF is written into the
 *                             //DMA-register DMA1DA
 *
 *
 * HS-Kempten, 30.12.2021
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>


#ifndef __DMA_SA_DA
  #define __DMA_SA_DA
  #if defined( SFR_20BIT )

     #define DMA_INIT(src,dst) { \
                volatile union { \
                  uint32_t value;   \
                  __SFR_FARPTR ptr; \
                } B;                    \
                B.value = (uint32_t)(src); \
                (*((volatile __SFR_FARPTR *)(&dst))) = B.ptr;  /*write 20-Bit source address into a volatile destination*/ \
              };\

  #else

    #define DMA_INIT(src,dst) (dst) = (uint16_t)(src);

  #endif
#endif








