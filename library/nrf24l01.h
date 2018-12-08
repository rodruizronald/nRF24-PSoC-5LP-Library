//*****************************************************************************
//
//  Prototypes and defines for the NRF24.
//  File:     nrf24.h
//  Version:  1.0v
//  Author:   Ronald Rodriguez Ruiz.
//  Date:     May 12, 2017.
//  ---------------------------------------------------------------------------
//  Specifications:
//  This library is meant to run on a Cypress PSoC 5LP. However, if the header
//  fiel definitions, the ISR and SPI are modified to match the hadware-software
//  interface provided for the host MCU, then it should work on other MCUs.
//
//*****************************************************************************

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

//*****************************************************************************
//
//  The following are defines for the hardware API. These definitions should be
//  modified based on the MCU hadware/software interface for GPIO.
//
//*****************************************************************************

//
//  NRF24 CE pin controller.
//
#define _nrf24_enable()                       CE_Write(HIGH)
#define _nrf24_disable()                      CE_Write(LOW)

//
//  NRF24 CSN pin controller.
//
#define _nrf24_select()                       SPI_SS_Write(LOW)
#define _nrf24_deselect()                     SPI_SS_Write(HIGH)

//
//  Delay function from the hosting MCU,
//  the functions should provide a delay in us and ms.
//
#define _delay_ms(...)                        CyDelay(__VA_ARGS__)
#define _delay_us(...)                        CyDelayUs(__VA_ARGS__)

//*****************************************************************************
//
//  The following are defines for the operation modes.
//
//*****************************************************************************

#define PRX                                   0
#define PTX                                   1

//*****************************************************************************
//
//  Prototypes for the API
//
//*****************************************************************************

//
//  NRF24L01 module initialization.
//
extern void NRF24_init(uint8_t mode, uint8_t id);

//
//  Data transmission (PTX/PRX).
//
extern char* NRF24_get_data(void);
extern void NRF24_assign_int(uint8_t var, int value);
extern void NRF24_assign_float(uint8_t var, float value);
extern void NRF24_assign_string(uint8_t var, char* value);
extern uint8_t NRF24_request_data(uint8_t var, uint8_t prx_id);

//
//  ISR for interrupt requests.
//
CY_ISR_PROTO(nrf24_Int_Handler);

#endif
