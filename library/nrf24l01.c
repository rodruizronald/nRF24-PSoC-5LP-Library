//*****************************************************************************
//
//  Prototypes for the NRF24.
//  API and private functions for the NRF24.
//  File:     nrf24.c
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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "nrf24l01.h"

//*****************************************************************************
//
//  The following are defines for the memory map.
//
//*****************************************************************************

#define CONFIG                          0x00
#define EN_AA                           0x01
#define EN_RXADDR                       0x02
#define SETUP_AW                        0x03
#define SETUP_RETR                      0x04
#define RF_CH                           0x05
#define RF_SETUP                        0x06
#define STATUS                          0x07
#define OBSERVE_TX                      0x08
#define CD                              0x09
#define RX_ADDR_P0                      0x0A
#define RX_ADDR_P1                      0x0B
#define RX_ADDR_P2                      0x0C
#define RX_ADDR_P3                      0x0D
#define RX_ADDR_P4                      0x0E
#define RX_ADDR_P5                      0x0F
#define TX_ADDR                         0x10
#define RX_PW_P0                        0x11
#define RX_PW_P1                        0x12
#define RX_PW_P2                        0x13
#define RX_PW_P3                        0x14
#define RX_PW_P4                        0x15
#define RX_PW_P5                        0x16
#define FIFO_STATUS                     0x17
#define DYNPD                           0x1C
#define FEATURE                         0x1D

//*****************************************************************************
//
//  The following are defines for the general status register.
//
//*****************************************************************************

#define RX_DR                           6
#define TX_DS                           5
#define MAX_RT                          4
#define RX_P_NO                         1

//*****************************************************************************
//
//  The following are defines for the fifo status register.
//
//*****************************************************************************

#define TX_REUSE                        6
#define TX_FULL                         5
#define TX_EMPTY                        4
#define RX_FULL                         1
#define RX_EMPTY                        0

//*****************************************************************************
//
//  The following are defines for bit and instruction mnemonics.
//
//*****************************************************************************

#define MASK_RX_DR                      6
#define MASK_TX_DS                      5
#define MASK_MAX_RT                     4
#define EN_CRC                          3
#define CRCO                            2
#define PWR_UP                          1
#define PRIM_RX                         0

#define R_REGISTER                      0x00
#define W_REGISTER                      0x20
#define REGISTER_MASK                   0x1F
#define R_RX_PAYLOAD                    0x61
#define W_TX_PAYLOAD                    0xA0
#define W_ACK_PAYLOAD                   0xA8
#define FLUSH_TX                        0xE1
#define FLUSH_RX                        0xE2
#define REUSE_TX_PL                     0xE3
#define ACTIVATE                        0x50
#define R_RX_PL_WID                     0x60
#define NOP                             0xFF

//*****************************************************************************
//
//  The following are defines to enable auto acknowledgment and rx addresses.
//
//*****************************************************************************

#define ENAA_P5                         5
#define ENAA_P4                         4
#define ENAA_P3                         3
#define ENAA_P2                         2
#define ENAA_P1                         1
#define ENAA_P0                         0

#define ERX_P5                          5
#define ERX_P4                          4
#define ERX_P3                          3
#define ERX_P2                          2
#define ERX_P1                          1
#define ERX_P0                          0

//*****************************************************************************
//
//  The following are defines for RF setup register.
//
//*****************************************************************************

#define RF_DR_LOW                       5
#define PLL_LOCK                        4
#define RF_DR_HIGH                      3
#define RF_PWR                          1

//*****************************************************************************
//
//  The following are defines for dynamic len paylod.
//
//*****************************************************************************

#define DPL_P0                          0
#define DPL_P1                          1
#define DPL_P2                          2
#define DPL_P3                          3
#define DPL_P4                          4
#define DPL_P5                          5
#define EN_DPL                          2
#define EN_ACK_PAY                      1
#define EN_DYN_ACK                      0

//*****************************************************************************
//
//  The following are defines for setting up auto re-transmission, address
//  width and transmit observer.
//
//*****************************************************************************

#define AW                              0
#define ARD                             4
#define ARC                             0
#define PLOS_CNT                        4
#define ARC_CNT                         0

//*****************************************************************************
//
//  The following are defines for logic operations.
//
//*****************************************************************************

#define READ                            0
#define WRITE                           1
#define LOW                             0
#define HIGH                            1

//*****************************************************************************
//
//  The following are defines for the application.
//
//*****************************************************************************

#define FIRST_INDEX                     0
#define CONFIG_PARAM                    5
#define NUM_VARIABLES                   9
#define DATA_LENGHT                     10
#define REPLY_LENGHT                    32
#define VARIABLES_LENGTH                3
#define ADDRESS_WIDTH                   0x03
#define ADDRESS_LENGTH                  0x05
#define COMMAND_LENGTH                  0x01
#define RF_CHANNEL                      0x02
#define TX_OUTPUT_POWER                 0x03
#define IRQ_RESET                       0x70
#define MAX_RT_RESET                    0x10
#define MAX_PLOS_CTN                    0x0F
#define ADR_VALUE                       0x02
#define ARC_VALUE                       0x0F

//*****************************************************************************
//
//  The following are defines for the transmission status.
//
//*****************************************************************************

#define NRF24_ACK_PAYLOAD_RECEIVED      0
#define NRF24_TRANSMISSION_FAILED       1

//*****************************************************************************
//
//  The following are global arrays to store data and var_idiables
//  to save reponse codes and flags.
//
//*****************************************************************************

//
//  Address for the PRX devices (each one must be unique).
//
static const uint8_t g_pipe0_address[][ADDRESS_LENGTH] =
{
  {0x4A,0x2B,0x3C,0x5D,0x1E},
  {0x2B,0x6D,0x7E,0x7A,0x8F},
  {0x4C,0x5C,0x8D,0x6B,0x9D},
  {0x6D,0x4B,0x9C,0x5C,0x0C},
  {0x7E,0x3A,0x0B,0x4D,0x1B},
  {0x8F,0x2E,0x2A,0x3E,0x2A}
};

static const char *g_var_idiable_id[NUM_VARIABLES] =
{
  "0", "1", "2",
  "3", "4", "5"
};

//
//  Data buffers.
//
static char g_value_buffer[DATA_LENGHT];
static char g_reply_buffer[REPLY_LENGHT];
static uint8_t g_nrf24_config[CONFIG_PARAM];
static uint8_t g_data_int[NUM_VARIABLES];
static float g_data_float[VARIABLES_LENGTH];
static char *g_data_string[VARIABLES_LENGTH];

//
//  System flags and data bytes.
//
static volatile uint8_t g_last_id = 10;
static volatile uint8_t g_ptx = false;
static volatile uint8_t g_request_flag = false;
static volatile uint8_t g_status = 0;
static volatile uint8_t g_incoming_byte = 0;
static volatile uint8_t g_master_reset = false;

//*****************************************************************************
//
//  Prototypes for private functions.
//
//*****************************************************************************

//
//  NRF24L01 internal configuration.
//
static void nrf24_config(uint8_t ptx, uint8_t id);

//
//  Status and control registers.
//
static uint8_t check_status(void);
static void nrf24_irq_clear(void);
static void nrf24_plos_ctn_reset(void);
static uint8_t nrf24_get_status(void);
static uint8_t nrf24_get_lost_packets_count(void);

//
//  SPI Interface.
//
static char nrf24_spi_handler(uint8_t task, uint8_t reg, char *arr, uint8_t len);
static char spi_transfer(char data);

//
//  Flush data.
//
static void flush_tx(void);
static void flush_rx(void);
static void nrf24_reset(void);

//
//  Address configuration.
//
static void set_address(uint8_t id);

//
//  Power control.
//
static void pwr_state(uint8_t state);

//
//  PRX - Acknowledgments.
//
static void attach_tx_payload(char *data, uint8_t len);
static void attach_ack_int(uint8_t data);
static void attach_ack_float(float data);
static void attach_ack_string(char *data, uint8_t len);
static void attach_ack_payload(char *data, uint8_t len);

//
//  PTX - Packets request and transmission.
//
static void ptx_request(uint8_t var_id, uint8_t prx_id);
static void ptx_transmit_int(int data, uint8_t prx_id);
static void ptx_transmit_float(float data, uint8_t prx_id);
static void ptx_transmit_string(char *data, uint8_t len, uint8_t prx_id);

//*****************************************************************************
//
//  Functions for the API.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize the NRF24.
//!
//! This function initilizes the SPI interface, attaches an ISR to the nrf24
//! IRQ pin for interrupt reuqests and configures the nRF24 module based on the
//! passed parameters.
//!
//! @param[in] mode Operation mode of the NRF24 (PTX/PRX).
//! @param[in] id   Unique id of the NRF24 (address).
//!
//! @return None.
//
//*****************************************************************************
void
NRF24_init(uint8_t mode, uint8_t id)
{
  g_status = 0;
  g_last_id = 10;
  g_incoming_byte = 0;

  g_ptx = false;
  g_master_reset = false;
  g_request_flag = false;

  //
  //  Initilize Serial Peripheral Interface.
  //
  SPI_Start();

  //
  //  Attach an ISR to the nrf24.
  //
  nrf24_Int_StartEx(nrf24_Int_Handler);

  //
  //  Configure the nrf24.
  //
  nrf24_config(mode, id);

  //
  //  Perform general reset (buffers flushed and flags cleared).
  //
  nrf24_reset();
}

//*****************************************************************************
//
//! @brief Request data to an PRX device.
//!
//! This function requests an specific var_idiable already store in an PRX device.
//!
//! @param[in] var_id  ID of the variable requested to the PRX device.
//! @param[in] prx_id  Unique id of the PRX device.
//!
//! @return status     Result of request operation, if status is equal to true,
//!                    then the transmission was successful, if not, it failed.
//
//*****************************************************************************
uint8_t
NRF24_request_data(uint8_t var_id, uint8_t prx_id)
{
  //
  //  First request - notify PRX device which var_idiable is required.
  //
  g_request_flag = false;
  ptx_request(var_id, id);

  //
  //  Wait until packet is recieved.
  //
  while(!g_request_flag);

  //
  //  If debug is enable, UART will display the status of the latest reply.
  //  Check if transmission was successful.
  //
  if(!check_status()) return false;

  _delay_ms(350);

  //
  //  Second request - let the PRX device send that var_idiable.
  //
  g_request_flag = false;
  ptx_request(var_id, id);

  //
  //  Wait until packet is recieved.
  //
  while(!g_request_flag);

  //
  //  If debug is enable, UART will display the status of the latest reply.
  //  Check if transmission was successful.
  //
  if(!check_status()) return false;

  return true;
}

//*****************************************************************************
//
//! @brief Assign an integer value to a variable.
//!
//! This function assign or updates the current value on an interger variable
//! in the PRX device. Variables are manage by array inside the library.
//!
//! @param[in] var_id  ID of the variable store in the PRX device.
//! @param[in] value   Data to assign to the variable.
//!
//! @return None.
//
//*****************************************************************************
void
NRF24_assign_int(uint8_t var_id, int value)
{
  g_data_int[var_id] = value;
}

//*****************************************************************************
//
//! @brief Assign an string value to a variable.
//!
//! This function assign or updates the current value on an string variable
//! in the PRX device. Variables are manage by array inside the library.
//!
//! @param[in] var_id  ID of the variable store in the PRX device.
//! @param[in] value   Data to assign to the variable.
//!
//! @return None.
//
//*****************************************************************************
void
NRF24_assign_string(uint8_t var_id, char* value)
{
  g_data_string[var_id] = value;
}

//*****************************************************************************
//
//! @brief Assign an float value to a variable.
//!
//! This function assign or updates the current value on an float variable
//! in the PRX device. Variables are manage by array inside the library.
//!
//! @param[in] var_id  ID of the variable store in the PRX device.
//! @param[in] value   Data to assign to the variable.
//!
//! @return None.
//
//*****************************************************************************
void
NRF24_assign_float(uint8_t var_id, float value)
{
  g_data_float[var_id] = value;
}

//*****************************************************************************
//
//! @brief Get data requested.
//!
//! This function returns an string with the last requested variable.
//!
//! @return g_reply_buffer Buffer of characters.
//
//*****************************************************************************
char*
NRF24_get_data(void)
{
  return g_reply_buffer;
}

//*****************************************************************************
//
//! @brief Configure the NRF24 module.
//!
//! This function writes on the NRF24 registers the desired configuration based
//! on default and passed parameters.
//!
//! @param[in] ptx  Operation mode of the NRF24 (PTX -> true/false).
//! @param[in] id   Unique id of the NRF24 (address).
//!
//! @return None.
//
//*****************************************************************************
static void
nrf24_config(uint8_t ptx, uint8_t id)
{
  //
  //  Wait for power down mode.
  //
  _delay_ms(15);
  g_nrf24_config[FIRST_INDEX] = 0x00;

  //
  //  Enable/Disable Auto Acknowledgment Data Pipes (0-5).
  //  ENAA_P1 - P5: Auto Ack. Disable.
  //  ENAA_P0: Auto Ack. Enable.
  //
  g_nrf24_config[FIRST_INDEX] = HIGH << ENAA_P0
                      | LOW << ENAA_P1
                      | LOW << ENAA_P2
                      | LOW << ENAA_P3
                      | LOW << ENAA_P4
                      | LOW << ENAA_P5;
  nrf24_spi_handler(WRITE, EN_AA, g_nrf24_config, COMMAND_LENGTH);

  //
  //  Enable/Disable Data Pipes (0-5) RX address.
  //  ERX_P1 - P5: Data Pipes Disable.
  //  ERX_P0: Data Pipe Enable.
  //
  g_nrf24_config[FIRST_INDEX] = HIGH << ERX_P0
                      | LOW << ERX_P1
                      | LOW << ERX_P2
                      | LOW << ERX_P3
                      | LOW << ERX_P4
                      | LOW << ERX_P5;
  nrf24_spi_handler(WRITE, EN_RXADDR, g_nrf24_config, COMMAND_LENGTH);

  //
  //  RX/TX Address field width (SETUP_AW): 5 bytes.
  //  '0x00' - Ilegal.
  //  '0x01' - 3 bytes.
  //  '0x02' - 4 bytes.
  //  '0x03' - 5 bytes.
  //
  g_nrf24_config[FIRST_INDEX] = ADDRESS_WIDTH;
  nrf24_spi_handler(WRITE, SETUP_AW, g_nrf24_config, COMMAND_LENGTH);

  //
  //  Setup of Automatic Retransmission (SETUP_RETR):
  //  Auto Retransmission Delay (ARD): '0010' - Wait 750us + 86us.
  //  Auto Retransmission Count (ARC): '1111' - Up to 15 Re-Transmit.
  //
  g_nrf24_config[FIRST_INDEX] = ADR_VALUE << ARD | ARC_VALUE << ARC;
  nrf24_spi_handler(WRITE, SETUP_RETR, g_nrf24_config, COMMAND_LENGTH);

  //
  //  RF Channel Frequency (RF_CH): 2.402 GHz.
  //
  g_nrf24_config[FIRST_INDEX] = RF_CHANNEL;
  nrf24_spi_handler(WRITE, RF_CH, g_nrf24_config, COMMAND_LENGTH);

  //
  //  Air Data Rate (RF_DR): 250 kbps.
  //  RF output power in TX mode: -0 dbm.
  //
  g_nrf24_config[FIRST_INDEX] = HIGH << RF_DR_LOW
                      | TX_OUTPUT_POWER << RF_PWR;
  nrf24_spi_handler(WRITE, RF_SETUP, g_nrf24_config, COMMAND_LENGTH);

  //
  //  Feature Register (FEATURE):
  //  EN_DPL: Enables Dynamic Payload len.
  //  EN_ACK_PAY: Enables Payload with ACK.
  //
  g_nrf24_config[FIRST_INDEX] = HIGH << EN_DPL | HIGH << EN_ACK_PAY;
  nrf24_spi_handler(WRITE, FEATURE, g_nrf24_config, COMMAND_LENGTH);

  //
  //  Number of Bytes in RX Payload: dynamic payload len.
  //  DPL_P0: PTX/PRX handle dynamic payload len.
  //
  g_nrf24_config[FIRST_INDEX] = HIGH << DPL_P0
                      | LOW << DPL_P1
                      | LOW << DPL_P2
                      | LOW << DPL_P3
                      | LOW << DPL_P4
                      | LOW << DPL_P5;
  nrf24_spi_handler(WRITE, DYNPD, g_nrf24_config, COMMAND_LENGTH);

  //
  //  Receive Address Data Pipe: 5 bytes.
  //  RX_ADDR_P0: g_pipe0_address[prx_id].
  //
  nrf24_spi_handler(WRITE, RX_ADDR_P0, g_pipe0_address[id], ADDRESS_LENGTH);

  //
  //  CONFIG Register,
  //  EN_CRC: '1' - Enable CRC.
  //  Cyclic Redundancy Check (CRCO): '1' - 2 bytes.
  //  Power (PWR_UP): '0' - Power Down -> (nRF24 goes to POWER DOWN mode).
  //  '1' - Power Up -> (nrf24 goes to Standby-I mode).
  //  PRIM_RX: '0' - Primary Transmitter (PTX).
  //  '1' - Primary Receiver (PRX).
  //
  if(ptx)
  {
    //
    //  PTX mode.
    //
    g_nrf24_config[FIRST_INDEX] = LOW << PRIM_RX;
    g_ptx = true;
  }
  else
  {
    //
    //  PRX mode.
    //
    g_nrf24_config[FIRST_INDEX] = HIGH << PRIM_RX;
    _nrf24_enable();
    g_ptx = false;
  }
  g_nrf24_config[FIRST_INDEX] |= HIGH << EN_CRC | HIGH << CRCO;
  nrf24_spi_handler(WRITE, CONFIG, g_nrf24_config, COMMAND_LENGTH);

  //
  //  Power up nrf24.
  //
  pwr_state(HIGH);
}

//*****************************************************************************
//
//! @brief Handles the SPI interface between the NRF24 and the MCU.
//!
//! This function performs reading and writing operation on the registers.
//!
//! @param[in] task Operation to perform (read/write).
//! @param[in] reg  Register in which the task will be carried out.
//! @param[in] arr  Array of values to be writing, if task is eqaul to write.
//! @param[in] len  Length of the array.
//!
//! @return incoming_byte For reading operations, incoming byte the reply from
//!                       the register read.
//
//*****************************************************************************
static char
nrf24_spi_handler(uint8_t task, uint8_t reg, char *arr, uint8_t len)
{
  uint8_t i = 0;
  uint8_t incoming_byte = 0;
  //
  //  Task: READ/WRITE to a specific register.
  //  reg: nrf24 specific register.
  //  array: Array with the package to manipulate.
  //  len: Array len.
  //

  //
  //  CSN LOW - nrf24 starts to listen for commandS.
  //
  _nrf24_select();
  _delay_us(10);

  //
  //  Empty reply buffer.
  //
  bzero(g_reply_buffer, strlen(g_reply_buffer));

  if(task == WRITE)
  {
    //
    //  Write into low level register.
    //
    spi_transfer(W_REGISTER | reg);
  }
  else
  {
    //
    //  Read low level register.
    //
    spi_transfer(R_REGISTER | reg);
  }

  for (i = 0; i < len; i++)
  {
    if(task == READ)
    {
      //
      //  Send dummy bytes to read out the data.
      //
      if(R_RX_PAYLOAD == reg)
      {
        g_reply_buffer[i] = spi_transfer(NOP);
      }
      else
      {
        incoming_byte = spi_transfer(NOP);
      }
    }
    else
    {
      //
      //  Send the commands to the nrf24
      //  one at the time.
      //
      spi_transfer(array[i]);
    }
  }

  //
  //  CSN HIGH - nrf24 goes back to idle mode.
  //
  _nrf24_deselect();
  _delay_us(10);

  return incoming_byte;
}

//*****************************************************************************
//
//! @brief Handles the SPI transfer in the MCU.
//!
//! This function perfroms reading and writing operations for SPI interfaces.
//!
//! @param[in] data Data byte to be transferred over SPI.
//!
//! @return SPI_ReadRxData Reply from the SPI operation.
//
//*****************************************************************************
static char
spi_transfer(char data)
{
  //
  //  Clear the transmit buffer before
  //  next reading (good practice).
  //
  SPI_ClearTxBuffer();
  SPI_ClearRxBuffer();

  //
  //  Send byte.
  //
  SPI_WriteTxData(data);

  //
  //  Wait until transmission is complete.
  //
  while(!(SPI_ReadTxStatus() & SPI_STS_SPI_DONE));

  return SPI_ReadRxData();
}

//*****************************************************************************
//
//! @brief Clear NRF24 interrupt falgs.
//!
//! This function clears up the registers holding the data falgs.
//!
//! @return None.
//
//*****************************************************************************
static void
nrf24_irq_clear(void)
{
    //
    //  Clear nrf24 interrupt flag.
    //
    g_nrf24_config[FIRST_INDEX] = IRQ_RESET;
    nrf24_spi_handler(WRITE, STATUS, g_nrf24_config, COMMAND_LENGTH);
}

//*****************************************************************************
//
//! @brief Reset the retransmission counter.
//!
//! This function overwrites the channel frecuency in order to reset the counter.
//!
//! @return None.
//
//*****************************************************************************
static void
nrf24_plos_ctn_reset(void)
{
    //
    //  Overwrite channel frecuency 2.402 GHz.
    //
    g_nrf24_config[FIRST_INDEX] = RF_CHANNEL;
    nrf24_spi_handler(WRITE, RF_CH, g_nrf24_config, COMMAND_LENGTH);
}

//*****************************************************************************
//
//! @brief Get general status register.
//!
//! This function reads the general status register and returns its content.
//!
//! @return status An interger holing the status register based on bit position.
//
//*****************************************************************************
static uint8_t nrf24_get_status(void)
{
    //
    //  Get the current status,
    //  i.e. pending data, irq, fifo...
    //
    return nrf24_spi_handler(READ, STATUS, g_nrf24_config, COMMAND_LENGTH);
}

//*****************************************************************************
//
//! @brief Get lost packets count.
//!
//! This function reads the observer tx register to get the amount of packets
//! lost since the last transmission.
//!
//! @return count Amount fo packets already lost.
//
//*****************************************************************************
uint8_t nrf24_get_lost_packets_count(void)
{
    //
    //  Get the amount of packets already lost.
    //
    return nrf24_spi_handler(READ, OBSERVE_TX, g_nrf24_config, COMMAND_LENGTH);
}

//*****************************************************************************
//
//! @brief Flush TX register.
//!
//! @return None.
//
//*****************************************************************************
static void
flush_tx(void)
{
  //
  //  CSN LOW - nrf24 starts to listen for commands.
  //
  _nrf24_select();
  _delay_us(10);

  //
  //  Flush old data.
  //
  spi_transfer(FLUSH_TX);

  //
  //  CSN HIGH - nrf24 goes back to idle.
  //
  _nrf24_deselect();
  _delay_us(10);
}

//*****************************************************************************
//
//! @brief Flush RX register.
//!
//! @return None.
//
//*****************************************************************************
static void
flush_rx(void)
{
  //
  //  CSN LOW - nrf24 starts to listen for commands.
  //
  _nrf24_select();
  _delay_us(10);

  //
  //  Flush old data.
  //
  spi_transfer(FLUSH_RX);

  //
  //  CSN HIGH - nrf24 goes back to idle.
  //
  _nrf24_deselect();
  _delay_us(10);
}

//*****************************************************************************
//
//! @brief General reset of the NRF24.
//!
//! This function flush both TX and RX register and clear the IRQ flags.
//!
//! @return None.
//
//*****************************************************************************
static void
nrf24_reset(void)
{
    flush_tx();
    flush_rx();
    nrf24_irq_clear();
}

//*****************************************************************************
//
//! @brief Power up/down the NRF24 module.
//!
//! This function reads the power control registers and modifies the power
//! state to enable payload transmission.
//!
//! @param[in] state Power state (1/0).
//!
//! @return None.
//
//*****************************************************************************
static void
pwr_state(uint8_t state)
{
  //
  //  Get register of power control.
  //
  uint8_t reg = nrf24_spi_handler(READ, CONFIG, g_nrf24_config, COMMAND_LENGTH);

  if(state)
  {
    //
    // Power up.
    //
    _set_bit(reg, PWR_UP);
  }
  else
  {
    //
    //  Power down.
    //
    _clear_bit(reg, PWR_UP);
  }

  //
  //  Send reuqest and wait for the power state to change.
  //
  g_nrf24_config[FIRST_INDEX] = reg;
  nrf24_spi_handler(WRITE, CONFIG, g_nrf24_config, COMMAND_LENGTH);
  _delay_ms(25);
}

//*****************************************************************************
//
//! @brief Set the address of the NRF24 module.
//!
//! This function write the address of the module on the TX_ADDR register.
//!
//! @param[in] id Unique identifier (address).
//!
//! @return None.
//
//*****************************************************************************
static void
set_address(uint8_t id)
{
  //
  //  Power down mode to write on registers.
  //
  pwr_state(LOW);

  //
  //  Transmit Address (PTX device only): 5 bytes.
  //  TX_ADDR: mAddress[prx_id].
  //
  nrf24_spi_handler(WRITE, TX_ADDR, g_pipe0_address[id], ADDRESS_LENGTH);

  //
  //  Return to standby mode to transmit payload .
  //
  pwr_state(HIGH);
}

//*****************************************************************************
//
//! @brief Request data to a PRX device.
//!
//! This function requests an specific variable to an PRX device in the same
//! channel.
//!
//! @param[in] var_id  ID of the variable requested to the PRX device.
//! @param[in] prx_id  Unique id of the PRX device (address).
//!
//! @return None.
//
//*****************************************************************************
static void
ptx_request(uint8_t var_id, uint8_t prx_id)
{
  //
  //  Check if the address of PRX device was already set.
  //
  if (g_last_id != id) set_address(id);

  //
  //  Attach payload to TX FIFO.
  //
  attach_tx_payload(g_var_idiable_id[var_id], strlen(g_var_idiable_id[var_id]));
  _delay_ms(10);

  //
  //  CE HIGH - Start transmitting data
  //  CE LOW - Stop transmitting data
  //
  _nrf24_enable();
  _delay_us(25);
  _nrf24_disable();

  g_last_id = id;
}

//*****************************************************************************
//
//! @brief Transmit a string.
//!
//! This function transmit a string from the PTX device to an specific PRX
//! device in the same channel.
//!
//! @param[in] data    Data to be transmitted.
//! @param[in] len     Length of the data to be transmitted.
//! @param[in] prx_id  Unique id of the PRX device (address).
//!
//! @return None.
//
//*****************************************************************************
static void
ptx_transmit_string(char *data, uint8_t len, uint8_t prx_id)
{
  //
  //  Check if the address of PRX device was already set.
  //
  if(g_last_id != id) set_address(id);

  //
  //  Attach payload to TX FIFO.
  //
  attach_tx_payload(data, len);
  _delay_ms(10);

  //
  //  CE HIGH - Start transmitting data
  //  CE LOW - Stop transmitting data
  //
  _nrf24_enable();
  _delay_us(25);
  _nrf24_disable();

  g_last_id = id;
}

//*****************************************************************************
//
//! @brief Transmit an integer.
//!
//! This function transmit an integer value from the PTX device to an specific
//! PRX device in the same channel.
//!
//! @param[in] data    Data to be transmitted.
//! @param[in] prx_id  Unique id of the PRX device (address).
//!
//! @return None.
//
//*****************************************************************************
static void
ptx_transmit_int(int data, uint8_t prx_id)
{
  //
  //  Check if the address of PRX device was already set.
  //
  if(g_last_id != id) set_address(id);

  //
  //  Attach payload to TX FIFO.
  //
  sprintf(g_value_buffer, "%d", data);
  attach_tx_payload(g_value_buffer, strlen(g_value_buffer));
  _delay_ms(10);

  //
  //  CE HIGH - Start transmitting data
  //  CE LOW - Stop transmitting data
  //
  _nrf24_enable();
  _delay_us(25);
  _nrf24_disable();

  g_last_id = id;
}

//*****************************************************************************
//
//! @brief Transmit a float.
//!
//! This function transmit a float value from the PTX device to an specific PRX
//! device in the same channel.
//!
//! @param[in] data    Data to be transmitted.
//! @param[in] prx_id  Unique id of the PRX device (address).
//!
//! @return None.
//
//*****************************************************************************
static void
ptx_transmit_float(float data, uint8_t prx_id)
{
  //
  //  Check if the address of PRX device was already set.
  //
  if(g_last_id != id) set_address(id);

  //
  //  Attach payload to TX FIFO.
  //
  sprintf(g_value_buffer, "%.2f", data);
  attach_tx_payload(g_value_buffer, strlen(g_value_buffer));
  _delay_ms(10);

  //
  //  CE HIGH - Start transmitting data
  //  CE LOW - Stop transmitting data
  //
  _nrf24_enable();
  _delay_us(25);
  _nrf24_disable();

  g_last_id = id;
}

//*****************************************************************************
//
//! @brief Attach payload to the TX FIFO register - PTX device.
//!
//! This function accesses to TX FIFO register and writes the payload on it.
//!
//! @param[in] data Payload to be written.
//! @param[in] len  Length of the payload.
//!
//! @return None.
//
//*****************************************************************************
static void
attach_tx_payload(char *data, uint8_t len)
{
  uint8_t i;

  //
  //  Used in PTX Mode.
  //  FLUSH_TX and W_TX_PAYLOAD are from the highest registers level.
  //

  //
  //  Flush old data.
  //
  flush_tx();

  //
  //  CSN LOW - nrf24 starts to listen for commands.
  //
  _nrf24_select();
  _delay_us(10);

  //
  //  Access to TX FIFO register and
  //  writes the payload to TX FIFO.
  //
  spi_transfer(W_TX_PAYLOAD);
  for(i=0; i<len; i++)
  {
      spi_transfer(*data++);
  }

  //
  //  CSN HIGH - nrf24 goes back to idle.
  //
  _nrf24_deselect();
  _delay_us(10);
}

//*****************************************************************************
//
//! @brief Attach a string as an acknowledgment payload to TX FIFO register.
//!
//! @param[in] data String to be written.
//! @param[in] len  Length of the string.
//!
//! @return None.
//
//*****************************************************************************
static void
attach_ack_string(char* data, uint8_t len)
{
  //
  //  Attach payload to TX FIFO.
  //
  attach_ack_payload(data, len);
}

//*****************************************************************************
//
//! @brief Attach an integer value as an acknowledgment payload to TX FIFO
//!        register.
//!
//! @param[in] data Integer acknowledgment to be written.
//!
//! @return None.
//
//*****************************************************************************
static void
attach_ack_int(uint8_t data)
{
  //
  //  Attach payload to TX FIFO.
  //
  sprintf(g_value_buffer, "%d", data);
  attach_ack_payload(g_value_buffer, strlen(g_value_buffer));
}

//*****************************************************************************
//
//! @brief Attach a float value as an acknowledgment payload to TX FIFO
//!        register.
//!
//! @param[in] data Float value to be written.
//!
//! @return None.
//
//*****************************************************************************
static void
attach_ack_float(float data)
{
  //
  //  Attach payload to TX FIFO.
  //
  sprintf(g_value_buffer, "%.2f", data);
  attach_ack_payload(g_value_buffer, strlen(g_value_buffer));
}

//*****************************************************************************
//
//! @brief Attach payload to the TX FIFO register - PRX device.
//!
//! This function accesses to TX FIFO register and writes the payload on it.
//!
//! @param[in] data Payload to be written.
//! @param[in] len  Length of the payload.
//!
//! @return None.
//
//*****************************************************************************
static void
attach_ack_payload(char *data, uint8_t len)
{
  uint8_t i;

  //
  //  CSN LOW - nrf24 starts to listen for commands.
  //
  _nrf24_select();
  _delay_us(10);

  //
  //  Access to TX FIFO register and
  //  writes the payload to TX FIFO.
  //
  spi_transfer(W_ACK_PAYLOAD);
  for(i=0; i<len; i++)
  {
      spi_transfer(*data++);
  }

  //
  //  CSN HIGH - nrf24 goes back to idle.
  //
  _nrf24_deselect();
  _delay_us(10);
}

//*****************************************************************************
//
//! @brief Attach payload to the TX FIFO register - PRX device.
//!
//! This function accesses to TX FIFO register and writes the payload on it.
//!
//! @param[in] data Payload to be written.
//! @param[in] len  Length of the payload.
//!
//! @return None.
//
//*****************************************************************************
static uint8_t
check_status(void)
{
    switch(g_status)
    {
        case NRF24_ACK_PAYLOAD_RECEIVED:
            nrf24_spi_handler(READ, R_RX_PAYLOAD, g_nrf24_config, NRF24_get_reply_len());
            break;
        case NRF24_TRANSMISSION_FAILED:
            return false;
            break;
    }
    return true;
}

//*****************************************************************************
//
//! @brief ISR for the NRF24 module.
//!
//! This function is triggered every time a new packet has been recieved. It
//! handles a different procedure for both PTX and PRX. In general, the ISR
//! makes sure that the data was recieved without errors and performs reset
//! operation in flag and counters to ensure proper functionality.
//!
//! @return None.
//
//*****************************************************************************
CY_ISR(nrf24_Int_Handler)
{
  uint8_t status_register = nrf24_get_status();

  if(g_ptx)
  {
    //
    //  PTX mode:
    //
    //  If the ACK packet contains a
    //  payload RX_DR IRQ is asserted.
    //
    if(status_register & (HIGH << RX_DR))
    {
      //
      //  ACK Payload Received - Transmission successful.
      //
      g_status = NRF24_ACK_PAYLOAD_RECEIVED;
    }
    else if(status_register & (HIGH << MAX_RT))
    {
      //
      //  Transmission failed.
      //
      g_status = NRF24_TRANSMISSION_FAILED;

      //
      //  Reset PLOS_CNT by overwriting to
      //  the RF_CH register.
      //
      nrf24_plos_ctn_reset();
    }
  }
  else
  {
    //
    //  PRX mode:
    //
    if(status_register & (HIGH << RX_DR))
    {
      //
      //  ACK Payload Received.
      //
      g_status = NRF24_ACK_PAYLOAD_RECEIVED;

      //
      //  Get PTX reply.
      //
      nrf24_spi_handler(READ, R_RX_PAYLOAD, g_nrf24_config, NRF24_get_reply_len());

      //
      //  Check which var_idiable was requested
      //  and attach ack payload to TX FIFO.
      //
      uint8_t num = (uint8_t)atoi(g_reply_buffer);
      attach_ack_int(g_data_int[num]);
    }
  }

  //
  //  Clear nrf24 interrupt flag.
  //
  nrf24_irq_clear();
  g_request_flag = true;

  //
  //  Clear psoc digital pin interrupt.
  //
  Pin_IRQ_ClearInterrupt();
}
