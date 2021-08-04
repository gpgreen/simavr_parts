/*
	mcp2515_virt.h

    Copyright 2021 Greg Green <ggreen@bit-builder.com>
 */

/*
 *  A virtual MCP2515 Can controller on the SPI bus.
 *
 */

#ifndef MCP2515_VIRT_H_
#define MCP2515_VIRT_H_

#include "simavr/sim_irq.h"
#include "simavr/sim_avr.h"
#include "simavr/avr_ioport.h"
#include "simavr/avr_spi.h"

typedef struct can_msg 
{
    uint32_t id;
    uint8_t rtr : 1;
    uint8_t eid : 1;
    uint8_t dlc : 4;
    uint8_t data[8];
} can_msg_t;

typedef enum canbus_status {
    Idle,
    TxBusy,
    RxBusy,
} canbus_status_t;

typedef struct canbus 
{
    canbus_status_t status;
    can_msg_t on_wire;
} canbus_t;

enum {
	MCP2515_SPI_BYTE_IN = 0,
	MCP2515_SPI_BYTE_OUT,
    MCP2515_CS_IN,
    MCP2515_INT_OUT,
	MCP2515_CLK_OUT,
    MCP2515_RESET_IN,
    MCP2515_RX0BF_OUT,
    MCP2515_RX1BF_OUT,
    MCP2515_TX0RTS_IN,
    MCP2515_TX1RTS_IN,
    MCP2515_TX2RTS_IN,
    MCP2515_CHKBUS,
    CANBUS_BUSY_PART_TX,
    CANBUS_BUSY_PART_RX,
    CANBUS_IDLE,
	MCP2515_COUNT
};

typedef struct mcp2515_pin_t
{
	char port;
	uint8_t pin;
} mcp2515_pin_t;

// set the port to 'Z' if pin
// is not connected on part
// chip_select, interrupt are required
// to be connected
typedef struct mcp2515_wiring_t
{
    // required pins
    mcp2515_pin_t chip_select;
    mcp2515_pin_t interrupt;
    // optional pins
    mcp2515_pin_t reset;
    mcp2515_pin_t rx0bf;
    mcp2515_pin_t rx1bf;
    mcp2515_pin_t tx0rts;
    mcp2515_pin_t tx1rts;
    mcp2515_pin_t tx2rts;
} mcp2515_wiring_t;

typedef enum mcp2515_op_mode
{
    Normal = 0,
    Sleep,
    Loopback,
    ListenOnly,
    Configuration,
} mcp2515_op_mode_t;

/*
 * MCP2515 Can Controller
 * tx_msg_ready
 * bit 0 - tx buf 0 flagged for tx
 * bit 1 - tx buf 1 flagged for tx
 * bit 2 - tx buf 2 flagged for tx
 * bit 3,4,5 - which tx buf 0 to transmit first
 */
typedef struct mcp2515_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		// irq list
    uint8_t instruction;
    uint8_t address;
    uint8_t registers[256]; // all the registers for the mcp2515
    mcp2515_op_mode_t pending_opmode;  // is there an opmode change waiting for tx finish
    uint8_t quick_status;   // the data value used in read status instruction
    uint8_t rx_status;      // the data value used in rx status instruction
    uint8_t bit_mod_mask;   // mask value from bit mod instruction */
    int cs_pin;         // value of CS pin
    uint8_t spi_data_in;    // spi data received
    uint8_t spi_data_out;   // data to send on spi
    int tx_msg_ready;   // bit field set if any can messages ready to send
    int tx_reg_sending;  // which tx buffer txmitting, + 1
    int rx_read_flag;    // this is set to non-zero if read rx buffer instruction was used
    canbus_t canbus;        // data related to can bus
} mcp2515_virt_t;

extern void
mcp2515_virt_int_release(mcp2515_virt_t* part);
    
extern void
mcp2515_virt_int_enable(mcp2515_virt_t* part);

extern void
mcp2515_virt_init(struct avr_t * avr,
                  mcp2515_virt_t * p);

extern void
mcp2515_virt_connect(mcp2515_virt_t * p,
                     mcp2515_wiring_t * wiring);

#endif /* MCP2515_VIRT_H_ */
