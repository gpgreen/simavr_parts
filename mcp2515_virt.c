/* 
 * This file is part of the simavr_parts project (https://github.com/simavr_parts).
 * Copyright 2021 Greg Green <ggreen@bit-builder.com>
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdio.h>
#include "mcp2515_virt.h"
#include "mcp2515_def.h"

// get the actual register offset for the one passed in
// this is needed because certain registers are mirrored
// across more than one address, but we only want to keep
// track of one address for that register
static uint8_t
mcp2515_get_register_offset(uint8_t reg)
{
    // for CANSTAT and CANCTRL, mask off high bits to get
    // register offset
    if ((reg & 0xF) == 0xE
        || (reg & 0xF) == 0xF)
        return reg & 0xF;
    return reg;
}

// get a pointer to the actual register for the offset passed in
// this is needed because certain registers are mirrored
// across more than one address, but we only want to keep
// track of one address for that register
static uint8_t*
mcp2515_get_register(mcp2515_virt_t* part, uint8_t reg)
{
    // for CANSTAT and CANCTRL, mask off high bits to get
    // register offset
    if ((reg & 0xF) == 0xE
        || (reg & 0xF) == 0xF)
        return &part->registers[reg & 0xF];
    return &part->registers[reg];
}

// get the current op mode
static mcp2515_op_mode_t
mcp2515_get_op_mode(mcp2515_virt_t* part)
{
    uint8_t reg_offset = mcp2515_get_register_offset(MCP_CANSTAT);
    return (mcp2515_op_mode_t)((part->registers[reg_offset] & 0xe0) >> 5);
}

// is the register allowed to be changed
static int
mcp2515_allowed_reg_modify(mcp2515_virt_t* part, uint8_t reg_offset)
{
    mcp2515_op_mode_t opmode = mcp2515_get_op_mode(part);
    
    // check targeted register to see if bit mask allowed
    switch (reg_offset) {
        // configuration and filter/mask registers
    case MCP_CNF1: case MCP_CNF2: case MCP_CNF3:
    case MCP_RXF0SIDH: case MCP_RXF1SIDH: case MCP_RXF2SIDH: case MCP_RXF3SIDH:
    case MCP_RXF4SIDH: case MCP_RXF5SIDH:
    case MCP_RXF0SIDL: case MCP_RXF1SIDL: case MCP_RXF2SIDL: case MCP_RXF3SIDL:
    case MCP_RXF4SIDL: case MCP_RXF5SIDL:
    case MCP_RXF0EID8: case MCP_RXF1EID8: case MCP_RXF2EID8: case MCP_RXF3EID8:
    case MCP_RXF4EID8: case MCP_RXF5EID8:
    case MCP_RXF0EID0: case MCP_RXF1EID0: case MCP_RXF2EID0: case MCP_RXF3EID0:
    case MCP_RXF4EID0: case MCP_RXF5EID0:
    case MCP_RXM0SIDH: case MCP_RXM1SIDH:
    case MCP_RXM0SIDL: case MCP_RXM1SIDL:
    case MCP_RXM0EID0: case MCP_RXM1EID0:
    case MCP_TXRTSCTRL:
        if (opmode == Configuration)
            return 1;
        else
            return 0;
    }
    // now check that a transmission buffer can be written
    // the TXBnCTRL.TXREQ bit must be clear to write the following:
    // the TXBnSIDH, TXBnSIDL, TXBnDLC minimum
    // if data bytes, then TXBnDm registers
    // if extended, then TXBnEIDm registers, if TXBnSIDLE.EXIDE bit set
    if (reg_offset >= MCP_TXB0SIDH && reg_offset <= MCP_TXB0D7)
    {
        uint8_t* txb0ctrl = mcp2515_get_register(part, MCP_TXB0CTRL);
        if (*txb0ctrl & (1 << MCP_TXREQ))
            return 0;
    } else if (reg_offset >= MCP_TXB1SIDH && reg_offset <= MCP_TXB1D7)
    {
        uint8_t* txb1ctrl = mcp2515_get_register(part, MCP_TXB1CTRL);
        if (*txb1ctrl & (1 << MCP_TXREQ))
            return 0;
    } else if (reg_offset >= MCP_TXB2SIDH && reg_offset <= MCP_TXB2D7)
    {
        uint8_t* txb2ctrl = mcp2515_get_register(part, MCP_TXB2CTRL);
        if (*txb2ctrl & (1 << MCP_TXREQ))
            return 0;
    }
    return 1;
}

// set the CANSTAT register based on current interrupt flags
static void
mcp2515_canstat_icod_bits(mcp2515_virt_t* part)
{
    uint8_t* canstat = mcp2515_get_register(part, MCP_CANSTAT);
    uint8_t* ireg = mcp2515_get_register(part, MCP_CANINTE);
    uint8_t* reg = mcp2515_get_register(part, MCP_CANINTF);
    uint8_t bits = 0;
    if ((*ireg & (1<<MCP_MERRE)) && (*reg & (1<<MCP_MERRF)))
        bits = 0x1;
    else if ((*ireg & (1<<MCP_WAKIE)) && (*reg & (1<<MCP_WAKIF)))
        bits = 0x2;
    else if ((*ireg & (1<<MCP_TX0IE)) && (*reg & (1<<MCP_TX0IF)))
        bits = 0x3;
    else if ((*ireg & (1<<MCP_TX1IE)) && (*reg & (1<<MCP_TX1IF)))
        bits = 0x4;
    else if ((*ireg & (1<<MCP_TX2IE)) && (*reg & (1<<MCP_TX2IF)))
        bits = 0x5;
    else if ((*ireg & (1<<MCP_RX0IE)) && (*reg & (1<<MCP_RX0IF)))
        bits = 0x6;
    else if ((*ireg & (1<<MCP_RX1IE)) && (*reg & (1<<MCP_RX1IF)))
        bits = 0x7;
    *canstat = (*canstat & 0xf1) | bits << 1;
    printf("MCP2515: canstat icod bits:0x%02x\n", bits);
}

// get the filter register offset
static int
mcp2515_filter_reg_offset(int i)
{
    if (i <= 0)
        return 0;
    if (i == 1 || i == 2)
        return i * 4;
    if (i > 2)
        return i * 2 + 4;
    return 0;
}

// hold a mask or filter extracted from mcp2515 registers
struct mcp2515_maskfilter
{
    uint32_t bits;
    uint8_t extended;
};

// copy a mask or filter from the registers
static struct mcp2515_maskfilter
mcp2515_maskfilter_copy(mcp2515_virt_t* part, int which, int mask)
{
    struct mcp2515_maskfilter copy;
    uint8_t *rxfm;
    if (mask) {
        rxfm = mcp2515_get_register(part, MCP_RXM0SIDH + 4 * which);
        copy.extended = 0;
    }
    else
        rxfm = mcp2515_get_register(part, MCP_RXF0SIDH + mcp2515_filter_reg_offset(which));
    uint32_t bits = *rxfm << 3;
    rxfm++;
    bits += (*rxfm >> 5) | ((*rxfm & 0x3) << 16);
    if (*rxfm & (1<<3))
        copy.extended = 1;
    else
        copy.extended = 0;
    rxfm++;
    bits += *rxfm << 8;
    rxfm++;
    bits += *rxfm;
    copy.bits = bits;
    return copy;
}

// find whether a mask and filter combination match on a can msg
static int
mcp2515_filter_match(mcp2515_virt_t* part, int masknum, int filternum)
{
    struct mcp2515_maskfilter mask = mcp2515_maskfilter_copy(part, masknum, 1);
    struct mcp2515_maskfilter filter = mcp2515_maskfilter_copy(part, filternum, 0);
    uint32_t match = 0;
    // extended message
    if (part->canbus.on_wire.eid) {
        if (filter.extended == 0)
            return 0;
        for (int i=0; i<29; i++)
        {
            if (!(mask.bits & (1<<i))
                || ((mask.bits & (1<<i))
                    && ((filter.bits & (1<<i)) == (part->canbus.on_wire.id & (1<<i)))))
                match |= (1<<i);
        }
        if ((match & 0x1FFFFFFF) == 0x1FFFFFFF)
            return 1;
    } else {
        // standard message
        for (int i=0; i<11; i++)
        {
            if (!(mask.bits & (1<<i))
                || ((mask.bits & (1<<i))
                    && ((filter.bits & (1<<i)) == (part->canbus.on_wire.id & (1<<i)))))
                match |= (1<<i);
        }
        uint32_t success_mask = 0x7FF;
        // if the filter is for extended, then check against data bytes
        if (filter.extended == 1) {
            for (int i=11; i<27; i++)
            {
                uint8_t byte, j;
                if (i < 19) {
                    byte = part->canbus.on_wire.data[0];
                    j = 11;
                } else {
                    byte = part->canbus.on_wire.data[1];
                    j = 19;
                }
                if (!(mask.bits & (1<<i))
                    || ((mask.bits & (1<<i))
                        && ((filter.bits & (1<<i)) == (byte & (1<<(i-j))))))
                    match |= (1<<i);
            }
            success_mask = 0x7FFFFFF;
        }
        if ((match & success_mask) == success_mask)
            return 1;
    }
    return 0;
}

// hold results of a mask/filter check on a can message
struct mab_check
{
    uint8_t masks;
    uint8_t filters;
};

// check whether an incoming message matches a mask/filter combination
static struct mab_check
mcp2515_mab_match(mcp2515_virt_t* part)
{
    struct mab_check matches = {.masks = 0, .filters = 0};
    
    if (mcp2515_filter_match(part, 0, 0)) {
        matches.masks |= (1<<0);
        matches.filters |= (1<<0);
    }
    if (mcp2515_filter_match(part, 0, 1)) {
        matches.masks |= (1<<0);
        matches.filters |= (1<<1);
    }
    if (mcp2515_filter_match(part, 1, 2)) {
        matches.masks |= (1<<1);
        matches.filters |= (1<<2);
    }
    if (mcp2515_filter_match(part, 1, 3)) {
        matches.masks |= (1<<1);
        matches.filters |= (1<<3);
    }
    if (mcp2515_filter_match(part, 1, 4)) {
        matches.masks |= (1<<1);
        matches.filters |= (1<<4);
    }
    if (mcp2515_filter_match(part, 1, 5)) {
        matches.masks |= (1<<1);
        matches.filters |= (1<<5);
    }
    return matches;
}

// a can message has been received from the canbus, do a match and move
// into receive buffers if possible
static void
mcp2515_rx_complete(mcp2515_virt_t* part)
{
    struct mab_check matches = mcp2515_mab_match(part);

    uint8_t* rxb = 0;
    int have_match = 0;
    int which_rxb = 0;
    uint8_t* caninte = mcp2515_get_register(part, MCP_CANINTE);
    uint8_t* canintf = mcp2515_get_register(part, MCP_CANINTF);
    uint8_t* eflg = mcp2515_get_register(part, MCP_EFLG);
    int iflag, enaf;
    
    // check against RXB0CTRL
    uint8_t* rxbctrl = mcp2515_get_register(part, MCP_RXB0CTRL);
    // mask/filters off, any message matches
    if ((*rxbctrl & ((1<<MCP_RXM1)|(1<<MCP_RXM0))) == ((1<<MCP_RXM1)|(1<<MCP_RXM0)))
        have_match = 1;
    // only match extended msg ids
    else if ((*rxbctrl & ((1<<MCP_RXM1)|(1<<MCP_RXM0))) == (1<<MCP_RXM1)) {
        if (part->canbus.on_wire.eid == 0)
            have_match = 0;
    }
    // only match standard msg ids
    else if ((*rxbctrl & ((1<<MCP_RXM1)|(1<<MCP_RXM0))) == (1<<MCP_RXM0)) {
        if (part->canbus.on_wire.eid == 1)
            have_match = 0;
    }
    // filter 0,1 matches?
    else if (matches.filters & 0x3)
        have_match = 1;
    if (have_match) {
        // check against hardware to see if RXB0 can accept
        if (!(*canintf & (1<<MCP_RX0IF))) {
            // copy to RXB0
            rxb = rxbctrl;
            which_rxb = 0;
            // set filter hits
            goto copy_can_msg;
        }
        else if (*rxbctrl & (1<<MCP_BUKT)) {
            // rollover to RXB1
            rxb = mcp2515_get_register(part, MCP_RXB1CTRL);
            which_rxb = 1;
            // set filter hits
            goto copy_can_msg;
        } else {
            which_rxb = 0;
            goto can_msg_overflow;
        }
    }
    // check against RXB1CTRL since couldn't do RXB0
    have_match = 0;
    rxbctrl = mcp2515_get_register(part, MCP_RXB1CTRL);
    if ((*rxbctrl & ((1<<MCP_RXM1)|(1<<MCP_RXM0))) == ((1<<MCP_RXM1)|(1<<MCP_RXM0)))
        have_match = 1;
    // only match extended msg ids
    else if ((*rxbctrl & ((1<<MCP_RXM1)|(1<<MCP_RXM0))) == (1<<MCP_RXM1)) {
        if (part->canbus.on_wire.eid == 0)
            have_match = 0;
    }
    // only match standard msg ids
    else if ((*rxbctrl & ((1<<MCP_RXM1)|(1<<MCP_RXM0))) == (1<<MCP_RXM0)) {
        if (part->canbus.on_wire.eid == 1)
            have_match = 0;
    }
    // filter 2,3,4,5 matches?
    else if (matches.filters & 0x3c)
        have_match = 1;
    if (have_match ) {
        // check against hardware to see if RXB1 can accept
        if (!(*canintf & (1<<MCP_RX1IF))) {
            // copy to RXB1
            rxb = rxbctrl;
            which_rxb = 1;
            // set filter hits
            goto copy_can_msg;
        }
        // can't copy into RXB1
        which_rxb = 1;
        goto can_msg_overflow;
    } else {
        // can't copy into either
        return;
    }

copy_can_msg:
    // set ctrl flags
    if (part->canbus.on_wire.rtr)
        *rxb |= (1<<MCP_RXRTR);
    // copy message to receive buffer
    rxb++;
    can_msg_t* can_msg = &part->canbus.on_wire;
    // RXBnSIDH
    *rxb = (can_msg->id & 0x7F8) >> 3;
    rxb++;
    // extended id
    if (can_msg->eid) {
        // RXBnSIDL
        *rxb = ((can_msg->id & 0x7) << 5) | (1<<3) | ((can_msg->id & 0x30000) >> 16);
        rxb++;
        // RXBnEID8
        *rxb = (can_msg->id & 0xFF00) >> 8;
        rxb++;
        // RXBnEID0
        *rxb = can_msg->id & 0xFF;
        rxb++;
    } else {
        // standard id
        // RXBnSIDL
        *rxb = (can_msg->id & 0x7) << 5;
        rxb += 3;
    }
    // RXBnDLC
    *rxb = (can_msg->rtr << 6) | can_msg->dlc;
    rxb++;
    // RXBnDm
    for (int i=0; i<can_msg->dlc; i++,rxb++)
        *rxb = can_msg->data[0];

    // set interrupt flag and interrupt if enabled
    iflag = which_rxb == 0 ? MCP_RX0IF : MCP_RX1IF;
    enaf = which_rxb == 0 ? MCP_RX0IE : MCP_RX1IE;
    *canintf |= (1<<iflag);
    if (*caninte & (1<<enaf)) {
        // set canstat
        mcp2515_canstat_icod_bits(part);
        // raise interrupt
        mcp2515_virt_int_enable(part);
    }
    
can_msg_overflow:
    iflag = which_rxb == 0 ? MCP_RX0OVR : MCP_RX1OVR;
    *eflg |= (1<<iflag);
    if (*caninte & (1<<MCP_ERRIE)) {
        // generate interrupt
        mcp2515_virt_int_enable(part);
    }
}

static void
mcp2515_select_tx_buf_for_tx(mcp2515_virt_t* part)
{
    // quick check
    if (part->tx_msg_ready == 0)
        return;
    
    uint8_t* regs[3];
    regs[0] = mcp2515_get_register(part, MCP_TXB0CTRL);
    regs[1] = mcp2515_get_register(part, MCP_TXB1CTRL);
    regs[2] = mcp2515_get_register(part, MCP_TXB2CTRL);
    uint8_t priorities[3];
    uint8_t max_priority = 0;
    // find the priority of each buffer set for tx and the max priority
    for (int i=0; i<3; i++) {
        // if abort flag is set, and the tx buf is not currently sending
        // clear the flags
        priorities[i] = 0;
        if (*regs[i] & (1<<MCP_ABTF)) {
            if (part->tx_reg_sending && (part->tx_reg_sending - 1) == i)
                continue;
            *regs[i] &= ((1<<MCP_TXP0)|(1<<MCP_TXP1));
            printf("MCP2515: clearing tx ctrl %d due to ABTF\n", i);
        }
        else if (*regs[i] & (1<<MCP_TXREQ))
            priorities[i] = *regs[i] & ((1<<MCP_TXP0)|(1<<MCP_TXP1));
        if (max_priority < priorities[i])
            max_priority = priorities[i];
    }
    // now find the first buffer with max priority and set for tx
    int selected = 4;
    for (int i=0; i<3; i++) {
        if ((*regs[i] & (1<<MCP_TXREQ)) && !(*regs[i] & (1<<MCP_ABTF))
            && priorities[i] == max_priority) {
            selected = i;
            break;
        }
    }
    // no buffer set for tx
    if (selected == 4) {
        part->tx_msg_ready = 0;
        return;
    }
    
    // set the bit in tx_msg_ready that says which buffer to tx first
    part->tx_msg_ready = (part->tx_msg_ready & 0x7) | (selected << 4);
    printf("MCP2515: tx buffer %d selected for highest priority transmission\n", selected);
}

static void
mcp2515_tx_complete(mcp2515_virt_t* part)
{
    // get index of transmission buffer
    int n = part->tx_reg_sending - 1;
    
    // clear TXREQ bit
    uint8_t* txbctrl = mcp2515_get_register(part, MCP_TXB0CTRL + n * 16);
    *txbctrl &= ~(1<<MCP_TXREQ);

    printf("MCP2515: tx of msg in buf %d complete\n", n);
    // remove the tx rdy bit
    part->tx_msg_ready &= ~(1<<n);
    
    // set interrupt flag bit
    uint8_t* canintf = mcp2515_get_register(part, MCP_CANINTF);
    *canintf |= (1<<(n + MCP_TX0IF));

    // check if interrupt should be raised
    uint8_t* caninte = mcp2515_get_register(part, MCP_CANINTE);
    if (*caninte & (1<<(n + MCP_TX0IE))) {
        // set canstat
        mcp2515_canstat_icod_bits(part);
        // raise interrupt
        mcp2515_virt_int_enable(part);
    }

    // extract the can message from the transmit buffers
    can_msg_t* can_msg = &part->canbus.on_wire;
    txbctrl++;
    can_msg->id = *txbctrl << 3;
    txbctrl++;
    can_msg->id += *txbctrl >> 5;
    can_msg->eid = (*txbctrl & (1<<3)) ? 1 : 0;
    if (can_msg->eid) {
        can_msg->id += (*txbctrl & 0x3) << 16;
        txbctrl++;
        can_msg->id += *txbctrl << 8;
        txbctrl++;
        can_msg->id += *txbctrl;
    }
    else
        txbctrl += 2;
    can_msg->dlc = *txbctrl & 0x4;
    can_msg->rtr = (*txbctrl & (1<<6)) ? 1 : 0;
    for (int i=0; i<8; i++) {
        txbctrl++;
        can_msg->data[i] = *txbctrl;
    }

    // print the sent can message in slcan format
    if (can_msg->eid)
        printf("can message sent: %c%08x%02x",
               can_msg->rtr ? 'R' : 'T', can_msg->id, can_msg->dlc);
    else
        printf("can message sent: %c%03x%02x", can_msg->rtr ? 'r' : 't', can_msg->id, can_msg->dlc);
    for (int i=0; i<can_msg->dlc; ++i) {
        txbctrl++;
        printf("%02x", *txbctrl);
    }
    printf("\n");
    
    // now check if more tx bufs are ready
    mcp2515_select_tx_buf_for_tx(part);
}

static uint8_t
mcp2515_txctl_write(mcp2515_virt_t* part, uint8_t reg_offset, uint8_t new_regval)
{
    // get index of transmission buffer
    int n = (reg_offset - 0x30) >> 4;
    uint8_t* reg = mcp2515_get_register(part, reg_offset);
    uint8_t curreg = *reg;
    uint8_t val = 0;
    
    // if TXREQ bit is to be set, and isn't set
    if (!(curreg & (1<<MCP_TXREQ)) && (new_regval & (1<<MCP_TXREQ))) {
        printf("MCP2515: tx buf %d flagged for tx\n", n);
        part->tx_msg_ready |= (1<<n);
        // TXREQ and priority can be set if via SPI, other bits cleared
        val = new_regval & ((1<<MCP_TXP0)|(1<<MCP_TXP1)|(1<<MCP_TXREQ));
    } else if ((curreg & (1<<MCP_TXREQ)) && !(new_regval & (1<<MCP_TXREQ))) {
        // transmission is aborted
        printf("MCP2515: tx buf %d flagged for tx abort\n", n);
        part->tx_msg_ready &= ~(1<<n);
        val = new_regval & ((1<<MCP_TXP0)|(1<<MCP_TXP1));
    }
    return val;
}

static void
mcp2515_one_shot_tx(mcp2515_virt_t* part, uint8_t new_regval)
{
    printf("MCP2515: TODO one shot transmissions\n");
}

static void
mcp2515_abort_tx(mcp2515_virt_t* part)
{
    printf("MCP2515: TODO abort transmissions\n");
}

static void
mcp2515_clockout(mcp2515_virt_t* part, uint8_t new_regval)
{
    printf("MCP2515: TODO clockout pin enable/disable\n");
}

static void
mcp2515_change_op_mode(mcp2515_virt_t* part)
{
    if (mcp2515_get_op_mode(part) == part->pending_opmode)
        return;
    
    // check and see if any messages flagged for transmission
    int txflagged = 0;
    for (int i=MCP_TXB0CTRL; i<=MCP_TXB2CTRL; i+=0x10)
    {
        uint8_t* reg = mcp2515_get_register(part, i);
        if (*reg & (1<<MCP_TXREQ))
        {
            txflagged = 1;
            break;
        }
    }
    if (txflagged)
    {
        printf("MCP2515: change opmode after transmission's complete\n");
    } else {
        // change opmode flags in CANSTAT register
        uint8_t* canstat = mcp2515_get_register(part, MCP_CANSTAT);
        *canstat = (*canstat & 0x1f) | (part->pending_opmode << 6);
        printf("MCP2515: changed op mode to: 0x%02x\n", mcp2515_get_op_mode(part));
    }
}
    
// write a register in the part
// checks to see if allowed to write first
// then takes action to alter data input, or other registers if required
// writes the resulting value in the register
static void
mcp2515_write_register(mcp2515_virt_t* part, uint8_t reg_offset, uint8_t val)
{
    int clear_interrupt = 0;
    if (!mcp2515_allowed_reg_modify(part, reg_offset))
    {
        printf("MCP2515: not allowed to write register:0x%02x\n", reg_offset);
        return;
    }
    uint8_t* reg = mcp2515_get_register(part, reg_offset);
    if (reg_offset == MCP_CANCTRL) {
        // check for abort transmissions
        if (val & (1<<MCP_ABAT))
            mcp2515_abort_tx(part);
        // op mode change
        if ((val & 0xe0) != (*reg & 0xe0))
            part->pending_opmode = (mcp2515_op_mode_t)(val>>6);
        // one shot change
        if ((val & (1<<MCP_OSM)) != (*reg & (1<<MCP_OSM)))
            mcp2515_one_shot_tx(part, val);
        // clkout setting changes
        if ((val & 0x7) != (*reg & 0x7))
            mcp2515_clockout(part, val);
    }
    // check for message transmission
    else if (reg_offset == MCP_TXB0CTRL || reg_offset == MCP_TXB1CTRL || reg_offset == MCP_TXB2CTRL)
    {
        val = mcp2515_txctl_write(part, reg_offset, val);
    }
    // check for interrupt flag clearing
    else if (reg_offset == MCP_CANINTF)
    {
        uint8_t* ireg = mcp2515_get_register(part, MCP_CANINTE);
        if ((*ireg & (1<<MCP_MERRE)) && (*reg & (1<<MCP_MERRF)) && !(val & (1<<MCP_MERRF)))
            clear_interrupt = 1;
        if ((*ireg & (1<<MCP_WAKIE)) && (*reg & (1<<MCP_WAKIF)) && !(val & (1<<MCP_WAKIF)))
            clear_interrupt = 1;
        if ((*ireg & (1<<MCP_ERRIE)) && (*reg & (1<<MCP_ERRIF)) && !(val & (1<<MCP_ERRIF)))
            clear_interrupt = 1;
        if ((*ireg & (1<<MCP_TX2IE)) && (*reg & (1<<MCP_TX2IF)) && !(val & (1<<MCP_TX2IF)))
            clear_interrupt = 1;
        if ((*ireg & (1<<MCP_TX1IE)) && (*reg & (1<<MCP_TX1IF)) && !(val & (1<<MCP_TX1IF)))
            clear_interrupt = 1;
        if ((*ireg & (1<<MCP_TX0IE)) && (*reg & (1<<MCP_TX0IF)) && !(val & (1<<MCP_TX0IF)))
            clear_interrupt = 1;
        if ((*ireg & (1<<MCP_RX1IE)) && (*reg & (1<<MCP_RX1IF)) && !(val & (1<<MCP_RX1IF)))
            clear_interrupt = 1;
        if ((*ireg & (1<<MCP_RX0IE)) && (*reg & (1<<MCP_RX0IF)) && !(val & (1<<MCP_RX0IF)))
            clear_interrupt = 1;
        // if an interrupt flag is going to be cleared, recalc the icod bits
        *reg = val;
        mcp2515_canstat_icod_bits(part);
        // now if there is still bits that are set, don't clear the interrupt
        if (clear_interrupt && (val > 0))
            clear_interrupt = 0;
    }
    *reg = val;
    // now that register(s) have been written, update transmission selection
    mcp2515_select_tx_buf_for_tx(part);
    if (clear_interrupt)
        mcp2515_virt_int_release(part);
    printf("MCP2515: write [0x%02x]=0x%02x\n", reg_offset, val);
}

// do a bit modify instruction
static void
mcp2515_bit_modify(mcp2515_virt_t* part, uint8_t reg_offset)
{
    // check targeted register to see if bit mask allowed
    switch (reg_offset) {
    case MCP_CNF1: case MCP_CNF2: case MCP_CNF3:
    case MCP_CANINTE: case MCP_CANINTF: case MCP_EFLG:
    case MCP_BFPCTRL: case MCP_TXRTSCTRL:
    case MCP_TXB0CTRL: case MCP_TXB1CTRL: case MCP_TXB2CTRL:
    case MCP_RXB0CTRL: case MCP_RXB1CTRL:
        break;
    default:
        // canctrl and canstat mapped in multiple addresses
        if ((reg_offset & 0xf) == 0xf || (reg_offset & 0xf) == 0xe)
            break;
        // if not allowed register, mask changes to 0xFF so a register
        // write instead of just specific bits
        part->bit_mod_mask = 0xFF;
        printf("MCP2515: register doesn't allow bit mods, changing to full register write\n");
        break;

    }
    uint8_t* reg = mcp2515_get_register(part, reg_offset);
    uint8_t regcur = *reg;
    printf("MCP2515: reg[0x%02x] bitmod mask:0x%02x data:0x%02x =",
           reg_offset,
           part->bit_mod_mask,
           regcur);
    for (int i=0; i<8; i++)
    {
        if (part->bit_mod_mask & 0x1)
        {
            if (part->spi_data_in & 0x1)
                regcur |= (1 << i);
            else
                regcur &= ~(1 << i);
        }
        part->bit_mod_mask >>= 1;
    }
    printf(" 0x%02x\n", regcur);
    mcp2515_write_register(part, reg_offset, regcur);
    part->address = 0;
    part->instruction = 0;
    part->bit_mod_mask = 0;
}

/*
 * Function called on 3rd or more byte received in an SPI transaction
 * the first 2 bytes of a transaction (or 1 if a single byte transaction)
 * are handled in mcp2515_virt_in_hook
 * EXCEPT in the MCP_SPI_LOAD_TX/RX case where the address byte is set
 * via the first 3 bits of the instruction byte, so that is loaded
 * into the address instead and this function called on 2nd byte
 */
static void
mcp2515_handle_spi_byte(mcp2515_virt_t* part)
{
    uint8_t reg_offset = mcp2515_get_register_offset(part->address);
    
    switch (part->instruction) {
    case MCP_SPI_READ: case MCP_SPI_READ_RX0:
        part->spi_data_out = part->registers[reg_offset];
        part->address++;
        printf("MCP2515: read [0x%02x]=0x%02x\n", reg_offset, part->spi_data_out);
        break;
    case MCP_SPI_WRITE: case MCP_SPI_LOAD_TX:
        part->spi_data_out = 0;
        mcp2515_write_register(part, reg_offset, part->spi_data_in);
        part->address++;
        break;
    case MCP_SPI_READ_STATUS:
        part->address++;
        part->spi_data_out = part->quick_status;
        printf("MCP2515: read status:0x%02x\n", part->quick_status);
        break;
    case MCP_SPI_RX_STATUS:
        part->address++;
        part->spi_data_out = part->rx_status;
        printf("MCP2515: read rx status0x%02x\n", part->rx_status);
        break;
    case MCP_SPI_BIT_MOD:
        if (part->bit_mod_mask == 0)
            part->bit_mod_mask = part->spi_data_in;
        else
            mcp2515_bit_modify(part, reg_offset);
        part->spi_data_out = 0;
        break;
    }
}

// do a part software reset
static void
mcp2515_reset(mcp2515_virt_t* part)
{
    printf("MCP2515: software reset\n");
    // set registers to zero
    memset(part->registers, 0, sizeof(part->registers));
    
    // set control registers to default values
    part->registers[MCP_CANSTAT] = 0x80;
    part->registers[MCP_CANCTRL] = 0xe7;
    
    part->instruction = 0;
    part->address = 0;
    part->pending_opmode = Configuration;
    part->quick_status = 0;
    part->rx_status = 0;
    part->bit_mod_mask = 0;
    part->spi_data_in = 0;
    part->spi_data_out = 0;
    part->tx_msg_ready = 0;
    part->tx_reg_sending = 0;
    part->rx_read_flag = 0;
    part->canbus.status = Idle;
}

static void
mcp2515_virt_checkbus_hook(struct avr_irq_t * irq,
                           uint32_t value,
                           void * param)
{
    mcp2515_virt_t * part = (mcp2515_virt_t*) param;
    if (part->canbus.status == Idle)
    {
        if (part->tx_msg_ready) {
            avr_raise_irq(part->irq + CANBUS_BUSY_PART_TX, (uint32_t)part);
        }
        else {
            mcp2515_change_op_mode(part);
        }
    }
}

// the Can bus is busy with device transmission
static void
mcp2515_virt_canbus_idle_hook(struct avr_irq_t * irq,
                              uint32_t value,
                              void * param)
{
    mcp2515_virt_t * part = (mcp2515_virt_t*) param;
    part->canbus.status = Idle;
    if (part->tx_msg_ready & 0x7)
        avr_raise_irq(part->irq + CANBUS_BUSY_PART_TX, (uint32_t)part);
    else {
        mcp2515_change_op_mode(part);
    }
}

static avr_cycle_count_t
_mcp2515_canbus_tx_finished(avr_t * avr, avr_cycle_count_t when, void * param)
{
    mcp2515_virt_t* part = (mcp2515_virt_t*)param;
    if (part->tx_reg_sending != 0) {
        mcp2515_tx_complete(part);
        part->tx_reg_sending = 0;
    }
    avr_raise_irq(part->irq + CANBUS_IDLE, (uint32_t)part);
    printf("CANBUS: can tx finished\n");
    return 0;
}

static avr_cycle_count_t
_mcp2515_canbus_rx_finished(avr_t * avr, avr_cycle_count_t when, void * param)
{
    mcp2515_virt_t* part = (mcp2515_virt_t*)param;
    mcp2515_rx_complete(part);
    avr_raise_irq(part->irq + CANBUS_IDLE, (uint32_t)part);
    printf("CANBUS: can message rx finished\n");
    return 0;
}

// the Can bus is busy with device transmission
static void
mcp2515_virt_canbus_busy_tx_hook(struct avr_irq_t * irq,
                                 uint32_t value,
                                 void * param)
{
    mcp2515_virt_t * part = (mcp2515_virt_t*) param;
    if (part->tx_msg_ready & 0x7) {
        int which_tx = part->tx_msg_ready >> 4;
        part->tx_reg_sending = which_tx + 1;
        // get duration of message transmission
        uint32_t duration_usec = 208;
        avr_cycle_timer_register_usec(part->avr, duration_usec, _mcp2515_canbus_tx_finished, part);
        part->canbus.status = TxBusy;
        printf("CANBUS: sending can message from tx buf %d\n", which_tx);
    } else {
        avr_cycle_timer_cancel(part->avr, _mcp2515_canbus_tx_finished, part);
        avr_raise_irq(part->irq + CANBUS_IDLE, (uint32_t)part);
    }
}

// the Can bus is busy with message transmission
static void
mcp2515_virt_canbus_busy_rx_hook(struct avr_irq_t * irq,
                                 uint32_t value,
                                 void * param)
{
    mcp2515_virt_t * part = (mcp2515_virt_t*) param;
    // get duration of message transmission
    uint32_t duration_usec = 208;
    avr_cycle_timer_register_usec(part->avr, duration_usec, _mcp2515_canbus_rx_finished, part);
    part->canbus.status = RxBusy;
    printf("CANBUS: can bus receiving message\n");
}

// the CS pin has changed
static void
mcp2515_virt_cs_hook(struct avr_irq_t * irq,
                     uint32_t value,
                     void * param)
{
    mcp2515_virt_t * part = (mcp2515_virt_t*) param;
    part->cs_pin = value & 0xFF;
    printf("MCP2515: chip select: 0x%02x\n", value);
    // CS pin raised
    if (part->cs_pin != 0)
    {
        part->instruction = 0;
        part->address = 0;
        // the read rx buffer instruction clears flag on raising of CS
        if (part->rx_read_flag) {
            uint8_t* reg = mcp2515_get_register(part, MCP_CANINTF);
            uint8_t value = *reg;
            if (part->rx_read_flag == 1)
                value &= ~(1<<MCP_RX0IF);
            else
                value &= ~(1<<MCP_RX1IF);
            mcp2515_write_register(part, MCP_CANINTF, value);
            part->rx_read_flag = 0;
        }
    }
}

void
mcp2515_virt_int_release(mcp2515_virt_t* part)
{
    avr_raise_irq(part->irq + MCP2515_INT_OUT, 1);
    printf("MCP2515: int released\n");
}
    
void
mcp2515_virt_int_enable(mcp2515_virt_t* part)
{
    avr_raise_irq(part->irq + MCP2515_INT_OUT, 0);
    printf("MCP2515: int enabled\n");
}

/*
 * Called when a SPI byte is sent out by part
 */
static void
mcp2515_virt_out_hook(struct avr_irq_t * irq,
                    uint32_t value,
                    void * param)
{
    mcp2515_virt_t * part = (mcp2515_virt_t*)param;
    printf("MCP2515: spi byte out 0x%02x\n", part->spi_data_out);
}

/*
 * Called when a SPI byte is received by part
 */
static void
mcp2515_virt_in_hook(struct avr_irq_t * irq,
                    uint32_t value,
                    void * param)
{
    uint8_t status, reg_offset, masked_instruction;
    mcp2515_virt_t * part = (mcp2515_virt_t*)param;

    // chip select should be pulled low to enable
    if (part->cs_pin)
        return;

    part->spi_data_in = value & 0xFF;
    printf("MCP2515: spi data in: 0x%02x\n", part->spi_data_in);
    
    // do something with this data
    if (part->instruction == 0) {
        part->instruction = part->spi_data_in;
        if (part->instruction == MCP_SPI_RESET)
            mcp2515_reset(part);
        else {
            masked_instruction = part->instruction & 0xF8;
            if (masked_instruction == MCP_SPI_LOAD_TX) {
                switch (part->instruction & 0x7) {
                case 0:
                    part->address = MCP_TXB0SIDH;
                    break;
                case 1:
                    part->address = MCP_TXB0D0;
                    break;
                case 2:
                    part->address = MCP_TXB1SIDH;
                    break;
                case 3:
                    part->address = MCP_TXB1D0;
                    break;
                case 4:
                    part->address = MCP_TXB2SIDH;
                    break;
                case 5:
                    part->address = MCP_TXB2D0;
                    break;
                }
                printf("MCP2515: load tx start address:0x%02x\n", part->address);
            }
            else if (masked_instruction == MCP_SPI_READ_RX0) {
                part->spi_data_out = 0;
                switch (part->instruction & 0x6) {
                case 0:
                    part->address = MCP_RXB0SIDH;
                    part->rx_read_flag = 1;
                    break;
                case 2:
                    part->address = MCP_RXB0D0;
                    part->rx_read_flag = 1;
                    break;
                case 4:
                    part->address = MCP_RXB1SIDH;
                    part->rx_read_flag = 2;
                    break;
                case 5:
                    part->address = MCP_RXB1D0;
                    part->rx_read_flag = 2;
                    break;
                }
                printf("MCP2515: read rx start address:0x%02x\n", part->address);
            }
            else if (masked_instruction == MCP_SPI_RTS) {
                part->spi_data_out = 0;
                uint8_t txbuffers = part->instruction & 0x7;
                printf("MCP2515: rts buffers:0x%02x\n", txbuffers);
                uint8_t val;
                if (txbuffers & 0x1) {
                    val = mcp2515_txctl_write(part, MCP_TXB0CTRL, (1<<MCP_TXREQ));
                    *mcp2515_get_register(part, MCP_TXB0CTRL) = val;
                }
                if (txbuffers & 0x2) {
                    val = mcp2515_txctl_write(part, MCP_TXB1CTRL, (1<<MCP_TXREQ));
                    *mcp2515_get_register(part, MCP_TXB1CTRL) = val;
                }
                if (txbuffers & 0x4) {
                    val = mcp2515_txctl_write(part, MCP_TXB2CTRL, (1<<MCP_TXREQ));
                    *mcp2515_get_register(part, MCP_TXB2CTRL) = val;
                }
                mcp2515_select_tx_buf_for_tx(part);
                part->instruction = 0;
            }
        }
        part->spi_data_out = 0;
    } else if (part->instruction > 0 && part->address == 0) {
        switch (part->instruction) {
        case MCP_SPI_READ: case MCP_SPI_WRITE:
            part->address = part->spi_data_in;
            part->spi_data_out = 0;
            break;
        case MCP_SPI_READ_RX0: case MCP_SPI_READ_RX1:
            printf("MCP2515: **WARNING** should not happen\n");
            break;
        case MCP_SPI_READ_STATUS:
            /* 0=CANINTF.RX0IF
               1=CANINTF.RX1IF
               2=TXB0CNTRL.TXREQ
               3=CANINTF.TX0IF
               4=TXB1CNTRL.TXREQ
               5=CANINTF.TX1IF
               6=TXB2CNTRL.TXREQ
               7=CANINTF.TX2IF
            */
            reg_offset = mcp2515_get_register_offset(MCP_CANINTF);
            status = 0;
            if (part->registers[reg_offset] & (1<<MCP_RX0IF))
                status |= (1 << MCP_CANINTF_RX0IF);
            if (part->registers[reg_offset] & (1<<MCP_RX1IF))
                status |= (1 << MCP_CANINTF_RX1IF);
            if (part->registers[reg_offset] & (1<<MCP_TX0IF))
                status |= (1 << MCP_CANINTF_TX0IF);
            if (part->registers[reg_offset] & (1<<MCP_TX1IF))
                status |= (1 << MCP_CANINTF_TX1IF);
            if (part->registers[reg_offset] & (1<<MCP_TX2IF))
                status |= (1 << MCP_CANINTF_TX2IF);
            reg_offset = mcp2515_get_register_offset(MCP_TXB0CTRL);
            if (part->registers[reg_offset] & (1<<MCP_TXREQ))
                status |= (1 << MCP_TXB0CNTRL_TXREQ);
            reg_offset = mcp2515_get_register_offset(MCP_TXB1CTRL);
            if (part->registers[reg_offset] & (1<<MCP_TXREQ))
                status |= (1 << MCP_TXB1CNTRL_TXREQ);
            reg_offset = mcp2515_get_register_offset(MCP_TXB2CTRL);
            if (part->registers[reg_offset] & (1<<MCP_TXREQ))
                status |= (1 << MCP_TXB2CNTRL_TXREQ);
            part->quick_status = status;
            part->address = 1;
            part->spi_data_out = status;
            printf("MCP2515: read status:0x%02x\n", status);
            break;
        case MCP_SPI_RX_STATUS:
            reg_offset = mcp2515_get_register_offset(MCP_CANINTF);
            status = 0;
            if (part->registers[reg_offset] & (1<<MCP_RX0IF))
                status |= (1 << MCP_MSG_RXB0);
            if (part->registers[reg_offset] & (1<<MCP_RX1IF))
                status |= (1 << MCP_MSG_RXB1);
            // BUGBUG: do the rest of the matches here
            part->rx_status = status;
            part->address = 1;
            part->spi_data_out = status;
            printf("MCP2515: read rx status:0x%02x\n", status);
            break;
        case MCP_SPI_BIT_MOD:
            part->address = part->spi_data_in;
            part->spi_data_out = 0;
            break;
        default:
            part->spi_data_out = 0;
            part->address = 0;
            break;
        }
    } else {
        mcp2515_handle_spi_byte(part);
    }
    avr_raise_irq(part->irq + MCP2515_SPI_BYTE_OUT, part->spi_data_out);
    avr_raise_irq(part->irq + MCP2515_CHKBUS, (uint32_t)part);
}

static const char * _mcp2515_irq_names[MCP2515_COUNT] = {
	[MCP2515_SPI_BYTE_IN] = "8>mcp2515.in",
	[MCP2515_SPI_BYTE_OUT] = "8<mcp2515.out",
	[MCP2515_CS_IN] = ">mcp2515.cs",
	[MCP2515_INT_OUT] = "<mcp2515.int",
	[MCP2515_CLK_OUT] = "<mcp2515.clk",
    [MCP2515_RESET_IN] = ">mcp2515.reset",
    [MCP2515_RX0BF_OUT] = ">mcp2515.rx0bf",
    [MCP2515_RX1BF_OUT] = ">mcp2515.rx1bf",
    [MCP2515_TX0RTS_IN] = "<mcp2515.tx0rts",
    [MCP2515_TX1RTS_IN] = "<mcp2515.tx1rts",
    [MCP2515_TX2RTS_IN] = "<mcp2515.tx2rts",
    [MCP2515_CHKBUS] = "=mcp2515.checkbus",
    [CANBUS_BUSY_PART_TX] = "=canbus.busy_part_tx",
    [CANBUS_BUSY_PART_RX] = "=canbus.busy_part_rx",
    [CANBUS_IDLE] = "=canbus.idle",
};

/*
 * Initialise the MCP2515 virtual part. This should be called before anything else.
 */
void
mcp2515_virt_init(struct avr_t * avr,
                  mcp2515_virt_t * part)
{
	memset(part, 0, sizeof(mcp2515_virt_t));
    
	part->avr = avr;

	part->irq = avr_alloc_irq(&avr->irq_pool, 0, MCP2515_COUNT, _mcp2515_irq_names);
	avr_irq_register_notify(part->irq + MCP2515_SPI_BYTE_IN, mcp2515_virt_in_hook, part);
	avr_irq_register_notify(part->irq + MCP2515_SPI_BYTE_OUT, mcp2515_virt_out_hook, part);
	avr_irq_register_notify(part->irq + MCP2515_CS_IN, mcp2515_virt_cs_hook, part);
	avr_irq_register_notify(part->irq + MCP2515_CHKBUS, mcp2515_virt_checkbus_hook, part);
	avr_irq_register_notify(part->irq + CANBUS_BUSY_PART_TX, mcp2515_virt_canbus_busy_tx_hook, part);
	avr_irq_register_notify(part->irq + CANBUS_BUSY_PART_RX, mcp2515_virt_canbus_busy_rx_hook, part);
	avr_irq_register_notify(part->irq + CANBUS_IDLE, mcp2515_virt_canbus_idle_hook, part);

    part->canbus.status = Idle;
    
    mcp2515_reset(part);
    // set the interrupt pin to default high, because external pull-up
//    AVR_IOCTL_IOPORT_SET_EXTERNAL('B')
}

/*
 *  "Connect" the IRQs of the MCP2515 to the SPI master of the AVR.
 */
void
mcp2515_virt_connect(mcp2515_virt_t * part,
                     mcp2515_wiring_t * wiring)
{
	avr_connect_irq(
		part->irq + MCP2515_SPI_BYTE_OUT,
        avr_io_getirq (part->avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT));
	avr_connect_irq(
		avr_io_getirq (part->avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT),
		part->irq + MCP2515_SPI_BYTE_IN);
	avr_connect_irq(
		avr_io_getirq (part->avr, AVR_IOCTL_IOPORT_GETIRQ(wiring->chip_select.port), wiring->chip_select.pin),
		part->irq + MCP2515_CS_IN);
	avr_connect_irq(
        part->irq + MCP2515_INT_OUT,
		avr_io_getirq (part->avr, AVR_IOCTL_IOPORT_GETIRQ(wiring->interrupt.port), wiring->interrupt.pin));

    // make sure the interrupt pin is high
    avr_raise_irq(part->irq + MCP2515_INT_OUT, 1);

    if (wiring->reset.port != 'Z')
    {
        avr_connect_irq(
            avr_io_getirq (part->avr, AVR_IOCTL_IOPORT_GETIRQ(wiring->reset.port), wiring->reset.pin),
            part->irq + MCP2515_RESET_IN);
    }
    if (wiring->rx0bf.port != 'Z')
    {
        avr_connect_irq(
            part->irq + MCP2515_RX0BF_OUT,
            avr_io_getirq (part->avr, AVR_IOCTL_IOPORT_GETIRQ(wiring->rx0bf.port), wiring->rx0bf.pin));
    }
    if (wiring->rx1bf.port != 'Z')
    {
        avr_connect_irq(
            part->irq + MCP2515_RX1BF_OUT,
            avr_io_getirq (part->avr, AVR_IOCTL_IOPORT_GETIRQ(wiring->rx1bf.port), wiring->rx1bf.pin));
    
    }
    if (wiring->tx0rts.port != 'Z')
    {
        avr_connect_irq(
            avr_io_getirq (part->avr, AVR_IOCTL_IOPORT_GETIRQ(wiring->tx0rts.port), wiring->tx0rts.pin),
            part->irq + MCP2515_TX0RTS_IN);
    }
    if (wiring->tx1rts.port != 'Z')
    {
        avr_connect_irq(
            avr_io_getirq (part->avr, AVR_IOCTL_IOPORT_GETIRQ(wiring->tx1rts.port), wiring->tx1rts.pin),
            part->irq + MCP2515_TX1RTS_IN);
    }
    if (wiring->tx2rts.port != 'Z')
    {
        avr_connect_irq(
            avr_io_getirq (part->avr, AVR_IOCTL_IOPORT_GETIRQ(wiring->tx2rts.port), wiring->tx2rts.pin),
            part->irq + MCP2515_TX2RTS_IN);
    
    }
}
