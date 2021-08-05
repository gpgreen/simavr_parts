# simavr_parts
virtual parts for use with simavr

## MCP2515

Virtual device to emulate
![MCP2515](https://www.microchip.com/en-us/product/MCP2515) CAN
controller with SPI device. Works with ![simavr](https://github.com/buserror/simavr)

The virtual device implements all of the SPI instructions as shown in
the datasheet.

The device needs to have at least 2 pins connected in addition to the
SPI bus. One pin is the SPI 'CS' pin, the other is the interrupt pin
coming from the device. Optionally, the reset pin can be connected to
the simulated avr. Reset is active low on this device.

The device has 5 io pins that are used for triggering
transmission, signalling reception of CAN messages, or general IO. Those can also
be connected to the simulated avr. At this time, the functionality of
the virtual device for those pins is not implemented.

The device will transmit a can message when triggered by the
appropriate SPI instruction, when the bus is idle. Once the calculated
length of time has passed at the current CAN bitrate, the device will
signal completion of transmission. 

To have a message received by the device, the method
`mcp2515_receive_can_message()` can be used. This will wait for the
bus to be idle, and then put the message on the bus. After the
calculated length of time for the CAN bitrate, the device will process
CAN message reception. Depending on the state of the device
(filters), it may not respond to that particular message.

To check the current status of the CAN bus `mcp2515_canbus_status()` can be
used. The status differentiates between TxBusy and RxBusy for internal
use. An actual CAN bus will only be busy or idle.

### Current functions of part not implemented
 - Response to Reset pin
 - functionality of the RXnBF, and TXnRTS pins in either mode
 - Sleep, Loopback, ListenOnly operation modes
 - Time-Triggered protocols
 - One-shot mode
 - any type of CAN bus error, except device reception buffer overflow
 
## License
simvavr_parts is licensed under the `GNU General Public License v3.0 or later`. See [LICENSE](LICENSE) for more info.
