# can-cia-611-1-poc
PoC for CAN CiA 611-1 CC/FD tunneling

* implementation of CC/FD tunneling over CAN XL
* for discussion about the proposed SDTs 0x06 and 0x07

### Files

* ccfd2xl : convert CC/FD to CAN XL (switchable SDT 0x03 or 0x06/0x07)
* xl2ccfd : convert CAN XL to CC/FD (supports SDT 0x03/0x06/0x07)
* canxlrcv : display CAN CC/FD/XL traffic
* create_canxl_vcans.sh : script to create virtual CAN XL interfaces

### Build the tools

* Just type 'make' to build the tools.
* 'make install' would install the tools in /usr/local/bin (optional)

### Run the PoC

* download and install the latest can-utils : https://github.com/linux-can/can-utils
* create virtual CAN XL interfaces with `./create_canxl_vcans.sh start`
  * vcan0 .. vcan3 : virtual CAN bus for CAN CC/FD traffic
  * vcan4 .. vcan6 : virtual CAN bus for CAN CC/FD/XL traffic

* open 4 terminals to run different tools
  1. `cangen vcan0 -m -8`
  2. `./ccfd2xl vcan0 vcan4 -v` OR `./ccfd2xl vcan0 vcan4 -v -3`
  3. `./xl2ccfd vcan4 vcan1 -v`
  4. `./canxlrcv any | grep -v vcan4`

In terminal 4 you should see the identical CAN CC/FD traffic on vcan0 and vcan1.
