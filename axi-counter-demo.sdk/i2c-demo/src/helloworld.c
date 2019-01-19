/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xiic.h"
#include "xparameters.h"
#include "xiic.h"
#include "xil_io.h"
#include "xil_printf.h"

/************************** Constant Definitions *****************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define IIC_BASE_ADDRESS	XPAR_IIC_0_BASEADDR

/*
 * The following constant defines the address of the IIC Slave device on the
 * IIC bus. Note that since the address is only 7 bits, this constant is the
 * address divided by 2.
 * The 7 bit IIC Slave address of the IIC EEPROM on the ML300/ML310/ML403/ML410/
 * ML501/ML505/ML507/ML510 boards is 0x50. The 7 bit IIC Slave address of the
 * IIC EEPROM on the ML605/SP601/SP605 boards is 0x54.
 * Please refer the User Guide's of the respective boards for further
 * information about the IIC slave address of IIC EEPROM's.
 */
#define CAM_ADDR	60	 /* 0xA0 as an 8 bit number */

/*
 * The page size determines how much data should be written at a time.
 * The ML300 board supports a page size of 32 and 16
 * The write function should be called with this as a maximum byte count.
 */
#define PAGE_SIZE	16

/*
 * The Starting address in the IIC EEPROM on which this test is performed
 */
#define EEPROM_TEST_START_ADDRESS	128


/**************************** Type Definitions *******************************/

/*
 * The AddressType for ML300/ML310/ML510 boards should be u16 as the address
 * pointer in the on board EEPROM is 2 bytes.
 * The AddressType for ML403/ML501/ML505/ML507/ML605/SP601/SP605 boards should
 * be u8 as the address pointer in the on board EEPROM is 1 bytes.
 */
typedef u8 AddressType;


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/

int IicLowLevelEeprom();

int ReadWriteVerify(AddressType Address);

unsigned EepromWriteByte(AddressType Address, u8 *BufferPtr, u16 ByteCount);

unsigned EepromReadByte(AddressType Address, u8 *BufferPtr, u16 ByteCount);

unsigned camWrite(u8 Address, u8 value);
unsigned camRead(u8 Address, u8 *BufferPtr, u16 ByteCount);
unsigned camWrite2(u8 chip_addr, u8 Address, u8 value);


/************************** Variable Definitions **************************/

int ErrorCount;			  /* The Error Count */

u8 WriteBuffer[PAGE_SIZE];	  /* Write buffer for writing a page */
u8 ReadBuffer[PAGE_SIZE];	  /* Read buffer for reading a page */
u8 ReadBufferAll[PAGE_SIZE * 4];  /* Buffer used for reading all the data */

u8 cami2cAddr;		  /* Variable for storing Eeprom IIC address */
XIic iic;

int main()
{
    init_platform();

    print("Hello World\n\r");
    u8 buf[2];
    unsigned bytes;
    int status;

    // Clear out the buf so we can see if it gets filled
    buf[0] = 0;
    buf[1] = 0;

    // Init i2c
    cami2cAddr = 48;
    status = XIic_Initialize(&iic, XPAR_AXI_IIC_0_DEVICE_ID);

    bytes = camWrite(0xFF, 0x01);	// Unlock the 2nd reg page

    camRead(0x0A, buf, 1);	// hex 26
    camWrite(0xFF, 0x01);
    camRead(0x0B, buf, 1);	// hex 42 but should be 41


    camRead(0x08, buf, 1);	// should be 0x40
    camWrite(0x08, 0x11);
    camRead(0x08, buf, 1);	// should be 0x11


    cleanup_platform();
    return 0;
}



unsigned camWrite(u8 Address, u8 value){
	volatile unsigned SentByteCount;
	u8 buf[2];

	buf[0] = Address;
	buf[1] = value;

	SentByteCount = XIic_Send(IIC_BASE_ADDRESS,
						cami2cAddr,
						buf, 2,
						XIIC_STOP);
	return SentByteCount;
}

unsigned camWrite2(u8 chip_addr, u8 Address, u8 value){
	volatile unsigned SentByteCount;
	u8 buf[2];

	buf[0] = Address;
	buf[1] = value;

	SentByteCount = XIic_Send(IIC_BASE_ADDRESS,
						chip_addr,
						buf, 2,
						XIIC_STOP);
	return SentByteCount;
}

unsigned camRead(u8 Address, u8 *BufferPtr, u16 ByteCount){
	volatile unsigned ReceivedByteCount;
	u16 StatusReg;

	/*
	 * Set the address register to the specified address by writing
	 * the address to the device, this must be tried until it succeeds
	 * because a previous write to the device could be pending and it
	 * will not ack until that write is complete.
	 */
//	do {
//		StatusReg = XIic_ReadReg(IIC_BASE_ADDRESS, XIIC_SR_REG_OFFSET);
//		if(!(StatusReg & XIIC_SR_BUS_BUSY_MASK)) {
//			ReceivedByteCount = XIic_Send(IIC_BASE_ADDRESS,
//							cami2cAddr,
//							(u8 *)&Address,
//							sizeof(Address),
//							XIIC_STOP);
//
//			if (ReceivedByteCount != sizeof(Address)) {
//
//				/* Send is aborted so reset Tx FIFO */
//				XIic_WriteReg(IIC_BASE_ADDRESS,
//						XIIC_CR_REG_OFFSET,
//						XIIC_CR_TX_FIFO_RESET_MASK);
//				XIic_WriteReg(IIC_BASE_ADDRESS,
//						XIIC_CR_REG_OFFSET,
//						XIIC_CR_ENABLE_DEVICE_MASK);
//			}
//		}
//
//	} while (ReceivedByteCount != sizeof(Address));

	XIic_Send(IIC_BASE_ADDRESS,
								cami2cAddr,
								(u8 *)&Address,
								sizeof(Address),
								XIIC_STOP); // XIIC_REPEATED_START

	/*
	 * Read the number of bytes at the specified address from the EEPROM.
	 */
	ReceivedByteCount = XIic_Recv(IIC_BASE_ADDRESS, cami2cAddr,
					BufferPtr, ByteCount, XIIC_STOP);

	/*
	 * Return the number of bytes read from the EEPROM.
	 */
	return ReceivedByteCount;
}


/*****************************************************************************/
/**
* The function uses the low level driver of IIC to read and write to the
* IIC EEPROM board. The addresses tested are from 128 to 192.
*
* @param	None.
*
* @return	XST_SUCCESS if successful, XST_FAILURE if unsuccessful.
*
* @note		None.
*
****************************************************************************/
int IicLowLevelEeprom()
{
	int Status;
	unsigned BytesRead;
	cami2cAddr = CAM_ADDR;

	/*
	 * Read, write and verify a page of data at the specified address.
	 */
	Status = ReadWriteVerify(EEPROM_TEST_START_ADDRESS);
	if (Status != XST_SUCCESS) {
		ErrorCount++;
	}

	if (ErrorCount != 0x0) {
		Status = XST_FAILURE;
	} else {
		Status = XST_SUCCESS;
	}

	return Status;
}

/*****************************************************************************/
/**
* This function writes, reads, and verifies the read to the IIC EEPROM.  It
* does the write as a single page write, performs a buffered read, and also
* performs byte reads.
*
* @param	Address is the starting address of the page in the EEPROM device
*		to which the data is to be written.
*
* @return	 XST_FAILURE if the test fails, XST_SUCCESS if the test passes.
*
* @note 	None.
*
****************************************************************************/
int ReadWriteVerify(AddressType Address)
{
	unsigned BytesWritten;
	unsigned BytesRead;
	int Index;

	/*
	 * Initialize the data to written and the read buffer.
	 */
	for (Index = 0; Index < PAGE_SIZE; Index++) {
		WriteBuffer[Index] = Index;
		ReadBuffer[Index] = 0;
	}

	/*
	 * Write to the EEPROM.
	 */
	BytesWritten = EepromWriteByte(Address, WriteBuffer, PAGE_SIZE);
	if (BytesWritten != PAGE_SIZE) {
		return XST_FAILURE;
	}

	/*
	 * Read from the EEPROM.
	 */
	BytesRead = EepromReadByte(Address, ReadBuffer, PAGE_SIZE);
	if (BytesRead != PAGE_SIZE) {
		return XST_FAILURE;
	}

	/*
	 * Verify the data read against the data written.
	 */
	for (Index = 0; Index < PAGE_SIZE; Index++)
	{
		if (ReadBuffer[Index] != WriteBuffer[Index]) {
			return XST_FAILURE;
		}
		ReadBuffer[Index] = 0;
	}

	/*
	 * Read each byte one at a time and verify.
	 */
	for (Index = 0; Index < PAGE_SIZE; Index++)
	{
		BytesRead = EepromReadByte(Address + Index,
				&ReadBuffer[Index], 1);
		if (BytesRead != 1) {
			return XST_FAILURE;
		}

		if (ReadBuffer[Index] != WriteBuffer[Index]) {
			return XST_FAILURE;
		}
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* This function writes a buffer of bytes to the IIC serial EEPROM.
*
* @param	Address contains the address in the EEPROM to write to.
* @param	BufferPtr contains the address of the data to write.
* @param	ByteCount contains the number of bytes in the buffer to be written.
*		Note that this should not exceed the page size of the EEPROM as
*		noted by the constant PAGE_SIZE.
*
* @return	The number of bytes written, a value less than that which was
*		specified as an input indicates an error.
*
* @note		None.
*
****************************************************************************/



unsigned EepromWriteByte(AddressType Address, u8 *BufferPtr, u16 ByteCount)
{
	volatile unsigned SentByteCount;
	volatile unsigned AckByteCount;
	u8 WriteBuffer[sizeof(Address) + PAGE_SIZE];
	int Index;


	/*
	 * A temporary write buffer must be used which contains both the address
	 * and the data to be written, put the address in first based upon the
	 * size of the address for the EEPROM.
	 */
	if (sizeof(AddressType) == 2) {
		WriteBuffer[0] = (u8)(Address >> 8);
		WriteBuffer[1] = (u8)(Address);
	} else if (sizeof(AddressType) == 1) {
		WriteBuffer[0] = (u8)(Address);
		cami2cAddr |= (EEPROM_TEST_START_ADDRESS >> 8) & 0x7;
	}

	/*
	 * Put the data in the write buffer following the address.
	 */
	for (Index = 0; Index < ByteCount; Index++) {
		WriteBuffer[sizeof(Address) + Index] = BufferPtr[Index];
	}

	/*
	 * Set the address register to the specified address by writing
	 * the address to the device, this must be tried until it succeeds
	 * because a previous write to the device could be pending and it
	 * will not ack until that write is complete.
	 */
	do {
		SentByteCount = XIic_Send(IIC_BASE_ADDRESS,
					cami2cAddr,
					(u8 *)&Address, sizeof(Address),
					XIIC_STOP);
		if (SentByteCount != sizeof(Address)) {

			/* Send is aborted so reset Tx FIFO */
			XIic_WriteReg(IIC_BASE_ADDRESS,  XIIC_CR_REG_OFFSET,
					XIIC_CR_TX_FIFO_RESET_MASK);
			XIic_WriteReg(IIC_BASE_ADDRESS, XIIC_CR_REG_OFFSET,
					XIIC_CR_ENABLE_DEVICE_MASK);
		}

	} while (SentByteCount != sizeof(Address));

	/*
	 * Write a page of data at the specified address to the EEPROM.
	 */
	SentByteCount = XIic_Send(IIC_BASE_ADDRESS, cami2cAddr,
				  WriteBuffer, sizeof(Address) + PAGE_SIZE,
				  XIIC_STOP);

	/*
	 * Wait for the write to be complete by trying to do a write and
	 * the device will not ack if the write is still active.
	 */
	do {
		AckByteCount = XIic_Send(IIC_BASE_ADDRESS, cami2cAddr,
					(u8 *)&Address, sizeof(Address),
					XIIC_STOP);
		if (AckByteCount != sizeof(Address)) {

			/* Send is aborted so reset Tx FIFO */
			XIic_WriteReg(IIC_BASE_ADDRESS,  XIIC_CR_REG_OFFSET,
					XIIC_CR_TX_FIFO_RESET_MASK);
			XIic_WriteReg(IIC_BASE_ADDRESS, XIIC_CR_REG_OFFSET,
					XIIC_CR_ENABLE_DEVICE_MASK);
		}

	} while (AckByteCount != sizeof(Address));


	/*
	 * Return the number of bytes written to the EEPROM
	 */
	return SentByteCount - sizeof(Address);
}

/*****************************************************************************/
/**
* This function reads a number of bytes from the IIC serial EEPROM into a
* specified buffer.
*
* @param	Address contains the address in the EEPROM to read from.
* @param	BufferPtr contains the address of the data buffer to be filled.
* @param	ByteCount contains the number of bytes in the buffer to be read.
*		This value is not constrained by the page size of the device
*		such that up to 64K may be read in one call.
*
* @return	The number of bytes read. A value less than the specified input
*		value indicates an error.
*
* @note		None.
*
****************************************************************************/
unsigned EepromReadByte(AddressType Address, u8 *BufferPtr, u16 ByteCount)
{
	volatile unsigned ReceivedByteCount;
	u16 StatusReg;

	/*
	 * Set the address register to the specified address by writing
	 * the address to the device, this must be tried until it succeeds
	 * because a previous write to the device could be pending and it
	 * will not ack until that write is complete.
	 */
	do {
		StatusReg = XIic_ReadReg(IIC_BASE_ADDRESS, XIIC_SR_REG_OFFSET);
		if(!(StatusReg & XIIC_SR_BUS_BUSY_MASK)) {
			ReceivedByteCount = XIic_Send(IIC_BASE_ADDRESS,
							cami2cAddr,
							(u8 *)&Address,
							sizeof(Address),
							XIIC_STOP);

			if (ReceivedByteCount != sizeof(Address)) {

				/* Send is aborted so reset Tx FIFO */
				XIic_WriteReg(IIC_BASE_ADDRESS,
						XIIC_CR_REG_OFFSET,
						XIIC_CR_TX_FIFO_RESET_MASK);
				XIic_WriteReg(IIC_BASE_ADDRESS,
						XIIC_CR_REG_OFFSET,
						XIIC_CR_ENABLE_DEVICE_MASK);
			}
		}

	} while (ReceivedByteCount != sizeof(Address));

	/*
	 * Read the number of bytes at the specified address from the EEPROM.
	 */
	ReceivedByteCount = XIic_Recv(IIC_BASE_ADDRESS, cami2cAddr,
					BufferPtr, ByteCount, XIIC_STOP);

	/*
	 * Return the number of bytes read from the EEPROM.
	 */
	return ReceivedByteCount;
}
