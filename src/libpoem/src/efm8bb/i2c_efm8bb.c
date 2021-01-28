/*
 * i2c_efm8bb.c
 *
 *  Created on: 10 aug. 2017
 *      Author: erlandlewin
 */
#define NDEBUG
#include <assert.h>
#include <stddef.h>

#include <SI_EFM8BB1_Register_Enums.h>

#include <poem/efm8bb/i2c_efm8bb.h>
#include <poem/error.h>
#include <poem/i2c.h>

//-----------------------------------------------------------------------------
// Pin Definitions
//-----------------------------------------------------------------------------
SI_SBIT (DISP_EN, SFR_P0, 0);          // Display Enable
#define DISP_BC_DRIVEN   0             // 0 = Board Controller drives display
#define DISP_EFM8_DRIVEN 1             // 1 = EFM8 drives display

SI_SBIT (LED1, SFR_P1, 4);             // Green LED
#define LED_ON   0
#define LED_OFF  1

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define  SYSCLK               24500000 // System clock frequency in Hz
/*
#define  SMB_FREQUENCY           10000 // Target SMBus frequency
                                       // This example supports between 10kHz
                                       // and 100kHz
*/
#define  WRITE                    0x00 // SMBus WRITE command
#define  READ                     0x01 // SMBus READ command
/*
#define  SLAVE_ADDR               0xF0 // Device addresses (7 bits,
                                       // lsb is a don't care)
*/
// Status vector - top 4 bits only
#define  SMB_SRADD                0x20 // (SR) slave address received
                                       //    (also could be a lost
                                       //    arbitration)
#define  SMB_SRSTO                0x10 // (SR) STOP detected while SR or ST,
                                       //    or lost arbitration
#define  SMB_SRDB                 0x00 // (SR) data byte received, or
                                       //    lost arbitration
#define  SMB_STDB                 0x40 // (ST) data byte transmitted
#define  SMB_STSTO                0x50 // (ST) STOP detected during a
                                       //    transaction; bus error
// End status vector definition


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
static I2Cchannel slaveChannel = NULL;

/* Only i2c slave supported for now */
Error efm8bb_i2c_slave_init( I2Cchannel i2cChannel, uint8_t address,
							 uint8_t *inputBuffer, int inputBufferSize )
{
	uint8_t TCON_save = TCON;
	/* Setup timer as clock source */
	/* Should not need clock as we are clocked by master? */

	/* TODO: If slaveChanne != NULL, return error */
	slaveChannel = i2cChannel;
	slaveChannel->slaveAddress = address;
	slaveChannel->inputBuffer = inputBuffer;
	slaveChannel->inputBufferSize = inputBufferSize;
	slaveChannel->state = EFM8BB_I2C_SLAVE_STATE_IDLE;

	// SMBCS (SMBus Clock Source Selection) = TIMER1 (Timer 1 Overflow.)
	// ENSMB (SMBus Enable) = ENABLED (Enable the SMBus module.)
	// SMBFTE (SMBus Free Timeout Detection Enable) = FREE_TO_ENABLED (Enable
	//     bus free timeouts. The bus the bus will be considered free if SCL and
	//     SDA remain high for more than 10 SMBus clock source periods.)
	// SMBTOE (SMBus SCL Timeout Detection Enable) = SCL_TO_ENABLED (Enable
	//     SCL low timeouts.)
	// EXTHOLD (SMBus Setup and Hold Time Extension Enable) = ENABLED (Enable
	//     SDA extended setup and hold times.)
	// SMB0ADM
	SMB0ADM = 0xff; // Enable automatic ACK generation, and an address mask of 0x7f
	SMB0ADR = address << 1 | 1;

	SMB0CF &= ~SMB0CF_SMBCS__FMASK;
	// This means it needs timer 1?
	// SMB0CF |= SMB0CF_SMBCS__TIMER1
	//	 | SMB0CF_ENSMB__ENABLED | SMB0CF_SMBFTE__FREE_TO_ENABLED | SMB0CF_SMBTOE__SCL_TO_ENABLED
	//	 | SMB0CF_EXTHOLD__ENABLED;
	SMB0CF |= SMB0CF_SMBCS__TIMER1
		 | SMB0CF_ENSMB__ENABLED
		 | SMB0CF_EXTHOLD__ENABLED;
	// [SMB0CF - SMBus 0 Configuration]$

	// set XBR0_SMB0E here? Currently done in pins.
	// XBR0 |= XBR0_SMB0E;

	/* Enabled SMBus ISR */
	EIE1 |= EIE1_ESMB0__ENABLED;

	// $[IE - Interrupt Enable]
	/*
	// EA (All Interrupts Enable) = ENABLED (Enable each interrupt according
	//     to its individual mask setting.)
	 */
	IE = IE_EA__ENABLED;

	// muck with timer 1
	// TODO: Generic timer handling

	//================================================================================
	// TIMER01_0_enter_DefaultMode_from_RESET
	//================================================================================
		// $[Timer Initialization]
		//Save Timer Configuration
		//Stop Timers
		TCON &= TCON_TR0__BMASK & TCON_TR1__BMASK;

		// [Timer Initialization]$

		// $[TH0 - Timer 0 High Byte]
		/*
		// TH0 (Timer 0 High Byte) = 0x38
		*/
		TH0 = (0x38 << TH0_TH0__SHIFT);
		// [TH0 - Timer 0 High Byte]$

		// $[TL0 - Timer 0 Low Byte]
		/*
		// TL0 (Timer 0 Low Byte) = 0x13
		*/
		TL0 = (0x13 << TL0_TL0__SHIFT);
		// [TL0 - Timer 0 Low Byte]$

		// $[TH1 - Timer 1 High Byte]
		/*
		// TH1 (Timer 1 High Byte) = 0x34
		*/
		TH1 = (0x34 << TH1_TH1__SHIFT);
		// [TH1 - Timer 1 High Byte]$

		// $[TL1 - Timer 1 Low Byte]
		/*
		// TL1 (Timer 1 Low Byte) = 0x34
		*/
		TL1 = (0x34 << TL1_TL1__SHIFT);
		// [TL1 - Timer 1 Low Byte]$

		// $[Timer Restoration]
		//Restore Timer Configuration
		TCON = TCON_save;

		// [Timer Restoration]$


	return NULL;
}

bool i2c_slave_service( I2Cchannel i2cChannel )
{
	switch( i2cChannel->state )
	{
		case EFM8BB_I2C_SLAVE_STATE_RECEIVED:
			return true;

		default:
			return false;
	}
}

Error i2c_slave_getInputBufferCount( const sI2Cchannel *i2c, int *count )
{
	// only valid if i2c is in states EFM8BB_I2C_SLAVE_STATE_RECEIVED,
	// EFM8BB_I2C_SLAVE_STATE_GOT_READ, and EFM8BB_I2C_SLAVE_STATE_SENDING

	*count = i2c->inputBufferCursor;

	return NULL; // TODO: Check state
}

Error i2c_slave_write( I2Cchannel i2c, int byteCount, const uint8_t *buffer )
{
	assert( i2c->state == EFM8BB_I2C_SLAVE_STATE_RECEIVED );

	i2c->outputBufferSize = byteCount;
	i2c->outputBuffer = buffer;
	i2c->outputBufferCursor = 1;
	i2c->state = EFM8BB_I2C_SLAVE_STATE_SENDING;

	// start sending
	SMB0DAT = buffer[0];
    SMB0CN0_ACK = 1;        // Send ack?
	SMB0CN0_SI = 0;                     // Clear SMBus interrupt flag

	return NULL;
}

//-----------------------------------------------------------------------------
// SMBUS0_ISR
//-----------------------------------------------------------------------------
//
// SMBUS0 ISR Content goes here. Remember to clear flag bits:
// SMB0CN0::SI (SMBus Interrupt Flag)
//
//
// SMBus ISR state machine
// - Slave only implementation - no master states defined
// - All incoming data is written to global variable <SMB_DATA_IN>
// - All outgoing data is read from global variable <SMB_DATA_OUT>
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (SMBUS0_ISR, SMBUS0_IRQn)
{
   assert( slaveChannel != NULL );

   if (SMB0CN0_ARBLOST == 0)
   {
      switch (SMB0CN0 & 0xF0)          // Decode the SMBus status vector
      {
         // Slave Receiver: Start+Address received
         case  SMB_SRADD:
            SMB0CN0_STA = 0;           // Clear SMB0CN0_STA bit
            if ((SMB0DAT & 0xFE) == ((slaveChannel->slaveAddress << 1) /* & 0xFE */)) // Decode address
            {                          // If the received address matches,
               SMB0CN0_ACK = 1;        // SMB0CN0_ACK the received slave address
               if ((SMB0DAT & 0x01) == READ) // If the transfer is a master READ,
               {
            	   if( slaveChannel->state == EFM8BB_I2C_SLAVE_STATE_RECEIVED )
            	   {
            		   if( slaveChannel->outputBufferReady )
            		   {
            			   assert( slaveChannel->outputBufferCursor == 0 );

            			   if( slaveChannel->outputBufferSize > 0 )
            			   {
            				   slaveChannel->outputBufferCursor = 1;
            				   SMB0DAT = slaveChannel->outputBuffer[0];
            				   slaveChannel->state = EFM8BB_I2C_SLAVE_STATE_SENDING;
            			   }
            			   else
            				   assert( false ); // we should never return 0 bytes to a read
            		   }
            		   else
            		   {
            			   slaveChannel->state = EFM8BB_I2C_SLAVE_STATE_GOT_READ;
            			   // don't clear SI.
            			   return;
            		   }
            	   }
            	   else
            		   assert( false ); // some error. How handle?
               }
               else
               {
            	   // TODO: if channel state is not idle, don't do the receiving,
            	   // and signal overrun error.
            	   slaveChannel->inputBufferCursor = 0;
            	   slaveChannel->state = EFM8BB_I2C_SLAVE_STATE_RECEIVING;
               }
            }
            else                       // If received slave address does not
            {                          // match,
               SMB0CN0_ACK = 0;        // NACK received address
            }
            break;

         // Slave Receiver: Data received
         case  SMB_SRDB:
         	if( slaveChannel->inputBufferCursor < slaveChannel->inputBufferSize )
         	{
         		slaveChannel->inputBuffer[ slaveChannel->inputBufferCursor ] = SMB0DAT;
         		slaveChannel->inputBufferCursor++;
         	}
            // SMB_DATA = SMB0DAT;        // Store incoming data
            // DATA_READY = 1;            // Indicate new data received
            SMB0CN0_ACK = 1;           // SMB0CN0_ACK received data

            break;

         // Slave Receiver: Stop received while either a Slave Receiver or
         // Slave Transmitter
         case  SMB_SRSTO:
            SMB0CN0_STO = 0;           // SMB0CN0_STO must be cleared by software when
                                       // a STOP is detected as a slave
            slaveChannel->state = EFM8BB_I2C_SLAVE_STATE_RECEIVED;
            break;

         // Slave Transmitter: Data byte transmitted
         case  SMB_STDB:
        	 if( slaveChannel->outputBufferCursor < slaveChannel->outputBufferSize )
        	 {
        		 SMB0DAT = slaveChannel->outputBuffer[ slaveChannel->outputBufferCursor ];
        		 // We use EHACK, so don't send explicit ACK
                 // SMB0CN0_ACK = 1;        // SMB0CN0_ACK the received slave address

        		 slaveChannel->outputBufferCursor++;
        		 if( slaveChannel->outputBufferCursor >= slaveChannel->outputBufferSize )
        			 slaveChannel->state = EFM8BB_I2C_SLAVE_STATE_IDLE;
        	 }
        	 else
        	 {
        		 // read past data. Return 0
        		 SMB0DAT = 0;
        	 }
            break;

         // Slave Transmitter: Arbitration lost, Stop detected
         //
         // This state will only be entered on a bus error condition.
         // In normal operation, the slave is no longer sending data or has
         // data pending when a STOP is received from the master, so the SMB0CN0_TXMODE
         // bit is cleared and the slave goes to the SRSTO state.
         case  SMB_STSTO:
            SMB0CN0_STO = 0;           // SMB0CN0_STO must be cleared by software when
                                       // a STOP is detected as a slave
            break;

         // Default: all other cases undefined
         default:
            SMB0CF &= ~0x80;           // Reset communication
            SMB0CF |= 0x80;
            SMB0CN0_STA = 0;
            SMB0CN0_STO = 0;
            SMB0CN0_ACK = 0;
            break;
      }
   }
   // SMB0CN0_ARBLOST = 1, Abort failed transfer
   else
   {
      SMB0CN0_STA = 0;
      SMB0CN0_STO = 0;
      SMB0CN0_ACK = 0;
      slaveChannel->state = EFM8BB_I2C_SLAVE_STATE_IDLE;
   }

   SMB0CN0_SI = 0;                     // Clear SMBus interrupt flag
}

//-----------------------------------------------------------------------------
// TIMER3_ISR
//-----------------------------------------------------------------------------
//
// TIMER3 ISR Content goes here. Remember to clear flag bits:
// TMR3CN::TF3H (Timer # High Byte Overflow Flag)
// TMR3CN::TF3L (Timer # Low Byte Overflow Flag)
//
// A Timer3 interrupt indicates an SMBus SCL low timeout.
// The SMBus is disabled and re-enabled here
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (TIMER3_ISR, TIMER3_IRQn)
{
   SMB0CF  &= ~0x80;                   // Disable SMBus
   SMB0CF  |=  0x80;                   // Re-enable SMBus
   TMR3CN0 &= ~0x80;                   // Clear Timer3 interrupt-pending flag
}



