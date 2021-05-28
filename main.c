// SYSC 3310 PROJECT
// NAME: OLUWASEYI SEHINDE-IBINI
// STUDENT #: 101092822

#include "msp.h"

static volatile int count = 0;

// function to increment state
void increment_state(){
	count = (count + 1) % 4;
}

// function to decrement state
void decrement_state(){
    if (count - 1 < 0)
    {
        count = 3;
    }
    else
    {
        count = (count - 1) % 4;
    }
	}

void change_state(){
	if(count == 0)
	{
		P1->OUT &= (uint8_t)(~(1<<0));
		P2->OUT &= (uint8_t)(~((1<<0)|(1<<1)|(1<<2)));
		EUSCI_A0->TXBUF='0';
	}
	else if(count == 1)
	{
		P1->OUT &= (uint8_t)(~((1<<0)));
		P2->OUT |= (uint8_t)((1<<0)|(1<<1)|(1<<2));
		EUSCI_A0->TXBUF='1';
	}
	else if(count == 2)
	{
		P1->OUT |= (uint8_t)((1<<0));
		P2->OUT &= (uint8_t)(~((1<<0)|(1<<1)|(1<<2)));
		EUSCI_A0->TXBUF='2';
	}
	else if(count == 3)
	{
		P1->OUT |= (uint8_t)((1<<0));
		P2->OUT |= (uint8_t)((1<<0)|(1<<1)|(1<<2));
		EUSCI_A0->TXBUF='3';
	}
}

/* Initializing the GPIO Pins */
void init_gpio(){
	
    //Set SEL0 and SEL1 to 0 to confirm its set to 0 for P1 and P2
    P1SEL0&=~(uint8_t)((1<<0)|(1<<1)|(1<<4));
    P1SEL1&=~(uint8_t)((1<<0)|(1<<1)|(1<<4));
		
    P2SEL0&=~(uint8_t)((1<<0)|(1<<1)|(1<<2));
    P2SEL1&=~(uint8_t)((1<<0)|(1<<1)|(1<<2));

    
    P1->REN |= (uint8_t)(((1<<1))); // Enable pull resistor
    P1->OUT |= (uint8_t)(((1<<1))); // Setting the pull direction

    
    P1->REN |= (uint8_t)(((1<<4))); // Enable pull resistor
    P1->OUT |= (uint8_t)(((1<<4))); // Setting the pull direction

    // Setting the direction of the ports to output
    P1->DIR |= (uint8_t)(((1<<0)));
    P2->DIR |= (uint8_t)(((1<<0)));
    P2->DIR |= (uint8_t)(((1<<1)));
    P2->DIR |= (uint8_t)(((1<<2)));

    //Initialize to driven low
    P1->OUT &= (uint8_t)(~(((1<<0))));
    P2->OUT &= (uint8_t)(~(((1<<0))));
    P2->OUT &= (uint8_t)(~(((1<<1))));
    P2->OUT &= (uint8_t)(~(((1<<2))));
    
    /* Configure Interrupts */
    P1IES |= (uint8_t)((1<<1)|(1<<4)); // Trigger interrupts on falling edge
    P1IFG &= (uint8_t)~((1<<1)|(1<<4)); // Clear interrupt flags
    P1IE |= (uint8_t)((1<<1)|(1<<4)); // Enable pin interrupts

    
    NVIC_SetPriority(PORT1_IRQn, 2);
    NVIC_ClearPendingIRQ(PORT1_IRQn);
    NVIC_EnableIRQ(PORT1_IRQn);
}

void init_UART(){
	//From UART Echo Example on CULearn
		CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    // Configure UART pins
    P1->SEL0 |= (1<<2) | (1<<3);                // set 2-UART pin as secondary function

    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
            EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    // Baud Rate calculation
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.125-78)*16) = 2
    EUSCI_A0->BRW = 78;                     // 12000000/16/9600
    EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
}

void interruptEnabler(){
	
		// Enable global interrupt for UART
    __enable_irq();

    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);

		//Enable Global Interrupts for Buttons
		__ASM("CPSIE I");

}

/* Interrupt Service Routines */
void PORT1_IRQHandler(void){
		
	if( P1IFG & (uint8_t)((1<<1))  ){
			
			P1IFG&=~(uint8_t)((1<<1));				
			increment_state(); //Increase by 1
				
	}else if( P1IFG & (uint8_t)((1<<4))  ){
		
			P1IFG&=~(uint8_t)((1<<4)); 				
			decrement_state();
		
	}
	
	change_state();
}

// UART interrupt service routine
void EUSCIA0_IRQHandler(){
	
  if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG){
			volatile  uint8_t n;   // volatile temp to avoid compiler optimization
			
			// read RXBUF then clear RX IFG to ensure the flag has been cleared
			// Note that reading will clear the flag but clear line was added for security
			n = EUSCI_A0->RXBUF;

       // From CULearn		
      // Check if the TX buffer is empty first
      while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
				
				//Determine input variable and change state where F is forward and B is backward.
				if(n == 'F'){
					increment_state(); // calling function to increment state
				}else if( n == 'B'){
					decrement_state(); // calling function to decrement state
					}
				}
				change_state(); // calling function to change state
    }
int main()
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer
	
		init_gpio(); // initialize the GPIOs
		init_UART();  // making the UART system work
		interruptEnabler(); //Enable interrupts
    
		while(1){}	// continuous loop
}
