

#include "eint.h"
#include <stdlib.h>

typedef struct eint3_entry {
    uint32_t pin_mask;          ///< Port pin's concatenated pin-mask
    void_func_t callback;       ///< Callback when interrupt occurs
    struct eint3_entry* next;   ///< The pointer to the next entry
} eint3_entry_t;

static inline void handle_eint_list(uint32_t *isr_bits_ptr, volatile uint32_t *int_clr_ptr,
                                    eint3_entry_t *list_head_ptr)
{

}

void EINT3_IRQHandler(void)
{
//use IO0IntClr
	uint32_t p0_rising  = LPC_GPIOINT->IO0IntStatR;
	uint32_t p2_rising  = LPC_GPIOINT->IO2IntStatR;
	 //handle_eint_list(&p0_rising,  &(LPC_GPIOINT->IO0IntClr), gp_port0_rising_list);
	 //handle_eint_list(&p2_rising,  &(LPC_GPIOINT->IO2IntClr), gp_port2_rising_list);
}
static void eint3_enable(uint8_t pin_num, eint_intr_t type, void_func_t func,
                         eint3_entry_t **list_head_ptr, volatile uint32_t *int_en_reg_ptr)
{

}
void eint3_enable_port0(uint8_t pin_num, eint_intr_t type, void_func_t func)
{
	//IO0IntClr
//LPC_GPIOINT needs to be used
	//use IO0IntClr
}

void eint3_enable_port2(uint8_t pin_num, eint_intr_t type, void_func_t func)
{

//LPC_GPIOINT needs to be used
	//use IO2IntClr

	//can use printf_lib.h in ISR to print
}


