/*
 * jcnet.h
 *
 *  Created on: Apr 19, 2024
 *      Author: isjeon
 */

#ifndef INC_JCNET_H_
#define INC_JCNET_H_

__attribute__( ( always_inline )) __STATIC_INLINE uint32_t cli_context(void)
{
	uint32_t result;
	__ASM volatile ("MRS %0,PRIMASK" : "=r" (result));
	__disable_irq();
	return result;
}

__attribute__( ( always_inline )) __STATIC_INLINE uint32_t sei_context(uint32_t org)
{
	if(!org) __enable_irq();
}

#define ARDUINO_CPP_ATLIB_USE
#endif /* INC_JCNET_H_ */
