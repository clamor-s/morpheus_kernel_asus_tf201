#ifndef __ASM_GENERIC_CMPXCHG_H
#define __ASM_GENERIC_CMPXCHG_H

/*
 * Generic cmpxchg
 *
 * Uses the local cmpxchg. Does not support SMP.
 */
#ifdef CONFIG_SMP
#error "Cannot use generic cmpxchg on SMP"
#endif

/*
 * Atomic compare and exchange.
 */
#define cmpxchg(ptr, o, n)	cmpxchg_local((ptr), (o), (n))
#define cmpxchg64(ptr, o, n)	cmpxchg64_local((ptr), (o), (n))

#endif
