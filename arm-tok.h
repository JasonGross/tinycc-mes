/* ------------------------------------------------------------------ */
/* Minimal ARM asm token table for the bootstrap inline assembler.     */
/* WARNING: relative order of register tokens matters: r0 first, pc    */
/* last, all register/alias tokens contiguous (asm_parse_regvar range).*/

/* hardware registers r0..r15 */
 DEF_ASM(r0)
 DEF_ASM(r1)
 DEF_ASM(r2)
 DEF_ASM(r3)
 DEF_ASM(r4)
 DEF_ASM(r5)
 DEF_ASM(r6)
 DEF_ASM(r7)
 DEF_ASM(r8)
 DEF_ASM(r9)
 DEF_ASM(r10)
 DEF_ASM(r11)
 DEF_ASM(r12)
 DEF_ASM(r13)
 DEF_ASM(r14)
 DEF_ASM(r15)

/* APCS/AAPCS register aliases (musl crt_arch.h uses a1,a2,ip; syscalls
   use r0-r7; keep the full alias set so we accept unmodified musl asm) */
 DEF_ASM(a1)  /* r0 */
 DEF_ASM(a2)  /* r1 */
 DEF_ASM(a3)  /* r2 */
 DEF_ASM(a4)  /* r3 */
 DEF_ASM(v1)  /* r4 */
 DEF_ASM(v2)  /* r5 */
 DEF_ASM(v3)  /* r6 */
 DEF_ASM(v4)  /* r7 */
 DEF_ASM(v5)  /* r8 */
 DEF_ASM(v6)  /* r9 */
 DEF_ASM(v7)  /* r10 */
 DEF_ASM(v8)  /* r11 */
 DEF_ASM(sb)  /* r9  */
 DEF_ASM(sl)  /* r10 */
 DEF_ASM(fp)  /* r11 */
 DEF_ASM(ip)  /* r12 */
 DEF_ASM(sp)  /* r13 */
 DEF_ASM(lr)  /* r14 */
 DEF_ASM(pc)  /* r15 - MUST be the last register token */

/* coprocessor + coprocessor-register names (musl pthread_arch.h: mrc p15,..,c13,c0,..) */
 DEF_ASM(p0)
 DEF_ASM(p1)
 DEF_ASM(p2)
 DEF_ASM(p3)
 DEF_ASM(p4)
 DEF_ASM(p5)
 DEF_ASM(p6)
 DEF_ASM(p7)
 DEF_ASM(p8)
 DEF_ASM(p9)
 DEF_ASM(p10)
 DEF_ASM(p11)
 DEF_ASM(p12)
 DEF_ASM(p13)
 DEF_ASM(p14)
 DEF_ASM(p15)
 DEF_ASM(c0)
 DEF_ASM(c1)
 DEF_ASM(c2)
 DEF_ASM(c3)
 DEF_ASM(c4)
 DEF_ASM(c5)
 DEF_ASM(c6)
 DEF_ASM(c7)
 DEF_ASM(c8)
 DEF_ASM(c9)
 DEF_ASM(c10)
 DEF_ASM(c11)
 DEF_ASM(c12)
 DEF_ASM(c13)
 DEF_ASM(c14)
 DEF_ASM(c15)

/* whitelist mnemonics (exactly what musl-1.1.24 arch/arm emits) */
 DEF_ASM(svc)
 DEF_ASM(mov)
 DEF_ASM(add)
 DEF_ASM(and)
 DEF_ASM(ldr)
 DEF_ASM(bl)
 DEF_ASM(dmb)
 DEF_ASM(clz)
 DEF_ASM(rbit)
 DEF_ASM(ldrex)
 DEF_ASM(strex)
 DEF_ASM(mrc)
/* load/store-multiple + return (musl setjmp.S/longjmp.S core save/restore) */
 DEF_ASM(stmia)
 DEF_ASM(ldmia)
 DEF_ASM(bx)
