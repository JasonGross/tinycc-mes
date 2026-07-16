/*************************************************************/
/*
 *  ARM inline assembler for TCC  --  bootstrap minimal, STAGE A
 *
 *  Scope-disciplined: implements exactly the instruction/constraint
 *  surface that musl-1.1.24 arch/arm inline asm emits, and errors loudly
 *  on anything outside it. Structure and the operand/constraint machinery
 *  follow mainline tinycc's arm-asm.c (Danny Milosavljevic); the register
 *  MOVE generation is reimplemented to target hardware regs r0..r15
 *  directly (raw ldr/str/mov via o()), because tcc-arm's load()/store()
 *  (via intr()) only model r0-r3 and r12 -- musl pins r4-r7 as syscall
 *  register-vars, which the generic path cannot reach.
 *
 *  STAGE A: `svc` + the register-var syscall path (no %-operands, all
 *  operands pinned to hardware regs). Stage B adds crt encoders
 *  (mov/ldr/add/and/bl); Stage C adds atomics (ldrex/strex/dmb/clz/rbit)
 *  and the "Q" memory constraint.
 */

#ifdef TARGET_DEFS_ONLY

#define CONFIG_TCC_ASM
#define NB_ASM_REGS 16

ST_FUNC void g(int c);
ST_FUNC void gen_le16(int c);
ST_FUNC void gen_le32(int c);

/*************************************************************/
#else
/*************************************************************/

#include "tcc.h"

/* XXX: make it faster ? */
ST_FUNC void g(int c)
{
    int ind1;
    if (nocode_wanted)
        return;
    ind1 = ind + 1;
    if (ind1 > cur_text_section->data_allocated)
        section_realloc(cur_text_section, ind1);
    cur_text_section->data[ind] = c;
    ind = ind1;
}

ST_FUNC void gen_le16 (int i)
{
    g(i);
    g(i>>8);
}

ST_FUNC void gen_le32 (int i)
{
    gen_le16(i);
    gen_le16(i>>16);
}

ST_FUNC void gen_expr32(ExprValue *pe)
{
    gen_le32(pe->v);
}

/* ---- raw register-move emission, hardware reg numbers 0..15 ---------- *
 * These bypass tcc's load()/store()/intr() (which only model r0-r3,r12)
 * so we can move operand values in and out of ANY hardware register the
 * asm needs (musl pins r4-r7). Only the SValue kinds that musl's asm
 * operands actually present are handled; anything else errors loudly.   */

/* mov hw, #imm  (or ldr hw,[pc]+ .word for non-encodable / symbolic) */
static void asm_emit_movi(int hw, SValue *sv)
{
    uint32_t op = stuff_const(0xE3A00000 | (hw << 12), sv->c.i);
    if ((sv->r & VT_SYM) || !op) {
        o(0xE59F0000 | (hw << 12));      /* ldr hw,[pc]  */
        o(0xEA000000);                    /* b .+4 (skip the literal) */
        if (sv->r & VT_SYM)
            greloc(cur_text_section, sv->sym, ind, R_ARM_ABS32);
        o(sv->c.i);                       /* .word imm */
    } else {
        o(op);
    }
}

/* load the RVALUE of sv into hardware register hw */
static void asm_emit_load(int hw, SValue *sv)
{
    int fr = sv->r;
    int v = fr & VT_VALMASK;
    uint32_t base, op;
    int fc, sign;

    if (fr & VT_LVAL) {
        if (v == VT_LOCAL) {
            base = 0xB; /* fp */
            fc = sv->c.i; sign = 0;
            if (fc < 0) { sign = 1; fc = -fc; }
            calcaddr(&base, &fc, &sign, 4095, 0);
            op = 0xE5100000;              /* ldr */
            if (!sign) op |= 0x800000;    /* U: add offset */
            o(op | (hw << 12) | fc | (base << 16));
            return;
        } else if (v == VT_LLOCAL) {      /* value's ptr is at [fp,#off] */
            SValue t;
            t.type.t = VT_PTR;
            t.r = VT_LOCAL | VT_LVAL;
            t.c.i = sv->c.i;
            asm_emit_load(14, &t);        /* ldr lr, [fp,#off] (lr=r14) */
            o(0xE5100000 | 0x800000 | (hw << 12) | (14 << 16)); /* ldr hw,[lr] */
            return;
        } else if (v < VT_CONST) {        /* address already in a tcc reg */
            o(0xE5100000 | 0x800000 | (hw << 12) | (intr(v) << 16));
            return;
        }
        tcc_error("arm asm: unsupported lvalue operand kind 0x%x", fr);
    } else {
        if (v == VT_CONST) {
            asm_emit_movi(hw, sv);
            return;
        } else if (v == VT_LOCAL) {       /* address of a local: add hw,fp,#off */
            op = stuff_const(0xE28B0000 | (hw << 12), sv->c.i);
            if (!op)
                tcc_error("arm asm: local address offset not encodable");
            o(op);
            return;
        } else if (v < VT_CONST) {        /* value already in a tcc reg */
            o(0xE1A00000 | (hw << 12) | intr(v));   /* mov hw, rv */
            return;
        }
        tcc_error("arm asm: unsupported rvalue operand kind 0x%x", fr);
    }
}

/* store hardware register hw into the LVALUE described by sv */
static void asm_emit_store(int hw, SValue *sv)
{
    int fr = sv->r;
    int v = fr & VT_VALMASK;
    uint32_t base, op;
    int fc, sign;

    if (v == VT_LOCAL) {
        base = 0xB; /* fp */
        fc = sv->c.i; sign = 0;
        if (fc < 0) { sign = 1; fc = -fc; }
        calcaddr(&base, &fc, &sign, 4095, 0);
        op = 0xE5000000;                  /* str */
        if (!sign) op |= 0x800000;
        o(op | (hw << 12) | fc | (base << 16));
        return;
    } else if (v < VT_CONST) {            /* address in a tcc reg */
        o(0xE5000000 | 0x800000 | (hw << 12) | (intr(v) << 16));
        return;
    }
    tcc_error("arm asm: unsupported store destination 0x%x", fr);
}

/* ---- opcode encoders (whitelist only) -------------------------------- */

/* parse a bare core register at the current token, return hw number 0..15 */
static int asm_parse_reg(void)
{
    int reg = asm_parse_regvar(tok);
    if (reg < 0)
        expect("ARM core register");
    next();
    return reg;
}

/* parse "[ Rn ]" (single-register, no offset) and return Rn */
static int asm_parse_mem_reg(void)
{
    int reg;
    skip('[');
    reg = asm_parse_reg();
    skip(']');
    return reg;
}

/* is the current token a core register? (used to disambiguate a flexible
   operand: register vs immediate) */
static int asm_tok_is_reg(void)
{
    return asm_parse_regvar(tok) >= 0;
}

/* parse "#imm" or "imm" into e */
static void asm_parse_imm(TCCState *s1, ExprValue *e)
{
    if (tok == '#' || tok == '$')
        next();
    asm_expr(s1, e);
}

/* map a 2-char ARM condition suffix to its 4-bit code, or 0xFF if unknown.
   Only the conditions musl's arch/arm .s files use need be here; growth is
   per-need with the same loud-error discipline. */
static uint32_t asm_parse_cond(const char *s)
{
    if (s[0] && s[1] && !s[2]) {
        char a = s[0], b = s[1];
        if (a == 'e' && b == 'q') return 0;
        if (a == 'n' && b == 'e') return 1;
        if (a == 'c' && b == 's') return 2;
        if (a == 'h' && b == 's') return 2;
        if (a == 'c' && b == 'c') return 3;
        if (a == 'l' && b == 'o') return 3;
        if (a == 'm' && b == 'i') return 4;
        if (a == 'p' && b == 'l') return 5;
        if (a == 'v' && b == 's') return 6;
        if (a == 'v' && b == 'c') return 7;
        if (a == 'h' && b == 'i') return 8;
        if (a == 'l' && b == 's') return 9;
        if (a == 'g' && b == 'e') return 10;
        if (a == 'l' && b == 't') return 11;
        if (a == 'g' && b == 't') return 12;
        if (a == 'l' && b == 'e') return 13;
        if (a == 'a' && b == 'l') return 14;
    }
    return 0xFF;
}

/* parse "{ Rn [, Rn | - Rn ]... }" into a 16-bit register-set mask */
static int asm_parse_reglist(void)
{
    int mask = 0, r, r2;
    skip('{');
    for (;;) {
        r = asm_parse_reg();
        if (tok == '-') {                 /* range: rA-rB */
            next();
            r2 = asm_parse_reg();
            if (r2 < r)
                tcc_error("arm asm: bad register range in list");
            while (r <= r2) { mask |= 1 << r; r++; }
        } else {
            mask |= 1 << r;
        }
        if (tok == ',') { next(); continue; }
        break;
    }
    skip('}');
    return mask;
}

ST_FUNC void asm_opcode(TCCState *s1, int token)
{
    ExprValue e;
    int rd, rn, rm;
    uint32_t op;
    const char *mn = get_tok_str(token, NULL);

    /* mov family: plain `mov`, `movs` (set flags), and `mov<cc>` (conditional).
       musl setjmp.S/longjmp.S use `movs r0,r1` + `moveq r0,#1`; the crt uses
       plain `mov`. cond defaults to AL (0xE). Handled here so the conditional
       forms need no separate tokens (tcc tokenizes `moveq` as one ident). */
    if (mn[0] == 'm' && mn[1] == 'o' && mn[2] == 'v') {
        uint32_t cond = 0xE, sbit = 0;
        const char *suf = mn + 3;
        if (suf[0] == 's' && suf[1] == '\0') {
            sbit = 1 << 20;
        } else if (suf[0] != '\0') {
            cond = asm_parse_cond(suf);
            if (cond == 0xFF)
                tcc_error("arm asm: unknown mov condition/suffix '%s'", suf);
        }
        rd = asm_parse_reg();
        skip(',');
        if (asm_tok_is_reg()) {
            rm = asm_parse_reg();
            o((cond << 28) | 0x01A00000 | sbit | (rd << 12) | rm);
        } else {
            asm_parse_imm(s1, &e);
            op = stuff_const((cond << 28) | 0x03A00000 | sbit | (rd << 12), e.v);
            if (!op)
                tcc_error("arm asm: mov #%d not encodable", (int)e.v);
            o(op);
        }
        return;
    }

    switch (token) {
    case TOK_ASM_svc:
        asm_parse_imm(s1, &e);
        o(0xEF000000 | (e.v & 0x00FFFFFF));
        return;

    /* ---- crt_arch.h _start: add/and/ldr/bl (mov handled above) ---- */
    case TOK_ASM_add:                   /* add Rd, Rn, Rm | add Rd, Rn, #imm */
        rd = asm_parse_reg();
        skip(',');
        rn = asm_parse_reg();
        skip(',');
        if (asm_tok_is_reg()) {
            rm = asm_parse_reg();
            o(0xE0800000 | (rn << 16) | (rd << 12) | rm);
        } else {
            asm_parse_imm(s1, &e);
            op = stuff_const(0xE2800000 | (rn << 16) | (rd << 12), e.v);
            if (!op)
                tcc_error("arm asm: add #%d not encodable", (int)e.v);
            o(op);
        }
        return;
    case TOK_ASM_and:                   /* and Rd, Rn, Rm | and Rd, Rn, #imm */
        rd = asm_parse_reg();
        skip(',');
        rn = asm_parse_reg();
        skip(',');
        if (asm_tok_is_reg()) {
            rm = asm_parse_reg();
            o(0xE0000000 | (rn << 16) | (rd << 12) | rm);
        } else {
            asm_parse_imm(s1, &e);
            op = stuff_const(0xE2000000 | (rn << 16) | (rd << 12), e.v);
            if (op) {
                o(op);
            } else {
                /* gas encodes an AND immediate that isn't representable as a
                   BIC of the one's complement (e.g. #-16 -> bic #15) */
                op = stuff_const(0xE3C00000 | (rn << 16) | (rd << 12),
                                 ~(uint32_t)e.v);
                if (!op)
                    tcc_error("arm asm: and #%d not encodable", (int)e.v);
                o(op);
            }
        }
        return;
    case TOK_ASM_bl:                    /* bl symbol */
        asm_expr(s1, &e);
        if (!e.sym)
            tcc_error("arm asm: bl requires a symbol operand");
        greloc(cur_text_section, e.sym, ind, R_ARM_PC24);
        o(encbranch(ind, ind + e.v, 1) | 0xE1000000);   /* 0xEA.. -> 0xEB.. */
        return;

    /* ---- pthread_arch.h: read the TLS pointer from CP15 ----------- */
    case TOK_ASM_mrc: {                 /* mrc pC, #o1, Rd, cN, cM, #o2 */
        int coproc, opc1, crn, crm, opc2;
        if (tok < TOK_ASM_p0 || tok > TOK_ASM_p15)
            expect("coprocessor (pN)");
        coproc = tok - TOK_ASM_p0; next();
        skip(',');
        asm_parse_imm(s1, &e); opc1 = e.v;
        skip(',');
        rd = asm_parse_reg();
        skip(',');
        if (tok < TOK_ASM_c0 || tok > TOK_ASM_c15)
            expect("coprocessor register (cN)");
        crn = tok - TOK_ASM_c0; next();
        skip(',');
        if (tok < TOK_ASM_c0 || tok > TOK_ASM_c15)
            expect("coprocessor register (cN)");
        crm = tok - TOK_ASM_c0; next();
        skip(',');
        asm_parse_imm(s1, &e); opc2 = e.v;
        o(0xEE100010 | (opc1 << 21) | (crn << 16) | (rd << 12) |
          (coproc << 8) | (opc2 << 5) | crm);
        return;
    }

    /* ---- v6/v7 synchronization primitives (musl atomic_arch.h) ---- */
    case TOK_ASM_ldrex:                 /* ldrex Rd, [Rn] */
        rd = asm_parse_reg();
        skip(',');
        rn = asm_parse_mem_reg();
        o(0xE1900F9F | (rn << 16) | (rd << 12));
        return;
    case TOK_ASM_strex:                 /* strex Rd, Rm, [Rn] */
        rd = asm_parse_reg();
        skip(',');
        rm = asm_parse_reg();
        skip(',');
        rn = asm_parse_mem_reg();
        o(0xE1800F90 | (rn << 16) | (rd << 12) | rm);
        return;
    case TOK_ASM_clz:                   /* clz Rd, Rm */
        rd = asm_parse_reg();
        skip(',');
        rm = asm_parse_reg();
        o(0xE16F0F10 | (rd << 12) | rm);
        return;
    case TOK_ASM_rbit:                  /* rbit Rd, Rm */
        rd = asm_parse_reg();
        skip(',');
        rm = asm_parse_reg();
        o(0xE6FF0F30 | (rd << 12) | rm);
        return;
    case TOK_ASM_dmb:                   /* dmb ish (only option musl uses) */
        if (tok >= TOK_IDENT)           /* consume the barrier-option token */
            next();
        o(0xF57FF05B);                  /* dmb ish */
        return;

    /* ---- setjmp.S/longjmp.S: block load/store + return ------------- */
    case TOK_ASM_stmia:                 /* stmia Rn[!], {reglist} */
    case TOK_ASM_ldmia: {               /* ldmia Rn[!], {reglist} */
        int wb = 0, mask;
        uint32_t base = (token == TOK_ASM_ldmia) ? 0xE8900000  /* LDMIA */
                                                 : 0xE8800000; /* STMIA */
        rn = asm_parse_reg();
        if (tok == '!') { wb = 1; next(); }   /* writeback base */
        skip(',');
        mask = asm_parse_reglist();
        o(base | (wb ? 0x00200000 : 0) | (rn << 16) | (mask & 0xFFFF));
        return;
    }
    case TOK_ASM_bx:                    /* bx Rn */
        rm = asm_parse_reg();
        o(0xE12FFF10 | rm);
        return;

    default:
        tcc_error("arm asm: instruction not implemented in bootstrap "
                  "assembler: '%s'", get_tok_str(token, NULL));
    }
}

/* ---- gcc extended-asm operand substitution --------------------------- */

ST_FUNC void subst_asm_operand(CString *add_str, SValue *sv, int modifier)
{
    int r, reg;
    char buf[64];

    r = sv->r;
    if ((r & VT_VALMASK) == VT_CONST) {
        if (!(r & VT_LVAL) && modifier != 'c' && modifier != 'n' &&
            modifier != 'P')
            cstr_ccat(add_str, '#');
        if (r & VT_SYM) {
            const char *name = get_tok_str(sv->sym->v, NULL);
            if (sv->sym->v >= SYM_FIRST_ANOM)
                get_asm_sym(tok_alloc(name, strlen(name))->tok, sv->sym);
            if (tcc_state->leading_underscore)
                cstr_ccat(add_str, '_');
            cstr_cat(add_str, name, -1);
            if ((uint32_t) sv->c.i == 0)
                goto no_offset;
            cstr_ccat(add_str, '+');
        }
        {
            int val = sv->c.i;
            if (modifier == 'n')
                val = -val;
            snprintf(buf, sizeof(buf), "%d", (int) sv->c.i);
            cstr_cat(add_str, buf, -1);
        }
      no_offset:;
    } else if ((r & VT_VALMASK) == VT_LOCAL) {
        snprintf(buf, sizeof(buf), "[fp,#%d]", (int) sv->c.i);
        cstr_cat(add_str, buf, -1);
    } else if (r & VT_LVAL) {
        reg = r & VT_VALMASK;
        if (reg >= VT_CONST)
            tcc_error("arm asm: internal: bad lvalue reg");
        snprintf(buf, sizeof(buf), "[%s]",
                 get_tok_str(TOK_ASM_r0 + reg, NULL));
        cstr_cat(add_str, buf, -1);
    } else {
        /* register case */
        reg = r & VT_VALMASK;
        if (reg >= VT_CONST)
            tcc_error("arm asm: internal: bad operand reg");
        snprintf(buf, sizeof(buf), "%s", get_tok_str(TOK_ASM_r0 + reg, NULL));
        cstr_cat(add_str, buf, -1);
    }
}

/* ---- prolog/epilog: move operand values into/out of asm registers ---- */

ST_FUNC void asm_gen_code(ASMOperand *operands, int nb_operands,
                          int nb_outputs, int is_output,
                          uint8_t *clobber_regs,
                          int out_reg)
{
    uint8_t regs_allocated[NB_ASM_REGS];
    ASMOperand *op;
    int i, reg;
    uint32_t saved_regset = 0;

    /* r4..r11 are callee-saved; save/restore any we touch */
    static const uint8_t reg_saved[] = { 4, 5, 6, 7, 8, 9, 10, 11 };

    memcpy(regs_allocated, clobber_regs, sizeof(regs_allocated));
    for (i = 0; i < nb_operands; i++) {
        op = &operands[i];
        if (op->reg >= 0)
            regs_allocated[op->reg] = 1;
    }
    for (i = 0; i < (int)(sizeof(reg_saved)/sizeof(reg_saved[0])); i++) {
        reg = reg_saved[i];
        if (regs_allocated[reg])
            saved_regset |= 1 << reg;
    }

    if (!is_output) {
        /* prolog: save clobbered callee-saved regs, then load inputs */
        if (saved_regset)
            o(0xE92D0000 | saved_regset);       /* push {...} */

        for (i = 0; i < nb_operands; i++) {
            op = &operands[i];
            if (op->reg >= 0) {
                if ((op->vt->r & VT_VALMASK) == VT_LLOCAL && op->is_memory) {
                    /* pointer for the memory operand is stored in the local
                       slot: load it (ldr reg,[fp,#off]) so subst can emit
                       "[reg]" */
                    SValue sv = *op->vt;
                    sv.r = (sv.r & ~VT_VALMASK) | VT_LOCAL | VT_LVAL;
                    sv.type.t = VT_PTR;
                    asm_emit_load(op->reg, &sv);
                } else if (i >= nb_outputs || op->is_rw) {
                    asm_emit_load(op->reg, op->vt);
                    if (op->is_llong)
                        tcc_error("arm asm: long long operand not implemented");
                }
            }
        }
    } else {
        /* epilog: store outputs, then restore callee-saved regs */
        for (i = 0; i < nb_outputs; i++) {
            op = &operands[i];
            if (op->reg >= 0) {
                if ((op->vt->r & VT_VALMASK) == VT_LLOCAL) {
                    if (!op->is_memory) {
                        SValue sv = *op->vt;
                        sv.r = (sv.r & ~VT_VALMASK) | VT_LOCAL | VT_LVAL;
                        sv.type.t = VT_PTR;
                        asm_emit_load(out_reg, &sv);
                        sv = *op->vt;
                        sv.r = (sv.r & ~VT_VALMASK) | out_reg;
                        asm_emit_store(op->reg, &sv);
                    }
                } else {
                    asm_emit_store(op->reg, op->vt);
                    if (op->is_llong)
                        tcc_error("arm asm: long long operand not implemented");
                }
            }
        }
        if (saved_regset)
            o(0xE8BD0000 | saved_regset);        /* pop {...} */
    }
}

/* ---- constraint solving (mainline structure) ------------------------- */

static inline int constraint_priority(const char *str)
{
    int priority, c, pr;
    priority = 0;
    for (;;) {
        c = *str;
        if (c == '\0')
            break;
        str++;
        switch (c) {
        case 'l':
        case 'r':
        case 'p':
        case 'Q':               /* memory addressed by a single register */
            pr = 3;
            break;
        case 'I':
        case 'i':
        case 'm':
            pr = 4;
            break;
        default:
            tcc_error("arm asm: unknown constraint '%c'", c);
            pr = 0;
        }
        if (pr > priority)
            priority = pr;
    }
    return priority;
}

static const char *skip_constraint_modifiers(const char *p)
{
    while (*p == '=' || *p == '&' || *p == '+' || *p == '%')
        p++;
    return p;
}

#define REG_OUT_MASK 0x01
#define REG_IN_MASK  0x02
#define is_reg_allocated(reg) (regs_allocated[reg] & reg_mask)

ST_FUNC void asm_compute_constraints(ASMOperand *operands,
                                    int nb_operands, int nb_outputs,
                                    const uint8_t *clobber_regs,
                                    int *pout_reg)
{
    ASMOperand *op;
    int sorted_op[MAX_ASM_OPERANDS];
    int i, j, k, p1, p2, tmp, reg, c, reg_mask;
    const char *str;
    uint8_t regs_allocated[NB_ASM_REGS];

    for (i = 0; i < nb_operands; i++) {
        op = &operands[i];
        op->input_index = -1;
        op->ref_index = -1;
        op->reg = -1;
        op->is_memory = 0;
        op->is_rw = 0;
        op->is_llong = 0;
    }
    for (i = 0; i < nb_operands; i++) {
        op = &operands[i];
        str = op->constraint;
        str = skip_constraint_modifiers(str);
        if (isnum(*str) || *str == '[') {
            k = find_constraint(operands, nb_operands, str, NULL);
            if ((unsigned) k >= i || i < nb_outputs)
                tcc_error("arm asm: invalid reference in constraint %d ('%s')",
                          i, str);
            op->ref_index = k;
            if (operands[k].input_index >= 0)
                tcc_error("arm asm: cannot reference twice the same operand");
            operands[k].input_index = i;
            op->priority = 5;
        } else if ((op->vt->r & VT_VALMASK) == VT_LOCAL
                   && op->vt->sym
                   && (reg = op->vt->sym->r & VT_VALMASK) < VT_CONST) {
            /* operand is a `register T x __asm__("rN")' var: pin to rN */
            op->priority = 1;
            op->reg = reg;
        } else {
            op->priority = constraint_priority(str);
        }
    }

    for (i = 0; i < nb_operands; i++)
        sorted_op[i] = i;
    for (i = 0; i < nb_operands - 1; i++) {
        for (j = i + 1; j < nb_operands; j++) {
            p1 = operands[sorted_op[i]].priority;
            p2 = operands[sorted_op[j]].priority;
            if (p2 < p1) {
                tmp = sorted_op[i];
                sorted_op[i] = sorted_op[j];
                sorted_op[j] = tmp;
            }
        }
    }

    for (i = 0; i < NB_ASM_REGS; i++) {
        if (clobber_regs[i])
            regs_allocated[i] = REG_IN_MASK | REG_OUT_MASK;
        else
            regs_allocated[i] = 0;
    }
    regs_allocated[13] = REG_IN_MASK | REG_OUT_MASK;   /* sp */
    regs_allocated[11] = REG_IN_MASK | REG_OUT_MASK;   /* fp */

    for (i = 0; i < nb_operands; i++) {
        j = sorted_op[i];
        op = &operands[j];
        str = op->constraint;
        if (op->ref_index >= 0)
            continue;
        if (op->input_index >= 0) {
            reg_mask = REG_IN_MASK | REG_OUT_MASK;
        } else if (j < nb_outputs) {
            reg_mask = REG_OUT_MASK;
        } else {
            reg_mask = REG_IN_MASK;
        }
        if (op->reg >= 0) {
            if (is_reg_allocated(op->reg))
                tcc_error("arm asm: regvar requests register that's taken");
            reg = op->reg;
        }
      try_next:
        c = *str++;
        switch (c) {
        case '=':
            goto try_next;
        case '+':
            op->is_rw = 1;
            /* fall through */
        case '&':
            if (j >= nb_outputs)
                tcc_error("arm asm: '%c' modifier can only be applied to outputs", c);
            reg_mask = REG_IN_MASK | REG_OUT_MASK;
            goto try_next;
        case 'l':
        case 'r':
        case 'p':
            if ((reg = op->reg) >= 0)
                goto reg_found;
            else for (reg = 0; reg <= 10; reg++) {
                if (!is_reg_allocated(reg) && reg != 11 && reg != 13)
                    goto reg_found;
            }
            goto try_next;
          reg_found:
            op->is_llong = 0;
            op->reg = reg;
            regs_allocated[reg] |= reg_mask;
            break;
        case 'Q':
            /* "Q" = memory addressed by a single register. tcc's minimal
               bootstrap assembler does not implement it (it has hazardous
               tcc-index-vs-hardware rendering after parse-time gv). musl is
               patched to use `"r"(ptr)` + an explicit `[%N]` in the template
               instead (patches/musl-1.1.24-arm/atomic-no-q.patch), which the
               validated plain-register path handles. Error loudly if it ever
               reaches here. */
            tcc_error("arm asm: 'Q' constraint not implemented "
                      "(patch musl atomics to \"r\"(ptr)+[%%N])");
            break;
        case 'I':
        case 'i':
            if (!((op->vt->r & (VT_VALMASK | VT_LVAL)) == VT_CONST))
                goto try_next;
            break;
        case 'm':
            if (j < nb_outputs || c == 'm') {
                if ((op->vt->r & VT_VALMASK) == VT_LLOCAL) {
                    for (reg = 0; reg <= 10; reg++) {
                        if (!(regs_allocated[reg] & REG_IN_MASK) && reg != 11 && reg != 13)
                            goto reg_found1;
                    }
                    goto try_next;
                  reg_found1:
                    regs_allocated[reg] |= REG_IN_MASK;
                    op->reg = reg;
                    op->is_memory = 1;
                }
            }
            break;
        default:
            tcc_error("arm asm: constraint %d ('%s') could not be satisfied",
                      j, op->constraint);
            break;
        }
        if (op->input_index >= 0) {
            operands[op->input_index].reg = op->reg;
            operands[op->input_index].is_llong = op->is_llong;
        }
    }

    *pout_reg = -1;
    for (i = 0; i < nb_operands; i++) {
        op = &operands[i];
        if (op->reg >= 0 &&
            (op->vt->r & VT_VALMASK) == VT_LLOCAL && !op->is_memory) {
            for (reg = 0; reg <= 10; reg++) {
                if (!(regs_allocated[reg] & REG_OUT_MASK) && reg != 11 && reg != 13)
                    goto reg_found2;
            }
            tcc_error("arm asm: could not find free output register for reloading");
          reg_found2:
            *pout_reg = reg;
            break;
        }
    }
}

ST_FUNC void asm_clobber(uint8_t *clobber_regs, const char *str)
{
    int reg;
    TokenSym *ts;

    if (!strcmp(str, "memory") ||
        !strcmp(str, "cc") ||
        !strcmp(str, "flags"))
        return;
    ts = tok_alloc(str, strlen(str));
    reg = asm_parse_regvar(ts->tok);
    if (reg == -1)
        tcc_error("arm asm: invalid clobber register '%s'", str);
    clobber_regs[reg] = 1;
}

/* If T names an ARM core register (r0..r15 or an APCS alias) return its
   hardware number 0..15, else -1. */
ST_FUNC int asm_parse_regvar (int t)
{
    if (t >= TOK_ASM_r0 && t <= TOK_ASM_pc) {
        switch (t) {
        case TOK_ASM_a1: return 0;
        case TOK_ASM_a2: return 1;
        case TOK_ASM_a3: return 2;
        case TOK_ASM_a4: return 3;
        case TOK_ASM_v1: return 4;
        case TOK_ASM_v2: return 5;
        case TOK_ASM_v3: return 6;
        case TOK_ASM_v4: return 7;
        case TOK_ASM_v5: return 8;
        case TOK_ASM_v6: return 9;
        case TOK_ASM_v7: return 10;
        case TOK_ASM_v8: return 11;
        case TOK_ASM_sb: return 9;
        case TOK_ASM_sl: return 10;
        case TOK_ASM_fp: return 11;
        case TOK_ASM_ip: return 12;
        case TOK_ASM_sp: return 13;
        case TOK_ASM_lr: return 14;
        case TOK_ASM_pc: return 15;
        default: return t - TOK_ASM_r0;
        }
    }
    return -1;
}

/*************************************************************/
#endif /* ndef TARGET_DEFS_ONLY */
