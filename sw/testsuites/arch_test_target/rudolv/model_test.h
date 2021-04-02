#ifndef _COMPLIANCE_TEST_H
#define _COMPLIANCE_TEST_H


#define RVMODEL_BOOT            \
    .global _start             ;\
_start:                        ;\
    csrw mtvec, x0             ;
    // Otherwise MTVEC would be undefined in simulation.
    // That's a problem for exit_cleanup in RVTEST_CODE_END.

#define RVMODEL_HALT       \
    la t0, begin_signature      ;\
    csrw 0x3fd, t0              ;\
    la t0, end_signature        ;\
    csrw 0x3fe, t0              ;\
    csrw 0x3ff, 2               ;\
1:  j 1b                        ;

//#define RVMODEL_CODE_BEGIN
//#define RVMODEL_CODE_END

#define RVMODEL_DATA_BEGIN \
    .align 4; .global begin_signature; begin_signature:
#define RVMODEL_DATA_END \
    .align 4; .global end_signature; end_signature:

#define RVMODEL_SET_MSW_INT
#define RVMODEL_CLEAR_MSW_INT
#define RVMODEL_CLEAR_MTIMER_INT
#define RVMODEL_CLEAR_MEXT_INT

#define RVMODEL_IO_ASSERT_GPR_EQ(_SP, _R, _I)
#define RVMODEL_IO_WRITE_STR(_SP, _STR)

#endif



