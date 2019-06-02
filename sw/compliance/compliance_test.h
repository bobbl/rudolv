#ifndef _COMPLIANCE_TEST_H
#define _COMPLIANCE_TEST_H

#include "riscv_test.h"

#define RV_COMPLIANCE_HALT              RVTEST_EXIT
#define RV_COMPLIANCE_RV32M
#define RV_COMPLIANCE_CODE_BEGIN        RVTEST_CODE_BEGIN
#define RV_COMPLIANCE_CODE_END          RVTEST_CODE_END
#define RV_COMPLIANCE_DATA_BEGIN        .section .signature
#define RV_COMPLIANCE_DATA_END

#endif
