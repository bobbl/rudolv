set_component FCCC_C0_FCCC_C0_0_FCCC
# Microsemi Corp.
# Date: 2019-Sep-09 19:08:08
#

create_clock -period 20 [ get_pins { CCC_INST/RCOSC_25_50MHZ } ]
create_generated_clock -multiply_by 12 -divide_by 50 -source [ get_pins { CCC_INST/RCOSC_25_50MHZ } ] -phase 0 [ get_pins { CCC_INST/GL0 } ]
