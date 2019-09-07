# DEVICE = up5k_miv
ifeq ($(PACKAGE),)
    $(error For DEVICE=up5k a PACKAGE= (sg48 or uwg30) must be given)
endif
ARACHNE_DEVICE = 5k
ARACHNE_PACKAGE = -P $(PACKAGE)
NEXTPNR_PACKAGE = --package $(PACKAGE)
