##
# @author Jacob Kiggins <jkiggins@d3engineering.com>

dtc_cpp_flags := @DTC_CPP_FLAGS@

# Work around for SUBDIR=../docs (or other non-subdir relative path)
top_builddir=$(abs_top_builddir)
top_srcdir=$(abs_top_srcdir)

# These are defaults that should be overridden in the carrier board
FPDLINK_PORTS ?= 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
GMSL_PORTS ?= 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15

# .PRECIOUS: %.dts.pre isx031fpd_%.dts.pre isx031gmsl_%.dts.pre
isx031fpd_%.dts.pre: d3-cam-isx031-fpdlink.dtsi $(DTC_INCLUDE_FILES)
	$(CC) -nostdinc -E $(dtc_cpp_flags) -x assembler-with-cpp -DPORT_IDX=$* -o $@ $<

isx031fpd_%.dtbo: isx031fpd_%.dts.pre
	dtc -@ -I dts -O dtb -o $@ $<

isx031gmsl_%.dts.pre: d3-cam-isx031-gmsl.dtsi $(DTC_INCLUDE_FILES)
	$(CC) -nostdinc -E $(dtc_cpp_flags) -x assembler-with-cpp -DPORT_IDX=$* -o $@ $<

isx031gmsl_%.dtbo: isx031gmsl_%.dts.pre
	dtc -@ -I dts -O dtb -o $@ $<

dtbo_targets = $(foreach port,$(FPDLINK_PORTS),isx031fpd_$(port).dtbo) $(foreach port,$(GMSL_PORTS),isx031gmsl_$(port).dtbo)
cpp_targets = $(foreach port,$(FPDLINK_PORTS),isx031fpd_$(port).dts.tmp) $(foreach port,$(GMSL_PORTS),isx031gmsl_$(port).dts.pre)

.PHONY: all dtbos deploy

dtbos: $(dtbo_targets)

deploy-local: dtbos
	install -D -t $(CAM_DTBO_INSTALL_PATH) $(dtbo_targets)

all: deploy

clean-local:
	rm -f *.dtbo
	rm -f *.dts.pre
