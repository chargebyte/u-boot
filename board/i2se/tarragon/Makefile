#
# Copyright (C) 2017 I2SE GmbH
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := tarragon.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
