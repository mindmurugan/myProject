include $(top_srcdir)/common.mk

all-local: $(hex)

hex =

noinst_PROGRAMS =

# 328Px8
noinst_PROGRAMS += razor9DofImu_328Px8.elf
razor9DofImu_328Px8_elf_CPPFLAGS = $(CPPFLAGS_common) $(MCU_328Px8)
razor9DofImu_328Px8_elf_LDADD = \
	$(top_builddir)/oooarkavr/liboooarkavr_328Px8.la \
	$(top_builddir)/oooarkavr/liboooarkavr_arduino_328Px8.la \
	$(LDADD_common)
razor9DofImu_328Px8_elf_LDFLAGS = $(LDFLAGS_common) $(MCU_328Px8)
razor9DofImu_328Px8_elf_SOURCES = razor9DofImu.cpp
hex += razor9DofImu_328Px8.hex

# 328x16
noinst_PROGRAMS += razor9DofImu_328Px16.elf
razor9DofImu_328Px16_elf_CPPFLAGS = $(CPPFLAGS_common) $(MCU_328Px16)
razor9DofImu_328Px16_elf_LDADD = \
	$(top_builddir)/oooarkavr/liboooarkavr_328Px16.la \
	$(top_builddir)/oooarkavr/liboooarkavr_arduino_328Px16.la \
	$(LDADD_common)
razor9DofImu_328Px16_elf_LDFLAGS = $(LDFLAGS_common) $(MCU_328Px16)
razor9DofImu_328Px16_elf_SOURCES = razor9DofImu.cpp
hex += razor9DofImu_328Px16.hex

# 1280x16
noinst_PROGRAMS += ugv_1280x16.elf
ugv_1280x16_elf_CPPFLAGS = $(CPPFLAGS_common) $(MCU_1280x16)
ugv_1280x16_elf_LDADD = \
	$(top_builddir)/oooarkavr/liboooarkavr_1280x16.la \
	$(top_builddir)/oooarkavr/liboooarkavr_arduino_1280x16.la \
	$(LDADD_common)
ugv_1280x16_elf_LDFLAGS = $(LDFLAGS_common) $(MCU_1280x16)
ugv_1280x16_elf_SOURCES = ugv.cpp
hex += ugv_1280x16.hex

noinst_PROGRAMS += ugvArdupilot_1280x16.elf
ugvArdupilot_1280x16_elf_CPPFLAGS = $(CPPFLAGS_common) $(MCU_1280x16)
ugvArdupilot_1280x16_elf_LDADD = \
	$(top_builddir)/oooarkavr/liboooarkavr_1280x16.la \
	$(top_builddir)/oooarkavr/liboooarkavr_arduino_1280x16.la \
	$(LDADD_common)
ugvArdupilot_1280x16_elf_LDFLAGS = $(LDFLAGS_common) $(MCU_1280x16)
ugvArdupilot_1280x16_elf_SOURCES = ugvArdupilot.cpp
hex += ugvArdupilot_1280x16.hex
