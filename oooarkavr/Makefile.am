include $(top_srcdir)/common.mk

lib_LTLIBRARIES =

liboooarkavr_SOURCES_ = \
\
communication/CommProtocol.cpp \
communication/Sirf.cpp \
communication/Ubx.cpp \
communication/I2c.cpp \
\
systems/System.cpp \
\
sensors/Sensor.cpp \
sensors/Gps.cpp \
sensors/Lea5h.cpp \
sensors/Em406.cpp \
\
control/Actuator.cpp \
control/Controller.cpp \
control/ServoManager.cpp \
control/Pid.cpp \
\
guidance/Guide.cpp \
\
math/basic.cpp

liboooarkavr_HEADERS = \
\
communication/CommProtocol.hpp \
communication/Sirf.hpp \
communication/Ubx.hpp \
communication/I2c.hpp \
\
systems/System.hpp \
\
sensors/Sensor.hpp \
sensors/Gps.hpp \
sensors/Lea5h.hpp \
sensors/Em406.hpp \
\
control/Actuator.hpp \
control/Controller.hpp \
control/ServoManager.hpp \
control/FeedbackController.hpp \
control/Pid.hpp \
\
guidance/Guide.hpp \
\
utilities/definitions.hpp \
utilities/Timer.hpp \
\
math/basic.hpp \
math/Quaternion.hpp \
math/Vector.hpp \
math/Matrix.hpp \
math/SmoothScalar.hpp \
math/SmoothInt.hpp

liboooarkavrdir = $(includedir)/oooarkavr

liboooarkavr_arduino_SOURCES_ = \
utilities/cPlusPlus.cpp \
utilities/Timer.cpp \
\
arduino/cores/wiring.c \
arduino/cores/wiring_shift.c \
arduino/cores/pins_arduino.c \
arduino/cores/wiring_digital.c \
arduino/cores/WMath.cpp \
arduino/cores/main.cpp \
arduino/cores/wiring_pulse.c \
arduino/cores/Tone.cpp \
arduino/cores/HardwareSerial.cpp \
arduino/cores/Print.cpp \
arduino/cores/WInterrupts.c \
arduino/cores/wiring_analog.c \
\
arduino/libraries/SoftwareSerial/SoftwareSerial.cpp \
arduino/libraries/Servo/Servo.cpp \
arduino/libraries/Wire/Wire.cpp \
arduino/libraries/Wire/utility/twi.c \
arduino/libraries/EEPROM/EEPROM.cpp

liboooarkavr_arduino_HEADERS= \
utilities/cPlusPlus.hpp \
\
arduino/cores/wiring_private.h \
arduino/cores/WProgram.h \
arduino/cores/wiring.h \
arduino/cores/pins_arduino.h \
arduino/cores/binary.h \
arduino/cores/HardwareSerial.h \
arduino/cores/Print.h \
arduino/cores/WConstants.h \
\
arduino/libraries/SoftwareSerial/SoftwareSerial.h \
arduino/libraries/Servo/Servo.h \
arduino/libraries/Wire/Wire.h \
arduino/libraries/Wire/utility/twi.h \
arduino/libraries/EEPROM/EEPROM.h

liboooarkavr_arduinodir=$(includedir)/oooarkavr/arduino;

lib_LTLIBRARIES += liboooarkavr_328Px8.la
liboooarkavr_328Px8_la_LIBADD = $(LDADD_common)
liboooarkavr_328Px8_la_CPPFLAGS = $(CPPFLAGS_common) $(MCU_328Px8)
liboooarkavr_328Px8_la_LDFLAGS = $(LDFLAGS_common) $(MCU_328Px8)
liboooarkavr_328Px8_la_SOURCES = $(liboooarkavr_SOURCES_)

lib_LTLIBRARIES += liboooarkavr_arduino_328Px8.la
liboooarkavr_arduino_328Px8_la_LIBADD = $(LDADD_common)
liboooarkavr_arduino_328Px8_la_CPPFLAGS = $(CPPFLAGS_common) $(MCU_328Px8)
liboooarkavr_arduino_328Px8_la_LDFLAGS = $(LDFLAGS_common) $(MCU_328Px8)
liboooarkavr_arduino_328Px8_la_SOURCES = $(liboooarkavr_arduino_SOURCES_)
liboooarkavr_arduino_328Px8_la_CFLAGS = $(ARDUINO_CFLAGS)
liboooarkavr_arduino_328Px8_la_CXXFLAGS = $(ARDUINO_CXXFLAGS)

lib_LTLIBRARIES += liboooarkavr_328Px16.la
liboooarkavr_328Px16_la_LIBADD = $(LDADD_common)
liboooarkavr_328Px16_la_CPPFLAGS = $(CPPFLAGS_common) $(MCU_328Px16)
liboooarkavr_328Px16_la_LDFLAGS = $(LDFLAGS_common) $(MCU_328Px16)
liboooarkavr_328Px16_la_SOURCES = $(liboooarkavr_SOURCES_)

lib_LTLIBRARIES += liboooarkavr_arduino_328Px16.la
liboooarkavr_arduino_328Px16_la_LIBADD = $(LDADD_common)
liboooarkavr_arduino_328Px16_la_CPPFLAGS = $(CPPFLAGS_common) $(MCU_328Px16)
liboooarkavr_arduino_328Px16_la_LDFLAGS = $(LDFLAGS_common) $(MCU_328Px16)
liboooarkavr_arduino_328Px16_la_SOURCES = $(liboooarkavr_arduino_SOURCES_)
liboooarkavr_arduino_328Px16_la_CFLAGS = $(ARDUINO_CFLAGS)
liboooarkavr_arduino_328Px16_la_CXXFLAGS = $(ARDUINO_CXXFLAGS)

lib_LTLIBRARIES += liboooarkavr_1280x16.la
liboooarkavr_1280x16_la_LIBADD = $(LDADD_common)
liboooarkavr_1280x16_la_CPPFLAGS = $(CPPFLAGS_common) $(MCU_1280x16)
liboooarkavr_1280x16_la_LDFLAGS = $(LDFLAGS_common) $(MCU_1280x16)
liboooarkavr_1280x16_la_SOURCES = $(liboooarkavr_SOURCES_)

lib_LTLIBRARIES += liboooarkavr_arduino_1280x16.la
liboooarkavr_arduino_1280x16_la_LIBADD = $(LDADD_common)
liboooarkavr_arduino_1280x16_la_CPPFLAGS = $(CPPFLAGS_common) $(MCU_1280x16)
liboooarkavr_arduino_1280x16_la_LDFLAGS = $(LDFLAGS_common) $(MCU_1280x16)
liboooarkavr_arduino_1280x16_la_SOURCES = $(liboooarkavr_arduino_SOURCES_)
liboooarkavr_arduino_1280x16_la_CFLAGS = $(ARDUINO_CFLAGS)
liboooarkavr_arduino_1280x16_la_CXXFLAGS = $(ARDUINO_CXXFLAGS)

lib_LTLIBRARIES += liboooarkavr_168x16.la
liboooarkavr_168x16_la_LIBADD = $(LDADD_common)
liboooarkavr_168x16_la_CPPFLAGS = $(CPPFLAGS_common) $(MCU_168x16)
liboooarkavr_168x16_la_LDFLAGS = $(LDFLAGS_common) $(MCU_168x16)
liboooarkavr_168x16_la_SOURCES = $(liboooarkavr_SOURCES_)

lib_LTLIBRARIES += liboooarkavr_arduino_168x16.la
liboooarkavr_arduino_168x16_la_LIBADD = $(LDADD_common)
liboooarkavr_arduino_168x16_la_CPPFLAGS = $(CPPFLAGS_common) $(MCU_168x16)
liboooarkavr_arduino_168x16_la_LDFLAGS = $(LDFLAGS_common) $(MCU_168x16)
liboooarkavr_arduino_168x16_la_SOURCES = $(liboooarkavr_arduino_SOURCES_)
liboooarkavr_arduino_168x16_la_CFLAGS = $(ARDUINO_CFLAGS)
liboooarkavr_arduino_168x16_la_CXXFLAGS = $(ARDUINO_CXXFLAGS)
