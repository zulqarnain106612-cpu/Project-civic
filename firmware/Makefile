HOST_CC      ?= cc
HOST_CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -pedantic -O2
ARM_GCC      ?= /home/s752/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc
ARM_SIZE     ?= /home/s752/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-size
ARM_OBJCOPY  ?= /home/s752/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-objcopy
CMSIS_INC     = Drivers/CMSIS
HAL_INC       = Drivers/STM32F4xx_HAL_Driver/Inc

# ── Transceiver: TJA1051 (NXP) — ONLY supported CAN transceiver. ───────
# No substitutes, variants, or successors are permitted.
# Set TRANSCEIVER_SILENT_CONFIRMED=1 after physically verifying the TJA1051
# wiring (STB=GND, CANH/CANL to OBD-II bus pins 6 & 14, VCC supplied).
# Safe defaults: CAN stays off until TJA1051 silent-mode is confirmed.
CAN_BITRATE_HZ             ?= 0
TRANSCEIVER_SILENT_CONFIRMED ?= 0
CAN_TX_CONFIRMED             ?= 0
USE_HSE_168MHZ               ?= 0

ARM_FLAGS = \
  -mcpu=cortex-m4 -mthumb \
  -DSTM32F407xx \
  -DPROJECT_CAN_BITRATE_HZ=$(CAN_BITRATE_HZ)u \
  -DPROJECT_TRANSCEIVER_SILENT_CONFIRMED=$(TRANSCEIVER_SILENT_CONFIRMED)u \
  -DPROJECT_TRANSCEIVER_TJA1051=1u \
  -DPROJECT_CAN_TX_CONFIRMED=$(CAN_TX_CONFIRMED)u \
  -DPROJECT_USE_HSE_168MHZ=$(USE_HSE_168MHZ)u \
  -ffunction-sections -fdata-sections \
  -Wall -Wextra -Werror -Os \
  -ICore/Inc \
  -I$(CMSIS_INC) \
  -I$(HAL_INC)

# All firmware source files
FIRMWARE_SRCS = \
  Core/Src/main.c \
  Core/Src/can_frame_queue.c \
  Core/Src/can_frame_format.c \
  Core/Src/can_signal_profiler.c \
  Core/Src/obd2_decoder.c \
  Core/Src/system_stm32f4xx.c \
  Core/Src/uart_dma.c \
  startup_stm32f407xx.s

.PHONY: test-host test-sanitize firmware firmware-tx clean

# ── Shortcut: passive capture build (TJA1051 confirmed, no TX) ─────────
firmware: build/project_civic.elf build/project_civic.bin
	$(ARM_SIZE) build/project_civic.elf

# ── Shortcut: bidirectional build (TJA1051 confirmed + TX enabled) ──────
firmware-tx: TRANSCEIVER_SILENT_CONFIRMED=1
firmware-tx: CAN_TX_CONFIRMED=1
firmware-tx: build/project_civic.elf
	$(ARM_SIZE) $<

build/project_civic.elf: $(FIRMWARE_SRCS)
	mkdir -p build
	$(ARM_GCC) $(ARM_FLAGS) -g --specs=nosys.specs \
	  -TSTM32F407VGTX_FLASH.ld \
	  -Wl,--gc-sections \
	  -Wl,-Map=build/project_civic.map \
	  $(FIRMWARE_SRCS) -o $@

build/project_civic.bin: build/project_civic.elf
	$(ARM_OBJCOPY) -O binary $< $@
	@echo "Flat binary: $@ (upload this to dashboard for STM32 OTA)"

# ── Host tests ──────────────────────────────────────────────────────────
TEST_SRCS_C = \
  Core/Src/can_frame_queue.c \
  Core/Src/can_frame_format.c \
  Core/Src/can_signal_profiler.c \
  Core/Src/obd2_decoder.c

build/test_can_frame_queue: tests/test_can_frame_queue.c $(TEST_SRCS_C)
	mkdir -p build
	$(HOST_CC) $(HOST_CFLAGS) -ICore/Inc \
	  tests/test_can_frame_queue.c \
	  Core/Src/can_frame_queue.c Core/Src/can_frame_format.c -o $@

build/test_obd2_decoder: tests/test_obd2_decoder.c $(TEST_SRCS_C)
	mkdir -p build
	$(HOST_CC) $(HOST_CFLAGS) -ICore/Inc \
	  tests/test_obd2_decoder.c \
	  Core/Src/obd2_decoder.c Core/Src/can_frame_queue.c -o $@

build/test_can_signal_profiler: tests/test_can_signal_profiler.c $(TEST_SRCS_C)
	mkdir -p build
	$(HOST_CC) $(HOST_CFLAGS) -ICore/Inc \
	  tests/test_can_signal_profiler.c \
	  Core/Src/can_signal_profiler.c Core/Src/can_frame_queue.c -o $@

build/test_can_frame_queue_san: tests/test_can_frame_queue.c $(TEST_SRCS_C)
	mkdir -p build
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -pedantic -O1 -g \
	  -fsanitize=address,undefined -fno-omit-frame-pointer \
	  -ICore/Inc \
	  tests/test_can_frame_queue.c \
	  Core/Src/can_frame_queue.c Core/Src/can_frame_format.c -o $@

build/test_obd2_decoder_san: tests/test_obd2_decoder.c $(TEST_SRCS_C)
	mkdir -p build
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -pedantic -O1 -g \
	  -fsanitize=address,undefined -fno-omit-frame-pointer \
	  -ICore/Inc \
	  tests/test_obd2_decoder.c \
	  Core/Src/obd2_decoder.c Core/Src/can_frame_queue.c -o $@

build/test_can_signal_profiler_san: tests/test_can_signal_profiler.c $(TEST_SRCS_C)
	mkdir -p build
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -pedantic -O1 -g \
	  -fsanitize=address,undefined -fno-omit-frame-pointer \
	  -ICore/Inc \
	  tests/test_can_signal_profiler.c \
	  Core/Src/can_signal_profiler.c Core/Src/can_frame_queue.c -o $@

test-host: build/test_can_frame_queue build/test_obd2_decoder build/test_can_signal_profiler
	./build/test_can_frame_queue
	./build/test_obd2_decoder
	./build/test_can_signal_profiler
	PYTHONPATH=tools python3 -m unittest discover -s tests -p 'test_*.py' -v

test-sanitize: build/test_can_frame_queue_san build/test_obd2_decoder_san build/test_can_signal_profiler_san
	./build/test_can_frame_queue_san
	./build/test_obd2_decoder_san
	./build/test_can_signal_profiler_san

clean:
	rm -rf build
