# 
# Automatically generated makefile from Xtensa Xplorer - DO NOT EDIT
# 

%:: RCS/%,v
%:: RCS/%
%:: s.%
%:: SCCS/s.%
.SUFFIXES:
.SUFFIXES: .a .o .c .cc .C .cpp .s .S .h

COREFLAG_nxp_rt600_RI2019_newlib_TC = --xtensa-system=C:/usr/xtensa/XtDevTools/install/builds/RI-2019.1-win32/nxp_rt600_RI2019_newlib/config --xtensa-core=nxp_rt600_RI2019_newlib
COREFLAG_nxp_rt600_RI2019_newlib = --xtensa-system=C:/usr/xtensa/XtDevTools/install/builds/RI-2019.1-win32/nxp_rt600_RI2019_newlib/config --xtensa-core=nxp_rt600_RI2019_newlib --xtensa-params=

FEEDBACK_CREATE = C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/bin/nxp_rt600_RI2019_newlib/Release_Feedback/feedback
FEEDBACK_OPT =  -fb_opt C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/bin/nxp_rt600_RI2019_newlib/Release_Feedback/feedback

VPATH = C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa
.PHONY:all clean submake 
.PHONY:pre-all pre-link post-all pre-clean  
all:pre-all eight_channel_dmic_i2s_playback post-all 

# Variables
PERL = C:/usr/xtensa/XtDevTools/install/tools/RI-2019.1-win32/XtensaTools/Tools/perl/bin/MSWin32-x86/perl.exe
WORKSPACE_ROOT = C:/usr/xtensa/Xplorer-8.0.10-workspaces/workspace
PROJECT_HOME = C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa
PROJECT_NAME = eight_channel_dmic_i2s_playback
TARGET_CONFIG = nxp_rt600_RI2019_newlib
TARGET_BUILD = nxp_rt600_RI2019_newlib
TARGET_TIE = lpc_sigmoid
STAMP = echo timestamp >

.PRECIOUS:%.props 
%.props:
	@$(PERL) -e 'if (! -f "$@") { open(F,">$@"); print F "timestamp"; close(F); }' 

# Link command
eight_channel_dmic_i2s_playback:__exe__eight_channel_dmic_i2s_playback__.pstamp MIMXRT685S/system_MIMXRT685S_dsp.o \
	board/board_hifi4.o board/pin_mux.o component/lists/generic_list.o component/serial_manager/serial_manager.o \
	component/serial_manager/serial_port_uart.o component/uart/usart_adapter.o drivers/fsl_clock.o \
	drivers/fsl_common.o drivers/fsl_flexcomm.o drivers/fsl_gpio.o drivers/fsl_reset.o \
	drivers/fsl_usart.o drivers_audio/fsl_dma.o drivers_audio/fsl_dmic.o drivers_audio/fsl_dmic_dma.o \
	drivers_audio/fsl_i2s.o drivers_audio/fsl_i2s_dma.o drivers_audio/fsl_inputmux.o \
	user/main.o user/user_pin_mux.o utilities/fsl_debug_console.o utilities/fsl_str.o \
	C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/bin/nxp_rt600_RI2019_newlib/Release/objfiles.list \
	C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/devices/MIMXRT685S/xtensa/min-rt/ldscripts/elf32xtensa.x \
	C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/devices/MIMXRT685S/xtensa/min-rt/specs \
	pre-link 
	@-echo " "
	C:/usr/xtensa/XtDevTools/install/tools/RI-2019.1-win32/XtensaTools/bin/xt-xc++ -Wl,@C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/bin/nxp_rt600_RI2019_newlib/Release/objfiles.list -o eight_channel_dmic_i2s_playback -mlsp=C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/../../../../../../devices/MIMXRT685S/xtensa/min-rt -lxos $(COREFLAG_nxp_rt600_RI2019_newlib) 
	@-echo " "
	C:/usr/xtensa/XtDevTools/install/tools/RI-2019.1-win32/XtensaTools/bin/xt-xc++ -Wl,@C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/bin/nxp_rt600_RI2019_newlib/Release/objfiles.list -o eight_channel_dmic_i2s_playback.minsize -mlsp=C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/../../../../../../devices/MIMXRT685S/xtensa/min-rt -Wl,-r -nostdlib -lxos $(COREFLAG_nxp_rt600_RI2019_newlib) 
	@-echo " "
	-@$(PERL) -e 'open(O, ">__exe__eight_channel_dmic_i2s_playback__.props"); print O "-mlsp=C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/../../../../../../devices/MIMXRT685S/xtensa/min-rt -lxos $(COREFLAG_nxp_rt600_RI2019_newlib) ";'

	@echo ***** Code size summary ************
	C:/usr/xtensa/XtDevTools/install/tools/RI-2019.1-win32/XtensaTools/bin/xt-size C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/bin/nxp_rt600_RI2019_newlib/Release/eight_channel_dmic_i2s_playback -C 
	@echo ************************************
__exe__eight_channel_dmic_i2s_playback__.pstamp:__exe__eight_channel_dmic_i2s_playback__.props \
	FORCE 
	-@$(PERL) -e 'open(I, "$<"); while (<I>) { chomp; if ("-mlsp=C:/Users/nxf55062/Documents/MCU_SDK/RT600/SDK_2.8.0_EVK-MIMXRT685_RC2_0/boards/evkmimxrt685/dsp_examples/RT600_DMIC_AUDIO/dsp/xtensa/../../../../../../devices/MIMXRT685S/xtensa/min-rt -lxos $(COREFLAG_nxp_rt600_RI2019_newlib) " ne $$_) { open(O, ">$@"); print O "timestamp"; unlink "__exe__eight_channel_dmic_i2s_playback__.d"; } exit 0; }'



MIMXRT685S/system_MIMXRT685S_dsp.o board/board_hifi4.o board/pin_mux.o component/lists/generic_list.o \
	component/serial_manager/serial_manager.o component/serial_manager/serial_port_uart.o \
	component/uart/usart_adapter.o drivers/fsl_clock.o drivers/fsl_common.o drivers/fsl_flexcomm.o \
	drivers/fsl_gpio.o drivers/fsl_reset.o drivers/fsl_usart.o drivers_audio/fsl_dma.o \
	drivers_audio/fsl_dmic.o drivers_audio/fsl_dmic_dma.o drivers_audio/fsl_i2s.o drivers_audio/fsl_i2s_dma.o \
	drivers_audio/fsl_inputmux.o user/main.o user/user_pin_mux.o utilities/fsl_debug_console.o \
	utilities/fsl_str.o :submake 
	

submake:FORCE 
	$(MAKE) -C MIMXRT685S all 
	$(MAKE) -C component\serial_manager all 
	$(MAKE) -C drivers_audio all 
	$(MAKE) -C component\uart all 
	$(MAKE) -C drivers all 
	$(MAKE) -C utilities all 
	$(MAKE) -C component\lists all 
	$(MAKE) -C user all 
	$(MAKE) -C board all 

FORCE:


clean:pre-clean 
	-@del "eight_channel_dmic_i2s_playback" "eight_channel_dmic_i2s_playback" "eight_channel_dmic_i2s_playback.ipalib" \
	"eight_channel_dmic_i2s_playback.minsize" "eight_channel_dmic_i2s_playback.minsize.a" \
	"__exe__eight_channel_dmic_i2s_playback__.pstamp" "__exe__eight_channel_dmic_i2s_playback__.props" \
	"eight_channel_dmic_i2s_playback.map" "eight_channel_dmic_i2s_playback-minsize.map" \
	"eight_channel_dmic_i2s_playback.ipakeep.pl" "eight_channel_dmic_i2s_playback.minsize.ipakeep.pl" \
	"eight_channel_dmic_i2s_playback.opt" "eight_channel_dmic_i2s_playback.minsize.opt" \
	"eight_channel_dmic_i2s_playback.bin"  >NUL 2>&1
	-@del "gmon.xx.bdelay" "gmon.xx.cyc" "gmon.xx.dcmiss" "gmon.xx.dcmiss_cyc" "gmon.xx.icmiss" \
	"gmon.xx.icmiss_cyc" "gmon.xx.cc_miss" "gmon.xx.cc_miss_cyc" "gmon.xx.interlock" \
	"gmon.xx.insns" "gmon.xx.cenergy" "gmon.xx.menergy" "gmon.xx.energy"  >NUL 2>&1
	-@cmd /c if exist eight_channel_dmic_i2s_playback.ipakeep rmdir /q /s eight_channel_dmic_i2s_playback.ipakeep
	-@cmd /c if exist eight_channel_dmic_i2s_playback.minsize.ipakeep rmdir /q /s eight_channel_dmic_i2s_playback.minsize.ipakeep
	-@del "MIMXRT685S\system_MIMXRT685S_dsp.o" "MIMXRT685S\system_MIMXRT685S_dsp.d" \
	"MIMXRT685S\system_MIMXRT685S_dsp.pstamp" "MIMXRT685S\system_MIMXRT685S_dsp.props" \
	"MIMXRT685S\system_MIMXRT685S_dsp.B" "MIMXRT685S\system_MIMXRT685S_dsp.i" "MIMXRT685S\system_MIMXRT685S_dsp.spin" \
	"MIMXRT685S\system_MIMXRT685S_dsp.opt"  >NUL 2>&1
	-@del "board\board_hifi4.o" "board\board_hifi4.d" "board\board_hifi4.pstamp" "board\board_hifi4.props" \
	"board\board_hifi4.B" "board\board_hifi4.i" "board\board_hifi4.spin" "board\board_hifi4.opt" \
	 >NUL 2>&1
	-@del "board\pin_mux.o" "board\pin_mux.d" "board\pin_mux.pstamp" "board\pin_mux.props" \
	"board\pin_mux.B" "board\pin_mux.i" "board\pin_mux.spin" "board\pin_mux.opt"  >NUL 2>&1
	-@del "component\lists\generic_list.o" "component\lists\generic_list.d" "component\lists\generic_list.pstamp" \
	"component\lists\generic_list.props" "component\lists\generic_list.B" "component\lists\generic_list.i" \
	"component\lists\generic_list.spin" "component\lists\generic_list.opt"  >NUL 2>&1
	-@del "component\serial_manager\serial_manager.o" "component\serial_manager\serial_manager.d" \
	"component\serial_manager\serial_manager.pstamp" "component\serial_manager\serial_manager.props" \
	"component\serial_manager\serial_manager.B" "component\serial_manager\serial_manager.i" \
	"component\serial_manager\serial_manager.spin" "component\serial_manager\serial_manager.opt" \
	 >NUL 2>&1
	-@del "component\serial_manager\serial_port_uart.o" "component\serial_manager\serial_port_uart.d" \
	"component\serial_manager\serial_port_uart.pstamp" "component\serial_manager\serial_port_uart.props" \
	"component\serial_manager\serial_port_uart.B" "component\serial_manager\serial_port_uart.i" \
	"component\serial_manager\serial_port_uart.spin" "component\serial_manager\serial_port_uart.opt" \
	 >NUL 2>&1
	-@del "component\uart\usart_adapter.o" "component\uart\usart_adapter.d" "component\uart\usart_adapter.pstamp" \
	"component\uart\usart_adapter.props" "component\uart\usart_adapter.B" "component\uart\usart_adapter.i" \
	"component\uart\usart_adapter.spin" "component\uart\usart_adapter.opt"  >NUL 2>&1
	-@del "drivers\fsl_clock.o" "drivers\fsl_clock.d" "drivers\fsl_clock.pstamp" "drivers\fsl_clock.props" \
	"drivers\fsl_clock.B" "drivers\fsl_clock.i" "drivers\fsl_clock.spin" "drivers\fsl_clock.opt" \
	 >NUL 2>&1
	-@del "drivers\fsl_common.o" "drivers\fsl_common.d" "drivers\fsl_common.pstamp" \
	"drivers\fsl_common.props" "drivers\fsl_common.B" "drivers\fsl_common.i" "drivers\fsl_common.spin" \
	"drivers\fsl_common.opt"  >NUL 2>&1
	-@del "drivers\fsl_flexcomm.o" "drivers\fsl_flexcomm.d" "drivers\fsl_flexcomm.pstamp" \
	"drivers\fsl_flexcomm.props" "drivers\fsl_flexcomm.B" "drivers\fsl_flexcomm.i" "drivers\fsl_flexcomm.spin" \
	"drivers\fsl_flexcomm.opt"  >NUL 2>&1
	-@del "drivers\fsl_gpio.o" "drivers\fsl_gpio.d" "drivers\fsl_gpio.pstamp" "drivers\fsl_gpio.props" \
	"drivers\fsl_gpio.B" "drivers\fsl_gpio.i" "drivers\fsl_gpio.spin" "drivers\fsl_gpio.opt" \
	 >NUL 2>&1
	-@del "drivers\fsl_reset.o" "drivers\fsl_reset.d" "drivers\fsl_reset.pstamp" "drivers\fsl_reset.props" \
	"drivers\fsl_reset.B" "drivers\fsl_reset.i" "drivers\fsl_reset.spin" "drivers\fsl_reset.opt" \
	 >NUL 2>&1
	-@del "drivers\fsl_usart.o" "drivers\fsl_usart.d" "drivers\fsl_usart.pstamp" "drivers\fsl_usart.props" \
	"drivers\fsl_usart.B" "drivers\fsl_usart.i" "drivers\fsl_usart.spin" "drivers\fsl_usart.opt" \
	 >NUL 2>&1
	-@del "drivers_audio\fsl_dma.o" "drivers_audio\fsl_dma.d" "drivers_audio\fsl_dma.pstamp" \
	"drivers_audio\fsl_dma.props" "drivers_audio\fsl_dma.B" "drivers_audio\fsl_dma.i" \
	"drivers_audio\fsl_dma.spin" "drivers_audio\fsl_dma.opt"  >NUL 2>&1
	-@del "drivers_audio\fsl_dmic.o" "drivers_audio\fsl_dmic.d" "drivers_audio\fsl_dmic.pstamp" \
	"drivers_audio\fsl_dmic.props" "drivers_audio\fsl_dmic.B" "drivers_audio\fsl_dmic.i" \
	"drivers_audio\fsl_dmic.spin" "drivers_audio\fsl_dmic.opt"  >NUL 2>&1
	-@del "drivers_audio\fsl_dmic_dma.o" "drivers_audio\fsl_dmic_dma.d" "drivers_audio\fsl_dmic_dma.pstamp" \
	"drivers_audio\fsl_dmic_dma.props" "drivers_audio\fsl_dmic_dma.B" "drivers_audio\fsl_dmic_dma.i" \
	"drivers_audio\fsl_dmic_dma.spin" "drivers_audio\fsl_dmic_dma.opt"  >NUL 2>&1
	-@del "drivers_audio\fsl_i2s.o" "drivers_audio\fsl_i2s.d" "drivers_audio\fsl_i2s.pstamp" \
	"drivers_audio\fsl_i2s.props" "drivers_audio\fsl_i2s.B" "drivers_audio\fsl_i2s.i" \
	"drivers_audio\fsl_i2s.spin" "drivers_audio\fsl_i2s.opt"  >NUL 2>&1
	-@del "drivers_audio\fsl_i2s_dma.o" "drivers_audio\fsl_i2s_dma.d" "drivers_audio\fsl_i2s_dma.pstamp" \
	"drivers_audio\fsl_i2s_dma.props" "drivers_audio\fsl_i2s_dma.B" "drivers_audio\fsl_i2s_dma.i" \
	"drivers_audio\fsl_i2s_dma.spin" "drivers_audio\fsl_i2s_dma.opt"  >NUL 2>&1
	-@del "drivers_audio\fsl_inputmux.o" "drivers_audio\fsl_inputmux.d" "drivers_audio\fsl_inputmux.pstamp" \
	"drivers_audio\fsl_inputmux.props" "drivers_audio\fsl_inputmux.B" "drivers_audio\fsl_inputmux.i" \
	"drivers_audio\fsl_inputmux.spin" "drivers_audio\fsl_inputmux.opt"  >NUL 2>&1
	-@del "user\main.o" "user\main.d" "user\main.pstamp" "user\main.props" "user\main.B" \
	"user\main.i" "user\main.spin" "user\main.opt"  >NUL 2>&1
	-@del "user\user_pin_mux.o" "user\user_pin_mux.d" "user\user_pin_mux.pstamp" "user\user_pin_mux.props" \
	"user\user_pin_mux.B" "user\user_pin_mux.i" "user\user_pin_mux.spin" "user\user_pin_mux.opt" \
	 >NUL 2>&1
	-@del "utilities\fsl_debug_console.o" "utilities\fsl_debug_console.d" "utilities\fsl_debug_console.pstamp" \
	"utilities\fsl_debug_console.props" "utilities\fsl_debug_console.B" "utilities\fsl_debug_console.i" \
	"utilities\fsl_debug_console.spin" "utilities\fsl_debug_console.opt"  >NUL 2>&1
	-@del "utilities\fsl_str.o" "utilities\fsl_str.d" "utilities\fsl_str.pstamp" "utilities\fsl_str.props" \
	"utilities\fsl_str.B" "utilities\fsl_str.i" "utilities\fsl_str.spin" "utilities\fsl_str.opt" \
	 >NUL 2>&1


# Dummies in case user does not provide some targets

pre-all:

pre-link:

post-all:

pre-clean:


# User included Makefile.include below here

# get build target from current directory and make it's first letter lowercase
build_target = $(notdir $(CURDIR))
build_target := $(subst R,r,$(build_target))
build_target := $(subst D,d,$(build_target))

post-all:
	@echo post all rule
	@echo --xtensa-system=C:/usr/xtensa/XtDevTools/install/builds/RI-2019.1-win32/nxp_rt600_RI2019_newlib/config --xtensa-core=nxp_rt600_RI2019_newlib --xtensa-params=
	@echo eight_channel_dmic_i2s_playback
	xt-objcopy --xtensa-system=C:/usr/xtensa/XtDevTools/install/builds/RI-2019.1-win32/nxp_rt600_RI2019_newlib/config --xtensa-core=nxp_rt600_RI2019_newlib --xtensa-params= -O binary eight_channel_dmic_i2s_playback ../../../../binary/dsp_text_$(build_target).bin \
--only-section=.dram0.rodata \
--only-section=.Level2InterruptVector.literal \
--only-section=.Level3InterruptVector.literal \
--only-section=.DebugExceptionVector.literal \
--only-section=.NMIExceptionVector.literal \
--only-section=.KernelExceptionVector.literal \
--only-section=.UserExceptionVector.literal \
--only-section=.DoubleExceptionVector.literal \
--only-section=.iram0.literal \
--only-section=.ResetVector.literal \
--only-section=.dram0.literal \
--only-section=.dram0.data \
--only-section=.dram0.bss \
--only-section=.ResetVector.text \
--only-section=.ResetHandler.literal \
--only-section=.ResetHandler.text \
--only-section=.WindowVectors.text \
--only-section=.Level2InterruptVector.text \
--only-section=.Level3InterruptVector.text \
--only-section=.DebugExceptionVector.text \
--only-section=.NMIExceptionVector.text \
--only-section=.KernelExceptionVector.text \
--only-section=.UserExceptionVector.text \
--only-section=.DoubleExceptionVector.text \
--only-section=.iram0.text
	xt-objcopy --xtensa-system=C:/usr/xtensa/XtDevTools/install/builds/RI-2019.1-win32/nxp_rt600_RI2019_newlib/config --xtensa-core=nxp_rt600_RI2019_newlib --xtensa-params= -O binary eight_channel_dmic_i2s_playback ../../../../binary/dsp_data_$(build_target).bin \
--only-section=.text \
--only-section=.rodata \
--only-section=.data \
--only-section=.bss \
--only-section=.literal \
--only-section=.sram.text \
--only-section=.sram.rodata \
--only-section=.sram.data \
--only-section=.sram.bss \
--only-section=.sram.literal \
--only-section=.clib.rodata \
--only-section=.clib.text \
--only-section=.clib.data \
--only-section=.clib.data \
--only-section=.clib.percpu.data \
--only-section=.clib.bss \
--only-section=.clib.percpu.bss \
--only-section=.clib.literal \
--only-section=.rtos.rodata \
--only-section=.rtos.text \
--only-section=.rtos.data \
--only-section=.rtos.percpu.data \
--only-section=.rtos.bss \
--only-section=.rtos.percpu.bss \
--only-section=.rtos.literal
	xt-objcopy --xtensa-system=C:/usr/xtensa/XtDevTools/install/builds/RI-2019.1-win32/nxp_rt600_RI2019_newlib/config --xtensa-core=nxp_rt600_RI2019_newlib --xtensa-params= -O binary eight_channel_dmic_i2s_playback ../../../../binary/dsp_ncache_$(build_target).bin \
--only-section=NonCacheable

#
bin_path = ../../../../binary/
ifdef OS
	# windows
	rm_cmd = del /Q
	bin_path := $(subst /,\,$(bin_path))
else
	# unix
	rm_cmd = rm -f
endif

pre-clean:
	@echo pre-clean rule
	cmd /c if exist "$(bin_path)dsp_text_$(build_target).bin" $(rm_cmd) "$(bin_path)dsp_text_$(build_target).bin"
	cmd /c if exist "$(bin_path)dsp_data_$(build_target).bin" $(rm_cmd) "$(bin_path)dsp_data_$(build_target).bin"
	cmd /c if exist "$(bin_path)dsp_ncache_$(build_target).bin" $(rm_cmd) "$(bin_path)dsp_ncache_$(build_target).bin"
