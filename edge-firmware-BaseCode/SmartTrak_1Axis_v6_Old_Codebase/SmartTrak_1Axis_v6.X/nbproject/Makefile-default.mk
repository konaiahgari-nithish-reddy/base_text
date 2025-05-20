#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/SmartTrak_1Axis_v6.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/SmartTrak_1Axis_v6.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../BSP/SerialPort.c ../BSP/SST25VF016.c ../BSP/DS3232.c ../BSP/I2CBus.c ../BSP/PCA9554.c ../BSP/mma845x.c ../BSP/SerialTimer.c ../BSP/ADC10Read.c ../source/Debug.c ../source/main.c ../source/SerialDisplay.c ../source/MenuFSM.c ../source/Stubs.c ../source/ftoa.c ../source/StrConversions.c ../source/TimeDelay.c ../source/SystemParameters.c ../source/MotorPWM.c ../source/MotionStats.c ../source/MotionSensor.c ../source/AppTimer.c ../source/MotionFSM.c ../source/MotionLimits.c ../source/MotionPhaseFSM.c ../source/MoveSequenceFSM.c ../source/CoordTranslate.c ../source/SunPosition.c ../source/RTCC.c ../source/Debounce.c ../source/ButtonProcessingFSM.c ../source/LEDs.c ../source/Inclinometer.c ../source/RxMessage.c ../source/UpdateParameters.c ../source/PanelPositionFSM.c ../source/Zigbee.c ../source/BMS.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360889168/SerialPort.o ${OBJECTDIR}/_ext/1360889168/SST25VF016.o ${OBJECTDIR}/_ext/1360889168/DS3232.o ${OBJECTDIR}/_ext/1360889168/I2CBus.o ${OBJECTDIR}/_ext/1360889168/PCA9554.o ${OBJECTDIR}/_ext/1360889168/mma845x.o ${OBJECTDIR}/_ext/1360889168/SerialTimer.o ${OBJECTDIR}/_ext/1360889168/ADC10Read.o ${OBJECTDIR}/_ext/812168374/Debug.o ${OBJECTDIR}/_ext/812168374/main.o ${OBJECTDIR}/_ext/812168374/SerialDisplay.o ${OBJECTDIR}/_ext/812168374/MenuFSM.o ${OBJECTDIR}/_ext/812168374/Stubs.o ${OBJECTDIR}/_ext/812168374/ftoa.o ${OBJECTDIR}/_ext/812168374/StrConversions.o ${OBJECTDIR}/_ext/812168374/TimeDelay.o ${OBJECTDIR}/_ext/812168374/SystemParameters.o ${OBJECTDIR}/_ext/812168374/MotorPWM.o ${OBJECTDIR}/_ext/812168374/MotionStats.o ${OBJECTDIR}/_ext/812168374/MotionSensor.o ${OBJECTDIR}/_ext/812168374/AppTimer.o ${OBJECTDIR}/_ext/812168374/MotionFSM.o ${OBJECTDIR}/_ext/812168374/MotionLimits.o ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o ${OBJECTDIR}/_ext/812168374/CoordTranslate.o ${OBJECTDIR}/_ext/812168374/SunPosition.o ${OBJECTDIR}/_ext/812168374/RTCC.o ${OBJECTDIR}/_ext/812168374/Debounce.o ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o ${OBJECTDIR}/_ext/812168374/LEDs.o ${OBJECTDIR}/_ext/812168374/Inclinometer.o ${OBJECTDIR}/_ext/812168374/RxMessage.o ${OBJECTDIR}/_ext/812168374/UpdateParameters.o ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o ${OBJECTDIR}/_ext/812168374/Zigbee.o ${OBJECTDIR}/_ext/812168374/BMS.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360889168/SerialPort.o.d ${OBJECTDIR}/_ext/1360889168/SST25VF016.o.d ${OBJECTDIR}/_ext/1360889168/DS3232.o.d ${OBJECTDIR}/_ext/1360889168/I2CBus.o.d ${OBJECTDIR}/_ext/1360889168/PCA9554.o.d ${OBJECTDIR}/_ext/1360889168/mma845x.o.d ${OBJECTDIR}/_ext/1360889168/SerialTimer.o.d ${OBJECTDIR}/_ext/1360889168/ADC10Read.o.d ${OBJECTDIR}/_ext/812168374/Debug.o.d ${OBJECTDIR}/_ext/812168374/main.o.d ${OBJECTDIR}/_ext/812168374/SerialDisplay.o.d ${OBJECTDIR}/_ext/812168374/MenuFSM.o.d ${OBJECTDIR}/_ext/812168374/Stubs.o.d ${OBJECTDIR}/_ext/812168374/ftoa.o.d ${OBJECTDIR}/_ext/812168374/StrConversions.o.d ${OBJECTDIR}/_ext/812168374/TimeDelay.o.d ${OBJECTDIR}/_ext/812168374/SystemParameters.o.d ${OBJECTDIR}/_ext/812168374/MotorPWM.o.d ${OBJECTDIR}/_ext/812168374/MotionStats.o.d ${OBJECTDIR}/_ext/812168374/MotionSensor.o.d ${OBJECTDIR}/_ext/812168374/AppTimer.o.d ${OBJECTDIR}/_ext/812168374/MotionFSM.o.d ${OBJECTDIR}/_ext/812168374/MotionLimits.o.d ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o.d ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o.d ${OBJECTDIR}/_ext/812168374/CoordTranslate.o.d ${OBJECTDIR}/_ext/812168374/SunPosition.o.d ${OBJECTDIR}/_ext/812168374/RTCC.o.d ${OBJECTDIR}/_ext/812168374/Debounce.o.d ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o.d ${OBJECTDIR}/_ext/812168374/LEDs.o.d ${OBJECTDIR}/_ext/812168374/Inclinometer.o.d ${OBJECTDIR}/_ext/812168374/RxMessage.o.d ${OBJECTDIR}/_ext/812168374/UpdateParameters.o.d ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o.d ${OBJECTDIR}/_ext/812168374/Zigbee.o.d ${OBJECTDIR}/_ext/812168374/BMS.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360889168/SerialPort.o ${OBJECTDIR}/_ext/1360889168/SST25VF016.o ${OBJECTDIR}/_ext/1360889168/DS3232.o ${OBJECTDIR}/_ext/1360889168/I2CBus.o ${OBJECTDIR}/_ext/1360889168/PCA9554.o ${OBJECTDIR}/_ext/1360889168/mma845x.o ${OBJECTDIR}/_ext/1360889168/SerialTimer.o ${OBJECTDIR}/_ext/1360889168/ADC10Read.o ${OBJECTDIR}/_ext/812168374/Debug.o ${OBJECTDIR}/_ext/812168374/main.o ${OBJECTDIR}/_ext/812168374/SerialDisplay.o ${OBJECTDIR}/_ext/812168374/MenuFSM.o ${OBJECTDIR}/_ext/812168374/Stubs.o ${OBJECTDIR}/_ext/812168374/ftoa.o ${OBJECTDIR}/_ext/812168374/StrConversions.o ${OBJECTDIR}/_ext/812168374/TimeDelay.o ${OBJECTDIR}/_ext/812168374/SystemParameters.o ${OBJECTDIR}/_ext/812168374/MotorPWM.o ${OBJECTDIR}/_ext/812168374/MotionStats.o ${OBJECTDIR}/_ext/812168374/MotionSensor.o ${OBJECTDIR}/_ext/812168374/AppTimer.o ${OBJECTDIR}/_ext/812168374/MotionFSM.o ${OBJECTDIR}/_ext/812168374/MotionLimits.o ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o ${OBJECTDIR}/_ext/812168374/CoordTranslate.o ${OBJECTDIR}/_ext/812168374/SunPosition.o ${OBJECTDIR}/_ext/812168374/RTCC.o ${OBJECTDIR}/_ext/812168374/Debounce.o ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o ${OBJECTDIR}/_ext/812168374/LEDs.o ${OBJECTDIR}/_ext/812168374/Inclinometer.o ${OBJECTDIR}/_ext/812168374/RxMessage.o ${OBJECTDIR}/_ext/812168374/UpdateParameters.o ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o ${OBJECTDIR}/_ext/812168374/Zigbee.o ${OBJECTDIR}/_ext/812168374/BMS.o

# Source Files
SOURCEFILES=../BSP/SerialPort.c ../BSP/SST25VF016.c ../BSP/DS3232.c ../BSP/I2CBus.c ../BSP/PCA9554.c ../BSP/mma845x.c ../BSP/SerialTimer.c ../BSP/ADC10Read.c ../source/Debug.c ../source/main.c ../source/SerialDisplay.c ../source/MenuFSM.c ../source/Stubs.c ../source/ftoa.c ../source/StrConversions.c ../source/TimeDelay.c ../source/SystemParameters.c ../source/MotorPWM.c ../source/MotionStats.c ../source/MotionSensor.c ../source/AppTimer.c ../source/MotionFSM.c ../source/MotionLimits.c ../source/MotionPhaseFSM.c ../source/MoveSequenceFSM.c ../source/CoordTranslate.c ../source/SunPosition.c ../source/RTCC.c ../source/Debounce.c ../source/ButtonProcessingFSM.c ../source/LEDs.c ../source/Inclinometer.c ../source/RxMessage.c ../source/UpdateParameters.c ../source/PanelPositionFSM.c ../source/Zigbee.c ../source/BMS.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/SmartTrak_1Axis_v6.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX360F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360889168/SerialPort.o: ../BSP/SerialPort.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SerialPort.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SerialPort.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/SerialPort.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/SerialPort.o.d" -o ${OBJECTDIR}/_ext/1360889168/SerialPort.o ../BSP/SerialPort.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/SST25VF016.o: ../BSP/SST25VF016.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SST25VF016.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SST25VF016.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/SST25VF016.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/SST25VF016.o.d" -o ${OBJECTDIR}/_ext/1360889168/SST25VF016.o ../BSP/SST25VF016.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/DS3232.o: ../BSP/DS3232.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/DS3232.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/DS3232.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/DS3232.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/DS3232.o.d" -o ${OBJECTDIR}/_ext/1360889168/DS3232.o ../BSP/DS3232.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/I2CBus.o: ../BSP/I2CBus.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/I2CBus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/I2CBus.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/I2CBus.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/I2CBus.o.d" -o ${OBJECTDIR}/_ext/1360889168/I2CBus.o ../BSP/I2CBus.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/PCA9554.o: ../BSP/PCA9554.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/PCA9554.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/PCA9554.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/PCA9554.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/PCA9554.o.d" -o ${OBJECTDIR}/_ext/1360889168/PCA9554.o ../BSP/PCA9554.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/mma845x.o: ../BSP/mma845x.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/mma845x.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/mma845x.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/mma845x.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/mma845x.o.d" -o ${OBJECTDIR}/_ext/1360889168/mma845x.o ../BSP/mma845x.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/SerialTimer.o: ../BSP/SerialTimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SerialTimer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SerialTimer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/SerialTimer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/SerialTimer.o.d" -o ${OBJECTDIR}/_ext/1360889168/SerialTimer.o ../BSP/SerialTimer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/ADC10Read.o: ../BSP/ADC10Read.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/ADC10Read.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/ADC10Read.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/ADC10Read.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/ADC10Read.o.d" -o ${OBJECTDIR}/_ext/1360889168/ADC10Read.o ../BSP/ADC10Read.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Debug.o: ../source/Debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Debug.o.d" -o ${OBJECTDIR}/_ext/812168374/Debug.o ../source/Debug.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/main.o: ../source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/main.o.d" -o ${OBJECTDIR}/_ext/812168374/main.o ../source/main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/SerialDisplay.o: ../source/SerialDisplay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/SerialDisplay.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/SerialDisplay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/SerialDisplay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/SerialDisplay.o.d" -o ${OBJECTDIR}/_ext/812168374/SerialDisplay.o ../source/SerialDisplay.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MenuFSM.o: ../source/MenuFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MenuFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MenuFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MenuFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MenuFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/MenuFSM.o ../source/MenuFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Stubs.o: ../source/Stubs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Stubs.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Stubs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Stubs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Stubs.o.d" -o ${OBJECTDIR}/_ext/812168374/Stubs.o ../source/Stubs.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/ftoa.o: ../source/ftoa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/ftoa.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/ftoa.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/ftoa.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/ftoa.o.d" -o ${OBJECTDIR}/_ext/812168374/ftoa.o ../source/ftoa.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/StrConversions.o: ../source/StrConversions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/StrConversions.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/StrConversions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/StrConversions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/StrConversions.o.d" -o ${OBJECTDIR}/_ext/812168374/StrConversions.o ../source/StrConversions.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/TimeDelay.o: ../source/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/TimeDelay.o.d" -o ${OBJECTDIR}/_ext/812168374/TimeDelay.o ../source/TimeDelay.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/SystemParameters.o: ../source/SystemParameters.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/SystemParameters.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/SystemParameters.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/SystemParameters.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/SystemParameters.o.d" -o ${OBJECTDIR}/_ext/812168374/SystemParameters.o ../source/SystemParameters.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotorPWM.o: ../source/MotorPWM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotorPWM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotorPWM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotorPWM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotorPWM.o.d" -o ${OBJECTDIR}/_ext/812168374/MotorPWM.o ../source/MotorPWM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionStats.o: ../source/MotionStats.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionStats.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionStats.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionStats.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionStats.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionStats.o ../source/MotionStats.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionSensor.o: ../source/MotionSensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionSensor.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionSensor.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionSensor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionSensor.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionSensor.o ../source/MotionSensor.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/AppTimer.o: ../source/AppTimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/AppTimer.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/AppTimer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/AppTimer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/AppTimer.o.d" -o ${OBJECTDIR}/_ext/812168374/AppTimer.o ../source/AppTimer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionFSM.o: ../source/MotionFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionFSM.o ../source/MotionFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionLimits.o: ../source/MotionLimits.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionLimits.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionLimits.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionLimits.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionLimits.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionLimits.o ../source/MotionLimits.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o: ../source/MotionPhaseFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o ../source/MotionPhaseFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o: ../source/MoveSequenceFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o ../source/MoveSequenceFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/CoordTranslate.o: ../source/CoordTranslate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/CoordTranslate.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/CoordTranslate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/CoordTranslate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/CoordTranslate.o.d" -o ${OBJECTDIR}/_ext/812168374/CoordTranslate.o ../source/CoordTranslate.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/SunPosition.o: ../source/SunPosition.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/SunPosition.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/SunPosition.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/SunPosition.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/SunPosition.o.d" -o ${OBJECTDIR}/_ext/812168374/SunPosition.o ../source/SunPosition.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/RTCC.o: ../source/RTCC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/RTCC.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/RTCC.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/RTCC.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/RTCC.o.d" -o ${OBJECTDIR}/_ext/812168374/RTCC.o ../source/RTCC.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Debounce.o: ../source/Debounce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Debounce.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Debounce.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Debounce.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Debounce.o.d" -o ${OBJECTDIR}/_ext/812168374/Debounce.o ../source/Debounce.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o: ../source/ButtonProcessingFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o ../source/ButtonProcessingFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/LEDs.o: ../source/LEDs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/LEDs.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/LEDs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/LEDs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/LEDs.o.d" -o ${OBJECTDIR}/_ext/812168374/LEDs.o ../source/LEDs.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Inclinometer.o: ../source/Inclinometer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Inclinometer.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Inclinometer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Inclinometer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Inclinometer.o.d" -o ${OBJECTDIR}/_ext/812168374/Inclinometer.o ../source/Inclinometer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/RxMessage.o: ../source/RxMessage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/RxMessage.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/RxMessage.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/RxMessage.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/RxMessage.o.d" -o ${OBJECTDIR}/_ext/812168374/RxMessage.o ../source/RxMessage.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/UpdateParameters.o: ../source/UpdateParameters.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/UpdateParameters.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/UpdateParameters.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/UpdateParameters.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/UpdateParameters.o.d" -o ${OBJECTDIR}/_ext/812168374/UpdateParameters.o ../source/UpdateParameters.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o: ../source/PanelPositionFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o ../source/PanelPositionFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Zigbee.o: ../source/Zigbee.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Zigbee.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Zigbee.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Zigbee.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Zigbee.o.d" -o ${OBJECTDIR}/_ext/812168374/Zigbee.o ../source/Zigbee.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/BMS.o: ../source/BMS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/BMS.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/BMS.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/BMS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/BMS.o.d" -o ${OBJECTDIR}/_ext/812168374/BMS.o ../source/BMS.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
else
${OBJECTDIR}/_ext/1360889168/SerialPort.o: ../BSP/SerialPort.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SerialPort.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SerialPort.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/SerialPort.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/SerialPort.o.d" -o ${OBJECTDIR}/_ext/1360889168/SerialPort.o ../BSP/SerialPort.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/SST25VF016.o: ../BSP/SST25VF016.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SST25VF016.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SST25VF016.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/SST25VF016.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/SST25VF016.o.d" -o ${OBJECTDIR}/_ext/1360889168/SST25VF016.o ../BSP/SST25VF016.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/DS3232.o: ../BSP/DS3232.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/DS3232.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/DS3232.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/DS3232.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/DS3232.o.d" -o ${OBJECTDIR}/_ext/1360889168/DS3232.o ../BSP/DS3232.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/I2CBus.o: ../BSP/I2CBus.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/I2CBus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/I2CBus.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/I2CBus.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/I2CBus.o.d" -o ${OBJECTDIR}/_ext/1360889168/I2CBus.o ../BSP/I2CBus.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/PCA9554.o: ../BSP/PCA9554.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/PCA9554.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/PCA9554.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/PCA9554.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/PCA9554.o.d" -o ${OBJECTDIR}/_ext/1360889168/PCA9554.o ../BSP/PCA9554.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/mma845x.o: ../BSP/mma845x.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/mma845x.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/mma845x.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/mma845x.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/mma845x.o.d" -o ${OBJECTDIR}/_ext/1360889168/mma845x.o ../BSP/mma845x.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/SerialTimer.o: ../BSP/SerialTimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SerialTimer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/SerialTimer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/SerialTimer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/SerialTimer.o.d" -o ${OBJECTDIR}/_ext/1360889168/SerialTimer.o ../BSP/SerialTimer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/1360889168/ADC10Read.o: ../BSP/ADC10Read.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360889168" 
	@${RM} ${OBJECTDIR}/_ext/1360889168/ADC10Read.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360889168/ADC10Read.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360889168/ADC10Read.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360889168/ADC10Read.o.d" -o ${OBJECTDIR}/_ext/1360889168/ADC10Read.o ../BSP/ADC10Read.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Debug.o: ../source/Debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Debug.o.d" -o ${OBJECTDIR}/_ext/812168374/Debug.o ../source/Debug.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/main.o: ../source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/main.o.d" -o ${OBJECTDIR}/_ext/812168374/main.o ../source/main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/SerialDisplay.o: ../source/SerialDisplay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/SerialDisplay.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/SerialDisplay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/SerialDisplay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/SerialDisplay.o.d" -o ${OBJECTDIR}/_ext/812168374/SerialDisplay.o ../source/SerialDisplay.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MenuFSM.o: ../source/MenuFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MenuFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MenuFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MenuFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MenuFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/MenuFSM.o ../source/MenuFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Stubs.o: ../source/Stubs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Stubs.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Stubs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Stubs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Stubs.o.d" -o ${OBJECTDIR}/_ext/812168374/Stubs.o ../source/Stubs.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/ftoa.o: ../source/ftoa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/ftoa.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/ftoa.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/ftoa.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/ftoa.o.d" -o ${OBJECTDIR}/_ext/812168374/ftoa.o ../source/ftoa.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/StrConversions.o: ../source/StrConversions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/StrConversions.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/StrConversions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/StrConversions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/StrConversions.o.d" -o ${OBJECTDIR}/_ext/812168374/StrConversions.o ../source/StrConversions.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/TimeDelay.o: ../source/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/TimeDelay.o.d" -o ${OBJECTDIR}/_ext/812168374/TimeDelay.o ../source/TimeDelay.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/SystemParameters.o: ../source/SystemParameters.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/SystemParameters.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/SystemParameters.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/SystemParameters.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/SystemParameters.o.d" -o ${OBJECTDIR}/_ext/812168374/SystemParameters.o ../source/SystemParameters.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotorPWM.o: ../source/MotorPWM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotorPWM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotorPWM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotorPWM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotorPWM.o.d" -o ${OBJECTDIR}/_ext/812168374/MotorPWM.o ../source/MotorPWM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionStats.o: ../source/MotionStats.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionStats.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionStats.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionStats.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionStats.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionStats.o ../source/MotionStats.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionSensor.o: ../source/MotionSensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionSensor.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionSensor.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionSensor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionSensor.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionSensor.o ../source/MotionSensor.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/AppTimer.o: ../source/AppTimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/AppTimer.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/AppTimer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/AppTimer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/AppTimer.o.d" -o ${OBJECTDIR}/_ext/812168374/AppTimer.o ../source/AppTimer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionFSM.o: ../source/MotionFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionFSM.o ../source/MotionFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionLimits.o: ../source/MotionLimits.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionLimits.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionLimits.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionLimits.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionLimits.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionLimits.o ../source/MotionLimits.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o: ../source/MotionPhaseFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/MotionPhaseFSM.o ../source/MotionPhaseFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o: ../source/MoveSequenceFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/MoveSequenceFSM.o ../source/MoveSequenceFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/CoordTranslate.o: ../source/CoordTranslate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/CoordTranslate.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/CoordTranslate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/CoordTranslate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/CoordTranslate.o.d" -o ${OBJECTDIR}/_ext/812168374/CoordTranslate.o ../source/CoordTranslate.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/SunPosition.o: ../source/SunPosition.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/SunPosition.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/SunPosition.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/SunPosition.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/SunPosition.o.d" -o ${OBJECTDIR}/_ext/812168374/SunPosition.o ../source/SunPosition.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/RTCC.o: ../source/RTCC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/RTCC.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/RTCC.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/RTCC.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/RTCC.o.d" -o ${OBJECTDIR}/_ext/812168374/RTCC.o ../source/RTCC.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Debounce.o: ../source/Debounce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Debounce.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Debounce.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Debounce.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Debounce.o.d" -o ${OBJECTDIR}/_ext/812168374/Debounce.o ../source/Debounce.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o: ../source/ButtonProcessingFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/ButtonProcessingFSM.o ../source/ButtonProcessingFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/LEDs.o: ../source/LEDs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/LEDs.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/LEDs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/LEDs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/LEDs.o.d" -o ${OBJECTDIR}/_ext/812168374/LEDs.o ../source/LEDs.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Inclinometer.o: ../source/Inclinometer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Inclinometer.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Inclinometer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Inclinometer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Inclinometer.o.d" -o ${OBJECTDIR}/_ext/812168374/Inclinometer.o ../source/Inclinometer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/RxMessage.o: ../source/RxMessage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/RxMessage.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/RxMessage.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/RxMessage.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/RxMessage.o.d" -o ${OBJECTDIR}/_ext/812168374/RxMessage.o ../source/RxMessage.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/UpdateParameters.o: ../source/UpdateParameters.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/UpdateParameters.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/UpdateParameters.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/UpdateParameters.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/UpdateParameters.o.d" -o ${OBJECTDIR}/_ext/812168374/UpdateParameters.o ../source/UpdateParameters.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o: ../source/PanelPositionFSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o.d" -o ${OBJECTDIR}/_ext/812168374/PanelPositionFSM.o ../source/PanelPositionFSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/Zigbee.o: ../source/Zigbee.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/Zigbee.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/Zigbee.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/Zigbee.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/Zigbee.o.d" -o ${OBJECTDIR}/_ext/812168374/Zigbee.o ../source/Zigbee.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
${OBJECTDIR}/_ext/812168374/BMS.o: ../source/BMS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/812168374" 
	@${RM} ${OBJECTDIR}/_ext/812168374/BMS.o.d 
	@${RM} ${OBJECTDIR}/_ext/812168374/BMS.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/812168374/BMS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DNOT_PIC32_STARTER_KIT -DNOT___ENABLEIO_STARTERKIT_DEBUG -I".." -I"../Include" -I"../Include/BSP" -I"../../Include/BSP" -I"../../xc32/v1.20/pic32mx/include/peripheral" -Wall -MMD -MF "${OBJECTDIR}/_ext/812168374/BMS.o.d" -o ${OBJECTDIR}/_ext/812168374/BMS.o ../source/BMS.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wchar-subscripts -Wchar-subscripts
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/SmartTrak_1Axis_v6.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/SmartTrak_1Axis_v6.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    -Wchar-subscripts $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=_min_heap_size=4096,--defsym=_min_stack_size=512,-L"..",-Map="$(BINDIR_)$(TARGETBASE).map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/SmartTrak_1Axis_v6.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/SmartTrak_1Axis_v6.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    -Wchar-subscripts $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=4096,--defsym=_min_stack_size=512,-L"..",-Map="$(BINDIR_)$(TARGETBASE).map"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/SmartTrak_1Axis_v6.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
