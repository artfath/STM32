@echo off
ECHO Computing CRC
ECHO -------------------------------------
REM Batch script for generating CRC in STM32 project
REM Must be placed at Project/<TARGET> folder

REM Path configuration
SET SREC_PATH="C:\Program Files\srecord\bin"
SET TARGET_NAME=selftest_h7
SET COMPUTE_CRC=1
SET COMPARE_HEX=1
SET CRC_ADDR_FROM_MAP=1
REM Not used when CRC_ADDR_FROM_MAP=1
SET CRC_ADDR=0x08000000

REM Derived configuration
SET MAP_FILE=%TARGET_NAME%.map
SET INPUT_BIN=%TARGET_NAME%.bin -binary
SET INPUT_HEX=%TARGET_NAME%.hex -intel
SET OUTPUT_HEX=%TARGET_NAME%_CRC.hex -intel
SET TMP_FILE=crc_tmp_file.txt

IF NOT "%CRC_ADDR_FROM_MAP%"=="1" goto:end_of_map_extraction
REM Extract CRC address from MAP file
REM -----------------------------------------------------------
REM Load line with checksum location to crc_search variable
ECHO Extracting CRC address from MAP file
FINDSTR "(_Check_Sum" %MAP_FILE%>%TMP_FILE%
SET /p crc_search=<%TMP_FILE%
DEL %TMP_FILE%
REM get first word at line, which should be CRC address in HEX format
for /f "tokens=1 delims= " %%a in ("%crc_search%") do set CRC_ADDR=%%a
REM -----------------------------------------------------------
REM End of CRC address extraction
:end_of_map_extraction
REM Convert BIN to HEX with offset
%SREC_PATH%\srec_cat.exe ^
	%INPUT_BIN% ^
	-offset 0x08000000 ^
	-o %INPUT_HEX%
	
REM Compute CRC and store it to new HEX file
ECHO CRC address: %CRC_ADDR%
if "%COMPUTE_CRC%"=="1" (
REM ECHO to see what is going on
ECHO %SREC_PATH%\srec_cat.exe ^
	%INPUT_HEX% ^
	-crop 0x08000000 %CRC_ADDR% ^
	-stm32 %CRC_ADDR% ^
	-o %OUTPUT_HEX%	
%SREC_PATH%\srec_cat.exe ^
	%INPUT_HEX% ^
	-crop 0x08000000 %CRC_ADDR% ^
	-stm32 %CRC_ADDR% ^
	-o %OUTPUT_HEX%
)

ECHO Modified HEX file with CRC stored at %OUTPUT_HEX%

REM Compare input HEX file with output HEX file
if "%COMPARE_HEX%"=="1" (
ECHO Comparing %INPUT_HEX% with %OUTPUT_HEX%
%SREC_PATH%\srec_cmp.exe ^
	%INPUT_HEX% %OUTPUT_HEX% -v
)
ECHO -------------------------------------