# STM32 chip flash using ESP32 chip
In this method bin file of STM32 was added to the ESP32 script. This method use for overcome the limitation of HTTP and MQTT methods.
-----------------------

-----------------
## 1) python file
--------------
### 1. target : convert bin file to .h file

### 2. process : 
1. read the .bin file
2. define the header file
3. 8bit data type used to creat veriables
4. store read data in 8 bit 2D array with lenght of 1024 and width according to bin file size (2d matrix)
5. 
### 3. instructions : 
1. place .py file in same folder that .bin file placed
2. run the pyhton file with first argument with the .bin file name. ex - ( python3 bin_to_hex.py stm32_bin_file.bin)
3. stm32_bin.h file now creat in the folder
4. move the stm32_bin.h file to the main directory of the espidf project
----------------
## 2) ESPidf file
-----------------
### 1. target : firmware update of stm32f103

### 2. functions :
1. init_uart_2() - UART configuration ( baud rate -115200)
2. gpio_init_2() - gpio pin initializing
3. gpio_boot_set() - enable boot mode of the stm32
4. gpio_boot_rset() - disable boot mode of the stm32
5. write_address_checksum() - checksum of the 32bit address
6. enter_bootmode_2() - flasing process
7. stm32_flash()
8. stm32_check() - check the stm32 firmware version
9. 
--------------------------
## 3) stm32 firmupdate step
-----------------------

1. BOOT0 to 1
2. RESET to 1
3. RESET to 0

### boot mode enter
1. TX 0x7F
2. RX 0x79

### flash erase
1. TX 0x43
2. TX 0xBC
3. RX 0x79		// erasing flash
4. TX 0xFF
5. TX 0x00
6. RX 0x79		// flash Erase success

### ---------------- loop start ---------------
1. TX 0x31
2. TX 0xCE
3. RX 0x79

#### send address starting 0x08000000<br>
0x12345678<br>
|31-----------24|23-----------16|15------------8|7---------bit 0|<br>
+---------------+---------------+---------------+---------------+<br>
|0 0 0 1 0 0 1 0|0 0 1 1 0 1 0 0|0 1 0 1 0 1 1 0|0 1 1 1 1 0 0 0|<br>
+---------------+---------------+---------------+---------------+<br>

4. TX - <br>
1byte - 0 0 0 1 0 0 1 0<br>
2byte - 0 0 1 1 0 1 0 0<br>
3byte - 0 1 0 1 0 1 1 0<br>
4byte - 0 1 1 1 1 0 0 0<br>
5byte - XOR (|0 0 0 1 0 0 1 0|0 0 1 1 0 1 0 0|0 1 0 1 0 1 1 0|0 1 1 1 1 0 0 0|)<br>

5. RX 0x79		// address send<br>

6. TX number of bytes --- (128 -1)<br>

7. Tx 128 byte send<br>

#### checksunm TX -- XOR(send byte)<br>
8. RX 0x79		// packet send done .........................<br>

------------ loop end ------------------------<br>

### boot mode exit
1. BOOT0 to 0
2. RESET to 1
3. RESET to 0
