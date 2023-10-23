# Vega_stm32Flash_eps32

1) python file
	target : convert bin file to .h file

	process : 
			1. read the .bin file
			2. define the header file
			3. 8bit data type used to creat veriables
			4. store read data in 8 bit 2D array with lenght of 1024 and width according to bin file size (2d matrix)

	instructions : 
			1. place .py file in same folder that .bin file placed
			2. run the pyhton file with first argument with the .bin file name
					ex - ( python3 bin_to_hex.py stm32_bin_file.bin)
			3. stm32_bin.h file now creat in the folder
			4. move the stm32_bin.h file to the main directory of the espidf project

2) ESPidf file
	target : firmware update of stm32f103

	functions :
			1. init_uart_2()
					UART configuration ( baud rate -115200)
			2. gpio_init_2()
					gpio pin initializing
			3. gpio_boot_set()
					enable boot mode of the stm32
			4. gpio_boot_rset()
					disable boot mode of the stm32
			5. write_address_checksum()
					checksum of the 32bit address
			6. enter_bootmode_2()
					flasing process
			7. stm32_flash()
			8. stm32_check()
					check the stm32 firmware version
