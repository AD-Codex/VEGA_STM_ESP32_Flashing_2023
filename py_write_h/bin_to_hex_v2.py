'''
2023/08
Divakaran A.
convert bin file to .h file

--------------- instructions
1. place .py file in same folder that .bin file placed
2. run the pyhton file with first argument with the .bin file name
		ex - ( python3 bin_to_hex.py stm32_bin_file.bin)
3. stm32_bin.h file now creat in the folder
4. move the stm32_bin.h file to the main directory of the espidf project
---------------
'''

import sys

bin_location = str(sys.argv[1])
bin_version = str(sys.argv[2])


# ------------------ read bin file
f = open(bin_location, 'rb')
bin_file_len = len(f.read())
# print(bin_file_len)
f = open(bin_location, 'rb')
bin_file = list(f.read(bin_file_len))
f.close()

bin_array_size = 1024
number_of_bin_arrays = int(bin_file_len / bin_array_size)
if (bin_file_len % bin_array_size > 0) :
	number_of_bin_arrays = number_of_bin_arrays+1


# ---------------------- h file make

f = open('stm32_bin.h', 'w')

f.write("#include <stdio.h>\n")
f.write("#include <stdint.h>\n")
f.write("\n")
f.write("unsigned int stm32_bin_len =" + str(bin_file_len) + ";\n")

f.write("unsigned int stm32_bin_arrays =" + str(number_of_bin_arrays) + ";\n")

f.write("int bin_version =" + bin_version.split('.')[0] + bin_version.split('.')[1] + bin_version.split('.')[2] + ";\n")

f.write("uint8_t bin[][1024] = {\n")

for arrays in range(number_of_bin_arrays):
	f.write("{")

	if (arrays < number_of_bin_arrays-1):
		i_size = bin_array_size
	else :
		i_size = bin_file_len - bin_array_size*arrays

	for i in range(i_size):
		f.write(str(int(bin_file[i+arrays*bin_array_size])))
		if (i != i_size-1):
			f.write(",")

	f.write("}")
	if (arrays<number_of_bin_arrays-1):
		f.write(",\n")

f.write("\n};")

f.close()

print("stm32_bin file created ...")
