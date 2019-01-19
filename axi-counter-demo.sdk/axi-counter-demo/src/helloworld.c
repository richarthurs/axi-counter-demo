/*
 * AXI Counter Demo Linux App
 * Author: Richard Arthurs
 * Date: January 14, 2018
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>
#include "ps7_init.h"
int main(void){



unsigned addr, offset;

int reg_addr = 0x43c00000;

int value;
int fd;

void *ptr;
unsigned pg_sz = sysconf(_SC_PAGESIZE);

printf("Access through /dev/mem - pg_sz: %x\n", pg_sz);

// Open the mem file
fd = open("/dev/mem", O_RDWR);
if(fd < 1){
	perror("can't open");
	return -1;
}

// mmap the device into memory
addr = (reg_addr & (~(pg_sz - 1)));
offset = (reg_addr - addr);
ptr = mmap(NULL, pg_sz, PROT_READ|PROT_WRITE, MAP_SHARED, fd, addr);


// Open up the log file
FILE *log_file;
log_file = fopen("/home/root/counter_log.txt","w");	// Note: must use /home/root, not ~

if(log_file == NULL){
  printf("Error! Cannot open log file");
  exit(1);
}


// Write one to reg 0 to start the LED blinky
*((unsigned *)(ptr + offset + 0)) = 1;

// Write 1 to reg 2 to start the FIFO fill process
*((unsigned *)(ptr + offset + 8)) = 1;

// Read 100x values from reg 3 (the "image" from the FIFO)
// We will know how many values to expect
for(int i = 0; i < 100; i +=1){
	value = *((unsigned *)(ptr + offset + 12));
	printf("Value %d from fifo: %x\n",i, value);
	fprintf(log_file,"%d\n",value);
}

// Write 0 to reg 2 to reset the FIFO
*((unsigned *)(ptr + offset + 8)) = 0;

// Read the LED counter register (reg 1) for fun
value = *((unsigned *)(ptr + offset + 4));
printf("LED Counter: %x\n", value);

// Close the log file
fclose(log_file);

return 0;
}
