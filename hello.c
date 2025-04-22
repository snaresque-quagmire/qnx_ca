#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <hw/pci.h>
#include <hw/inout.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

// setup
#define	INTERRUPT		iobase[1] + 0				// Badr1 + 0 : also ADC register
#define	MUXCHAN			iobase[1] + 2				// Badr1 + 2
#define	TRIGGER			iobase[1] + 4				// Badr1 + 4
#define	AUTOCAL			iobase[1] + 6				// Badr1 + 6
#define DA_CTLREG		iobase[1] + 8				// Badr1 + 8

// adc output data and fifo
#define AD_DATA			iobase[2] + 0				// Badr2 + 0
#define AD_FIFOCLR		iobase[2] + 2				// Badr2 + 2

// unused
#define	TIMER0				iobase[3] + 0			// Badr3 + 0
#define	TIMER1				iobase[3] + 1			// Badr3 + 1
#define	TIMER2				iobase[3] + 2			// Badr3 + 2
#define	COUNTCTL		iobase[3] + 3				// Badr3 + 3

// port c is not used
#define	DIO_PORTA		iobase[3] + 4				// Badr3 + 4
#define	DIO_PORTB		iobase[3] + 5				// Badr3 + 5
#define	DIO_PORTC		iobase[3] + 6				// Badr3 + 6
#define	DIO_CTLREG		iobase[3] + 7				// Badr3 + 7

// unused
#define	PACER1				iobase[3] + 8			// Badr3 + 8
#define	PACER2				iobase[3] + 9			// Badr3 + 9
#define	PACER3				iobase[3] + a			// Badr3 + a
#define	PACERCTL			iobase[3] + b			// Badr3 + b

// dac output data and fifo
#define DA_Data			iobase[4] + 0				// Badr4 + 0
#define DA_FIFOCLR		iobase[4] + 2				// Badr4 + 2

#define	DEBUG			1
#define VVAVE_FILE_MAGIC 	0x6429
#define INTERVAL_NS		100000

int badr[5];								// PCI 2.2 assigns 6 IO base addresses


// ==================== GLOBAL VARIABLE DECLARATION  ========================================

struct pci_dev_info info;
void *hdl;
uintptr_t iobase[6];
unsigned int i,count;
unsigned short chan;

unsigned int int_flag;

int dio_kill = 0;
int debounce_flag = 0;
void* read_adc_ptr;
void* adc_data_ptr; 

// ==================== END OF GLOBAL VARIABLE DECLARATION  ========================================

typedef struct{
	size_t size;
	size_t capacity;
	uint16_t *data;
} Buffer;

typedef struct{
	uint16_t magic;
	size_t size;
	size_t capacity;
} Vvave_signature;

void adc_read(uint16_t *adc_in_1, uint16_t *adc_in_2);
void dio(uintptr_t *dio_in);
void dac(uint16_t *data);
void buffer_resize(Buffer* buffer, size_t new_capacity);
void buffer_write(Buffer *buffer, const uint16_t *data, size_t size);
void buffer_free(Buffer *buffer);
int buffer_read(Buffer *buffer, size_t index, uint16_t *out_value);
void adc_load_file(Buffer* buffer, const char* file_path);
void adc_save_file(Buffer *buffer, const char* file_path);
char* shift(int *argc, char*** argv);
void handle_sigtstp(int sig);
void handle_sigusr1(int sig);
void handle_sigint(int sig);
void handle_sigalrm(int sig);
void* monitor_dio(void* arg);
void cleanup();
void precise_sleep(long nanosecond);

int main(int argc, char* argv[]){

// ==================== MAIN LOCAL VARIABLE DECLARATION  ========================================

	pthread_t monitor_thread;
	pthread_attr_t attr;
	uint16_t adc_in_1, adc_in_2, adc_in_1_prev=0;
	uintptr_t dio_in, dio_in_prev = 0;
	uint16_t data_to_scr;

	const char* program;
	const char* flag;
	const char* file_pathname;

	timer_t timerid;
	struct itimerspec its;
	struct timespec rem;
	struct timespec req;

// ====================  END OF MAIN LOCAL VARIABLE DECLARATION  ========================================

// ====================  MAIN VARIABLE INIT  ========================================

	Buffer* adc_data = (Buffer*)malloc(sizeof(Buffer));
	Buffer* read_adc = (Buffer*)malloc(sizeof(Buffer));

	adc_data->data = 0;
	adc_data->size = 0;
	adc_data->capacity = 0;
	read_adc->data = 0;
	read_adc->size = 0;
	read_adc->capacity = 0;

	int_flag = 0;
// ====================  END OF MAIN VARIABLE INIT  ========================================


// ====================  IO MAPPING  ========================================

	memset(&info,0,sizeof(info));     // set info to 0
	if(pci_attach(0)<0) {             // try connecting to PCI, but the PCI PATH is set somewhere else
	  perror("pci_attach");
	  exit(EXIT_FAILURE);
	  }
	
																			/* Vendor and Device ID */
	info.VendorId=0x1307;
	info.DeviceId=0x01;
	
	if ((hdl=pci_attach_device(0, PCI_SHARE|PCI_INIT_ALL, 0, &info))==0) {
	  perror("pci_attach_device");
	  exit(EXIT_FAILURE);
	  }
	/*  
	  for(i=0;i<6;i++) {							// Another printf BUG ? - Break printf to two statements
		if(info.BaseAddressSize[i]>0) {
		  printf("Aperture %d  Base 0x%x Length %d Type %s\n", i, 
			PCI_IS_MEM(info.CpuBaseAddress[i]) ?  (int)PCI_MEM_ADDR(info.CpuBaseAddress[i]) : 
			(int)PCI_IO_ADDR(info.CpuBaseAddress[i]),info.BaseAddressSize[i], 
			PCI_IS_MEM(info.CpuBaseAddress[i]) ? "MEM" : "IO");
		  }
	  }  
	*/															
	//printf("IRQ %d\n",info.Irq); 		
															// Assign BADRn IO addresses for PCI-DAS1602			
	if(DEBUG) {
	//printf("\nDAS 1602 Base addresses:\n\n");
	for(i=0;i<5;i++) {
	  badr[i]=PCI_IO_ADDR(info.CpuBaseAddress[i]);
	  //if(DEBUG) printf("Badr[%d] : %x\n", i, badr[i]);
	  }
	 
		//printf("\nReconfirm Iobase:\n");  			// map I/O base address to user space						
	for(i=0;i<5;i++) {								// expect CpuBaseAddress to be the same as iobase for PC
	  iobase[i]=mmap_device_io(0x0f,badr[i]);	
	  //printf("Index %d : Address : %x ", i,badr[i]);
	  //printf("IOBASE  : %x \n",iobase[i]);
	  }													
	}
															// Modify thread control privity
	if(ThreadCtl(_NTO_TCTL_IO,0)==-1) {
	  perror("Thread Control");
	  exit(1);
	  }			 																								

// ====================  END OF IO MAPPING  ========================================

// ====================  MAIN  ========================================

	system("clear");
	read_adc_ptr = read_adc;
	adc_data_ptr = adc_data;
	atexit(cleanup);
	printf("\n");

	if(signal(SIGTSTP,handle_sigtstp) < 0){
		fprintf(stderr, "[ERROR] Failed at sigtstp, %s\n", strerror(errno));
		exit(1);
	}

	if(signal(SIGUSR1,handle_sigusr1) < 0){
		fprintf(stderr, "[ERROR] Failed at sigusr1, %s\n", strerror(errno));
		exit(1);
	}
	if(signal(SIGINT,handle_sigint) < 0){
		fprintf(stderr, "[ERROR] Failed at sigint, %s\n", strerror(errno));
		exit(1);
	}
	if(signal(SIGALRM,handle_sigalrm) < 0){
		fprintf(stderr, "[ERROR] Failed at sigalrm, %s\n", strerror(errno));
		exit(1);
	}


	program = shift(&argc, &argv);

	if(argc == 0){
		fprintf(stderr, "[ERROR] No flag provided\n");
		exit(1);
	}

	flag = shift(&argc, &argv);
	if(strcmp(flag, "-i") == 0){
		i = 0;
		if(argc == 0){
			fprintf(stderr, "[ERROR] No file provided to write into\n");
			exit(1);
		}else{
			file_pathname = shift(&argc, &argv);
		}
	}else if(strcmp(flag, "-p") == 0){
		if(argc == 0){
			fprintf(stderr, "[ERROR] No file provided to write into\n");
			exit(1);
		}else{
			file_pathname = shift(&argc, &argv);
		}
		adc_load_file(read_adc, file_pathname);
		printf("[INFO] Buffer loaded successfully\n");
	}else if(strcmp(flag, "-w") == 0){
		if(argc == 0){
			fprintf(stderr, "[ERROR] No file provided to write into\n");
			exit(1);
		}else{
			file_pathname = shift(&argc, &argv);
		}
		adc_load_file(read_adc, file_pathname);
		printf("[INFO] Buffer loaded successfully\n");
	}else{
		fprintf(stderr, "[ERROR] Unknown flag '%s' provided\n", flag);
		exit(1);
	}	

		// =====================  DIO INIT CHECKER  ======================
	dio(&dio_in);
	dio_in_prev = 0x00;
	if(dio_in != 0xf0){
		fprintf(stdout, "Program will hang until 0xf0 is set\n\n");
	}
	while(dio_in != 0xf0){
		if(dio_in != dio_in_prev){
			fprintf(stdout, "[WARN] Please set digital input switches to %02X\n", dio_in);
			dio_in_prev = dio_in;
		} else if(int_flag == 1){
			printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
			exit(1);
		}		
		dio(&dio_in);
		usleep(1000);
	}

	dio_kill = 1;
	adc_in_1 = 0;
	adc_in_1_prev = 0;

		// =====================  END OF DIO INIT CHECKER  ======================

		// =====================   PTHREAD INITIALIZATION ======================
	if(pthread_attr_init(&attr) != 0){
		fprintf(stderr, "[ERROR] Failed to init attr\n");
		fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
		exit(1);
	}
	if(pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0){
		fprintf(stderr, "[ERROR] Failed to set detached attr\n");
		fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
		exit(1);
	}
	if(pthread_create(&monitor_thread, &attr, monitor_dio, NULL) != 0){
		fprintf(stderr, "[ERROR] Failed to create monitor_thread\n");
		fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
		exit(1);
	}
		// =====================  END OF PTHREAD INITIALIZATION  ======================

		// =====================   CLOCK INITIALIZATION  ======================

	if(timer_create(CLOCK_REALTIME, NULL, &timerid) < 0){
		fprintf(stderr, "[ERROR] Failed to create timer\n");
		fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
		exit(1);				
	}

	its.it_value.tv_sec = 0;
	its.it_value.tv_nsec = INTERVAL_NS;
	its.it_interval.tv_sec = 0;
	its.it_interval.tv_nsec = INTERVAL_NS;

	if(timer_settime(timerid, 0, &its, NULL) < 0){
		fprintf(stderr, "[ERROR] Failed to set timer\n");
		fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
		exit(1);				
	}

		// =====================  END OF CLOCK INITIALIZATION  ======================

	if(strcmp(flag, "-i") == 0){
		while (i < 64){
			adc_read(&adc_in_1, &adc_in_2);
			if ( abs((unsigned int)adc_in_1_prev - (unsigned int)adc_in_1) > 100){
				buffer_write(adc_data, &adc_in_1, sizeof(uint16_t));
				printf("inserted %d: %d: %d \n", i, (unsigned int)adc_in_1_prev, (unsigned int)adc_in_1);
				adc_in_1_prev = adc_in_1;
				i++;
			}
			if(int_flag != 0){
				break;	
			}
		}
		if(int_flag == 0){
			printf("[INFO] SAVING . . . . . .\n");
			adc_save_file(adc_data, file_pathname);
			printf("[INFO] Finished\n");
		} else {
			printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
			printf("[INFO] Freeing buffers\n");
		}

	}else if(strcmp(flag, "-p") == 0){
		i = 0;
		while( (i < (read_adc->size/sizeof(read_adc->data[0])) ) & (int_flag == 0) ){
			buffer_read(read_adc, (size_t)i,&data_to_scr); 
			printf("inserted %d: %d \n", i, (unsigned int)data_to_scr);
			i++;
		}
		if(int_flag == 0){
			printf("[INFO] SAVING . . . . . .\n");
			printf("[INFO] Data saved successfully\n");
		} else {
			printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
		}
	}else if(strcmp(flag, "-w") == 0){
		count = read_adc->size/sizeof(read_adc->data[0]);
		printf("%u\n", (unsigned int)count);
		printf("[INFO] Data loaded successfully\n");
		printf("[INFO] Sending data to dac . . . . . . \n");
		while(1){

			for(i=0; i<count; ++i){
				buffer_read(read_adc, (size_t)i,&data_to_scr); 
				dac(&data_to_scr);
				//precise_sleep(INTERVAL_NS);
				usleep(1);
			} 		
			i = 0;	
			if(int_flag != 0){
				printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
				break;
			}
		}
	}else{
		fprintf(stderr, "[ERROR] Unknown flag '%s' provided\n", flag);
		exit(1);
	}	

	pthread_join(monitor_thread, NULL);
	return 0;
}
// ====================  END OF MAIN  ========================================


// ====================  DAQ RELATED FUNCTIONS  ========================================

void adc_read(uint16_t *adc_in_1, uint16_t *adc_in_2){


	/*
		reads adc, and return 2^16 value of adc_in_1 and adc_in_2
	*/

	// setup sequences, might not need to run every poll, but meh following the tutorial to run every poll

	if(debounce_flag == 1){	
		out16(INTERRUPT,0x60c0);		
		out16(TRIGGER,0x2081);
		out16(AUTOCAL,0x007f);
		out16(AD_FIFOCLR,0);
		out16(MUXCHAN,0x0D00);
		count = 0x00;
		while(count < 0x01) {
			//chan = 0x0D00 | count;
			out16(MUXCHAN,0x0D00);
			delay(1);						
			out16(AD_DATA,0); 
			while(!(in16(MUXCHAN) & 0x4000));			
	
			/*
				The EOC bit in BADR1 + 0 may be polled until true
				This will hang the program until polled true, but hang time <1ms
			*/
			if (count == 0x00){
				*adc_in_1 = in16(AD_DATA);
			}else{
				*adc_in_2 = in16(AD_DATA); 
				printf("4444\n");
			}    
			count++;
			delay(5);
		}
		return;
	}
	debounce_flag = 1;
	alarm(1);
	
}

void dio(uintptr_t *dio_in){
	uintptr_t local_dio;

	out8(DIO_CTLREG,0x90);				// There is Port A, Port C(Upper), Port B, Port C(Lower). Sets input/output		
	*dio_in =in8(DIO_PORTA); 																							
	local_dio = *dio_in;
	out8(DIO_PORTB, local_dio);
}

void dac(uint16_t *data){
	//for(i=0x8fff;i<0xffff;i=i+0x80) {
		out16(DA_CTLREG,0x0a23);			// DA Enable, #0, #1, SW 5V unipolar		2/6
		out16(DA_FIFOCLR, 0);					// Clear DA FIFO  buffer
		out16(DA_Data,(short) *data);																																		
		out16(DA_CTLREG,0x0a43);			// DA Enable, #1, #1, SW 5V unipolar		2/6
		out16(DA_FIFOCLR, 0);					// Clear DA FIFO  buffer
		out16(DA_Data,(short) *data);																																		

		//printf("DAC Data [%4x]: %4x\r", i, i);		// print DAC													
		//fflush( stdout ); 
		//delay(2);
	//}
															// Reset DAC to 5v
	// out16(DA_CTLREG,(short)0x0a23);	
	// out16(DA_FIFOCLR,(short) 0);			
	// out16(DA_Data, 0x8fff);						// Mid range - Unipolar																											
	
	// out16(DA_CTLREG,(short)0x0a43);	
	// out16(DA_FIFOCLR,(short) 0);			
	// out16(DA_Data, 0x8fff);		
}

// ====================  END OF DAQ RELATED FUNCTIONS  ========================================


// ====================  BUFFER RELATED FUNCTIONS  ========================================


/*  
    resize buffer to new_capacity, my current code, size is same as capacity. just by
    reading  [ buffer->size / sizeof(int) ], i know how many integer inside the buffer
*/
void buffer_resize(Buffer *buffer, size_t new_capacity){
    buffer->capacity = new_capacity;
    buffer->data = realloc(buffer->data, buffer->capacity);
	//printf("resizing\n");
}

void buffer_write(Buffer *buffer, const uint16_t *data, size_t size){
    /*  How does buffer_write work
        stores uint16_t as bytes in an array
        
        before storing, need to realloc the old buffer so there is enough space for new data

        before exiting, need to free;
    */
    if(buffer->size + size > buffer->capacity){
        buffer_resize(buffer, buffer->capacity + size);
    }
	//printf("copying\n");
    memcpy(buffer->data + (buffer->size / sizeof(uint16_t)), data, size);
    buffer->size += size;
}

void buffer_free(Buffer *buffer){
    free(buffer->data);
    buffer->data = NULL;
    buffer->size = 0;
    buffer->capacity = 0;
}

int buffer_read(Buffer *buffer, size_t index, uint16_t *out_value){
    /*	How does buffer_read work ?
    
        buffer_read access the buffer like an array
    */
    if (index >= buffer->size){
        return 0;
    }
    *out_value = buffer->data[index];
    return 1;
}

// ====================  END OF BUFFER RELATED FUNCTIONS  ========================================

void adc_load_file(Buffer* buffer, const char* file_path){
	FILE *f;
	Vvave_signature meta; 		
	size_t n;

	f = fopen(file_path,"rb");
	if(f == NULL){
		fprintf(stderr, "[ERROR] Could not open file '%s' : %s\n", file_path, strerror(errno));
		exit(1);
	}
	
	n = fread(&meta, sizeof(meta),1,f);
	if(n < 1){
		fprintf(stderr, "ERROR: Could not read meta data from file '%s':%s\n", file_path, strerror(errno));
		exit(1);
	}
	if(meta.magic != VVAVE_FILE_MAGIC){
		fprintf(stderr, "[ERROR] %s does not appear to be a valid file\n", file_path);
		fprintf(stderr, "[ERROR] Magic number detected to be %04X. Expected %04X.\n", meta.magic, VVAVE_FILE_MAGIC);
		exit(1);
	}
	buffer->capacity = meta.capacity;
	buffer->size = meta.size;
	buffer_resize(buffer, meta.capacity);
	//printf("filling buffer %lu, %lu\n", (unsigned long)meta.size, (unsigned long)meta.capacity);

	n = fread(buffer->data, sizeof(uint16_t), (meta.size/sizeof(uint16_t)),f);	
}

char* shift(int *argc, char ***argv){
 	char* result;
 	assert(*argc > 0);
	result = **argv;
 	*argv += 1;
 	*argc -= 1;
	return result;
}

void* monitor_dio(void* arg){
	uintptr_t current_dio;

	while(dio_kill){
		dio(&current_dio);
		if(current_dio == 0xF1){
			raise(SIGUSR1);
			dio_kill = 0;
		}
		usleep(500);
	}
	return NULL;
}

void adc_save_file(Buffer *buffer, const char* file_path){
	Vvave_signature meta; 		
	FILE *f;
	meta.magic = VVAVE_FILE_MAGIC;
	meta.size = buffer->size; 
	meta.capacity = buffer->capacity;
	f = fopen(file_path, "wb");
	if(f == NULL){
		fprintf(stderr, "ERROR: Could not open file '%s':%s\n", file_path, strerror(errno));
		exit(1);
	}

	fwrite(&meta, sizeof(meta),1,f);
	if(ferror(f)){
		fprintf(stderr, "ERROR: Could not write to file '%s':%s\n", file_path, strerror(errno));
		exit(1);
	}

	fwrite(buffer->data,buffer->size,1,f);
	if(ferror(f)){
		fprintf(stderr, "ERROR: Could not write to file '%s':%s\n", file_path, strerror(errno));
		exit(1);
	}


	fclose(f);
}
void cleanup(){
	buffer_free(read_adc_ptr);
	buffer_free(adc_data_ptr);
	printf("[INFO] EXIT SUCCESS\n");

}
void precise_sleep(long nanosecond){
	struct timespec req, rem;
	req.tv_sec = 0;
	req.tv_nsec = nanosecond;
	clock_nanosleep(CLOCK_MONOTONIC,0, &req, &rem);
}
void handle_sigtstp(int sig){
	printf("[INFO] Stop not allowed\n");

}
void handle_sigusr1(int sig){
	++int_flag;
}
void handle_sigint(int sig){
	++int_flag;
}
void handle_sigalrm(int sig){
	debounce_flag = 0;	
}
