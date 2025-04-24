// MA4830 - Realtime Software for Mechatronic Systems
// Major CA Report
// Academic Year: AY24/25 Sem 2
// Tutorial Class: MA1 (Wed, 1:30 PM to 2:20PM)
// Tutor: Dr Ahmad Khairyanto
// Group Members:
// Yap Jia Jun (U2121081K)
// Ho Min Han (U2121659)
// Gan Yi Xiang (U2123533D)
// Feng Qi Hao (U2120147B)
// Liang Fu Den (U2123453C)
// Daniel Lim Wei Hung (U2123353F)
// Ng Hong Xi (U2123024A)

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <hw/pci.h>
#include <hw/inout.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

#define INTERRUPT   iobase[1] + 0 // Badr1 + 0 : also ADC register
#define MUXCHAN     iobase[1] + 2   // Badr1 + 2
#define TRIGGER     iobase[1] + 4   // Badr1 + 4
#define AUTOCAL     iobase[1] + 6   // Badr1 + 6
#define DA_CTLREG   iobase[1] + 8   // Badr1 + 8

#define AD_DATA		iobase[2] + 0   // Badr2 + 0
#define AD_FIFOCLR	iobase[2] + 2   // Badr2 + 2

#define TIMER0		iobase[3] + 0   // Badr3 + 0
#define TIMER1		iobase[3] + 1   // Badr3 + 1
#define TIMER2		iobase[3] + 2   // Badr3 + 2
#define COUNTCTL	iobase[3] + 3   // Badr3 + 3
#define DIO_PORTA	iobase[3] + 4 	// Badr3 + 4
#define DIO_PORTB	iobase[3] + 5 	// Badr3 + 5
#define DIO_PORTC	iobase[3] + 6   // Badr3 + 6
#define DIO_CTLREG	iobase[3] + 7 	// Badr3 + 7
#define PACER1		iobase[3] + 8   // Badr3 + 8
#define PACER2		iobase[3] + 9   // Badr3 + 9
#define PACER3		iobase[3] + 0xA // Badr3 + a
#define PACERCTL	iobase[3] + 0xB // Badr3 + b

#define DA_Data     iobase[4] + 0   // Badr4 + 0
#define DA_FIFOCLR  iobase[4] + 2   // Badr4 + 2
#define DEBUG               1
#define VVAVE_FILE_MAGIC    0x6429
#define INTERVAL_NS         100000
#define CALIB_COUNT         200

int badr[5]; // PCI 2.2 assigns 6 IO base addresses
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//==========global var=========

// variables to store standard wave data
char waveform;
double frequency;
double amplitude;
unsigned int data[500];
int numOfSteps;
// end of  standard wave variables

struct pci_dev_info info;
void *hdl;
uintptr_t iobase[6];

unsigned int int_flag;

int dio_kill = 0;
int debounce_flag = 0;
void *read_adc_ptr;
void *adc_data_ptr;

pthread_t pot2_calib;
pthread_attr_t* attr_ptr;

double adc_scale;
unsigned int calib1, calib2;

typedef struct
{
    size_t size;
    size_t capacity;
    uint16_t *data;
} Buffer;

typedef struct
{
    uint16_t magic;
    size_t size;
    size_t capacity;
} Vvave_signature;

//============functions declaration=============

        
void adc_read(uint16_t *adc_in, unsigned int port);                     // interface with PCI DAS 1602/16
void dio(uintptr_t *dio_in);
void dac(uint16_t *data);
void buffer_resize(Buffer *buffer, size_t new_capacity);                // buffer used to store uint16_t data before saving/printing
void buffer_write(Buffer *buffer, const uint16_t *data, size_t size);
void buffer_free(Buffer *buffer);
int buffer_read(Buffer *buffer, size_t index, uint16_t *out_value);
void adc_load_file(Buffer *buffer, const char *file_path);              // save file , load file
void adc_save_file(Buffer *buffer, const char *file_path);
char *shift(int *argc, char ***argv);        // reading flags in user input
void handle_sigusr1(int sig);                // signal handling ( Ctrl-C or 0xF1 )
void handle_sigint(int sig);
void *monitor_dio(void *arg);                // runs in a separate thread to poll digital input, for terminate and changing waveform
void* monitor_pot(void* arg);                // runs in another thread to poll adc for scaling
void cleanup();                              // runs when exit (end of code), to properly free data
void precise_sleep_until(struct timespec *next);                        // delay between dac data points
void *get_param(void *arg);
void* calibration_func(void* arg);           // calibrate to find the fluctuation range of potentiometer
void create_wave();

int main(int argc, char *argv[])
{
    //=================== MAIN LOCAL VARIABLE DECLARATION  ========================================

	pthread_t monitor_thread, pot_thread, pot1_calib;
	unsigned int adc1_chan, adc2_chan;
	pthread_attr_t attr;
	uint16_t adc_in_1, adc_in_2, adc_in_1_prev;
	uintptr_t dio_in, dio_in_prev;
	uint16_t data_to_scr;

    const char *program;
    const char *flag;
    const char *file_pathname;
    unsigned int i, count;

    int data_size;
    char input_size[200];

	struct timespec next;
	unsigned int comparison_buf;

    // ====================  MAIN VARIABLE INIT  ========================================
    Buffer *adc_data = (Buffer *)malloc(sizeof(Buffer));
    Buffer *read_adc = (Buffer *)malloc(sizeof(Buffer));

    adc_data->data = 0;
    adc_data->size = 0;
    adc_data->capacity = 0;
    read_adc->data = 0;
    read_adc->size = 0;
    read_adc->capacity = 0;

    int_flag = 0;
    // ====================  END OF MAIN VARIABLE INIT  ========================================

    // ====================  IO MAPPING  ========================================

    memset(&info, 0, sizeof(info)); // set info to 0
    if (pci_attach(0) < 0)
    { // try connecting to PCI, but the PCI PATH is set somewhere else
        perror("pci_attach");
        exit(EXIT_FAILURE);
    }

    /* Vendor and Device ID */
    info.VendorId = 0x1307;
    info.DeviceId = 0x01;

    if ((hdl = pci_attach_device(0, PCI_SHARE | PCI_INIT_ALL, 0, &info)) == 0)
    {
        perror("pci_attach_device");
        exit(EXIT_FAILURE);
    }

    if (DEBUG)
    {
        // printf("\nDAS 1602 Base addresses:\n\n");
        for (i = 0; i < 5; i++)
        {
            badr[i] = PCI_IO_ADDR(info.CpuBaseAddress[i]);
            // if(DEBUG) printf("Badr[%d] : %x\n", i, badr[i]);
        }

        // printf("\nReconfirm Iobase:\n");  			// map I/O base address to user space
        for (i = 0; i < 5; i++)
        { // expect CpuBaseAddress to be the same as iobase for PC
            iobase[i] = mmap_device_io(0x0f, badr[i]);
            // printf("Index %d : Address : %x ", i,badr[i]);
            // printf("IOBASE  : %x \n",iobase[i]);
        }
    }
    // Modify thread control privity
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1)
    {
        perror("Thread Control");
        exit(1);
    }

    // ====================  END OF IO MAPPING  ========================================

        // ====================  INITIALIZATION  ========================================
	system("clear");                        // --------------------------------- clear screen before running code
	read_adc_ptr = read_adc;
	adc_data_ptr = adc_data;
	adc1_chan = 0x0D00;                     // --------------------------------- define MUX channel
	adc2_chan = 0x0D11;
    i = 0;
    count = 0;
	adc_in_1 = 0;
    adc_in_2 = 0;
    adc_in_1_prev = 0;
    atexit(cleanup);                        // --------------------------------- runs cleanup() during at exit
	printf("\n");
        // ====================  END OF INITIALIZATION  ========================================

    if (signal(SIGUSR1, handle_sigusr1) < 0)
    {
        fprintf(stderr, "[ERROR] Failed at sigusr1, %s\n", strerror(errno));
        exit(1);
    }
    if (signal(SIGINT, handle_sigint) < 0)
    {
        fprintf(stderr, "[ERROR] Failed at sigint, %s\n", strerror(errno));
        exit(1);
    }

    program = shift(&argc, &argv);

    if (argc == 0)
    {
        fprintf(stderr, "[ERROR] No flag provided\n");
        exit(1);
    }

    flag = shift(&argc, &argv);
    if (strcmp(flag, "-i") == 0)
    {
        i = 0;
        if (argc == 0)
        {
            fprintf(stderr, "[ERROR] No file provided to write into\n");
            exit(1);
        }
        else
        {
            file_pathname = shift(&argc, &argv);
        }
    }
    else if (strcmp(flag, "-w") == 0)
    {
        if (argc == 0)
        {
            fprintf(stderr, "[ERROR] No file provided to write into\n");
            exit(1);
        }
        else
        {
            file_pathname = shift(&argc, &argv);
        }
        adc_load_file(read_adc, file_pathname);
        printf("[INFO] Buffer loaded successfully\n");
    }
    else if (strcmp(flag, "-r") == 0)
    {
        if (argc == 0)
        {
            fprintf(stderr, "[ERROR] please enter <waveform> <frequency> <amplitude>\n");
            exit(1);
        }
    }
    else
    {
        fprintf(stderr, "[ERROR] Unknown flag '%s' provided\n", flag);
        exit(1);
    }

    // =====================  DIO INIT CHECKER  ======================
    dio(&dio_in);
    dio_in_prev = 0x00;
    if (dio_in != 0xf0)
    {
        fprintf(stdout, "Program will hang until 0xf0 is set\n\n");
    }
    while (dio_in != 0xf0)
    {
        if (dio_in != dio_in_prev)
        {
            fprintf(stdout, "[WARN] Please set digital input switches to %02X\n", dio_in);
            dio_in_prev = dio_in;
        }
        else if (int_flag == 1)
        {
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


		// =====================  ADC INIT  ======================

    calib1 = 0;
    calib2 = 0;
    adc_read(&adc_in_1, (unsigned int) 0x0D00);
    adc_read(&adc_in_2, (unsigned int) 0x0D11);
    while( adc_in_1 < 1000 || adc_in_1 > 60000 || adc_in_2 < 1000 || adc_in_2 > 60000){
        fprintf(stdout, "Please set both potentiometer 1000 < value < 60000 -- adc1: %u, adc2: %u\n", (unsigned int) adc_in_1, (unsigned int) adc_in_2);
        delay(1000);
        adc_read(&adc_in_1, (unsigned int) 0x0D00);
        adc_read(&adc_in_2, (unsigned int) 0x0D11);
        if(int_flag != 0){
            printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
            exit(1);
        }
    }	 
    fprintf(stdout, "Calibration running in 3 seconds, please do not touch potentiometer\n");
    delay(1000);
    fprintf(stdout, "Calibration running in 2 seconds, please do not touch potentiometer\n");
    delay(1000);
    fprintf(stdout, "Calibration running in 1 seconds, please do not touch potentiometer\n");
    delay(1000);
    fprintf(stdout, "Calibration running at background\n");
        // =====================  END OF ADC INIT ======================

		// =====================   PTHREAD INITIALIZATION ======================
        if(pthread_attr_init(&attr) != 0){
            fprintf(stderr, "[ERROR] Failed to init switch attr\n");
            fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
            exit(1);
        }
        if(pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0){
            fprintf(stderr, "[ERROR] Failed to set detached switch attr\n");
            fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
            exit(1);
        }
        attr_ptr = &attr;
        if(pthread_create(&monitor_thread, &attr, monitor_dio, NULL) != 0){
            fprintf(stderr, "[ERROR] Failed to create monitor_thread\n");
            fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
            exit(1);
        }
        if(pthread_create(&pot_thread, &attr, monitor_pot, NULL) != 0){
            fprintf(stderr, "[ERROR] Failed to create pot_thread\n");
            fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
            exit(1);
        }
        if(pthread_create(&pot1_calib, NULL, calibration_func, &adc1_chan) != 0){
            fprintf(stderr, "[ERROR] Failed to create pot_thread\n");
            fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
            exit(1);
        }
        if(pthread_create(&pot2_calib, NULL, calibration_func, &adc2_chan) != 0){
            fprintf(stderr, "[ERROR] Failed to create pot_thread\n");
            fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
            exit(1);
        }
            // =====================  END OF PTHREAD INITIALIZATION  ======================

    if (strcmp(flag, "-i") == 0)
    {

        while (1)
        {

            if(int_flag != 0){
                printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
                exit(1);
            }
            printf("Please key in the size of the data that you would like to record.\n");
            if (fgets(input_size, sizeof(input_size), stdin) != NULL)
            {
                if (sscanf(input_size, "%d", &data_size) == 1 && input_size[0] != '\n')
                {
                    data_size = atoi(input_size);
                    printf("\nYou entered: %ld \n \nPlease adjust the potentiometer to record the data into the file.\n", data_size);
                    break;
                }
                else
                {
                    printf("\nInvalid Input, Please enter a valid number.\n");
                }
            }
            else
            {
                printf("\nError reading input. Please try again.\n");
            }
        }
        if(pthread_join(pot2_calib, NULL) != 0){
			fprintf(stderr, "[ERROR] pthread_join failed. Err: %s\n", strerror(errno));
			exit(1);
		}
		if(pthread_create(&pot1_calib, NULL, calibration_func, &adc1_chan) != 0){
			fprintf(stderr, "[ERROR] Failed to create pot_thread\n");
			fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno)); 
			exit(1);
		}
		if(pthread_join(pot1_calib, NULL) != 0){
			fprintf(stderr, "[ERROR] pthread_join failed. Err: %s\n", strerror(errno));
			exit(1);
		}
        while (i < data_size)
        {
            adc_read(&adc_in_1, adc1_chan);
			if( adc_in_1 < adc_in_1_prev){
				comparison_buf = (unsigned int)adc_in_1_prev - (unsigned int)adc_in_1;
			} else {
				comparison_buf = (unsigned int)adc_in_1 - (unsigned int)adc_in_1_prev;
			}
			if ( comparison_buf > calib1){
				buffer_write(adc_data, &adc_in_1, sizeof(uint16_t));
				printf("inserted %d: %d: %d \n", i, (unsigned int)adc_in_1_prev, (unsigned int)adc_in_1);
				adc_in_1_prev = adc_in_1;
				i++;
			}
            if (int_flag != 0)
            {
                break;
            }
        }
        if (int_flag == 0)
        {
            printf("\n[INFO] SAVING . . . . . .\n");
            adc_save_file(adc_data, file_pathname);
            printf("[INFO] Finished\n");
        }
        else
        {
            printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
        }
    }
    else if (strcmp(flag, "-w") == 0)
    {
		i = 0;
		count = read_adc->size/sizeof(read_adc->data[0]);
		while( (i < (read_adc->size/sizeof(read_adc->data[0])) ) & (int_flag == 0) ){
			buffer_read(read_adc, (size_t)i,&data_to_scr); 
			printf("inserted %d: %d \n", i, (unsigned int)data_to_scr);
			i++;
		}

		printf("[INFO] Number of points read: %u\n", (unsigned int)count);
		printf("[INFO] Data loaded successfully\n");
		printf("[INFO] Sending data to dac . . . . . . \n");

		if(clock_gettime(CLOCK_MONOTONIC, &next)){
			fprintf(stderr, "[ERROR] clock_gettime failed 'next'\n");
			fprintf(stderr, "[ERROR] Error code: %s\n", strerror(errno));
			exit(1);
		}

        while (1)
        {

			for(i=0; i<count; ++i){
				buffer_read(read_adc, (size_t)i,&data_to_scr); 
				data_to_scr = (uint16_t) ((double) data_to_scr *adc_scale); 
				dac(&data_to_scr);
				next.tv_nsec += INTERVAL_NS;

				if(next.tv_nsec >= 1000000000){
					next.tv_nsec -= 1000000000;
					next.tv_sec += 1;
				}
				precise_sleep_until(&next);
			} 		
			i = 0;	

			if(int_flag != 0){
				printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
				break;
			}
		}
    }
    else if (strcmp(flag, "-r") == 0)
    {
    	if(pthread_join(pot2_calib, NULL) != 0){
			fprintf(stderr, "[ERROR] pthread_join failed. Err: %s\n", strerror(errno));
			exit(1);
		}
		printf("Calibration complete\n");
        //================================= standard wave mode ===============================
        numOfSteps = 500;
        waveform = *argv[0];
        frequency = atof(argv[1]);
        amplitude = atof(argv[2]); 
        create_wave();

        pthread_create(NULL, NULL, &get_param, NULL); // only start this thread if -r flag raised
        
        while (1)
        {
            for (i = 0; i < numOfSteps; i++)
            {            	
                // output wave
                out16(DA_CTLREG, 0x0a23); // DA Enable, #0, #1, SW 5V unipolar		2/6
                out16(DA_FIFOCLR, 0);     // Clear DA FIFO  buffer
                out16(DA_Data, (short)(unsigned int) (  (double) data[i] * adc_scale));
                out16(DA_CTLREG, 0x0a43); // DA Enable, #1, #1, SW 5V unipolar		2/6
                out16(DA_FIFOCLR, 0);     // Clear DA FIFO  buffer
                out16(DA_Data, (short)(unsigned int) (  (double) data[i] * adc_scale));

                if (i % (int)(numOfSteps / frequency) == 0)
                {
                    fflush(stdout);
                    printf("\a");
                    fflush(stdout);
                }

                usleep(300);
            }
            if (int_flag != 0)
            {
                printf("\n[INFO] Interrupt signal triggered . . . . . . \n");
                break;
            }
        }
        // Unreachable code
        // Reset DAC to 5v

        printf("\n\nExit Demo Program\n");
        pci_detach_device(hdl);
        return (0);
    }
    else
    {
        fprintf(stderr, "[ERROR] Unknown flag '%s' provided\n", flag);
        exit(1);
    }

	if(pthread_join(monitor_thread, NULL) < 0){
		if( errno != EINVAL){
			fprintf(stderr, "monitor_thread not freed properly, %s\n", strerror(errno));
			exit(1);
		}
	}

	if(pthread_join(pot_thread, NULL) < 0){
		fprintf(stderr, "pthread_join pot_thread failed\n");
		exit(1);
	}

    return 0;
}

// ====================  END OF MAIN  ========================================

//============function to create standard wave=============

void create_wave()
{
    int samplesPerCycle;
    int halfCycle;
    float delta, dummy;
    unsigned int i;
    numOfSteps = 500;                         // number of samples/steps total
    samplesPerCycle = numOfSteps / frequency; // how many samples/steps a cycle will take

    halfCycle = samplesPerCycle / 2; // number of steps for half a cycle
    switch (waveform)
    {
    case 's':
        // create sine wave
        delta = (2.0 * 3.142 * frequency) / numOfSteps; // increment

        for (i = 0; i < numOfSteps; i++)
        {
            dummy = ((sinf((float)(i * delta))) + 1.0) * 0x8000 * amplitude / 5.0;
            data[i] = (unsigned)dummy; // add offset
        }
        break;
    case 'q':
        // create square wave
        for (i = 0; i < numOfSteps; i++)
        {

            if ((i % samplesPerCycle) < halfCycle)
            {
                data[i] = amplitude / 5.0 * 0xFFFF;
            }
            else
            {
                data[i] = (unsigned)0.0 * 0x8000;
            }
        }
        break;
    case 'w':
        // create saw tooth wave
        delta = (0xFFFF / numOfSteps * frequency) * (amplitude / 5);
        for (i = 0; i < numOfSteps; i++)
        {
            dummy = (i % samplesPerCycle);
            dummy = dummy * delta;
            data[i] = dummy;
        }
        break;
    case 't':
        // create triangle wave
        delta = (float)((0xFFFF / numOfSteps * frequency) * amplitude / 5.0);
        for (i = 0; i < numOfSteps; i++)
        {
            if ((i % samplesPerCycle) > halfCycle)
            {
                dummy = (float)((samplesPerCycle * delta) - (((i % samplesPerCycle)) * delta));
            }
            else
            {
                dummy = (float)(i % samplesPerCycle) * delta;
            }
            data[i] = (unsigned int)dummy * 2;
        }
        break;
    default:
        printf("Invalid wave type.");
        break;
    }
}
//=====================thread that continuously check for user input==========================
void *get_param(void *arg)
{
    char input[200];
    char w;
    double f;
    double a;
    int samplesPerCycle;
    int halfCycle;

    float delta, dummy;
    unsigned int i;
    numOfSteps = 500;

    // Display welcome message and instructions
    printf("\n+-----------------------------------------+\n");
    printf("|       Welcome to the Wave Generator     |\n");
    printf("+-----------------------------------------+\n");
    printf("| Usage: ./wavegen [options] <args>       |\n");
    printf("| Options:                                |\n");
    printf("|   -r : Change waveform parameters       |\n");
    printf("|   -i : Record ADC data                  |\n");
    printf("|   -w : Output waveform to DAC           |\n");
    printf("| Instructions:                           |\n");
    printf("| 1. Waveform Generation (-r):            |\n");
    printf("|    Format: <type> <freq> <amp>          |\n");
    printf("|    Type: s(sine), q(square),            |\n");
    printf("|          t(triangle), w(sawtooth)       |\n");
    printf("|    Freq: 1-25 Hz, Amp: 0-5V             |\n");
    printf("|    Ex: ./wavegen -r s 10 2.5 for 10 Hz  |\n");
    printf("|        sine, 2.5V                       |\n");
    printf("+-----------------------------------------+\n");
    printf("| 2. Record ADC Data (-i):                |\n");
    printf("|    Ex: ./wavegen -i file.wave           |\n");
    printf("|    To record wave, adjust A/D 1         |\n");
    printf("+-----------------------------------------+\n");
    printf("| 3. Output to DAC (-w):                  |\n");
    printf("|    Ex: ./wavegen -w file.wave           |\n");
    printf("+-----------------------------------------+\n");
    printf("| 4. Run: Set waveform via input switch:  |\n");
    printf("|    s:0xF2(sine), q:0xF4(square),        |\n");
    printf("|    w:0xF6(saw), t:0xF6(triangle)        |\n");
    printf("+-----------------------------------------+\n");
    printf("| 5. Adjust amplitude:                    |\n");
    printf("|    Adjust A/D 0                         |\n");
    printf("+-----------------------------------------+\n");
    printf("| 6. Exit: Ctrl+C or set input to 0xF1    |\n");
    printf("+-----------------------------------------+\n");
    printf("| Developed by: Daniel Lim Wei Hung,      |\n");
    printf("| Feng Qi Hao, Gan Yi Xiang, Ho Min Han,  |\n");
    printf("| Liang Fu Den, Ng Hong Xi, Yap Jia Jun   |\n");
    printf("+-----------------------------------------+\n");

    while (1)
    {
        if (fgets(input, sizeof(input), stdin) == NULL) // break if no input
            break;
        if (sscanf(input, "%c %lf %lf", &w, &f, &a) == 3) // update with new wave
        {
            pthread_mutex_lock(&mutex);
            waveform = w;
            frequency = f;
            amplitude = a;
            create_wave();
            pthread_mutex_unlock(&mutex);
        }
        else
        {
            printf("invalid input\n");
            printf("Usage: <waveform> <frequency> <amplitude>\n");
            printf("Waveforms: s = sine, q = square, t = triangular, w = sawtooth\n");
        }
    }
    return 0;
}
//======================end of thread===============================================

// ====================  DAQ RELATED FUNCTIONS  ========================================


void adc_read(uint16_t *adc_in, unsigned int port){
	out16(INTERRUPT,0x60c0);		
	out16(TRIGGER,0x2081);
	out16(AUTOCAL,0x007f);
	out16(AD_FIFOCLR,0);
	out16(MUXCHAN,0x0D00);
	out16(MUXCHAN,port);
	delay(1);						
	out16(AD_DATA,0); 
	while(!(in16(MUXCHAN) & 0x4000));			
	*adc_in = in16(AD_DATA);
	delay(5);
}


void dio(uintptr_t *dio_in){
	uintptr_t local_dio;

	out8(DIO_CTLREG,0x90);				// There is Port A, Port C(Upper), Port B, Port C(Lower). Sets input/output		
	*dio_in =in8(DIO_PORTA); 																							
	local_dio = *dio_in;
	out8(DIO_PORTB, local_dio);
}
void dac(uint16_t *data){
    out16(DA_Data,(short) *data);																																		
    out16(DA_FIFOCLR, 0);					// Clear DA FIFO  buffer	
}

// ====================  END OF DAQ RELATED FUNCTIONS  ========================================

// ====================  BUFFER RELATED FUNCTIONS  ========================================

/*
    resize buffer to new_capacity, my current code, size is same as capacity. just by
    reading  [ buffer->size / sizeof(int) ], i know how many integer inside the buffer
*/
void buffer_resize(Buffer *buffer, size_t new_capacity)
{
    buffer->capacity = new_capacity;
    buffer->data = realloc(buffer->data, buffer->capacity);
    // printf("resizing\n");
}

void buffer_write(Buffer *buffer, const uint16_t *data, size_t size)
{
    /*  How does buffer_write work
        stores uint16_t as bytes in an array

        before storing, need to realloc the old buffer so there is enough space for new data

        before exiting, need to free;
    */
    if (buffer->size + size > buffer->capacity)
    {
        buffer_resize(buffer, buffer->capacity + size);
    }
    // printf("copying\n");
    memcpy(buffer->data + (buffer->size / sizeof(uint16_t)), data, size);
    buffer->size += size;
}

void buffer_free(Buffer *buffer)
{
    free(buffer->data);
    buffer->data = NULL;
    buffer->size = 0;
    buffer->capacity = 0;
}

int buffer_read(Buffer *buffer, size_t index, uint16_t *out_value)
{
    /*	How does buffer_read work ?

        buffer_read access the buffer like an array
    */
    if (index >= buffer->size)
    {
        return 0;
    }
    *out_value = buffer->data[index];
    return 1;
}

// ====================  END OF BUFFER RELATED FUNCTIONS  ========================================

void adc_load_file(Buffer *buffer, const char *file_path)
{
    FILE *f;
    Vvave_signature meta;
    size_t n;

    f = fopen(file_path, "rb");
    if (f == NULL)
    {
        fprintf(stderr, "[ERROR] Could not open file '%s' : %s\n", file_path, strerror(errno));
        exit(1);
    }

    n = fread(&meta, sizeof(meta), 1, f);
    if (n < 1)
    {
        fprintf(stderr, "ERROR: Could not read meta data from file '%s':%s\n", file_path, strerror(errno));
        exit(1);
    }
    if (meta.magic != VVAVE_FILE_MAGIC)
    {
        fprintf(stderr, "[ERROR] %s does not appear to be a valid file\n", file_path);
        fprintf(stderr, "[ERROR] Magic number detected to be %04X. Expected %04X.\n", meta.magic, VVAVE_FILE_MAGIC);
        exit(1);
    }
    buffer->capacity = meta.capacity;
    buffer->size = meta.size;
    buffer_resize(buffer, meta.capacity);

    n = fread(buffer->data, sizeof(uint16_t), (meta.size / sizeof(uint16_t)), f);
}

char *shift(int *argc, char ***argv)
{
    char *result;
    assert(*argc > 0);
    result = **argv;
    *argv += 1;
    *argc -= 1;
    return result;
}

void *monitor_dio(void *arg)
{
    uintptr_t current_dio;

    while (dio_kill)
    {
        dio(&current_dio);
        if (current_dio == 0xF1)
        {
            raise(SIGUSR1);
            dio_kill = 0;
        }
        else if (current_dio == 0xF2 && waveform != 's')
        {

            waveform = 's';
            create_wave();
        }
        else if (current_dio == 0xF4 && waveform != 'q')
        {

            waveform = 'q';
            create_wave();
        }
        else if (current_dio == 0xF8 && waveform != 'w')
        {

            waveform = 'w';
            create_wave();
        }
        else if (current_dio == 0xF6 && waveform != 't')
        {

            waveform = 't';
            create_wave();
        }
        usleep(500);
    }
    return NULL;
}

void* monitor_pot(void* arg){
	uint16_t adc2, adc2_prev;
	unsigned int comparison_buf;
	pthread_join(pot2_calib, NULL);

	adc_read(&adc2,0x0D11);
	adc2_prev = adc2;
	while(dio_kill){
		adc_read(&adc2,0x0D11);
		if( adc2 < adc2_prev){
			comparison_buf = (unsigned int)adc2_prev - (unsigned int)adc2;
		} else {
			comparison_buf = (unsigned int)adc2 - (unsigned int)adc2_prev;
		}
		if( comparison_buf > calib2 ){
			adc_scale = (double)adc2 / (double) 0xFFFF;
		}
		adc2_prev = adc2;
	}
	return NULL;
}


void* calibration_func(void* arg){
	unsigned int calib_local, port, comparison_buf, i;
	uint16_t adc, adc_prev;


	calib_local = 0;
	port = *(unsigned int*)arg;
	adc_read(&adc, port);
	adc_prev = adc;

	for(i = 0; i < CALIB_COUNT; ++i){
		adc_read(&adc, port);
		if( adc < adc_prev){
			comparison_buf = (unsigned int)adc_prev - (unsigned int)adc;
		} else {
			comparison_buf = (unsigned int)adc - (unsigned int)adc_prev;
		}
		if( comparison_buf > calib_local ){
			calib_local = comparison_buf;
		}
		adc_prev = adc;
	}
	if( calib_local > 200 ){
		if(port == 0x0D00){
			fprintf(stderr, "Large fluctuation detected at ADC1 at %u.\n",calib_local);
		} else if(port == 0x0D11){
			fprintf(stderr, "Large fluctuation detected at ADC2 at %u.\n",calib_local);
		} else {
			fprintf(stderr, "Unknown port %02x\n", (unsigned int) port);
			exit(1);
		}
		fprintf(stderr, "Please check power supply or potentiometer connections.\nAborting . . . . . . \n");
		exit(1);
	}
	if(port == 0x0D00){
		calib1 = calib_local;
	} else if(port == 0x0D11){
		calib2 = calib_local;
	} else {
		fprintf(stderr, "Unknown port %02x\n", (unsigned int) port);
		exit(1);
	}
	return NULL;

}

void adc_save_file(Buffer *buffer, const char *file_path)
{
    Vvave_signature meta;
    FILE *f;
    meta.magic = VVAVE_FILE_MAGIC;
    meta.size = buffer->size;
    meta.capacity = buffer->capacity;
    f = fopen(file_path, "wb");
    if (f == NULL)
    {
        fprintf(stderr, "ERROR: Could not open file '%s':%s\n", file_path, strerror(errno));
        exit(1);
    }

    fwrite(&meta, sizeof(meta), 1, f);
    if (ferror(f))
    {
        fprintf(stderr, "ERROR: Could not write to file '%s':%s\n", file_path, strerror(errno));
        exit(1);
    }

    fwrite(buffer->data, buffer->size, 1, f);
    if (ferror(f))
    {
        fprintf(stderr, "ERROR: Could not write to file '%s':%s\n", file_path, strerror(errno));
        exit(1);
    }

    fclose(f);
}
void cleanup(){
	buffer_free(read_adc_ptr);
	buffer_free(adc_data_ptr);
	pthread_attr_destroy(attr_ptr);
	printf("\n[INFO] EXIT SUCCESS\n");
}
void precise_sleep_until(struct timespec *next){
	int ret;
	ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
	if(ret < 0){
		if(errno != ENOENT){
			fprintf(stderr, "[ERROR] Exited with non-interrupt condition\n[ERROR] clock_nanosleep failed\n[ERROR] Err: %s\n", strerror(errno));
		}
		exit(1);
	}
}
void handle_sigusr1(int sig)
{
    ++int_flag;
}
void handle_sigint(int sig)
{
    ++int_flag;
}
