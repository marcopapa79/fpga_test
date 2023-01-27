/*
* =====================================================================================
*
*       Filename:  fpga_test_cfg_spi.c
*
*    Description:  FPGA registers access test
*
*        Version:  1.0
*        Created:  09/10/2020
*       Revision:  none
*             Id:  
*       Compiler:  gcc
*
*         Author:  m.papa
*   Organization:  Quixant Limited
*
*      Copyright:  (c) Quixant Italia
*        License:  Quixant Limited License (http://www.quixant.com)
*
* =====================================================================================
* This software is owned and published by:
* Quixant Italia srl, Contrada Strade Bruciate 1, Torrita Tiberina, Roma, Italy
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software constitutes of code for use in programming Quixant gaming
* platforms. This software is licensed by Quixant to be adapted only for
* use with Quixant's series of gaming platforms. Quixant is not responsible
* for misuse of illegal use of the software for platforms not supported herein.
* Quixant is providing this source code "AS IS" and will not be responsible for
* issues arising from incorrect user implementation of the source code herein.
*
* QUIXANT MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE, ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE
* INCLUDING, WITHOUT LIMITATION, NO IMPLIED WARRANTY OF MERCHANTABILITY FITNESS
* FOR A PARTICULAR PURPOSE OR USE, OR NONINFRINGEMENT. QUIXANT WILL HAVE NO
* LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT, NEGLIGENCE OR INCLUDING,
* WITHOUT LIMITATION, ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, OR
* CONSEQUENTIAL DAMAGES OF LOSS OF DATA, SAVINGS OR PROFITS, EVEN IF QUIXANT
* HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*
* =====================================================================================
*/
#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/io.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <cmath>

#include <asm/termbits.h>

#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <pthread.h>
#include "openssl/evp.h"
#endif

#include <stdio.h>
/*=============================
====== PCIe definition ======*/
#include "PCIe_access.h"
#define IO_DEVID    "19d40100"
#define NVRAM_DEVID "19d40200"
#define SECs_DEVID  "19d40300"
#define MUART_DEVID "19d40a00"
#define QLI_DEVID   "19d40e00"
#define SBC_DEVID   "19d40700"
#define ATS_DEVID   "19d40c00"
/*===========================*/


/*=============================
====== test definition ======*/

#define _DEBUG              0

#define PRINT_ALL           2
#define PRINT_VALUES     	1
#define NO_PRINT_VALUES     0


#define CPOL_1              0x08
#define CPOL_0              0x00
#define CPHA_1              0x04
#define CPHA_0              0x00
#define CS_ENA              0x01
#define CS_DIS              0x00
#define CORE_ENA            0x40
#define CORE_FPGA_ENA       0x80
#define CORE_DIS            0x00
#define CS_CFG_ROM_ENA      CORE_ENA | CPOL_1 | CPHA_1 | CS_ENA
//#define CS_CFG_ROM_DIS      CORE_ENA | CPOL_1 | CPHA_1 | CS_DIS
#define CS_FPGA_ROM_ENA     CORE_FPGA_ENA | CPOL_1 | CPHA_1 | CS_ENA
//#define CS_FPGA_ROM_DIS     CORE_FPGA_ENA | CPOL_1 | CPHA_1 | CS_DIS

#define FPGA_ROM_BASE       0x30
#define CFG_ROM_BASE        0x48

#define CMD_WRITE_ENA       0x06
#define CMD_WRITE_DISEB     0x04
#define CMD_CHIP_ERASE      0xC7
#define CMD_SECTOR_ERASE    0x20
#define CMD_DEVICE_ID       0x90
// Dummy - Dummy - 00h - (MF7-MF0) - (ID7-ID0)
#define CMD_READ_SR1        0x05
// 05h - (S7-S0)
#define CMD_WRITE_SR1       0x01
// 01h - (S7-S0)
#define CMD_READ_DATA       0x03
// 03h - A23-A16 - A15-A8 - A7-A0 - (D7-D0)
#define CMD_PAGE_PROGRAM    0x02
// 02h - A23-A16 - A15-A8 - A7-A0 - D7-D0 - D7-D0(3)

// Status Register (byte 0)
//        S7                      S6              S5                   S4 S3 S2                    S1                       S0 (busy) 
//Status Register Protect - Sector Protect - Top/Bottom Protect - Block Protect Bits[2:0] - Write Enable Latch - Erase/Write In Progress(volatile)

/*=============================
===== TAP SM definition =====*/
#define TEST_LOGIC_RESET	0x0
#define RUN_TEST_IDLE		0x1
#define SELECT_DR			0x2
#define CAPTURE_DR			0x3
#define SHIFT_DR			0x4
#define EXIT1_DR			0x5
#define PAUSE_DR			0x6
#define EXIT2_DR			0x7
#define UPDATE_DR			0x8
#define SELECT_IR			0x9
#define CAPTURE_IR			0xA
#define SHIFT_IR			0xB
#define EXIT1_IR			0xC
#define PAUSE_IR			0xD
#define EXIT2_IR			0xE
#define UPDATE_IR			0xF

#define F_2MHz				01
#define F_1MHz				02
#define F_500KHz			04
#define F_256KHz			08
/*===========================*/

/*===========================
===== SRAM Parameters =====*/
#define NORMAL_MODEx64		0x80
#define MIRROR_MODEx32		0x40

#define AUTOVERIFY_MODE		0x08
#define BANK_MODE			0x04
#define MIRROR_MODE			0x02
#define NORMAL_MODE			0x01

#define AES_SIZE_MODES     2  // defined only two AES modes 128 and 256
#define MAX_KEY_NUM       16  // FPGA hardcoded keys
/*===========================*/

/*===========================
===== DEBUG Parameters =====*/
#define STATUS_CMD			0x40
#define FILTER_CMD			0x30
#define START_CMD			0x10

/*===========================
===== CFG ROM old managment=====*/
#define _OLDCFGROM             0
/*===========================*/
// Utility details
//      Version
//       Date

uint8_t ver_major = 1;
uint8_t ver_minor = 13;
char date[]="27/01/2023";

/*===========================*/


/*===========================*/
//
// 		FPGA EEPROM
//	 Command line flags
//
uint8_t  f_binary = 0;              // Binary/Text output flag (default: text)
int f_read = 0;                     // FPGA read code command (default: no)
int f_write = 0;                    // FPGA write code command (default: no)
int f_save_fout = 0;                // Save the FPGA content in an output file (default: no) 
int f_help = 0;                		// Help menu (default: no) 

uint32_t mem_size = 4*1024*1024;
uint32_t sizeSecsBuffer = 32*16;  

/*===========================*/

#define SRAM_TYPE(x) ((x) == 0?"Standard SRAM":((x) == 1)?"Fast SRAM":"MRAM")
#define QLI_CAPAB(x) ((x) == 0?"Not_Present":((x) == 1)?"1-wire or 2-wires":"1-wire, 2-wires or pwm")
#define IO_TYPE(x)   ((x) == 0?"TI (without feedback)":((x) == 1)?"ST with feedback":"ONSEMI with feedback")
#define CK_MATCH(x)  ((x) == 0?"Checksum wrong":"Checksum match")
#define PULL_UP(x)   ((x) == 0?"10k pull-up":"10k-1k in parallel pull-up")
#define CCTALK_ENA(x)((x) == 0?"disabled":"enabled")
#define AUTHENTICA(x)((x) == 0?"FPGA-LP not authenticated":"FPGA-LP authenticated")
#define DMA(x)       ((x) == 0?"DMA not supported":"DMA supported")
#define RSA(x)       ((x) == 0?"RSA 2048 not supported":"RSA 2048 supported")
#define KEY_17th(x)  ((x) == 0?"KEY17_th_feature not supported":"KEY17_th_feature supported")
#define MASTER_KEY(x)((x) == 0?"Master Key Feature not supported":"Master Key Feature supported")
#define MASTER_KEY_FROM_LP(x)((x) == 1?"MASTER KEY FLAG ASSERTED":"MASTER KEY FLAG DEASSERTED")


#define ENABLE_ATS(x)((x) == 1?"ATS_ENABLED":"ATS_DISABLED")
#define ENABLE_ATS_DHCP(x)((x) == 1?"ATS_DHCP_ENABLED":"ATS_DHCP_DISABLED")
#define ENABLE_ATS_IPV6(x)((x) == 1?"ATS_IPV6_ENABLED":"ATS_IPV6_DISABLED")
#define ENABLE_ATS_VARIABLE_PACKET(x)((x) == 1?"VARIABLE_PACKET_ENABLED":"VARIABLE_PACKET_DISABLED")
#define DISABLE_ATS_SW_TRACING(x)((x) == 0?"ATS_SW_TRACING_ENABLED":"ATS_SW_TRACING_DISABLED")
#define DISABLE_ATS_HW_TRACING(x)((x) == 0?"ATS_HW_TRACING_ENABLED":"ATS_HW_TRACING_DISABLED")
#define ENABLE_ATS_LOOP(x)((x) == 1?"ATS_INTERNAL_LOOP_ENABLED":"ATS_INTERNAL_LOOP_DISABLED")
#define DISABLE_ATS_SYNC(x)((x) == 1?"ATS_SYNC_DISABLED":"ATS_SYNC_ENABLED")
#define TRACING_STATUS(x)((x) == 1?"TRACING_STARTED":"TRACING_NOT_STARTED")


/*====================================
= Pointer Declaration of PCIe BARs =*/
uint32_t *pcie_bar_io         = NULL;
uint32_t *pcie_bar_mem        = NULL;
uint32_t *pcie_bar_secs       = NULL;
uint32_t *pcie_bar_muart      = NULL;
uint32_t *pcie_bar_qli        = NULL;
uint32_t *pcie_bar_sbc        = NULL;
uint32_t *pcie_bar_ats        = NULL;
uint32_t *pcie_bar_size_io    = NULL;
uint32_t *pcie_bar_size_mem   = NULL;
uint32_t *pcie_bar_size_secs  = NULL;
uint32_t *pcie_bar_size_muart = NULL;
uint32_t *pcie_bar_size_qli   = NULL;
uint32_t *pcie_bar_size_sbc   = NULL;
uint32_t *pcie_bar_size_ats   = NULL;
/*==================================*/
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  serial_open
		 *  Description:  open serial port, 
		 *  Requiremets:  
		 *                buffer = pointer of buffer
		 * =====================================================================================
		 */
		 
		int serial_open(const char * serial_port)  
		{
			int fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
			if (fd != -1) {
				
				// Serial port setup
						
				int speed = 115200;//3000000;
				struct termios2 term2;
				int n = ioctl(fd, TCGETS2, &term2);
				if (n == 0) {
					term2.c_cflag = (term2.c_cflag & ~CSIZE) | CS8; // 8-bit chars 
					term2.c_iflag &= ~IGNBRK;                       // disable break processing 
					term2.c_lflag = 0;                              // no signaling chars, no echo, no canonical processing 
					term2.c_oflag = 0;                              // no remapping, no delays 
					term2.c_cc[VMIN]  = 0;                          // read doesn't block 
					term2.c_cc[VTIME] = 1;                          // 0.5 seconds read timeout 
					term2.c_iflag &= ~(IXON | IXOFF | IXANY);       // shut off xon/xoff ctrl 
					term2.c_cflag |= (CLOCAL | CREAD);              // ignore modem controls, enable reading 
					term2.c_cflag &= ~(PARENB | PARODD);            // shut off parity 
					term2.c_cflag &= ~CBAUD;                        // Remove current BAUD rate 
					term2.c_cflag |= BOTHER;                        // Allow custom BAUD rate using int input 
					term2.c_ispeed = speed;                         // Set the input BAUD rate 
					term2.c_ospeed = speed;                         // Set the output BAUD rate 
					n = ioctl(fd, TCSETS2, &term2);
					if (n == 0) {
						return fd;
					}
					else {
						perror("TCSETS"); 
						close(fd);
						return -1;
					}
				}
				else {
					perror("TCGETS"); 
					close(fd);
					return -1;
				}
			}
			perror("open");
			return -1;
		}
		/*
		//Open Serial Port   
		int open_port(const char * serial_port)   
		{   
			int fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
						
			fd = open( "/dev/ttyS4", O_RDWR|O_NOCTTY|O_NDELAY);   
			if (-1 == fd)  
			{   
				perror("Can't Open Serial Port");   
				return(-1);   
			}  
				
			//Restoring Serial Port to Blocking State   
			if(fcntl(fd, F_SETFL, 0)<0)   
					printf("fcntl failed!\n");   
			else   
			printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));   
			//Test whether it is a terminal device   
			if(isatty(STDIN_FILENO)==0)   
				printf("standard input is not a terminal device\n");   
			else   
				printf("isatty success!\n");   
			printf("fd-open=%d\n",fd);   
			return fd;   
		}  
			*/
		
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  ats wait for sync
		 *  Description:  ats wait 64 bits of DEBUG serial sync 
		 *  Requiremets:  
		 *                buffer = pointer of serial
		 * =====================================================================================
		 */
		uint32_t ats_wait_for_sync(uint8_t * data_rd, int fd1)  
		{	
			do
			{
				read(fd1,data_rd,1);
				if (memcmp(data_rd,"\xcc\xcc\xcc\xcc\xcc\xcc\xcc\xcc",8)==0)
				{
					printf("\n*** Sync 0xcc received \n");	
					break;									
				};
				data_rd[7]=data_rd[6];						
				data_rd[6]=data_rd[5];							
				data_rd[5]=data_rd[4];								
				data_rd[4]=data_rd[3];								
				data_rd[3]=data_rd[2];								
				data_rd[2]=data_rd[1];								
				data_rd[1]=data_rd[0];	
							
			}
			while (1); 
			return 1;			
		}
			
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  ats wait cmd acknoledge
		 *  Description:  ats wait 64 bits of DEBUG serial sync 
		 *  Requiremets:  
		 *                buffer = pointer of serial
		 * =====================================================================================
		 */
		void ats_wait_cmd_ack(uint8_t * data_rd, int fd1,uint8_t cmd)  
		{		
			uint8_t ack_cmd[8];
			uint32_t count=0;
			
			ack_cmd[4]=0xdd; ack_cmd[5]=0xdd; ack_cmd[6]=0xdd; ack_cmd[7]=0xdd;
			ack_cmd[1]=0xdd; ack_cmd[2]=0xdd; ack_cmd[3]=0xdd;
			ack_cmd[0]=0xd0+((cmd>>4)&0x0f);
			do
			{	
				read(fd1,data_rd,1);
				if (memcmp(data_rd,ack_cmd,8)==0)
				{
					printf("\n*** Command 0x%2.2x acknoledge received \n",(cmd>>4)&0xff);	
					break;									
				};
				data_rd[7]=data_rd[6];						
				data_rd[6]=data_rd[5];							
				data_rd[5]=data_rd[4];								
				data_rd[4]=data_rd[3];								
				data_rd[3]=data_rd[2];								
				data_rd[2]=data_rd[1];								
				data_rd[1]=data_rd[0];	
				
				usleep(150);// 150us
				++count;
				if (count==10000) printf("\n*** Error Exit for timeout \n");	
					
				 	
			}
			while (count<10000); //wait 1.5second
			//while (1); 
			
			//return data_rd;
		}
					
		
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  read_debug_response
		 *  Description:  debug serial response
		 *  Requiremets:  
		 *                buffer = pointer of serial
		 * =====================================================================================
		 */
		void * read_debug_response(uint8_t * data_rd, int fd1, uint32_t length, unsigned int debug_print)
		
		{		
			int ret_code;
			uint8_t *debug_response = (uint8_t *)calloc (length,sizeof(uint8_t));
			uint32_t i=0;	
			
			do//for(i = 0; i < length; i++)
			{				
				if ((ret_code=read(fd1,data_rd,1))>0)  //read(fd1,data_rd,1);
				{
					debug_response[i]=data_rd[0];
					i++;					
					if (debug_print == 1) printf("\nret code %d ",ret_code); 
					if (debug_print == 1) printf("0x%2.2x ",data_rd[0]&0xff); 	
				}; 
			} 	
			while (i < length);
								
			for(i = 0; i < length; i++)
			{		
				*(data_rd+i) = *(debug_response+i);
			}; 
			free(debug_response);
			return data_rd;
		}
		
		
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  ats_response_read
		 *  Description:  debug serial response
		 *  Requiremets:  
		 *                sync_numb = max num of sync accepted
		 *                length    = max num QWord read
		 * =====================================================================================
		 */
		void ats_response_read(int fd1, uint32_t sync_numb, uint32_t length, unsigned int debug_print)
		
		{		
			int ret_code;
			uint8_t *debug_response = (uint8_t *)calloc (length,sizeof(uint8_t));
			uint32_t i=0;	
			uint32_t count=0;	
			
			do
			{	
				read_debug_response(debug_response, fd1, 8, debug_print);								
				if (!memcmp(debug_response,"\xcc\xcc\xcc\xcc\xcc\xcc\xcc\xcc",8)==0)
				{	
					i++;
					printf("Module:%2.2x Function: %2.2x Time Stamp: %2.2x.%2.2x Payload: 0x%2.2x.%2.2x.%2.2x.%2.2x\n" ,debug_response[0]&0x7F,((debug_response[1]<<1)|(debug_response[0]>>7)),debug_response[3],debug_response[2],debug_response[7],debug_response[6],debug_response[5],debug_response[4]);
				} else ++count;
			}
			while ((count<sync_numb) && (i<length));
			
			free(debug_response);
		}
		
							
		/*
		 * =====================================================================================
		 *         Type:  aes_mode_t
		 *  Description:  AES algorithm choice: AES_CBC = CBC algorithm, AES_ECB = ECB algorithm
		 *				  Enum input parameter for QxtSecSOpen() and QxtSecSOpenDbg()
		 * =====================================================================================
		 */
		typedef enum {AES_ECB, AES_CBC} aes_mode_t;
		/*
		 * =====================================================================================
		 *         Type:  aes_size_t
		 *  Description:  AES key size: AES_128 = 128 bits length, AES_256 = 256 bits length
		 *				  Enum input parameter for QxtSecSOpen() and QxtSecSOpenDbg()
		 * =====================================================================================
		 */
		typedef enum {AES_128, AES_256} aes_size_t;
		
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  print_file
		 *  Description:  print value of memory, 
		 *  Requiremets:  
		 *                buffer = pointer of buffer
		 * =====================================================================================
		 */
		 
		 /*
		static void print_file(uint8_t * buffer)
		{
			uint32_t n;
			for (n=0; n<mem_size; n++) {
				if (f_binary==0) {
					if (n%16==0) {
						if (n) printf("\n");
						printf("[%8.8lx]", (long unsigned int)n);
					}
					printf( " %2.2x", (unsigned char)buffer[n] );
				}
				else {
					printf("%c", buffer[n]&0xff);
				}
			}
		}
*/
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  pot2
		 *  Description:  Calculate 2^y, 
		 *  Requiremets:  
		 *                y = power of two exponent
		 * =====================================================================================
		 */
		uint8_t pot2 (uint8_t y)
		{
			uint8_t z = 1;		
			if(y==0) z = 1;
			else
			{   
				for (int i=1; i<=y; i++) z=z*2; /* moltiplica x per x tante volte quanto e' il valore di esp.*/
					#if _DEBUG	
						{	
							printf("\n risultato  = %2.2x\n", z);
						}; 
					#endif 
							
			}
			return z;
		}
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  press enter to continue
		 *  Description:  wait enter to go ahead
		 *  Requiremets:  
		 *                
		 * =====================================================================================
		 */
		void wait_to_continue()
		{
			printf("Press ENTER key to Continue\n");  
			getchar();
			getchar(); 
			/*
			if ((ret_code=scanf("%d", &anykey))!=1)
				{
					printf("function read error %d\n",ret_code);
				};*/
		}
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  f_cs_enable
		 *  Description:  Enable EEPROM CSn, 
		 *  Requiremets:  
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */
		void f_cs_enable(uint8_t spi_offset, uint8_t setup_spi)
		{
			uint8_t *data_wr = (uint8_t *)calloc (2,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (2,sizeof(uint8_t));
			data_wr[0] = setup_spi;  //0x49	
			IOWr(pcie_bar_io[0] + spi_offset + 0x00, data_wr, 1, 1, 0, NO_PRINT_VALUES); 
			free(data_wr);
			free(data_rd);
		}
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  f_cs_disable
		 *  Description:  Enable EEPROM CSn, 
		 *  Requiremets:  
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */		
		void f_cs_disable(uint8_t spi_offset, uint8_t setup_spi)
		{
			uint8_t *data_wr = (uint8_t *)calloc (2,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (2,sizeof(uint8_t));
			data_wr[0] = (setup_spi & 0xFE);  //0x48	
			IOWr(pcie_bar_io[0] + spi_offset + 0x00, data_wr, 1, 1, 0, NO_PRINT_VALUES);
			free(data_wr);
			free(data_rd);
		}
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  wait_fifo_empty
		 *  Description:  wait SPI fifo empty
		 *  Requiremets:  		 
		 *				  data_rd 	   = status register byte 0 content
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 * =====================================================================================
		 */
		void wait_fifo_empty (uint8_t spi_offset)
		{
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
			do{ 
				IORd(pcie_bar_io[0] + spi_offset + 0x01, data_rd, 1, 1, NO_PRINT_VALUES);      
			} while (data_rd[0] & 0x01);
		}	
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  rd_dev_id_man_id
		 *  Description:  Read EEPROM Device ID and Manufactured ID, (W25Q32JV command 0x90)
		 *  Requiremets:  		 
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */
		void rd_dev_id_man_id (uint8_t spi_offset, uint8_t setup_spi)
		{
						
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
								
			f_cs_enable(spi_offset, setup_spi);				
			data_wr[0]= CMD_DEVICE_ID; //0x90;
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // DEV ID CMD
			wait_fifo_empty(spi_offset);			
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			data_wr[0]= 0x00;
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // dummy 
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // dummy      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);     
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // dummy      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);     
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // man id      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES); 
			printf("\n Manufacturer ID \t = %2.2x ", data_rd[0]&0xff);
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // dev id      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);   
			printf("\n Device ID \t\t = %2.2x\n\n ", data_rd[0]&0xff);
			
			f_cs_disable(spi_offset, setup_spi);
							
			free(data_wr); 
			free(data_rd); 
				
		}
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  read_sr
		 *  Description:  Read EEPROM Status Register, (W25Q32JV command 0x05)
		 *  Requiremets:  		 
		 *				  data_rd 	   = status register byte 0 content
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */
		void * read_sr (uint8_t * data_rd, uint8_t spi_offset, uint8_t setup_spi)
		{
			
				uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
				
				f_cs_enable(spi_offset, setup_spi);
				data_wr[0]= CMD_READ_SR1; //0x05;
				IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // READ SR CMD
				wait_fifo_empty(spi_offset);
				IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
				
				data_wr[0]= 0x00;						
				IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // status register     
				wait_fifo_empty(spi_offset);
				IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
				
				#if _DEBUG	
				{	
					printf("\n Status Register \t = %2.2x ", data_rd[0]&0xff);
				}; 
				#endif 	
				f_cs_disable(spi_offset, setup_spi);
				
				free(data_wr); 
			
			return data_rd;
		}		
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  wr_ena_latch
		 *  Description:  Write enable EEPROM latch, (W25Q32JV command 0x06) 
		 *  Requiremets:  
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */	
		void wr_ena_latch (uint8_t spi_offset, uint8_t setup_spi, unsigned int debug_print)
		{
			
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
								
			if (debug_print == 1) printf("\n** Set up Write Enable **\n"); 
			
			f_cs_enable(spi_offset, setup_spi);		
			data_wr[0]= CMD_WRITE_ENA; //0x06;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // CMD_WRITE_ENA CMD
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);  
			f_cs_disable(spi_offset, setup_spi);
				
			free(data_wr); 
			free(data_rd); 
			
		}	
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  wr_dise_latch
		 *  Description:  Write disable EEPROM latch, (W25Q32JV command 0x04)
		 *  Requiremets:  
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */	
		void wr_dise_latch (uint8_t spi_offset, uint8_t setup_spi, unsigned int debug_print)
		{
			
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
								
			if (debug_print == 1) printf("\n** Set up Write Disable **\n"); 
			
			f_cs_enable(spi_offset, setup_spi);	
			data_wr[0]= CMD_WRITE_DISEB; //0x04;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // CMD_WRITE_DISEB CMD
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			f_cs_disable(spi_offset, setup_spi);
				
			free(data_wr); 
			free(data_rd); 
			
		}			
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  write_sr
		 *  Description:  Write EEPROM Status Register, (W25Q32JV command 0x01)
		 *  Requiremets:  		 
		 *				  sr_byte      = status register byte 0 content
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */
		void write_sr (uint8_t * sr_byte, uint8_t spi_offset, uint8_t setup_spi, unsigned int debug_print)
		{
			
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
			wr_ena_latch(spi_offset, setup_spi, debug_print);

			if (debug_print == 1) printf("\n** Write EEPROM Status Register **\n"); 

			f_cs_enable(spi_offset, setup_spi);				
			data_wr[0]= CMD_WRITE_SR1; //0x01;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // WRITE SR CMD
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, sr_byte, 1, 1, 0, NO_PRINT_VALUES); // status register  0x9C   
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			f_cs_disable(spi_offset, setup_spi);
			fflush(stdout);
			
			read_sr(data_rd, spi_offset, setup_spi);
			do 
			{
				// code block to be executed				
				if (debug_print == 1) 
				{ 
					printf(".");
					fflush(stdout);
					usleep(1*100*1000);
				}
				read_sr(data_rd, spi_offset, setup_spi);						
			}
			while ( (data_rd[0]&0x01) != 0x00);
			
			wr_dise_latch(spi_offset, setup_spi,debug_print);
			#if _DEBUG	
			{	
				printf("\n Status Register \t = %2.2x ", data_rd[0]&0xff);
			}; 
			#endif 			
			free(data_rd); 
			free(data_wr); 
			
		}
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  sector_erase
		 *  Description:  Erase Sector for the EEPROM and then verify status register is not busy
		 *                (W25Q32JV command 0x20)
		 *  Requiremets:  Each sector is 4KB 
		 *                The W25Q32JV array is organized into 16,384 programmable pages of 256-bytes each.
		 *                Up to 256 bytes can be programmed at a time. Pages can be erased in groups of 16 (4KB sector erase)
		 *                The W25Q32JV has 1,024 erasable sectors 
		 *                
		 *                start_addr_v = (3 bytes uint8_t) 24-bit sector address
		 *                               To addessed any sector we need 10 digit, from 0 to 1023 is 0xSSS000
		 *                               Sector 0 start address is start_addr_v = 0x000000
		 *                               Sector 1 start address is start_addr_v = 0x001000
		 *                               ..
		 *                               Sector 15 start address is start_addr_v = 0x00F000
		 *                               ..
		 *                               Sector 1023 start address is start_addr_v = 0x3FF000
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */		
		void sector_erase (uint8_t * start_addr_v, uint8_t spi_offset, uint8_t setup_spi, unsigned int debug_print)
		{

			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
			wr_ena_latch(spi_offset, setup_spi, debug_print);
								
			if (debug_print == 1) printf("\n** SECTOR %d ERASE **\n",( ((start_addr_v[1] & 0xF0)>>4)+((start_addr_v[2] & 0xFF)<<4) )); 
								
			f_cs_enable(spi_offset, setup_spi);		
			data_wr[0]= CMD_SECTOR_ERASE; //0x20;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES);			// SECTOR ERASE
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 2, NO_PRINT_VALUES);	// Address 23..16      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 1, NO_PRINT_VALUES);	// Address 15..8       
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 0, NO_PRINT_VALUES);	// Address 7..0      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);			
			f_cs_disable(spi_offset, setup_spi);

			if (debug_print == 1) printf("\n Sector %d Erase ongoing ",( ((start_addr_v[1] & 0xF0)>>4)+((start_addr_v[2] & 0xFF)<<4) ));
			fflush(stdout);

			read_sr(data_rd, spi_offset, setup_spi);
			do 
			{
				// code block to be executed
				if (debug_print == 1) 
				{ 
					printf(".");
					fflush(stdout);
					usleep(1*100*1000);
				}
				read_sr(data_rd, spi_offset, setup_spi);
			}
			while ( (data_rd[0]&0x01) != 0x00);

			if (debug_print == 1) printf("\n Sector %d Erase DONE ",( ((start_addr_v[1] & 0xF0)>>4)+((start_addr_v[2] & 0xFF)<<4) ));
			free(data_wr); 
			free(data_rd);                                                      
		}
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  chip_erase
		 *  Description:  Erase entire Chip of the EEPROM and then verify status register 
		 *                is not busy. (W25Q32JV command 0xC7)
		 *  Requiremets:  
		 *				  spi_offset   = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi    = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */	
		void chip_erase (uint8_t spi_offset, uint8_t setup_spi, unsigned int debug_print)
		{

			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
			wr_ena_latch(spi_offset, setup_spi, debug_print);
								
			if (debug_print == 1) printf("\n** CHIP_ERASE **\n"); 	
			f_cs_enable(spi_offset, setup_spi);	
								
			data_wr[0]= CMD_CHIP_ERASE; //0xC7;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // READ SR CMD
			wait_fifo_empty(spi_offset);				
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			f_cs_disable(spi_offset, setup_spi);
								
			if (debug_print == 1) printf("\n Chipe Erase ongoing ");//\t = %2.2x ", data_read[0]&0xff);		
			fflush(stdout);

			read_sr(data_rd, spi_offset, setup_spi);
			do 
			{
				// code block to be executed
				if (debug_print == 1) 
				{ 
					printf(".");
					fflush(stdout);
					usleep(1*100*1000);
				}
				read_sr(data_rd, spi_offset, setup_spi);
			}
			while ( (data_rd[0]&0x01) != 0x00);

			if (debug_print == 1) printf("\n Chipe Erase DONE ");
			free(data_wr); 
			free(data_rd);                                                      
		}
		/*
		 * ===  FUNCTION  ======================================================================
		 *         Name:  read_data_nbyte
		 *  Description:  Read a number of byte from a starting address (W25Q32JV command 0x03)
		 *  Requiremets:  
		 *
		 *				  start_addr_v  = (3 bytes uint8_t) 24-bit read address
		 *                byte_lenght_v = number of byte to be read
		 *				  spi_offset    = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi     = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */
		void read_data_nbyte (uint8_t * start_addr_v, uint32_t byte_lenght_v, uint8_t spi_offset, uint8_t setup_spi)
		{
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
			uint32_t byten;
			printf("\n** READ DATA nBYTE **\n "); 
				
			f_cs_enable(spi_offset, setup_spi);	
			data_wr[0]= CMD_READ_DATA; //0x03;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // CMD_READ_DATA 
			wait_fifo_empty(spi_offset);				
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);			

			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 2, NO_PRINT_VALUES); // Address 23..16      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 1, NO_PRINT_VALUES); // Address 15..8       
			wait_fifo_empty(spi_offset);				
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 0, NO_PRINT_VALUES); // Address 7..0      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);     
			
			data_wr[0]= 0x00;
			for (byten = 0; byten < byte_lenght_v; byten++)
			{
				IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES);
				wait_fifo_empty(spi_offset);				
				IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES); // RD Byte byten 
				printf("Byte %d: \t = %2.2x\n ", byten, data_rd[0]&0xff);
			}; 
			f_cs_disable(spi_offset, setup_spi);
			
			free(data_wr);
			free(data_rd);
		}
		/*
		 * ===  FUNCTION  ======================================================================
		 *         Name:  read_data_nbyte_output
		 *  Description:  Read a number of byte from a starting address (W25Q32JV command 0x03)
		 *				  and return the content
		 *  Requiremets:  
		 *
		 *				  start_addr_v  = (3 bytes uint8_t) 24-bit read address
		 *                byte_lenght_v = number of byte to be read
		 *                data_read     = is the data read content
		 *				  spi_offset    = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi     = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */
		void * read_data_nbyte_output (uint8_t * start_addr_v, uint32_t byte_lenght_v, uint8_t * data_read, uint8_t spi_offset, uint8_t setup_spi)
		{
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
			uint32_t byten;
			
			//printf("\n** READ DATA nBYTE **\n "); 				
			f_cs_enable(spi_offset, setup_spi);	
			data_wr[0]= CMD_READ_DATA; //0x03;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // CMD_READ_DATA 
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 2, NO_PRINT_VALUES); // Address 23..16      
			wait_fifo_empty(spi_offset);				
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 1, NO_PRINT_VALUES); // Address 15..8       
			wait_fifo_empty(spi_offset);				
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 0, NO_PRINT_VALUES); // Address 7..0      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);     
		
			data_wr[0]= 0x00;
			for (byten = 0; byten < byte_lenght_v; byten++)
			{
				IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES);
				wait_fifo_empty(spi_offset);
				IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES); // RD Byte byten      
				*(data_read+byten) = *data_rd;			
			}; 
			
			f_cs_disable(spi_offset, setup_spi);
				
			free(data_wr);
			free(data_rd);
			
			return data_read;
		}
		/*
		 * ===  FUNCTION  ======================================================================
		 *         Name:  page_program
		 *  Description:  Write a number of bytes (from one byte to 256 bytes) from a starting 
		 *                address,then verify status register is not busy. (W25Q32JV command 0x02)		 				  
		 *  Requiremets:  
		 *
		 *				  start_addr_v  = (3 bytes uint8_t) 24-bit read address
		 *                byte_lenght_v = number of byte to be read
		 *                data_buffer_v = is the buffer of data to be written
		 *				  spi_offset    = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi     = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */
		void page_program (uint8_t * start_addr_v, uint8_t * data_buffer_v, uint32_t byte_lenght_v, uint8_t spi_offset, uint8_t setup_spi, unsigned int debug_print)
		{
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
			uint32_t byten;

			wr_ena_latch(spi_offset, setup_spi, debug_print);

			if (debug_print == 1) printf("\n** PAGE PROGRAM nBYTE **\n"); 

			f_cs_enable(spi_offset, setup_spi);	
			data_wr[0]= CMD_PAGE_PROGRAM; //0x02;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // CMD_PAGE_PROGRAM 
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			

			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 2, NO_PRINT_VALUES); // Address 23..16      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 1, NO_PRINT_VALUES); // Address 15..8       
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);     
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 0, NO_PRINT_VALUES); // Address 7..0      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);     
			

			for (byten = 0; byten < byte_lenght_v; byten++)
			{
				data_wr[0]=0x00;
				IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_buffer_v, 1, 1, byten, NO_PRINT_VALUES); // Byte byten     
				wait_fifo_empty(spi_offset);
				IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);    
				
			};	
			f_cs_disable(spi_offset, setup_spi);

			if (debug_print == 1) printf("\n Program ongoing ");	
			fflush(stdout);

			read_sr(data_rd, spi_offset, setup_spi);
			do 
			{
				// code block to be executed
				if (debug_print == 1) 
				{ 
					printf(".");
					fflush(stdout);
				}
				read_sr(data_rd, spi_offset, setup_spi);
			}
			while ( (data_rd[0]&0x01) != 0x00);

			if (debug_print == 1) printf("\n Program DONE ");	
			free(data_wr); 
			free(data_rd);                                                      
		}
		/*
		 * ===  FUNCTION  ======================================================================
		 *         Name:  read_page_output
		 *  Description:  Read a number of byte from a starting address (W25Q32JV command 0x03)
		 *				  and return the content
		 *  Requiremets:  
		 *
		 *				  start_addr_v  = (3 bytes uint8_t) 24-bit read address
		 *                block_size_v 	= number of byte to be read
		 *                data_read     = is the data read content
		 *				  spi_offset    = (0x20 base_spi, 0x48 cfg_rom spi)
		 *				  setup_spi     = (spi_ena,cpha,cpol,cs)
		 * =====================================================================================
		 */
		void * read_page_output (uint8_t * start_addr_v, uint32_t block_size_v, uint8_t * data_read, uint8_t spi_offset, uint8_t setup_spi, unsigned int debug_print)
		{
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (3,sizeof(uint8_t));
			uint32_t byten;
			uint32_t n_pages, page=0;
			
			n_pages = block_size_v/256;
			
			//if (debug_print == 1) printf("\n** READ DATA nBYTE **\n "); 				
			f_cs_enable(spi_offset, setup_spi);	
			data_wr[0]= CMD_READ_DATA; //0x03;
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES); // CMD_READ_DATA 
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 2, NO_PRINT_VALUES); // Address 23..16      
			wait_fifo_empty(spi_offset);				
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);      
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 1, NO_PRINT_VALUES); // Address 15..8       
			wait_fifo_empty(spi_offset);				
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);
			
			IOWr(pcie_bar_io[0] + spi_offset + 0x02, start_addr_v, 1, 1, 0, NO_PRINT_VALUES); // Address 7..0      
			wait_fifo_empty(spi_offset);
			IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES);     
		
			data_wr[0]= 0x00;
			for (byten = 0; byten < block_size_v; byten++)
			{
				if (!(byten%256)) 
				{
					page++; 
					printf("FPGA memory read %3.3d.%1.1d%%\r", (1000*page/n_pages)/10, (1000*page/n_pages)%10);
					fflush(stdout);
				};
				IOWr(pcie_bar_io[0] + spi_offset + 0x02, data_wr, 1, 1, 0, NO_PRINT_VALUES);
				wait_fifo_empty(spi_offset);
				IORd(pcie_bar_io[0] + spi_offset + 0x02, data_rd, 1, 1, NO_PRINT_VALUES); // RD Byte byten      
				*(data_read+byten) = *data_rd;	
			}; 
			
			// User notification
			printf("FPGA memory read %3.3d.%1.1d%%\r", (1000*page/n_pages)/10, (1000*page/n_pages)%10);
    
			f_cs_disable(spi_offset, setup_spi);
				
			free(data_wr);
			free(data_rd);
			
			return data_read;
		}		
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  writeFpgaCode
		 *  Description:  Write the FPGA microcode to the FLASH memory
		 *                Return the written microcode length in bytes or -1 in case of access error
		 * =====================================================================================
		 */
		uint32_t writeFpgaCode(uint8_t *fpga_wdata, uint32_t size, uint8_t padding, unsigned int debug_print)
		{
			uint32_t n_pages, remainders, page, page_4M;
			uint8_t  rem_buffer[256];
			uint32_t ret_code = size;	
			uint32_t byte_lenght=256;	
			uint8_t start_addr[3];				
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint8_t *data_rd = (uint8_t *)calloc (4,sizeof(uint8_t));
			uint32_t max_n_page;
			uint8_t  pad_buffer[256];
			
			max_n_page = (4*1024*1024)/256;
			n_pages = size/256;
			remainders = size%256;
							
			if (remainders) 
			{
				memcpy(rem_buffer, fpga_wdata+size-remainders, remainders);
				memset(rem_buffer+remainders, padding, 256-remainders);
			} 
							
			// Disable any protection						
			if (debug_print==1) printf("\n** WRITE_STATUS_REGISTER DISABLE PROTECTION **\n"); 
			data_wr[0]=0x00; //sr write unprotect ALL
			write_sr(data_wr,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,debug_print);
						
			if (debug_print==1) printf("\n** READ_STATUS_REGISTER **\n"); 
			read_sr(data_rd,FPGA_ROM_BASE,CS_FPGA_ROM_ENA);
			if (debug_print==1) printf("\n   Status Register \t = %2.2x \n", data_rd[0]&0xff);
						
			if (data_rd[0] |= 0) {
				printf("\n   FAILURE: cannot clear protection bit (check WP pin = 3.3V)\n");
				free(data_rd);
				free(data_wr); 
				return (uint32_t)-1;
			}
					
			// Erase the memory chip					
			chip_erase(FPGA_ROM_BASE,CS_FPGA_ROM_ENA,debug_print);

			// Program the memory pages						
			printf("\n");
			for(page=0; page<n_pages; page++) {				
							
				// set page address							
				start_addr[0]= 0x00;
				start_addr[1]= (page%256);
				start_addr[2]= page/256;
				byte_lenght  = 256;		
				page_program(start_addr,fpga_wdata+page*256,256,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,NO_PRINT_VALUES);
							
				// User notification
				printf("FPGA memory write %3.3d.%1.1d%%\r", (1000*page/n_pages)/10, (1000*page/n_pages)%10);
							
			}	
						
			if (remainders) 
			{				
				start_addr[0]= 0x00;
				start_addr[1]= page%256;
				start_addr[2]= page/256;
				byte_lenght  = 256;		
				page_program(start_addr,rem_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,NO_PRINT_VALUES);
			}
			printf("FPGA memory write %3.3d.%1.1d%%\r", (1000*page/n_pages)/10, (1000*page/n_pages)%10);
			fflush(stdout);
			
			page++;
			if (padding == 0x00) 
			{	
				memset(pad_buffer,padding,256);
				printf("\n Program padding \n");
				for(page_4M=page; page_4M<max_n_page; page_4M++) 
				{				
						
					start_addr[0]= 0x00;
					start_addr[1]= page_4M%256;
					start_addr[2]= page_4M/256;
					byte_lenght  = 256;		
					page_program(start_addr,pad_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,NO_PRINT_VALUES);
					// User notification
					printf("FPGA padding write %3.3d.%1.1d%%\r", (1000*page_4M/max_n_page)/10, (1000*page_4M/max_n_page)%10);
				}
				printf("FPGA padding write %3.3d.%1.1d%%\r", (1000*page_4M/max_n_page)/10, (1000*page_4M/max_n_page)%10);
				
			}
			
			free(data_rd);
			free(data_wr); 
			return ret_code;
		}
	/* 
	 * ===  FUNCTION  ======================================================================
	 *         Name:  readFpgaCode
	 *  Description:  Read the FPGA microcode from the FLASH memory
	 *                Return the read microcode length in bytes or <0 in case of access error
	 * =====================================================================================
	 */
	uint32_t readFpgaCode(uint8_t *fpga_rdata, uint32_t size, unsigned int debug_print)
	{
		uint32_t ret_code=size;
		uint32_t byte_lenght=0;
		uint8_t start_addr[3];
    
		//
		// Send the read command to the SPI bus (see SPI Flash Winbond W25Q32BV data sheet)
		start_addr[0]= 0x00;
		start_addr[1]= 0x00;
		start_addr[2]= 0x00;
		byte_lenght=size;
		read_page_output(start_addr,byte_lenght,fpga_rdata,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,debug_print);
		    
		return ret_code;
	}
		/*
		 * ===  FUNCTION  ======================================================================
		 *         Name:  TAP
		 *  Description:  Test Access Port State Machine
		 *				  
		 *  Requiremets:  
		 *
		 *				  mem_addr_sbc  = memory pointer
		 *                data 			= write data to memory
		 *                BitCount      = number of data bit
		 *				  StartTAP      = TAP Initial state
		 *				  EndTAP        = TAP End state
		 * =====================================================================================
		 */
		void TAP (uint8_t *mem_addr_sbc, uint8_t *data, uint8_t BitCount, uint8_t StartTAP, uint8_t EndTAP, unsigned int debug_print)
		{	
			uint8_t *data_wr = (uint8_t *)calloc (4,sizeof(uint8_t));	
			uint8_t *data_rd = (uint8_t *)calloc (4,sizeof(uint8_t));	
			
			
			data_wr[3]=0x00;
			data_wr[2]=0x00; //START TAP=0
			data_wr[1]=(BitCount & 0xFF);
			data_wr[0]=(((StartTAP & 0x0F) << 4) | (EndTAP & 0x0F));					
						 
			// ** Bitcount & StateStart & StateEnd **
			IOWr(pcie_bar_sbc[1] + 0x80, data_wr, 1, 1, 0, debug_print);
			IOWr(pcie_bar_sbc[1] + 0x81, data_wr, 1, 1, 1, debug_print);
			IOWr(pcie_bar_sbc[1] + 0x82, data_wr, 1, 1, 2, debug_print);
			IOWr(pcie_bar_sbc[1] + 0x83, data_wr, 1, 1, 3, debug_print);
				
			if (debug_print == 1) 
			{
				printf("\nINIT JTAG Memory \n"); 
			}
			switch(BitCount)
			{
			case 0: // 0 byte
				data_wr[0]=0x00;
				MWr32(mem_addr_sbc, data_wr, 1, 1, debug_print);
				break;

			case 8: // 1 byte
				data_wr[0]=data[0];
				MWr32(mem_addr_sbc, data_wr, 1, 1, debug_print);
				break;

			case 16: // 2 byte
				data_wr[0]=data[1];	
				MWr32(mem_addr_sbc, data_wr, 1, 1, debug_print);			
				data_wr[0]=data[0];
				MWr32(mem_addr_sbc + 0x04, data_wr, 1, 1, debug_print);
				break;

			case 32: // 4 byte 		
				data_wr[0]=data[3];	
				MWr32(mem_addr_sbc, data_wr, 1, 1, debug_print);			
				data_wr[0]=data[2];
				MWr32(mem_addr_sbc + 0x04, data_wr, 1, 1, debug_print);
				data_wr[0]=data[1];
				MWr32(mem_addr_sbc + 0x08, data_wr, 1, 1, debug_print);
				data_wr[0]=data[0];
				MWr32(mem_addr_sbc + 0x0C, data_wr, 1, 1, debug_print);
				break;

			case 64: // 8 byte 	
				data_wr[0]=data[7];	
				MWr32(mem_addr_sbc, data_wr, 1, 1, debug_print);			
				data_wr[0]=data[6];
				MWr32(mem_addr_sbc + 0x04, data_wr, 1, 1, debug_print);
				data_wr[0]=data[5];
				MWr32(mem_addr_sbc + 0x08, data_wr, 1, 1, debug_print);
				data_wr[0]=data[4];
				MWr32(mem_addr_sbc + 0x0C, data_wr, 1, 1, debug_print);
				data_wr[0]=data[3];	
				MWr32(mem_addr_sbc + 0x10, data_wr, 1, 1, debug_print);			
				data_wr[0]=data[2];
				MWr32(mem_addr_sbc + 0x14, data_wr, 1, 1, debug_print);
				data_wr[0]=data[1];
				MWr32(mem_addr_sbc + 0x18, data_wr, 1, 1, debug_print);
				data_wr[0]=data[0];
				MWr32(mem_addr_sbc + 0x1C, data_wr, 1, 1, debug_print);
				break;

			case 128: // 16 byte	
				data_wr[0]=data[15];	
				MWr32(mem_addr_sbc, data_wr, 1, 1, debug_print);			
				data_wr[0]=data[14];
				MWr32(mem_addr_sbc + 0x04, data_wr, 1, 1, debug_print);
				data_wr[0]=data[13];
				MWr32(mem_addr_sbc + 0x08, data_wr, 1, 1, debug_print);
				data_wr[0]=data[12];
				MWr32(mem_addr_sbc + 0x0C, data_wr, 1, 1, debug_print);

				data_wr[0]=data[11];	
				MWr32(mem_addr_sbc + 0x10, data_wr, 1, 1, debug_print);			
				data_wr[0]=data[10];
				MWr32(mem_addr_sbc + 0x14, data_wr, 1, 1, debug_print);
				data_wr[0]=data[9];
				MWr32(mem_addr_sbc + 0x18, data_wr, 1, 1, debug_print);
				data_wr[0]=data[8];
				MWr32(mem_addr_sbc + 0x1C, data_wr, 1, 1, debug_print);
				
				data_wr[0]=data[7];	
				MWr32(mem_addr_sbc + 0x20, data_wr, 1, 1, debug_print);			
				data_wr[0]=data[6];
				MWr32(mem_addr_sbc + 0x24, data_wr, 1, 1, debug_print);
				data_wr[0]=data[5];
				MWr32(mem_addr_sbc + 0x28, data_wr, 1, 1, debug_print);
				data_wr[0]=data[4];
				MWr32(mem_addr_sbc + 0x2C, data_wr, 1, 1, debug_print);

				data_wr[0]=data[3];	
				MWr32(mem_addr_sbc + 0x30, data_wr, 1, 1, debug_print);			
				data_wr[0]=data[2];
				MWr32(mem_addr_sbc + 0x34, data_wr, 1, 1, debug_print);
				data_wr[0]=data[1];
				MWr32(mem_addr_sbc + 0x38, data_wr, 1, 1, debug_print);
				data_wr[0]=data[0];
				MWr32(mem_addr_sbc + 0x3C, data_wr, 1, 1, debug_print);
				break;

			default:
				data_wr[0]=0x00;
				MWr32(mem_addr_sbc, data_wr, 1, 1, debug_print);
				break;
			}

			// ** start_access_point **
			//printf("\nStart Access Point command \n");			
			data_wr[3]=0x00;
			data_wr[2]=0x01; //START TAP=1
			data_wr[1]=(BitCount & 0xFF);
			data_wr[0]=(((StartTAP & 0x0F) << 4) | (EndTAP & 0x0F));					
						
			// ** Bitcount & StateStart & StateEnd **
			IOWr(pcie_bar_sbc[1] + 0x80, data_wr, 1, 1, 0, debug_print);
			IOWr(pcie_bar_sbc[1] + 0x81, data_wr, 1, 1, 1, debug_print);
			IOWr(pcie_bar_sbc[1] + 0x82, data_wr, 1, 1, 2, debug_print);
			IOWr(pcie_bar_sbc[1] + 0x83, data_wr, 1, 1, 3, debug_print);		

			// ** Wait Legacy interrupt **
			if (debug_print == 1) 
			{
				printf("\nWait Core Free \n"); 
			}
			IORd(pcie_bar_sbc[1] + 0x80, data_rd, 1, 1, debug_print);  
			do 
			{
				// code block to be executed
				if (debug_print == 1) 
				{ 
					printf(".");
					fflush(stdout);
					usleep(1*100*1000);
				}
				IORd(pcie_bar_sbc[1] + 0x80, data_rd, 1, 1, debug_print);   
			}
			while ( (data_rd[0]&0x01) == 0x00);

			// ** Stop Access Point **
			//printf("\nStop Access Point \n"); 
			data_wr[3]=0x00;
			data_wr[2]=0x00; 
			data_wr[1]=0x00;
			data_wr[0]=0x00;		
			IOWr(pcie_bar_sbc[1] + 0x80, data_wr, 4, 4, 0, debug_print);
			IORd(pcie_bar_sbc[1] + 0x80, data_rd, 1, 1, debug_print); 

			switch(BitCount)
			{
			case 0: // 0 byte
				data_wr[0]=0x00;
				MWr32(mem_addr_sbc, data_wr, 1, 1, NO_PRINT_VALUES);
				break;

			case 8: // 1 byte
				data_wr[0]=0x00;
				MWr32(mem_addr_sbc, data_wr, 1, 1, NO_PRINT_VALUES);
				break;

			case 16: // 2 byte
				data_wr[0]=0x00;	
				MWr32(mem_addr_sbc, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x04, data_wr, 1, 1, NO_PRINT_VALUES);
				break;

			case 32: // 4 byte 		
				data_wr[0]=0x00;	
				MWr32(mem_addr_sbc, data_wr, 1, 1, NO_PRINT_VALUES);	
				MWr32(mem_addr_sbc + 0x04, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x08, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x0C, data_wr, 1, 1, NO_PRINT_VALUES);
				break;
			
			case 64: // 8 byte 	
				data_wr[0]=0x00;
				MWr32(mem_addr_sbc, data_wr, 1, 1, NO_PRINT_VALUES);	
				MWr32(mem_addr_sbc + 0x04, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x08, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x0C, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x10, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x14, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x18, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x1C, data_wr, 1, 1, NO_PRINT_VALUES);
				break;

			case 128: // 16 byte	
				data_wr[0]=0x00;	
				MWr32(mem_addr_sbc, data_wr, 1, 1, NO_PRINT_VALUES);	
				MWr32(mem_addr_sbc + 0x04, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x08, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x0C, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x10, data_wr, 1, 1, NO_PRINT_VALUES);	
				MWr32(mem_addr_sbc + 0x14, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x18, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x1C, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x20, data_wr, 1, 1, NO_PRINT_VALUES);	
				MWr32(mem_addr_sbc + 0x24, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x28, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x2C, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x30, data_wr, 1, 1, NO_PRINT_VALUES);	
				MWr32(mem_addr_sbc + 0x34, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x38, data_wr, 1, 1, NO_PRINT_VALUES);
				MWr32(mem_addr_sbc + 0x3C, data_wr, 1, 1, NO_PRINT_VALUES);
				break;

			default:
				data_wr[0]=0x00;
				MWr32(mem_addr_sbc, data_wr, 1, 1, NO_PRINT_VALUES);
				break;
			}

			free(data_wr);
			free(data_rd);

		}
				
		/*
		 * ===  FUNCTION  ======================================================================
		 *         Name:  SIR
		 *  Description:  SIR instruction
		 *				  
		 *  Requiremets:  
		 *
		 *				  mem_addr_sbc  = memory pointer
		 *                data 			= write data to memory
		 *                BitCount      = number of data bit
		 * =====================================================================================
		 */
		void SIR (uint8_t *mem_addr_sbc, uint8_t *data, uint8_t BitCount, unsigned int debug_print)
		{	

			uint8_t *data_rd = (uint8_t *)calloc (4,sizeof(uint8_t));
			
			if (debug_print==1) printf("\n** SIR %d %2.2x. **\n",BitCount,(data[0]&0xff)); 
			if (debug_print==1) printf("\nWrite Operation Memory Instruction %2.2x. \n",(data[0]&0xff)); 

			TAP (mem_addr_sbc, data, BitCount, SHIFT_IR, PAUSE_IR, debug_print);	
		
			if (debug_print==1) 
				{
					IORd(pcie_bar_sbc[1] + 0x82, data_rd, 2, 2, 0); 
					printf("\nStateIn & Current_State %2.2x\n",(data_rd[0]&0xff)); 
					printf("\nStateJTAGMaster & TMSState %2.2x\n",(data_rd[1]&0xff)); 
					
					printf("\nFrom IR pause to DR pause \n"); 
				}
				// From IR pause to DR pause 
				data[0]=0x00;
				TAP (mem_addr_sbc, data, 0, EXIT1_DR, PAUSE_DR, debug_print);
				
			if (debug_print==1) 
				{
					IORd(pcie_bar_sbc[1] + 0x82, data_rd, 2, 2, 0); 
					printf("\nStateIn & Current_State %2.2x\n",(data_rd[0]&0xff)); 
					printf("\nStateJTAGMaster & TMSState %2.2x\n",(data_rd[1]&0xff)); 
				}
				
			free(data_rd);

		}
		/*
		 * ===  FUNCTION  ======================================================================
		 *         Name:  SDR
		 *  Description:  SDR instruction
		 *				  
		 *  Requiremets:  
		 *
		 *				  mem_addr_sbc  = memory pointer
		 *                data 			= write data to memory
		 *                BitCount      = number of data bit
		 * =====================================================================================
		 */
		void SDR (uint8_t *mem_addr_sbc, uint8_t *data, uint8_t BitCount, unsigned int debug_print)
		{	
			
			uint8_t *data_rd = (uint8_t *)calloc (4,sizeof(uint8_t));
			
			if (debug_print==1) printf("\n** SDR %d TDI=%2.2x. **\n",BitCount,(data[0]&0xff)); 
			if (debug_print==1) printf("\nWrite Operation Memory Instruction %2.2x. \n",(data[0]&0xff)); 
			
			TAP (mem_addr_sbc, data, BitCount, SHIFT_DR, PAUSE_DR, debug_print);	
		
			if (debug_print==1) 
				{
					IORd(pcie_bar_sbc[1] + 0x82, data_rd, 2, 2, 0); 
					printf("\nStateIn & Current_State %2.2x\n",(data_rd[0]&0xff)); 
					printf("\nStateJTAGMaster & TMSState %2.2x\n",(data_rd[1]&0xff)); 
					
					printf("\nFrom DR pause to IR pause \n"); 
				}		

				// From DR pause to IR pause 
				data[0]=0x00;
				TAP (mem_addr_sbc, data, 0, EXIT1_IR, PAUSE_IR, debug_print);
				

			if (debug_print==1) 
				{
					IORd(pcie_bar_sbc[1] + 0x82, data_rd, 2, 2, 0); 
					printf("\nStateIn & Current_State %2.2x\n",(data_rd[0]&0xff)); 	
					printf("\nStateJTAGMaster & TMSState %2.2x\n",(data_rd[1]&0xff)); 				
				}
				
			free(data_rd);

		}
		
		// ******************************************************************************************
		int openSSLEnc(uint8_t *key, uint8_t *iv, aes_size_t aes_size, aes_mode_t aes_mode,
		  uint8_t *cipher_data, uint8_t *data, uint32_t data_length)
		  // ******************************************************************************************
		{
		  const EVP_CIPHER *cipher = NULL;
		  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
		  //EVP_CIPHER_CTX ctx;
		  int32_t cipher_length, final_length;
		  uint8_t *ciphertext;

		  if (aes_mode == AES_CBC)
		  {
			if (aes_size == AES_128)
			  cipher = EVP_aes_128_cbc();
			else
			  cipher = EVP_aes_256_cbc();
		  }
		  else if (aes_mode == AES_ECB)
		  {
			if (aes_size == AES_128)
			  cipher = EVP_aes_128_ecb();
			else
			  cipher = EVP_aes_256_ecb();
		  }

		  EVP_CIPHER_CTX_init(ctx);
		  EVP_EncryptInit_ex(ctx, cipher, NULL, (uint8_t *)key, (uint8_t *)iv);

		  cipher_length = data_length + EVP_MAX_BLOCK_LENGTH;
		  ciphertext = (uint8_t *)malloc(cipher_length);

		  EVP_EncryptUpdate(ctx, ciphertext, &cipher_length, (uint8_t *)data, data_length);
		  EVP_EncryptFinal_ex(ctx, ciphertext + cipher_length, &final_length);

		  // Copy data
		  memcpy(cipher_data, ciphertext, data_length);

		  // release internal memory
		  free(ciphertext);

		  EVP_CIPHER_CTX_cleanup(ctx);
		  EVP_CIPHER_CTX_free(ctx);
		  return 0;
		}
/*================================================
==================================================
==================== MAIN ========================
==================================================
================================================*/

int main(int argc, char **argv)
{	
	int ret_code;
    int n;
    char * rom_file_name = 0;
    char * rom_out_file_name = 0;
	uint8_t padding = 0xFF;
	
	for (n=1; n<argc; n++) {
        if (strncmp(argv[n], "-s", 2)==0) {
            // Set the memory size
            if (n<argc-1) {
                mem_size = atoi(argv[n+1]);
            }
        }
        else if(strncmp(argv[n], "-help", 5)==0) {
            // Set the padding value to 00
            f_help = 1;
        }
		
		/*
        else if (strncmp(argv[n], "-b", 2)==0) {
            // Set the binary output flag
            f_binary = 1;
        }
        else if (strncmp(argv[n], "-t", 2)==0) {
            // Reset the binary output flag
            f_binary = 0;
        }*/
        else if(strncmp(argv[n], "-p", 2)==0) {
            // Set the padding value to 00
            padding = 0x00;
        }
        else if(strncmp(argv[n], "-of", 3)==0) {
            // Set the fpga rom output filename
			f_save_fout = 1;
            if (n<argc-1) {
                rom_out_file_name = strdup(argv[n+1]);
            }
        }
        else if(strncmp(argv[n], "-r", 2)==0) {
            // Set the read flag
            f_read = 1;
        }
        else if(strncmp(argv[n], "-w", 2)==0) {
            // Set the write flag
            f_write = 1;
            // Set the fpga rom filename
            if (n<argc-1) {
                rom_file_name = strdup(argv[n+1]);
            }
        }
    }

#ifndef _WIN32
	/*
	* Set privilege level 3 (all i/o)
	*/
	ret_code = iopl(3);
	if (ret_code != 0) 
	{
		perror("iopl");
		exit(-1);
	} 
	//==========================================
	//			Functions @ PCIe BARs 
	pcie_bar_io = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_mem = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_secs = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_muart = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_qli = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_sbc = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_ats = (uint32_t *)calloc (6,sizeof(uint32_t));
	
	pcie_bar_size_io = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_mem = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_secs = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_muart = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_qli = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_sbc = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_ats = (uint32_t *)calloc (6,sizeof(uint32_t));
  
	find_pci_reg(IO_DEVID,		pcie_bar_io,	pcie_bar_size_io,	NO_PRINT_VALUES);	
	find_pci_reg(NVRAM_DEVID,	pcie_bar_mem,	pcie_bar_size_mem,	NO_PRINT_VALUES);
	find_pci_reg(SECs_DEVID,	pcie_bar_secs,	pcie_bar_size_secs,	NO_PRINT_VALUES);	
	find_pci_reg(MUART_DEVID,	pcie_bar_muart,	pcie_bar_size_muart,NO_PRINT_VALUES);
	find_pci_reg(QLI_DEVID,		pcie_bar_qli,	pcie_bar_size_qli,	NO_PRINT_VALUES);		
	find_pci_reg(SBC_DEVID,		pcie_bar_sbc,	pcie_bar_size_sbc,	NO_PRINT_VALUES);		
	find_pci_reg(ATS_DEVID,		pcie_bar_ats,	pcie_bar_size_ats,	NO_PRINT_VALUES);	
	//find_pci_reg(SBC_DEVID,		pcie_bar_sbc,	pcie_bar_size_sbc,	PRINT_VALUES);
	//==========================================

	uint32_t nvramSize = 32768;
	uint32_t nvramMaxSize = pcie_bar_size_mem[0];//262144;

	int fd = open("/dev/mem", O_RDWR);
	//			MEM Map for PCIe mem space qli and nvram 
	if (fd != -1) {
		uint8_t * mem_addr;	
		uint8_t * mem_addr_led;		
		uint8_t * mem_addr_secs;		
		uint8_t * mem_addr_sbc;			
		uint8_t * mem_addr_ats;														
		mem_addr = (uint8_t*)mmap(0, nvramMaxSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_mem[0]);
		mem_addr_led = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_qli[0]);
		mem_addr_sbc = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_sbc[0]);
		mem_addr_secs = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_secs[0]);
		mem_addr_ats = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_ats[0]);
	
		
		uint8_t *data_read  = (uint8_t *)calloc (256,sizeof(uint8_t));
		uint8_t *data_write = (uint8_t *)calloc (256,sizeof(uint8_t));
		uint8_t *expected_data = (uint8_t *)calloc (256,sizeof(uint8_t));
		uint8_t *fpga_wdata = (uint8_t *)malloc(mem_size);
		uint8_t *fpga_rdata = (uint8_t *)malloc(mem_size);
		uint8_t trace_id[8];
		uint8_t expanded_key[16];
		uint8_t *key_p  	= (uint8_t *)calloc (sizeSecsBuffer,sizeof(uint8_t));
		
		if (f_help)
			{	
				printf(" \n");	
				printf(" ========== Help Menu =========\n");	
				printf(" ===   Shell command list   ===\n");	
				printf(" \n");	
				printf("  [-s  <mem size>  ]   Set the memory size (default value is 4*1024*1024)\n");
				printf("  [-p              ]   Enable padding, by set ROM unused byte to 0x00 (default value is no padding)  \n");
				printf("  [-w  <file name> ]   Program fpga ROM with input file_name \n");
				printf("  [-r              ]   Read fpga ROM content and store to output file_name (see -of command)\n");
				printf("  [-of <file name> ]   Define the name of output filename to store the ROM content readed\n");
				printf(" \n");
				exit(0);
			}
		//
        // FPGA read code command
        //
        if (f_read) 
			{
				int32_t rom_file_out;
				uint32_t r_size;
				
				readFpgaCode(fpga_rdata, mem_size, NO_PRINT_VALUES);
				printf("\n");
				// test		
				//print_file(fpga_rdata);
				if (!f_save_fout) 
					{
						strcpy(rom_out_file_name,"fpga_output.bit");
					}
					rom_file_out = open(rom_out_file_name, O_CREAT | O_WRONLY | O_TRUNC, 0777);		
					if (rom_file_out >= 0) 
							{
								r_size=write(rom_file_out, fpga_rdata, mem_size);	
								close(rom_file_out); 
								if (r_size != mem_size ) 
								{
									printf("FAILURE: on file writing\n");									
									exit(-1);									
								}									
								exit(0);
							}
						else 
							{			
								printf("FAILURE: error opening %s. Abort\n", rom_out_file_name);								
								exit(-1);
							}	
				
			}
		//
        // FPGA write code from file command
        //
        if (f_write) 
			{
				int32_t rom_file;
				uint32_t w_size, size;
				
				if (rom_file_name) {
					rom_file = open(rom_file_name, O_RDONLY);						
							
					if (rom_file >= 0) 
						{					
							size = read(rom_file, fpga_wdata, mem_size);
							w_size = writeFpgaCode(fpga_wdata, size, padding, NO_PRINT_VALUES);
							close(rom_file); 
							// test		
							if (w_size != size ) 
							{
								printf("FAILURE: memory chip protection error\n");									
								exit(-1);									
							}		
							printf("\nPROGRAM DONE\n");									
							exit(0);	
						}								
					else 
						{			
							printf("FAILURE: error opening %s. Abort\n", rom_file_name);								
							exit(-1);
						}
				}
				else {
					printf("FAILURE: invalid ROM file name. Abort\n");								
					exit(-1);
				}
			}

	/* =====================================
	========================================
	=========    Test execution    ========= 
	======================================== 
	========================================
	*/	
	
	printf("\n ====================================");  
	printf("\n === Quixant Configuration utlity ==="); 
	printf("\n ===      Ver %d.%d  %s     ===",ver_major,ver_minor,date); 
	printf("\n ====================================\n");  
	
	IORd(pcie_bar_io[0] + 0x2A, data_read, 2, 1, NO_PRINT_VALUES); 
	printf("\n FPGA VERSION Maj.Min\t= %2.2x.%2.2x\n\n", data_read[0]&0xff,data_read[1]&0xff);
	 
	
	IORd(pcie_bar_io[0] + 0x44, data_read, 2, 1, NO_PRINT_VALUES); 
	printf("\n ===================================");
	printf("\n ===  IO MODULE (HW parameters)  ===");
	printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
		
	#if _OLDCFGROM	
		{	
	IORd(pcie_bar_io[0] + 0x13, data_read, 1,1, NO_PRINT_VALUES);
	printf("\n HW CONF ID \t= %2.2x", data_read[0]&0xff);
		}; 
	#endif 
	
	
	IORd(pcie_bar_io[0] + 0x24, data_read, 2, 1, NO_PRINT_VALUES);
	printf("\n PLATFORM ID \t= %2.2x\n PLATFORM SP ID\t= %2.2x", data_read[0]&0xff,data_read[1]&0xff);	
	IORd(pcie_bar_io[0] + 0x27, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n CAPABILITIES \t= %2.2x", data_read[0]&0xff);
	
	printf("\n\n === CONFIGURATION ===");
	IORd(pcie_bar_io[0] + 0x26, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n DOUT_TYPE \t= %s", IO_TYPE((data_read[0]>>3)&0x1F));
	printf("\n FEEDBACK WIDTH\t= %d", (((data_read[0]&0x07)+1)*8));	
	printf("\n ===================================");
	
	#if _OLDCFGROM	
		{	
	IORd(pcie_bar_io[0] + 0x14, data_read, 4, 1, NO_PRINT_VALUES);
	printf("\n DIN NUM \t= %d", (data_read[0]&0xff));
	printf("\n DOUT NUM \t= %d", (data_read[1]&0xff));
	printf("\n FB MASK \t= 0x%2.2x", (data_read[2]&0xff));
	printf("\n DOUT CFG \t= 0x%2.2x", (data_read[3]&0xff));
	printf("\n ===================================");
		}; 
	#endif 
	
	printf("\n\n");
	IORd(pcie_bar_mem[1] + 0x14, data_read, 2, 1, NO_PRINT_VALUES); 
	printf("\n ===================================");
	printf("\n === SRAM MODULE (HW parameters) ===");
	printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
	#if _OLDCFGROM	
		{	
	IORd(pcie_bar_mem[1] + 0x2F, data_read, 1,1, NO_PRINT_VALUES);
	printf("\n HW CONF ID \t= %2.2x", data_read[0]&0xff);
		}; 
	#endif 
	IORd(pcie_bar_mem[1] + 0x30, data_read, 2, 1, NO_PRINT_VALUES);
	printf("\n PLATFORM ID \t= %2.2x\n PLATFORM SP ID\t= %2.2x", data_read[0]&0xff,data_read[1]&0xff);	
	IORd(pcie_bar_mem[1] + 0x32, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n CAPABILITIES \t= %2.2x", data_read[0]&0xff);	
	printf("\n\n === CONFIGURATION ===");
	IORd(pcie_bar_mem[1] + 0x33, data_read, 1,1, NO_PRINT_VALUES);
	printf("\n SRAM Type \t= %s", SRAM_TYPE(data_read[0]&0x03));
	IORd(pcie_bar_mem[1] + 0x22, data_read, 2,1, NO_PRINT_VALUES);	
	#if 1//_DEBUG	
		{	
 	    printf("\n SRAM CFG PARAMS ID \t  = %2.2x\n", data_read[1]&0xff);
 	  }; 
	#endif 
	printf("\n Number of Chip = %d", (pot2(data_read[1]&0x03)));
	if ( ((data_read[1]>>2)&0x0F)==0 )
		{
			printf("\n Chip Size \t= 0 kbit");	
		} 
	else
		{
			printf("\n Chip Size \t= %.0lf kbit", pow(2, (((data_read[1]>>2)&0x0F)+1) ));
		} 

	printf("\n READ Latency \t= %d", (data_read[0]&0x0F));
	printf("\n WRITE Latency \t= %d", ((data_read[0]>>4)&0x0F)); 	
	printf("\n ===================================");

	printf("\n\n");
	MRd32(mem_addr_led, data_read, 4, 1, NO_PRINT_VALUES); 
	printf("\n ===================================");
	printf("\n === QLI MODULE  (HW parameters) ===");
	printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
	#if _OLDCFGROM	
		{	
	MRd32(mem_addr_led + 0x38, data_read, 1,1, NO_PRINT_VALUES);
	printf("\n HW CONF ID \t= %2.2x", data_read[0]&0xff);
		}; 
	#endif 
	MRd32(mem_addr_led + 0x17, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n PLATFORM ID \t= %2.2x", data_read[0]&0xff);
	MRd32(mem_addr_led + 0x39, data_read, 1, 1, NO_PRINT_VALUES);	
	printf("\n PLATFORM SP ID\t= %2.2x",data_read[0]&0xff);
	//MRd32(mem_addr_led + 0x32, data_read, 1, 1, NO_PRINT_VALUES);
	//printf("\n CAPABILITIES \t= %2.2x", data_read[0]&0xff);
	printf("\n\n === CONFIGURATION ===");
	printf("\n LED Presence \t= %2.2x%2.2x", data_read[3]&0xff,data_read[2]&0xff);
	printf("\n QLI Ch0\t= %s", QLI_CAPAB(data_read[2]&0x03));
	printf("\n QLI Ch1\t= %s", QLI_CAPAB((data_read[2]>>2)&0x03));
	printf("\n QLI Ch2\t= %s", QLI_CAPAB((data_read[2]>>4)&0x03));
	printf("\n QLI Ch3\t= %s", QLI_CAPAB((data_read[2]>>6)&0x03));
	printf("\n QLI Ch4\t= %s", QLI_CAPAB(data_read[3]&0x03));
	printf("\n QLI Ch5\t= %s", QLI_CAPAB((data_read[3]>>2)&0x03));
	printf("\n QLI Ch6\t= %s", QLI_CAPAB((data_read[3]>>4)&0x03));
	printf("\n QLI Ch7\t= %s", QLI_CAPAB((data_read[3]>>6)&0x03));
	printf("\n ===================================\n");
	
	printf("\n ===================================");
	printf("\n === COM MODULE  (HW parameters) ===");
	printf("\n ===================================");
	//printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
	IORd(pcie_bar_io[0] + 0x68, data_read, 1,1, NO_PRINT_VALUES);
	printf("\n CCTALK1 status \t= %s", CCTALK_ENA(data_read[0]&0x01));
	printf("\n CCTALK1 pullup \t= %s", PULL_UP((data_read[0]>>1)&0x01));
	printf("\n CCTALK2 status \t= %s", CCTALK_ENA((data_read[0]>>2)&0x01));
	printf("\n CCTALK2 pullup \t= %s\n", PULL_UP((data_read[0]>>3)&0x01));
	//printf("\n CCTALK registers \t= %2.2x\n", (data_read[0]&0xFF));
	
	printf("\n ===================================");
	printf("\n === SECs MODULE (HW parameters) ===");
	printf("\n ===================================");
	IORd(pcie_bar_secs[1] + 0x3C, data_read, 1,1, NO_PRINT_VALUES);
	printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
	IORd(pcie_bar_secs[1] + 0x2A, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n CAPABILITIES \t= %2.2x", data_read[0]&0xff);	
	
	printf("\n DMA presence    \t= %s", DMA(data_read[0]&0x01));
	printf("\n RSA presence    \t= %s", RSA((data_read[0]>>1)&0x01));
	printf("\n KEY17th feature \t= %s", KEY_17th((data_read[0]>>2)&0x01));
	printf("\n Master KEY feature \t= %s", MASTER_KEY((data_read[0]>>3)&0x01));
	
	//printf("\n data ena \t= %d\n", data_read[0]);
	
	IORd(pcie_bar_io[0] + 0x10, data_read, 4,1, NO_PRINT_VALUES);
	printf("\n === VERIFY CHECKSUM ===");
	printf("\n       Sector 0");	
	printf("\n Checksum 64bytes \t= %2.2x", (data_read[0]&0xff));
	printf("\n Checksum 64bytes ok \t= %s\n", CK_MATCH(data_read[1]&0x01));
	//#if _OLDCFGROM	
		{	
	printf("\n       Sector 2");	
	printf("\n Checksum 64bytes \t= %2.2x", (data_read[2]&0xff));
	printf("\n Checksum 64bytes ok \t= %s", CK_MATCH((data_read[1]>>1)&0x01));
		}; 
	//#endif 
  
	
	printf("\n\n === TEST FIRST BYTE SRAM ===");
	data_write[0]=0x01;
	data_write[1]=0x02;
	data_write[2]=0x03;
	data_write[3]=0x04;
	MWr32(mem_addr, data_write, 4, 4, NO_PRINT_VALUES);
	MRd32(mem_addr, data_read, 4, 4, NO_PRINT_VALUES); 
	printf("\n SRAM first 4 bytes: byte0: 0x%2.2x byte1: 0x%2.2x byte2: 0x%2.2x byte3: 0x%2.2x\n",data_read[0]&0xff,data_read[1]&0xff, data_read[2]&0xff,data_read[3]&0xff); 
 
	IORd(pcie_bar_io[0] + 0x20, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n Authentication status: %s \n",AUTHENTICA(data_read[0]&0x01)); 

	#if _DEBUG	
		{	
			IORd(pcie_bar_io[0] + 0x4B, data_read, 1, 1, NO_PRINT_VALUES);
			printf("\n Frequency Divider \t  = %2.2x\n", data_read[0]&0xff);
			IORd(pcie_bar_io[0] + 0x48, data_read, 1, 1, NO_PRINT_VALUES);
			printf("\n Set up CPOL, CPHA, FLASH_CS and SPI ENABLE is \t  = %2.2x\n", data_read[0]&0xff);
		}; 
	#endif  
	
	
	uint32_t system_return;
	char path [1024]="sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0   -i ./../git/libsecuress/bin/x64/original_key    -o ./../git/libsecuress/bin/x64/key_enc  > /dev/null";
	
	sprintf (path, "sudo insmod ./../git/drvsecuress/linux/qxtsecs.ko");
	system_return = system(path);
	
	if (system_return != 0)
		{
			printf("system error path %s",path);	
		}
	sprintf (path, "sudo chmod 666 /dev/secS");
	system_return = system(path);	
	if (system_return != 0)
		{
			printf("system error path %s",path);	
		}		
	struct stat stat_file;
	if (stat("/dev/secS",&stat_file)!=1) printf("\n==========================\n/dev/secS not found\n You cannot use AES functionality\n==========================\n");	
	
		
	data_read[0]=0x00;
	data_read[1]=0x00;
	data_read[2]=0x00;
	data_read[3]=0x00;
	
	IORd(pcie_bar_secs[1] + 0x2B, data_read, 1,1, NO_PRINT_VALUES);
	printf("\n AES mode register: %2.2x",data_read[0]); 
	
			/*
				printf("\n\n\n ======================================= \t");
				printf("\n == enable debug =="); 
				data_write[0]=0x90;
				IOWr(pcie_bar_secs[1] + 0x20, data_write, 1, 1, 0, NO_PRINT_VALUES);
				printf("\n ======================================= \t\n\n");
										
			for(uint8_t num_key = 0; num_key < 16; num_key++)
				{
					printf("\n == AES KEY%d ==",num_key);
					data_write[0]=num_key*8;
						IOWr(pcie_bar_secs[1] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES); 
						printf("\n Key 255..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
					data_write[0]=(num_key*8+1);
						IOWr(pcie_bar_secs[1] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES); 
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
					data_write[0]=(num_key*8+2);
						IOWr(pcie_bar_secs[1] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES); 
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
					data_write[0]=(num_key*8+3);
						IOWr(pcie_bar_secs[1] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES); 
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
					data_write[0]=(num_key*8+4);
						IOWr(pcie_bar_secs[1] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES); 
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
					data_write[0]=(num_key*8+5);
						IOWr(pcie_bar_secs[1] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES); 
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
					data_write[0]=(num_key*8+6);
						IOWr(pcie_bar_secs[1] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES); 
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
					data_write[0]=(num_key*8+7);
						IOWr(pcie_bar_secs[1] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES); 
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
					}	
				
				printf("\n\n\n ======================================= \t");
				printf("\n == disable debug =="); 
				data_write[0]=0x80;
				IOWr(pcie_bar_secs[1] + 0x20, data_write, 1, 1, 0, NO_PRINT_VALUES);
				*/
	printf("\n\n =======================================");  
	printf("\n Config ROM SPI core frequency selection\n\n");  
	//printf("\n ==================================\n");  
 	/*
	printf(" 1: 64MHz \n");	//(x00+1)=1 	64/1=64
	printf(" 2: 32MHz \n");	//(x01+1)=2 	64/2=32  
	printf(" 3: 16MHz \n");	//(x03+1)=4 	64/4=16        
	printf(" 4: 8MHz  \n");	//(x07+1)=8 	64/8=8        
	printf(" 5: 4MHz  \n");	//(x0F+1)=16	64/16=4		  
	printf(" 6: 2MHz  \n");	//(x1F+1)=32	64/32=2
	printf(" 7: 1MHz  \n");	//(x3F+1)=64	64/64=1
	printf(" 8: Custom freq  \n");	//(xYY+1)=Z	64/Z=
		
	printf(" Select an item ? [1..8] ");
		if ((ret_code=scanf("%2hhx", &data_read[0]))!=1)
		{
			printf("function read error %d\n",ret_code);
		};
		
	if (data_read[0]==8) 
		{
			printf("Insert the freq divider (0x..) ");
			if ((ret_code=scanf("%2hhx", &data_read[1]))!=1)
			{
				printf("function read error %d\n",ret_code);
			};
		data_write[0]=data_read[1];
		}
	else 
		{
		data_write[0]=pot2(data_read[0])-0x01;
		};
		IOWr(pcie_bar_io[0] + 0x4B, data_write, 1, 1, 0, NO_PRINT_VALUES);
		#if _DEBUG	
			{	
				IORd(pcie_bar_io[0] + 0x4B, data_read, 1, 1, NO_PRINT_VALUES);
				printf("\n Frequency Divider \t  = %2.2x\n", data_read[0]&0xff);
			}; 
		#endif 
		*/
		
		data_write[0]=0x07;							
		IOWr(pcie_bar_io[0] + 0x4B, data_write, 1, 1, 0, NO_PRINT_VALUES);
		// setup clock freq fpga spi max freq
		data_write[0]=0x00;							
		IOWr(pcie_bar_io[0] + FPGA_ROM_BASE + 0x03, data_write, 1, 1, 0, NO_PRINT_VALUES);
							
	
	 /* ======================================
		======================================
		=========     MAIN MENU      ========= 
		======================================
		======================================
		*/  
		
		char exit=0;
		char chip_erase_choice[]="N";
		uint32_t testchoice;
		
		uint32_t byte_lenght=0;
		uint32_t byten;
		uint32_t sector;
		uint8_t start_addr[3];
		uint8_t data_buffer[256];
		uint8_t global_buffer[256];
		uint8_t sector_buffer[4096];
		uint8_t mem_buffer[mem_size];
		uint32_t bufferSize = 0;
		uint32_t i;
		uint32_t getpci_sel;
		uint32_t ats_sel;
		// NVram variable
		uint32_t nbanks;
		uint32_t chipsize;
		uint32_t sramtype;
		uint32_t rd_latency;
		uint32_t wr_latency;
		
		
		// qli variable
		uint8_t	qli_cfg[8];
		// io variable
		uint32_t fb_width;
		uint32_t dout_type;
		uint8_t fb_mask;
		uint8_t dout_cfg;
		uint8_t Hw_ID;
		uint8_t Sp_ID;
		// com_port
		uint8_t cctalk1_ena;
		uint8_t cctalk1_pu;
		uint8_t cctalk2_ena;
		uint8_t cctalk2_pu;
		uint8_t check_sum=0;
		// 
		uint32_t check_id_error=0;
		//uint32_t repetition=0;
		//uint32_t repetition_cnt=0;
		
		// init EEPROM page buffer data
		for (i = 0; i < 256; i++)
		{
			data_buffer[i]=0xFF;
			global_buffer[i]=0xFF;
		};	
		
	SEL_MENU:
	
		printf("\n\n==============================\n");  
		printf("===                        ===\n");  
		printf("=== SPI FLASH COMMAND MENU ===\n");  
		printf("===                        ===\n");  
		printf("==============================\n\n"); 
		
		printf("========== CONFIGURATION FLASH CMD ===========\n");
		printf("Cmd_01: Get device id and manufacture id (CFG FLASH) \n");
		printf("Cmd_02: Read Status Register (CFG FLASH) \n");
		printf("Cmd_03: Write SR protection enable ALL (CFG FLASH) \n");
		printf("Cmd_04: Program Page nByte (CFG FLASH) \n");
		printf("Cmd_05: Read Data nByte (CFG FLASH) \n");
		printf("Cmd_06: Chip Erase (CFG FLASH) \n");
		printf("Cmd_07: Sector Erase (CFG FLASH) \n");
		printf("Cmd_08: Advanced Program Sect 0 Header \n");
		printf("Cmd_09: Advanced Program SRAM CFG nByte \n");
		printf("Cmd_10: Advanced Program QLI CFG nByte \n");
		printf("Cmd_11: Advanced Program IO CFG nByte \n");
		printf("Cmd_12: Progam_Check_Sum Sect 0\n");
		printf("Cmd_13: Advanced Program CCTALK nByte \n");
		printf("Cmd_14: Progam_Check_Sum Sect 2\n");
		printf("Cmd_15: Set up SPI frequency (CFG FLASH) \n");
		printf("Cmd_16: Write SR protection disable ALL (CFG FLASH) \n");
		printf("Cmd_17: Check CFG ROM Automatic SelfTest \n");
		printf("============ FPGA FLASH CMD ===============        (last fpga byte 0x1d6B99)\n");
		printf("Cmd_20: Get device id and manufacture id (FPGA FLASH) \n");
		printf("Cmd_21: Read Status Register (FPGA FLASH) \n");
		printf("Cmd_22: Read Data nByte (FPGA FLASH) \n");//
		printf("Cmd_23: Program Page nByte (FPGA FLASH)  \n");
		printf("Cmd_24: FPGA program from input file (FPGA FLASH)  \n");
		printf("Cmd_25: FPGA readback and store to output file (FPGA FLASH)  \n");
		printf("Cmd_26: Chip Erase (FPGA FLASH) \n");
		printf("Cmd_27: Sector Erase (FPGA FLASH) \n");
		printf("Cmd_28: Advanced Program Sect 1020 Header \n");
		printf("Cmd_29: Advanced Program SRAM CFG nByte \n");
		printf("Cmd_30: Advanced Program QLI CFG nByte \n");
		printf("Cmd_31: Advanced Program IO CFG nByte \n");
		printf("Cmd_32: Progam_Check_Sum Sect 1020\n");
		printf("Cmd_33: Advanced Program CCTALK nByte \n");
		printf("Cmd_34: Progam_Check_Sum Sect 1022\n");
		printf("Cmd_39: Write SR protection disable ALL (FPGA FLASH) \n");
		printf("=============== AES TEST CMD ==================\n");
		printf("Cmd_40: Encrypt AES Keys with 17th key and write in Conf ROM sector 3 0x003000\n");	
		printf("Cmd_41: Verify Key0..15 with open SSL\n");	
		printf("Cmd_42: Encrypt AES Keys with 17th key and write in FPGA FLASH sector 1023 0x3FF000\n");	
		printf("Cmd_43: Test_Analyzer\n");	
		printf("Cmd_44: Encrypt AES Keys with Master key and write in FPGA FLASH sector 1023 0x3FF000\n");
		printf("Cmd_45: AES reset\n");
		printf("Cmd_46: Program Master Key injection\n");
		printf("Cmd_47: Get all AES debug register\n");
		printf("=============== JTAG CMD ==================\n");
		printf("Cmd_50: Get FPGA device id true jtag \n");
		printf("Cmd_51: Test and verification Trace ID true jtag \n");
		printf("Cmd_52: Test com port \n");		
		printf("=============== SRAM TEST ==================\n");
		printf("Cmd_60: SRAM Write Test from 0 to MAX size in Normal mode (stress test for consumption)\n");
		printf("Cmd_61: SRAM Write Test from 0 to MAX size in Mirror mode (stress test for consumption)\n");
		printf("Cmd_62: SRAM Write Test and Read back/verify from 0 to MAX size in Normal mode 64\n");
		printf("Cmd_63: SRAM Write Read access\n");
		printf("=============== DEBUG TEST ==================\n");
		printf("Cmd_64: DEBUG enable | control command\n");
		printf("Cmd_65: DEBUG status read\n");
		printf("Cmd_66: DEBUG serial command check\n");
		printf("Cmd_67: DEBUG serial basic test IO module\n");
		printf("Cmd_68: DEBUG serial basic test LP module\n");
		printf("=============== DIO TEST ==================\n");
		printf("Cmd_70: DIN status\n");
		printf("Cmd_71: DIN interrupt test\n");
		printf("=============== CFG PCIe SPACE ==================\n");
		printf("Cmd_80: Write IO Configuration Space\n");
		printf("=============== PCIe utility ==================\n");
		printf("Cmd_99: Get PCIe status/Enable PCIe/Set command reg\n");
	
		//IORd(pcie_bar_mem[1] + 0x14, data_read, 2, 1, NO_PRINT_VALUES); 
		printf("type 0 to exit \n");
		
		printf("Select an item ? [1..99] (other value to exit) ");
		if ((ret_code=scanf("%d", &testchoice))!=1)
		{
			printf("function read error %d\n",ret_code);
		};
		
		printf("\n");
		
		switch (testchoice) 
			{

				case 1:
					{					
						printf("\n** READ_DEVICE ID **\n"); 
						rd_dev_id_man_id(CFG_ROM_BASE,CS_CFG_ROM_ENA);
						
						wait_to_continue();	
						break;
					}
				case 2:
					{
						printf("\n** READ_STATUS_REGISTER **\n"); 
						read_sr(data_read,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						printf("\n Status Register \t = %2.2x ", data_read[0]&0xff);
						
						wait_to_continue();
						break;
					}
					
				case 3:
					{
						printf("\n** WRITE_STATUS_REGISTER PROTECTION **\n"); 
						data_write[0]=0x9C; //sr write protect ALL
						printf("\n   Byte0 \t = %2.2x ", data_write[0]&0xff);
						write_sr(data_write,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
					
				case 4: 
					{
					
						printf("Address to be written (0xAAAAAA) : 0x");
						if ((ret_code=scanf("%2hhx%2hhx%2hhx", &start_addr[2],&start_addr[1],&start_addr[0]))!=3)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Byte length ? [1..256]: ");
						if ((ret_code=scanf("%u", &byte_lenght))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						for (byten = 0; byten < byte_lenght; byten++)
						{
							printf("\n");
							printf("BYTE %2d to be written (0x..) : 0x", byten);
							if ((ret_code=scanf("%2hhx", &data_buffer[byten]))!=1)
							{
								printf("function read error %d\n",ret_code);
							};
						};

						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);

						wait_to_continue();
						break;
					}

				case 5: 
					{
						read_sr(data_read,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						if (((data_read[0])&0x01)!=0) 
							printf("\n Warning !!! The Device is busy in other operation \n");

						printf("Address to read from (0xAAAAAA) : 0x");
						if ((ret_code=scanf("%2hhx%2hhx%2hhx", &start_addr[2],&start_addr[1],&start_addr[0]))!=3)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Byte length ? [1..256]: ");
						if ((ret_code=scanf("%u", &byte_lenght))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						read_data_nbyte(start_addr,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA);

						wait_to_continue();
						break;
					} 

				case 6:
					{
						chip_erase(CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();	
						break;
					}
				
				case 7:
					{	
						printf("Which Sector of Block 0 to erase [0..1023] : ");
						if ((ret_code=scanf("%d", &sector))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						start_addr[2]=((sector >> 4) & 0xFF);//(((sector*4096) >> 16) & 0xFF);
						start_addr[1]=((sector << 4) & 0xFF);
						start_addr[0]=0x00;					 //(((sector*4096) & 0xFF));  
						
						#if _DEBUG	
							{	
								printf("sector address is %2hhx%2hhx%2hhx",start_addr[2],start_addr[1],start_addr[0]);
							}; 
						#endif 
						sector_erase(start_addr,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
	
						wait_to_continue();
						break;
					}
					
				case 8:
					{
						printf("Hardware Unique ID ? [0xHH] : 0x");
						if ((ret_code=scanf("%2hhx", &Hw_ID))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Platform Special ID ? [0xYY] : 0x");
						if ((ret_code=scanf("%2hhx", &Sp_ID))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						data_buffer[0]=(Hw_ID & 0xFF);
						data_buffer[1]=(Sp_ID & 0xFF);
						
						printf("byte0 is %2.2x\n",data_buffer[0]);
						printf("byte1 is %2.2x\n",data_buffer[1]);
						#if _DEBUG
						{
							printf("Hardware ID is %2.2x\n",(Hw_ID & 0xFF));
							printf("Special ID is %2.2x\n",(Sp_ID & 0xFF));
						}; 
						#endif

						start_addr[2]= 0x00;
						start_addr[1]= 0x00;
						start_addr[0]= 0x00;
						byte_lenght = 2;

						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
					
				case 9:
					{
						printf("-- ===================================================================================\n");
						printf("--    chip_num[1..0]    |          chip_size[3..0]                | sram_type[1..0] \n");
						printf("-- 00 SRAM1 installed   | 0000  0 Mbit SRAMs  1000  TBD           |    00   SRAM\n");
						printf("-- 01 SRAM1~2 installed | 0001  TBD           1001  TBD           |    01   Fast SRAM\n");
						printf("-- 10 SRAM1~4 installed | 0010  TBD           1010   2 Mbit SRAMs |    10   MRAM\n");
						printf("-- 11 SRAM1~8 installed | 0011  TBD           1011   4 Mbit SRAMs |   others tbd   \n");
						printf("--                      | 0100  TBD           1100   8 Mbit SRAMs | \n");
						printf("--                      | 0101  TBD           1101  16 Mbit SRAMs | \n");
						printf("--                      | 0110  TBD           1110  32 Mbit SRAMs | \n");
						printf("--                      | 0111  TBD           1111  64 Mbit SRAMs | \n");
						printf("-- ===================================================================================\n");
						//start_addr[3]= {0x00,0x00,0x00};
						printf("How Many Banks/Chips ? [0 = 1 SRAM module, 1 = 2 SRAM modules, 2 = 4 SRAM modules, 3 = 8 SRAM modules] ");
						if ((ret_code=scanf("%d", &nbanks))!=1)
						{
							printf("function read error %d\n",ret_code);
						};

						printf("\nInsert one of the following Chip size \n");
						printf("   ====================================\n");
						printf("   0  0 Mbit SRAMs    8   TBD           \n");
						printf("   1  TBD             9   TBD           \n");
						printf("   2  TBD             10   2 Mbit SRAMs \n");
						printf("   3  TBD             11   4 Mbit SRAMs \n");
						printf("   4  TBD             12   8 Mbit SRAMs \n");
						printf("   5  TBD             13  16 Mbit SRAMs \n");
						printf("   6  TBD             14  32 Mbit SRAMs \n");
						printf("   7  TBD             15  64 Mbit SRAMs \n");
						printf("   ====================================\n");
						printf("Which is the chip size ? [0..15] ");
						if ((ret_code=scanf("%d", &chipsize))!=1)
						{
							printf("function read error %d\n",ret_code);
						};

						printf("Which SRAM type is mounted ? [0 - SRAM, 1 - FAST SRAM, 2 - MRAM] ");
						if ((ret_code=scanf("%d", &sramtype))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Which is read latency ? [0..15] ");
						if ((ret_code=scanf("%d", &rd_latency))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Which is write latency ? [0..15] ");
						if ((ret_code=scanf("%d", &wr_latency))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						//byte0 = (((sramtype & 0xFF)<<6) | ((chipsize & 0xFF)<<2) | ((nbanks-1) & 0xFF));
						data_buffer[0]=(sramtype & 0x03);
						data_buffer[1]=(((chipsize & 0x0F)<<2) | ((nbanks) & 0x03));
						data_buffer[2]=((wr_latency & 0x0F)<<4 | (rd_latency & 0x0F));
						
						printf("byte8 is %2.2x\n",data_buffer[0]);
						printf("byte9 is %2.2x\n",data_buffer[1]);
						printf("byte10 is %2.2x\n",data_buffer[2]);
						#if _DEBUG
						{
							printf("nbanks is %2.2x\n",(pot2(nbanks) & 0xFF));
							printf("chipsize is %2.2x\n",(chipsize & 0xFF));
							printf("write latency is %2.2x\n",(wr_latency & 0xFF));
							printf("read latency is %2.2x\n",(rd_latency & 0xFF));
						}; 
						#endif

						start_addr[2]= 0x00;
						start_addr[1]= 0x00;
						start_addr[0]= 0x08;
						byte_lenght = 3;

						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
				
				case 10:
					{
						printf("-- ========================================\n");
						printf("--    QLI Channels Presence/Capabilities   \n");	
						printf("--        (0) not present                  \n");
						printf("--        (1) 1 wire or 2 wire             \n");
						printf("--        (2) 1 wire, 2 wire or pwm        \n");
						printf("--        (3) TBD                          \n");
						printf("-- ========================================\n");
						
						for (i = 0; i < 8; i++)
						{
							printf("Insert QLI %d config parameters [0..3] ",i);
							if ((ret_code=scanf("%d", &qli_cfg[i]))!=1)
							{
								printf("function read error %d\n",ret_code);
							};
						};	
						
						data_buffer[0]= (  ((qli_cfg[3] & 0x03)<<6) | ((qli_cfg[2] & 0x03)<<4) | ((qli_cfg[1] & 0x03)<<2) | (qli_cfg[0] & 0x03)  );
						data_buffer[1]= (  ((qli_cfg[7] & 0x03)<<6) | ((qli_cfg[6] & 0x03)<<4) | ((qli_cfg[5] & 0x03)<<2) | (qli_cfg[4] & 0x03)  );
						
						printf("byte16 is %2.2x\n",data_buffer[0]);
						printf("byte17 is %2.2x\n",data_buffer[1]);

						start_addr[2]= 0x00;
						start_addr[1]= 0x00;
						start_addr[0]= 0x10;
						byte_lenght = 2;

						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
					
				case 11:
					{
						printf("-- ==================================================\n");
						printf("--                IO Config Capabilities             \n");	
						printf("--        fb_width[2..0]       |  Dout type sel      \n");	
						printf("-- 000 - 8 bits  100 - 40 bits |    00000 TI         \n");
						printf("-- 001 - 16 bits 101 - 48 bits |    00001 ST         \n");
						printf("-- 010 - 24 bits 110 - 56 bits |    00010 ONSEMI     \n");
						printf("-- 011 - 32 bits 111 - TBD     |                     \n");
						printf("-- ==================================================\n");
						
						printf("Insert Feedback width params [0..6] (0 = 8 bits, .. ,6 = 56 bits): ");
						if ((ret_code=scanf("%d", &fb_width))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						printf("Dout octal buffer: 2 = ONSEMI (feedback); 1 = ST (feedback); 0 = TI (no feedback)\n type ? : ");
						if ((ret_code=scanf("%d", &dout_type))!=1)
						{
							printf("function read error %d\n",ret_code);
						};							
						
						data_buffer[0]=(  ((dout_type & 0x1F)<<3) | (fb_width & 0x07)  );
						printf("byte24 is %2.2x\n",data_buffer[0]);
						
						start_addr[2]= 0x00;
						start_addr[1]= 0x00;
						start_addr[0]= 0x18;
						byte_lenght = 1;
						
						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
					
				case 12:
					{	
						start_addr[2]= 0x00;
						start_addr[1]= 0x00;
						start_addr[0]= 0x08;
						byte_lenght=55;
						read_data_nbyte_output(start_addr,byte_lenght,global_buffer,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						// calculate checksum
						check_sum=0;
						for (i = 0; i < 56; i++)
						{
							check_sum=global_buffer[i]+check_sum;
							printf("%2.2x ",global_buffer[i]);
						};
						data_buffer[0]=check_sum;						
						//global_buffer[31]=check_sum;
						// 
						start_addr[2]= 0x00;
						start_addr[1]= 0x00;
						start_addr[0]= 0x02;
						byte_lenght = 1;
						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}	
				
				case 13:
					{
						printf("-- ========================================\n");
						printf("--       CCTALK Enable/Configuration       \n");		
						printf("--  cctalk[x] ena	                       \n");
						printf("--  (0) cctalk disabled COM enabled        \n");
						printf("--  (1) cctalk enabled COM disabled        \n");
						printf("--  cctalk[x] pull up                      \n");
						printf("--  (0) 10k pull-up                        \n");
						printf("--  (1) 10k-1k parallel pull-up            \n");		
						printf("-- ========================================\n");
						
						
						printf("CCTALK 1 management:    1 = ENABLED; 0 DISABLED ? : ");
						if ((ret_code=scanf("%hhu", &cctalk1_ena))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
												
						printf("CCTALK 1 pull-up setup: 1 = 10k-1k; 0 = 10k ?     : ");
												
						if ((ret_code=scanf("%hhu", &cctalk1_pu))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						printf("CCTALK 2 management:    1 = ENABLED; 0 DISABLED ? : ");
												
						if ((ret_code=scanf("%hhu", &cctalk2_ena))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						printf("CCTALK 2 pull-up setup: 1 = 10k-1k; 0 = 10k ?     : ");
												
						if ((ret_code=scanf("%hhu", &cctalk2_pu))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						data_buffer[0]= ((cctalk2_pu & 0x01)<<3) | ((cctalk2_ena & 0x01)<<2) | ((cctalk1_pu & 0x01)<<1) | (cctalk1_ena & 0x01);  
						
						printf("byte32 is %2.2x\n",data_buffer[0]);
						
						start_addr[2]= 0x00;
						start_addr[1]= 0x20;
						start_addr[0]= 0x20;
						byte_lenght = 1;

						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
					
				case 14:
					{	
						start_addr[2]= 0x00;
						start_addr[1]= 0x20;
						start_addr[0]= 0x08;
						byte_lenght=55;
						read_data_nbyte_output(start_addr,byte_lenght,global_buffer,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						// calculate checksum
						check_sum=0;
						for (i = 0; i < 56; i++)
						{
							check_sum=global_buffer[i]+check_sum;
							printf("%2.2x ",global_buffer[i]);
						};
						data_buffer[0]=check_sum;						
						//global_buffer[31]=check_sum;
						// 
						start_addr[2]= 0x00;
						start_addr[1]= 0x20;
						start_addr[0]= 0x02;
						byte_lenght = 1;
						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}	
					
				case 15:
					{
						printf("\n\n** Set up SPI FREQ **\n"); 	
						printf("1: 64MHz \n"); //(x00+1)=1 	64/1=64
						printf("2: 32MHz \n"); //(x01+1)=2 	64/2=32  
						printf("3: 16MHz \n"); //(x03+1)=4 	64/4=16        
						printf("4: 8MHz  \n"); //(x07+1)=8 	64/8=8        
						printf("5: 4MHz  \n"); //(x0F+1)=16 64/16=4		  
						printf("6: 2MHz  \n"); //(x1F+1)=32 64/32=2
						printf("7: 1MHz  \n"); //(x3F+1)=64 64/64=1
						printf("8: Custom freq  \n"); // (xYY+1)=Z 64/Z=
							
						printf("Select an item ? [1..8] ");
							if ((ret_code=scanf("%2hhx", &data_read[0]))!=1)
							{
								printf("function read error %d\n",ret_code);
							};
							
						if (data_read[0]==8) 
							{
							  printf("Insert the freq divider (0x..) ");
								if ((ret_code=scanf("%2hhx", &data_read[1]))!=1)
								{
									printf("function read error %d\n",ret_code);
								};
							data_write[0]=data_read[1];
							}
						else 
							{
							data_write[0]=pot2(data_read[0])-0x01;
							};
							IOWr(pcie_bar_io[0] + CFG_ROM_BASE + 0x03, data_write, 1, 1, 0, NO_PRINT_VALUES);
							#if _DEBUG	
								{	
									IORd(pcie_bar_io[0] + CFG_ROM_BASE + 0x03, data_read, 1, 1, NO_PRINT_VALUES);
									printf("\n Frequency Divider \t  = %2.2x\n", data_read[0]&0xff);
								}; 
							#endif 	
							
						wait_to_continue();				
						break;
					}  
					
				case 16:
					{
						printf("\n** WRITE_STATUS_REGISTER DISABLE PROTECTION **\n"); 
						data_write[0]=0x00; //sr write unprotect ALL
						printf("\n   Byte0 \t = %2.2x ", data_write[0]&0xff);
						write_sr(data_write,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}	
					
				case 17:
					{
	
						printf("\n** READ_DEVICE ID **\n"); 
						rd_dev_id_man_id(CFG_ROM_BASE,CS_CFG_ROM_ENA);
						
						printf("\n=======================\n");
						printf("** VERIFY PROTECTION **"); 
						printf("\n=======================\n"); 
						//** check SR 
						read_sr(data_read,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						//printf("\n Status Register \t = %2.2x ", data_read[0]&0xff);
						if (data_read[0]!=0) 
							{
								//** clear SR unprotect all
								data_write[0]=0x00; //sr write unprotect ALL
								write_sr(data_write,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
								//** check SR again
								read_sr(data_read,CFG_ROM_BASE,CS_CFG_ROM_ENA);
								//printf("\n Status Register \t = %2.2x ", data_read[0]&0xff);
								if (data_read[0]!=0) 
									{										
										printf("FAILURE: cannot access status register in W/R Abort\n");
										break;										
									};
							};						
						printf("** Status Register Access             : VERIFIED \n");
						
						// ** Enable status register protection ALL
						data_write[0]=0x9C; 
						write_sr(data_write,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						// ** check SR  
						read_sr(data_read,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						//printf("\n Status Register \t = %2.2x ", data_read[0]&0xff);
						if (data_read[0]!=data_write[0]) 
							{
								printf("FAILURE: cannot write status register protection \n");
								break;										
							};
						printf("** Status Register Write Protection   : VERIFIED \n");
							
						// ** Disable status register protection ALL 
						data_write[0]=0x00; 
						write_sr(data_write,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						// ** check SR  
						read_sr(data_read,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						//printf("\n Status Register \t = %2.2x ", data_read[0]&0xff);
						if (data_read[0]!=0) 
							{
								printf("FAILURE: cannot disable status register protection \n");
								break;										
							};
						printf("** Status Register Disable Protection : VERIFIED \n");
						
						printf("\n===========================\n");
						printf("** VERIFY READ/WRITE CMD **"); 
						printf("\n===========================\n"); 
						// ** Program first 256 bytes fourth sector 0x004000
						start_addr[2]= 0x00;
						start_addr[1]= 0x40;
						start_addr[0]= 0x00;
						// ** Erase fourth sector 0x004000
						sector_erase(start_addr,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						byte_lenght = 256;
						for (byten = 0; byten < byte_lenght; byten++)
						{
							data_buffer[byten]=byten;
						};
						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						
						// ** Read first 256 bytes fourth sector 0x004000 and verify match
						read_data_nbyte_output(start_addr,byte_lenght,global_buffer,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						for (byten = 0; byten < byte_lenght; byten++)
						{
							if (data_buffer[byten]!=global_buffer[byten])
								{
									printf("FAILURE: Read failure. Write expected data %d is different from Read got data %d \n", data_buffer[byten]&0xff, global_buffer[byten]&0xff);
									break;
								};
						};
						printf("** FLASH Program and Read commands    : VERIFIED \n");
						
						printf("\n==============================\n");
						printf("** VERIFY CHIP/SECTOR ERASE **"); 
						printf("\n==============================\n"); 
						// ** Verify flash protection on sector erase and chip erase 
						// ** Enable status register protection ALL
						data_write[0]=0x9C; 
						write_sr(data_write,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						// ** check SR  
						read_sr(data_read,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						//printf("\n Status Register \t = %2.2x ", data_read[0]&0xff);
						if (data_read[0]!=data_write[0]) 
							{
								printf("FAILURE: cannot write status register protection \n");
								break;										
							};
						// ** Erase fourth sector 0x004000
						sector_erase(start_addr,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						// ** Chip erase 
						chip_erase(CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						// ** Read first 256 bytes fourth sector 0x004000 and verify match
						read_data_nbyte_output(start_addr,byte_lenght,global_buffer,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						for (byten = 0; byten < byte_lenght; byten++)
						{
							if (data_buffer[byten]!=global_buffer[byten])
								{
									printf("FAILURE: Read failure. Write expected data %d is different from Read got data %d \n", data_buffer[byten]&0xff, global_buffer[byten]&0xff);
									break;
								};
						};	    
						printf("** sector and chip erase protection   : VERIFIED \n");
						
						// ** Disable status register protection ALL 
						data_write[0]=0x00; 
						write_sr(data_write,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						// ** Erase fourth sector 0x004000
						sector_erase(start_addr,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						// ** Read first 4096 bytes fourth sector 0x004000 and verify match
						byte_lenght = 4096;
						read_data_nbyte_output(start_addr,byte_lenght,sector_buffer,CFG_ROM_BASE,CS_CFG_ROM_ENA);
						for (byten = 0; byten < byte_lenght; byten++)
						{
							if (sector_buffer[byten]!=0xFF)
								{
									printf("FAILURE: Sector erase is failed. Read got data is: %d \n", sector_buffer[byten]&0xff);
									break;
								};
						};	  
						printf("** FLASH sector erase protection      : VERIFIED \n");  
						
						printf("FLASH Chip erase CMD will erease all flash content, are you sure ? [Y/N] ");
						if ((ret_code=scanf("%s", chip_erase_choice))!=1)  
							{
								printf("function read error %d\n",ret_code);
							};	
						if (strcmp(chip_erase_choice,"Y")==0)
							{
								// ** Chip erase 
								chip_erase(CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
								// ** Verify Chip erased
								byte_lenght = mem_size;
								//read_data_nbyte_output(start_addr,mem_size,mem_buffer,CFG_ROM_BASE,CS_CFG_ROM_ENA);
								read_page_output(start_addr,mem_size,mem_buffer,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
								for (byten = 0; byten < byte_lenght; byten++)
								{
									if (mem_buffer[byten]!=0xFF)
										{
											printf("FAILURE: Sector erase is failed. Read got data is: %d \n", mem_buffer[byten]&0xff);
											break;
										};
								};	 
								printf("** FLASH chip erase protection        : VERIFIED \n");  
								break;
							};
						printf("** FLASH chip erase protection        : to be VERIFY \n"); 
						break;
					}	
					
				case 20:
					{					
						printf("\n** READ_DEVICE ID **\n"); 
						rd_dev_id_man_id(FPGA_ROM_BASE,CS_FPGA_ROM_ENA);
						
						wait_to_continue();	
						break;
					}	
					
				case 21:
					{
						printf("\n** READ_STATUS_REGISTER **\n"); 
						read_sr(data_read,FPGA_ROM_BASE,CS_FPGA_ROM_ENA);
						printf("\n Status Register \t = %2.2x ", data_read[0]&0xff);
						
						wait_to_continue();
						break;
					}	
					
				case 22: 
					{
						read_sr(data_read,FPGA_ROM_BASE,CS_FPGA_ROM_ENA);
						if (data_read[0]|=0) 
							printf("\n Warning !!! The Device is busy in other operation \n");

						printf("Address to read from (0xAAAAAA) : 0x");
						if ((ret_code=scanf("%2hhx%2hhx%2hhx", &start_addr[2],&start_addr[1],&start_addr[0]))!=3)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Byte length ? [1..256]: ");
						if ((ret_code=scanf("%u", &byte_lenght))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						read_data_nbyte(start_addr,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA);

						wait_to_continue();
						break;
					} 
					
				case 23: 
					{					
						printf("\n Address to be written (0xAAAAAA) : 0x");
						if ((ret_code=scanf("%2hhx%2hhx%2hhx", &start_addr[2],&start_addr[1],&start_addr[0]))!=3)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Byte length ? [1..256]: ");
						if ((ret_code=scanf("%u", &byte_lenght))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						for (byten = 0; byten < byte_lenght; byten++)
						{
							printf("\n");
							printf("BYTE %2d to be written (0x..) : 0x", byten);
							if ((ret_code=scanf("%2hhx", &data_buffer[byten]))!=1)
							{
								printf("function read error %d\n",ret_code);
							};
						};

						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);

						wait_to_continue();
						break;
					}
					
				case 24: 
					{
						int rom_file;
						uint32_t w_size, size;
						char rom_file_name[]="Qxi_FPGA_30_Qxi_FPGA_30.bit";
						
						printf("Insert the name of FPGA input file: ");
						if ((ret_code=scanf("%s", rom_file_name))!=1)
						{
							printf("function read error %d\n",ret_code);
						};	

						rom_file = open(rom_file_name, O_RDONLY);						
						
						if (rom_file >= 0) 
							{					
								size = read(rom_file, fpga_wdata, mem_size);
								w_size = writeFpgaCode(fpga_wdata, size, padding, PRINT_VALUES);
								close(rom_file); 
								if (w_size != size ) 
								{
									printf("FAILURE: memory chip protection error\n");											
								}	
								wait_to_continue();									
								break;
							}								
						else 
							{			
								printf("FAILURE: error opening %s. Abort\n", rom_file_name);
								wait_to_continue();
								break;
							}
					}	
					
				case 25: 
					{
						int32_t rom_file_out;
						uint32_t r_size;
						uint32_t byte_size,size;
						char rom_out_file_name[]="fpga_output.bit";
						
						printf("Insert the name of FPGA output file: ");
						if ((ret_code=scanf("%s", rom_out_file_name))!=1)
						{
							printf("function read error %d\n",ret_code);
						};	
						printf("Insert how many byte to read and save [0=MAX]: ");
						if ((ret_code=scanf("%u", &byte_size))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						if (byte_size == 0) 
							{
								size=mem_size;
							}
						else if (byte_size > 256)
							{
								size=byte_size;
							}
						else 
							{
								size=256;
							}
						// read fpga code
						readFpgaCode(fpga_rdata, size, PRINT_VALUES);
						printf("\n");	
						// store to file
						rom_file_out = open(rom_out_file_name, O_CREAT | O_WRONLY | O_TRUNC, 0777);		
						if (rom_file_out >= 0) 
								{
									r_size=write(rom_file_out, fpga_rdata, size);	
									close(rom_file_out); 
									if (r_size != size ) 
									{
										printf("FAILURE: on file writing\n");								
										break;		
									}										
									break;
								}
							else 
								{			
									printf("FAILURE: error opening %s. Abort\n", rom_out_file_name);	
									break;
								}	
					}	
					
				case 26:
					{
						chip_erase(FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
							
						wait_to_continue();
						break;
					}
				
				case 27:
					{	
						printf("Which Sector of Block 0 to erase [0..1023] : ");
						if ((ret_code=scanf("%d", &sector))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						start_addr[2]=((sector >> 4) & 0xFF);//(((sector*4096) >> 16) & 0xFF);
						start_addr[1]=((sector << 4) & 0xFF);
						start_addr[0]=0x00;					 //(((sector*4096) & 0xFF));  
						
						#if _DEBUG	
							{	
								printf("sector address is %2hhx%2hhx%2hhx",start_addr[2],start_addr[1],start_addr[0]);
							}; 
						#endif 					
						sector_erase(start_addr,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
	
						wait_to_continue();
						break;
					}
					
				case 28:
					{
						printf("Hardware Unique ID ? [0xHH] : 0x");
						if ((ret_code=scanf("%2hhx", &Hw_ID))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Platform Special ID ? [0xYY] : 0x");
						if ((ret_code=scanf("%2hhx", &Sp_ID))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						data_buffer[0]=(Hw_ID & 0xFF);
						data_buffer[1]=(Sp_ID & 0xFF);
						
						printf("byte0 is %2.2x\n",data_buffer[0]);
						printf("byte1 is %2.2x\n",data_buffer[1]);
						#if _DEBUG
						{
							printf("Hardware ID is %2.2x\n",(Hw_ID & 0xFF));
							printf("Special ID is %2.2x\n",(Sp_ID & 0xFF));
						}; 
						#endif

						start_addr[2]= 0x3F;
						start_addr[1]= 0xC0;
						start_addr[0]= 0x00;
						byte_lenght = 2;

						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
					
				case 29:
					{
						printf("-- ===================================================================================\n");
						printf("--    chip_num[1..0]    |          chip_size[3..0]                | sram_type[1..0] \n");
						printf("-- 00 SRAM1 installed   | 0000  0 Mbit SRAMs  1000  TBD           |    00   SRAM\n");
						printf("-- 01 SRAM1~2 installed | 0001  TBD           1001  TBD           |    01   Fast SRAM\n");
						printf("-- 10 SRAM1~4 installed | 0010  TBD           1010   2 Mbit SRAMs |    10   MRAM\n");
						printf("-- 11 SRAM1~8 installed | 0011  TBD           1011   4 Mbit SRAMs |   others tbd   \n");
						printf("--                      | 0100  TBD           1100   8 Mbit SRAMs | \n");
						printf("--                      | 0101  TBD           1101  16 Mbit SRAMs | \n");
						printf("--                      | 0110  TBD           1110  32 Mbit SRAMs | \n");
						printf("--                      | 0111  TBD           1111  64 Mbit SRAMs | \n");
						printf("-- ===================================================================================\n");
						//start_addr[3]= {0x00,0x00,0x00};
						printf("How Many Banks/Chips ? [0 = 1 SRAM module, 1 = 2 SRAM modules, 2 = 4 SRAM modules, 3 = 8 SRAM modules] ");
						if ((ret_code=scanf("%d", &nbanks))!=1)
						{
							printf("function read error %d\n",ret_code);
						};

						printf("\nInsert one of the following Chip size \n");
						printf("   ====================================\n");
						printf("   0  0 Mbit SRAMs    8   TBD           \n");
						printf("   1  TBD             9   TBD           \n");
						printf("   2  TBD             10   2 Mbit SRAMs \n");
						printf("   3  TBD             11   4 Mbit SRAMs \n");
						printf("   4  TBD             12   8 Mbit SRAMs \n");
						printf("   5  TBD             13  16 Mbit SRAMs \n");
						printf("   6  TBD             14  32 Mbit SRAMs \n");
						printf("   7  TBD             15  64 Mbit SRAMs \n");
						printf("   ====================================\n");
						printf("Which is the chip size ? [0..15] ");
						if ((ret_code=scanf("%d", &chipsize))!=1)
						{
							printf("function read error %d\n",ret_code);
						};

						printf("Which SRAM type is mounted ? [0 - SRAM, 1 - FAST SRAM, 2 - MRAM] ");
						if ((ret_code=scanf("%d", &sramtype))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Which is read latency ? [0..15] ");
						if ((ret_code=scanf("%d", &rd_latency))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("Which is write latency ? [0..15] ");
						if ((ret_code=scanf("%d", &wr_latency))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						//byte0 = (((sramtype & 0xFF)<<6) | ((chipsize & 0xFF)<<2) | ((nbanks-1) & 0xFF));
						data_buffer[0]=(sramtype & 0x03);
						data_buffer[1]=(((chipsize & 0x0F)<<2) | ((nbanks) & 0x03));
						data_buffer[2]=((wr_latency & 0x0F)<<4 | (rd_latency & 0x0F));
						
						printf("byte8 is %2.2x\n",data_buffer[0]);
						printf("byte9 is %2.2x\n",data_buffer[1]);
						printf("byte10 is %2.2x\n",data_buffer[2]);
						#if _DEBUG
						{
							printf("nbanks is %2.2x\n",(pot2(nbanks) & 0xFF));
							printf("chipsize is %2.2x\n",(chipsize & 0xFF));
							printf("write latency is %2.2x\n",(wr_latency & 0xFF));
							printf("read latency is %2.2x\n",(rd_latency & 0xFF));
						}; 
						#endif

						start_addr[2]= 0x3F;
						start_addr[1]= 0xC0;
						start_addr[0]= 0x08;
						byte_lenght = 3;

						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
				
				case 30:
					{
						printf("-- ========================================\n");
						printf("--    QLI Channels Presence/Capabilities   \n");	
						printf("--        (0) not present                  \n");
						printf("--        (1) 1 wire or 2 wire             \n");
						printf("--        (2) 1 wire, 2 wire or pwm        \n");
						printf("--        (3) TBD                          \n");
						printf("-- ========================================\n");
						
						for (i = 0; i < 8; i++)
						{
							printf("Insert QLI %d config parameters [0..3] ",i);
							if ((ret_code=scanf("%d", &qli_cfg[i]))!=1)
							{
								printf("function read error %d\n",ret_code);
							};
						};	
						
						data_buffer[0]= (  ((qli_cfg[3] & 0x03)<<6) | ((qli_cfg[2] & 0x03)<<4) | ((qli_cfg[1] & 0x03)<<2) | (qli_cfg[0] & 0x03)  );
						data_buffer[1]= (  ((qli_cfg[7] & 0x03)<<6) | ((qli_cfg[6] & 0x03)<<4) | ((qli_cfg[5] & 0x03)<<2) | (qli_cfg[4] & 0x03)  );
						
						printf("byte16 is %2.2x\n",data_buffer[0]);
						printf("byte17 is %2.2x\n",data_buffer[1]);

						start_addr[2]= 0x3F;
						start_addr[1]= 0xC0;
						start_addr[0]= 0x10;
						byte_lenght = 2;

						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
					
				case 31:
					{
						printf("-- ==================================================\n");
						printf("--                IO Config Capabilities             \n");	
						printf("--        fb_width[2..0]       |  Dout type sel      \n");	
						printf("-- 000 - 8 bits  100 - 40 bits |    00000 TI         \n");
						printf("-- 001 - 16 bits 101 - 48 bits |    00001 ST         \n");
						printf("-- 010 - 24 bits 110 - 56 bits |    00010 ONSEMI     \n");
						printf("-- 011 - 32 bits 111 - TBD     |                     \n");
						printf("-- ==================================================\n");
						
						printf("Insert Feedback width params [0..6] (0 = 8 bits, .. ,6 = 56 bits): ");
						if ((ret_code=scanf("%d", &fb_width))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						printf("Dout octal buffer: 2 = ONSEMI (feedback); 1 = ST (feedback); 0 = TI (no feedback)\n type ? : ");
						if ((ret_code=scanf("%d", &dout_type))!=1)
						{
							printf("function read error %d\n",ret_code);
						};							
						
						data_buffer[0]=(  ((dout_type & 0x1F)<<3) | (fb_width & 0x07)  );
						printf("byte24 is %2.2x\n",data_buffer[0]);
						
						start_addr[2]= 0x3F;
						start_addr[1]= 0xC0;
						start_addr[0]= 0x18;
						byte_lenght = 1;
						
						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
					
				case 32:
					{	
						start_addr[2]= 0x3F;
						start_addr[1]= 0xC0;
						start_addr[0]= 0x08;
						byte_lenght=55;
						read_data_nbyte_output(start_addr,byte_lenght,global_buffer,FPGA_ROM_BASE,CS_FPGA_ROM_ENA);
						// calculate checksum
						check_sum=0;
						for (i = 0; i < 56; i++)
						{
							check_sum=global_buffer[i]+check_sum;
							printf("%2.2x ",global_buffer[i]);
						};
						data_buffer[0]=check_sum;						
						//global_buffer[31]=check_sum;
						// 
						start_addr[2]= 0x3F;
						start_addr[1]= 0xC0;
						start_addr[0]= 0x02;
						byte_lenght = 1;
						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}	
				
				case 33:
					{
						printf("-- ========================================\n");
						printf("--       CCTALK Enable/Configuration       \n");		
						printf("--  cctalk[x] ena	                       \n");
						printf("--  (0) cctalk disabled COM enabled        \n");
						printf("--  (1) cctalk enabled COM disabled        \n");
						printf("--  cctalk[x] pull up                      \n");
						printf("--  (0) 10k pull-up                        \n");
						printf("--  (1) 10k-1k parallel pull-up            \n");		
						printf("-- ========================================\n");
						
						
						printf("CCTALK 1 management:    1 = ENABLED; 0 DISABLED ? : ");
						if ((ret_code=scanf("%hhu", &cctalk1_ena))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
												
						printf("CCTALK 1 pull-up setup: 1 = 10k-1k; 0 = 10k ?     : ");
												
						if ((ret_code=scanf("%hhu", &cctalk1_pu))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						printf("CCTALK 2 management:    1 = ENABLED; 0 DISABLED ? : ");
												
						if ((ret_code=scanf("%hhu", &cctalk2_ena))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						printf("CCTALK 2 pull-up setup: 1 = 10k-1k; 0 = 10k ?     : ");
												
						if ((ret_code=scanf("%hhu", &cctalk2_pu))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						data_buffer[0]= ((cctalk2_pu & 0x01)<<3) | ((cctalk2_ena & 0x01)<<2) | ((cctalk1_pu & 0x01)<<1) | (cctalk1_ena & 0x01);  
						
						printf("byte32 is %2.2x\n",data_buffer[0]);
						
						start_addr[2]= 0x3F;
						start_addr[1]= 0xE0;
						start_addr[0]= 0x20;
						byte_lenght = 1;

						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
												
						wait_to_continue();
						break;
					}
					
				case 34:
					{	
						start_addr[2]= 0x3F;
						start_addr[1]= 0xE0;
						start_addr[0]= 0x08;
						byte_lenght=55;
						read_data_nbyte_output(start_addr,byte_lenght,global_buffer,FPGA_ROM_BASE,CS_FPGA_ROM_ENA);
						// calculate checksum
						check_sum=0;
						for (i = 0; i < 56; i++)
						{
							check_sum=global_buffer[i]+check_sum;
							printf("%2.2x ",global_buffer[i]);
						};
						data_buffer[0]=check_sum;						
						//global_buffer[31]=check_sum;
						// 
						start_addr[2]= 0x3F;
						start_addr[1]= 0xE0;
						start_addr[0]= 0x02;
						byte_lenght = 1;
						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}	
					
				case 39:
					{
						printf("\n** WRITE_STATUS_REGISTER DISABLE PROTECTION **\n"); 
						data_write[0]=0x00; //sr write unprotect ALL
						printf("\n   Byte0 \t = %2.2x ", data_write[0]&0xff);
						write_sr(data_write,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}	
				
				case 40:
					{
						printf("-- ========================================\n");
						printf("--      Program AES Keys in Conf ROM       \n");		
						printf("--          Sector 3 0x003000              \n");		
						printf("-- ========================================\n");
						
						data_write[0]=0x01;
						IOWr(pcie_bar_secs[1] + 0x2B, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x2B, data_read, 1,1, NO_PRINT_VALUES);
						printf("\n AES mode register: %2.2x",data_read[0]); 
						
						sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0 -i ./../git/libsecuress/bin/x64/original_key -o ./../git/libsecuress/bin/x64/key_enc > /dev/null");
						sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0 -i ./../git/libsecuress/bin/x64/original_key -o ./../git/libsecuress/bin/x64/key_enc -ow> /dev/null");
						system_return = system(path);
						if (system_return != 0)
						{
							printf("system error path %s",path);	
						}
						
						
						data_write[0]=0x00;
						IOWr(pcie_bar_secs[1] + 0x2B, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x2B, data_read, 1,1, NO_PRINT_VALUES);
						printf("\n AES mode register: %2.2x",data_read[0]); 
						
						for(i = 0; i < 8; i++)
						{
							printf("\n == AES KEY%d ==",i); 
							MRd32(mem_addr_secs, data_buffer, 256, 4, NO_PRINT_VALUES); 
							printf("\n Key 255..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+0]&0xff,data_buffer[(i*32)+1]&0xff,data_buffer[(i*32)+2]&0xff,data_buffer[(i*32)+3]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+4]&0xff,data_buffer[(i*32)+5]&0xff,data_buffer[(i*32)+6]&0xff,data_buffer[(i*32)+7]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+8]&0xff,data_buffer[(i*32)+9]&0xff,data_buffer[(i*32)+10]&0xff,data_buffer[(i*32)+11]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+12]&0xff,data_buffer[(i*32)+13]&0xff,data_buffer[(i*32)+14]&0xff,data_buffer[(i*32)+15]&0xff); 
							printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+16]&0xff,data_buffer[(i*32)+17]&0xff,data_buffer[(i*32)+18]&0xff,data_buffer[(i*32)+19]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+20]&0xff,data_buffer[(i*32)+21]&0xff,data_buffer[(i*32)+22]&0xff,data_buffer[(i*32)+23]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+24]&0xff,data_buffer[(i*32)+25]&0xff,data_buffer[(i*32)+26]&0xff,data_buffer[(i*32)+27]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+28]&0xff,data_buffer[(i*32)+29]&0xff,data_buffer[(i*32)+30]&0xff,data_buffer[(i*32)+31]&0xff); 
						}
						
						// ** Program first 256 bytes fourth sector 0x003000
						start_addr[0]= 0x00;
						start_addr[1]= 0x30;
						start_addr[2]= 0x00;
						// ** Erase fourth sector 0x003000
						sector_erase(start_addr,CFG_ROM_BASE,CS_CFG_ROM_ENA,NO_PRINT_VALUES);
						byte_lenght = 256;
						
						start_addr[0]= 0x00;
						start_addr[1]= 0x30;
						start_addr[2]= 0x00;
						byte_lenght = 256;
						
						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						for(i = 0; i < 8; i++)
						{
							printf("\n == AES KEY%d ==",i+8); 
							MRd32(mem_addr_secs+256, data_buffer, 256, 4, NO_PRINT_VALUES); 
							printf("\n Key 255..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+0]&0xff,data_buffer[(i*32)+1]&0xff,data_buffer[(i*32)+2]&0xff,data_buffer[(i*32)+3]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+4]&0xff,data_buffer[(i*32)+5]&0xff,data_buffer[(i*32)+6]&0xff,data_buffer[(i*32)+7]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+8]&0xff,data_buffer[(i*32)+9]&0xff,data_buffer[(i*32)+10]&0xff,data_buffer[(i*32)+11]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+12]&0xff,data_buffer[(i*32)+13]&0xff,data_buffer[(i*32)+14]&0xff,data_buffer[(i*32)+15]&0xff); 
							printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+16]&0xff,data_buffer[(i*32)+17]&0xff,data_buffer[(i*32)+18]&0xff,data_buffer[(i*32)+19]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+20]&0xff,data_buffer[(i*32)+21]&0xff,data_buffer[(i*32)+22]&0xff,data_buffer[(i*32)+23]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+24]&0xff,data_buffer[(i*32)+25]&0xff,data_buffer[(i*32)+26]&0xff,data_buffer[(i*32)+27]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+28]&0xff,data_buffer[(i*32)+29]&0xff,data_buffer[(i*32)+30]&0xff,data_buffer[(i*32)+31]&0xff); 
						}
						
						start_addr[0]= 0x00;
						start_addr[1]= 0x31;
						start_addr[2]= 0x00;
						byte_lenght = 256;
						
						page_program(start_addr,data_buffer,byte_lenght,CFG_ROM_BASE,CS_CFG_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}

				case 41:
					{
						printf("-- ========================================\n");
						printf("--     Program and verify with open SSL    \n");		
						printf("--  			 KEY0..15                  \n");		
						printf("-- ========================================\n");
						
						
						uint8_t iv[32] = "\x01\x23\x45\x67\x89\x01\x23\x45\x67\x89\x01\x23\x45\x67\x89\x00";
						uint8_t key[32] = "\x4f\x63\x8c\x73\x5f\x61\x43\x01\x56\x78\x24\xb1\xa2\x1a\x4f\x6a";
						//uint8_t key[32] = "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f";
  						 
						uint8_t *dst_ssl = (uint8_t *)malloc(sizeSecsBuffer);
						uint8_t *src = (uint8_t *)malloc(sizeSecsBuffer);
						uint8_t *data_enc_key_i = (uint8_t *)malloc(sizeSecsBuffer);	
						uint32_t cmp_err = 0;
						
						aes_size_t aes_size[AES_SIZE_MODES] = { AES_128, AES_256 };
						aes_mode_t aes_mode[AES_SIZE_MODES] = { AES_ECB, AES_CBC };
						
						char key_file_name[]="../git/libsecuress/bin/x64/original_key";
						int32_t key_file;
						uint32_t size;
						uint32_t key_idx;
							
							if (key_file_name) {
								key_file = open(key_file_name, O_RDONLY);						
										
								if (key_file >= 0) 
									{					
										size = read(key_file, key_p, sizeSecsBuffer);
										if (size == 0)
										{
											printf("FAILURE: file  %s. is empty\n", key_file_name);								
										};
										close(key_file);
										
										//printf("\nPROGRAM DONE\n");									
										exit=0;	
									}								
								else 
									{			
										printf("FAILURE: error opening %s. Abort\n", key_file_name);								
										exit=-1;
									}
							}
							else {
								printf("FAILURE: invalid original key file name. Abort\n");								
								exit=-1;
							}
						#if _DEBUG	
							{	
								for (int j = 0; j<16; j++)
									{
									printf("\nla chiave %d è: ",j);	
									for (int i = (32*j); i<(32*(j+1)); i++)
										{
												printf("%2.2x",key_p[i]);	
										}
									}
							}; 
						#endif 	
						
						for (key_idx = 0; key_idx<16; key_idx++)
						{ 	
							sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k %d -i ./../git/libsecuress/bin/x64/original_key -o ./../git/libsecuress/bin/x64/key_enc%d > /dev/null", key_idx, key_idx);
							system_return = system(path);
							if (system_return != 0)
							{
								printf("system error path %s",path);	
							}
						}	
			
						char enc_key_file_name[]="./../git/libsecuress/bin/x64/key_enc0 ";
						for (key_idx = 0; key_idx<16; key_idx++)
						{    
							printf("\nKey%d: ",key_idx);	
							memcpy(key+16, ((uint8_t*)key_p)+32*key_idx, 16);
							memcpy(key, (((uint8_t*)key_p)+32*key_idx)+16, 16);
								
							#if _DEBUG	
								for (int i = 0; i<32; i++)
								{
									printf("%2.2x",key[i]);	
								}
							#endif 
  						
							// Save a copy for later use
							memcpy(src, key_p, sizeSecsBuffer);
							//for (int size_idx = 0; size_idx<AES_SIZE_MODES; size_idx++)
							for (int size_idx = 0; size_idx<1; size_idx++)
							{
								//for (int mode_idx = 0; mode_idx < AES_SIZE_MODES; mode_idx++)
								for (int mode_idx = 0; mode_idx < 1; mode_idx++)
								{
									// Encrypt data using standard OpenSSL functions
									openSSLEnc(key, iv, aes_size[size_idx], aes_mode[mode_idx], dst_ssl, src, sizeSecsBuffer);
								}
							}
							
							sprintf(enc_key_file_name,"./../git/libsecuress/bin/x64/key_enc%d",key_idx);
							
							
							int32_t enc_key_file;	
							
								cmp_err = 0;		
								if (enc_key_file_name) 
									{
										enc_key_file = open(enc_key_file_name, O_RDONLY);//O_CREAT | O_WRONLY | O_TRUNC, 0777);		
										
										if (enc_key_file >= 0) 
											{
												size = read(enc_key_file, data_enc_key_i, sizeSecsBuffer);	
												if (size == 0)
												{
													printf("FAILURE: file  %s. is empty\n", enc_key_file_name);								
												};
												close(enc_key_file); 
																					
												exit=0;
											}
										else 
											{			
												printf("FAILURE: error opening %s. Abort\n", enc_key_file_name);								
												exit=-1;
											}	
									}
								else 
									{
										printf("FAILURE: invalid key file name. Abort\n");								
										exit=-1;
									}
									
								printf("\nCheck encryption with key%d\n",key_idx);	
								for (int j = 0; j<16; j++)
										{	
										for (int i = (32*j); i<(32*(j+1)); i++)
											{
												if (dst_ssl[i]!=data_enc_key_i[i])
													{
														printf("error @ byte %d. Expected encrypted data: %2.2x Got encrypted data %2.2x\n",i,dst_ssl[i],data_enc_key_i[i]);
														cmp_err++;	
													}
											}
										}
								if (cmp_err==0) 
									{ 
										printf("Key%d verification PASSED",key_idx);
									}
								else 
									{ 
										printf("Key%d verification FAIL",key_idx);
										cmp_err=0;
									}
								
							#if _DEBUG	
								{	
									for (int j = 0; j<16; j++)
										{
										printf("\nla riga cifrata %d è: ",j);	
										for (int i = (32*j); i<(32*(j+1)); i++)
											{
												printf("%2.2x",dst_ssl[i]);	
											}
										}
								}; 
								
								{	
									for (int j = 0; j<16; j++)
										{
										printf("\nla riga cifrata expected %d è: ",j);	
										for (int i = (32*j); i<(32*(j+1)); i++)
											{
												printf("%2.2x",data_enc_key_i[i]);	
											}
										}
								}; 
							#endif 
							
						};
						free(dst_ssl);			
						free(src);
						free(data_enc_key_i);
						
						wait_to_continue();
						break;
					}
					
				case 42:
					{
						printf("-- ========================================\n");
						printf("--    Program AES Keys in FPGA FLASH ROM   \n");		
						printf("--         Sector 1023 0x3FF000            \n");		
						printf("-- ========================================\n");
						
						data_write[0]=0x01;
						IOWr(pcie_bar_secs[1] + 0x2B, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x2B, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n AES mode register: %2.2x",data_read[0]); 
						
						sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0 -i ./../git/libsecuress/bin/x64/original_key -o ./../git/libsecuress/bin/x64/key_enc > /dev/null");
						sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0 -i ./../git/libsecuress/bin/x64/original_key -o ./../git/libsecuress/bin/x64/key_enc -ow> /dev/null");
						system_return = system(path);
						if (system_return != 0)
						{
							printf("system error path %s",path);	
						}
						
						
						data_write[0]=0x00;
						IOWr(pcie_bar_secs[1] + 0x2B, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x2B, data_read, 1,1, NO_PRINT_VALUES);
						printf("\n AES mode register: %2.2x",data_read[0]); 
						
						for(i = 0; i < 8; i++)
						{
							printf("\n == AES KEY%d ==",i); 
							MRd32(mem_addr_secs, data_buffer, 256, 4, NO_PRINT_VALUES); 
							printf("\n Key 255..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+0]&0xff,data_buffer[(i*32)+1]&0xff,data_buffer[(i*32)+2]&0xff,data_buffer[(i*32)+3]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+4]&0xff,data_buffer[(i*32)+5]&0xff,data_buffer[(i*32)+6]&0xff,data_buffer[(i*32)+7]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+8]&0xff,data_buffer[(i*32)+9]&0xff,data_buffer[(i*32)+10]&0xff,data_buffer[(i*32)+11]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+12]&0xff,data_buffer[(i*32)+13]&0xff,data_buffer[(i*32)+14]&0xff,data_buffer[(i*32)+15]&0xff); 
							printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+16]&0xff,data_buffer[(i*32)+17]&0xff,data_buffer[(i*32)+18]&0xff,data_buffer[(i*32)+19]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+20]&0xff,data_buffer[(i*32)+21]&0xff,data_buffer[(i*32)+22]&0xff,data_buffer[(i*32)+23]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+24]&0xff,data_buffer[(i*32)+25]&0xff,data_buffer[(i*32)+26]&0xff,data_buffer[(i*32)+27]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+28]&0xff,data_buffer[(i*32)+29]&0xff,data_buffer[(i*32)+30]&0xff,data_buffer[(i*32)+31]&0xff); 
						}
						
						// ** Program first 256 bytes fourth sector 0x3FF000
						start_addr[0]= 0x00;
						start_addr[1]= 0xF0;
						start_addr[2]= 0x3F;
						// ** Erase fourth sector 0x3FF000
						sector_erase(start_addr,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,NO_PRINT_VALUES);
						byte_lenght = 256;
						
						start_addr[0]= 0x00;
						start_addr[1]= 0xF0;
						start_addr[2]= 0x3F;
						byte_lenght = 256;
						
						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						for(i = 0; i < 8; i++)
						{
							printf("\n == AES KEY%d ==",i+8); 
							MRd32(mem_addr_secs+256, data_buffer, 256, 4, NO_PRINT_VALUES); 
							printf("\n Key 255..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+0]&0xff,data_buffer[(i*32)+1]&0xff,data_buffer[(i*32)+2]&0xff,data_buffer[(i*32)+3]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+4]&0xff,data_buffer[(i*32)+5]&0xff,data_buffer[(i*32)+6]&0xff,data_buffer[(i*32)+7]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+8]&0xff,data_buffer[(i*32)+9]&0xff,data_buffer[(i*32)+10]&0xff,data_buffer[(i*32)+11]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+12]&0xff,data_buffer[(i*32)+13]&0xff,data_buffer[(i*32)+14]&0xff,data_buffer[(i*32)+15]&0xff); 
							printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+16]&0xff,data_buffer[(i*32)+17]&0xff,data_buffer[(i*32)+18]&0xff,data_buffer[(i*32)+19]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+20]&0xff,data_buffer[(i*32)+21]&0xff,data_buffer[(i*32)+22]&0xff,data_buffer[(i*32)+23]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+24]&0xff,data_buffer[(i*32)+25]&0xff,data_buffer[(i*32)+26]&0xff,data_buffer[(i*32)+27]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+28]&0xff,data_buffer[(i*32)+29]&0xff,data_buffer[(i*32)+30]&0xff,data_buffer[(i*32)+31]&0xff); 
						}
						
						start_addr[0]= 0x00;
						start_addr[1]= 0xF1;
						start_addr[2]= 0x3F;
						byte_lenght = 256;
						
						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
					}
				case 43:
					{	
						data_write[0] = 0x01;
						IOWr(pcie_bar_io[0] + 0x24, data_write, 1, 1, 0, NO_PRINT_VALUES);
						data_write[0] = 0x00;
						IOWr(pcie_bar_io[0] + 0x24, data_write, 1, 1, 0, NO_PRINT_VALUES);
						
						wait_to_continue();
						break;
						return 0;
					} 
					
					
				case 44:
					{
						printf("-- ========================================\n");
						printf("--    Program AES Keys in FPGA FLASH ROM   \n");		
						printf("--         Sector 1023 0x3FF000            \n");		
						printf("-- ========================================\n");
						
						// temp
						IORd(pcie_bar_secs[1] + 0x23, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n\n\n ======================================= \t");
						printf("\n =========== Debug Part =========== \t");
						printf("\n\n == Master Key Ready flag == \t %s",MASTER_KEY_FROM_LP((data_read[0]>>7)&0x01));
						printf("\n == Autentication Status == \t %s",AUTHENTICA((data_read[0]>>6)&0x01));
						printf("\n\n ======================================= \t");
						
						
						printf("\n\n\n ======================================= \t");
						printf("\n == Master Key RECOVERED from LP =="); 
						IORd(pcie_bar_secs[1] + 0x6C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_secs[1] + 0x68, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						IORd(pcie_bar_secs[1] + 0x64, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						IORd(pcie_bar_secs[1] + 0x60, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t");
						
						printf("\n\n\n ======================================= \t");
						printf("\n == Master Key CRYPTED from LP =="); 
						IORd(pcie_bar_secs[1] + 0x7C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_secs[1] + 0x78, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						IORd(pcie_bar_secs[1] + 0x74, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						IORd(pcie_bar_secs[1] + 0x70, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n\n");
						
						printf("\n\n\n ======================================= \t");
						printf("\n == Master Key INJECTED to LP =="); 
						IORd(pcie_bar_secs[1] + 0x8C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_secs[1] + 0x88, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						IORd(pcie_bar_secs[1] + 0x84, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						IORd(pcie_bar_secs[1] + 0x80, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n\n");
						
						////
						data_write[0]=0x02;
						IOWr(pcie_bar_secs[1] + 0x2B, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x2B, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n AES mode register: %2.2x",data_read[0]);
						
												
						sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0 -i ./../git/libsecuress/bin/x64/original_key -o ./../git/libsecuress/bin/x64/key_enc > /dev/null");
						sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0 -i ./../git/libsecuress/bin/x64/original_key -o ./../git/libsecuress/bin/x64/key_enc -ow> /dev/null");
						
						system_return = system(path);
						if (system_return != 0)
						{
							printf("system error path %s",path);	
						}
						
						
						data_write[0]=0x00;
						IOWr(pcie_bar_secs[1] + 0x2B, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x2B, data_read, 1,1, NO_PRINT_VALUES);
						printf("\n AES mode register: %2.2x",data_read[0]); 
						
						for(i = 0; i < 8; i++)
						{
							printf("\n == AES KEY%d ==",i); 
							MRd32(mem_addr_secs, data_buffer, 256, 4, NO_PRINT_VALUES); 
							printf("\n Key 255..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+0]&0xff,data_buffer[(i*32)+1]&0xff,data_buffer[(i*32)+2]&0xff,data_buffer[(i*32)+3]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+4]&0xff,data_buffer[(i*32)+5]&0xff,data_buffer[(i*32)+6]&0xff,data_buffer[(i*32)+7]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+8]&0xff,data_buffer[(i*32)+9]&0xff,data_buffer[(i*32)+10]&0xff,data_buffer[(i*32)+11]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+12]&0xff,data_buffer[(i*32)+13]&0xff,data_buffer[(i*32)+14]&0xff,data_buffer[(i*32)+15]&0xff); 
							printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+16]&0xff,data_buffer[(i*32)+17]&0xff,data_buffer[(i*32)+18]&0xff,data_buffer[(i*32)+19]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+20]&0xff,data_buffer[(i*32)+21]&0xff,data_buffer[(i*32)+22]&0xff,data_buffer[(i*32)+23]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+24]&0xff,data_buffer[(i*32)+25]&0xff,data_buffer[(i*32)+26]&0xff,data_buffer[(i*32)+27]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+28]&0xff,data_buffer[(i*32)+29]&0xff,data_buffer[(i*32)+30]&0xff,data_buffer[(i*32)+31]&0xff); 
						}
						
						// ** Program first 256 bytes fourth sector 0x3FF000
						start_addr[0]= 0x00;
						start_addr[1]= 0xF0;
						start_addr[2]= 0x3F;
						// ** Erase fourth sector 0x3FF000
						sector_erase(start_addr,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,NO_PRINT_VALUES);
						byte_lenght = 256;
						
						start_addr[0]= 0x00;
						start_addr[1]= 0xF0;
						start_addr[2]= 0x3F;
						byte_lenght = 256;
						
						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						for(i = 0; i < 8; i++)
						{
							printf("\n == AES KEY%d ==",i+8); 
							MRd32(mem_addr_secs+256, data_buffer, 256, 4, NO_PRINT_VALUES); 
							printf("\n Key 255..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+0]&0xff,data_buffer[(i*32)+1]&0xff,data_buffer[(i*32)+2]&0xff,data_buffer[(i*32)+3]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+4]&0xff,data_buffer[(i*32)+5]&0xff,data_buffer[(i*32)+6]&0xff,data_buffer[(i*32)+7]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+8]&0xff,data_buffer[(i*32)+9]&0xff,data_buffer[(i*32)+10]&0xff,data_buffer[(i*32)+11]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+12]&0xff,data_buffer[(i*32)+13]&0xff,data_buffer[(i*32)+14]&0xff,data_buffer[(i*32)+15]&0xff); 
							printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+16]&0xff,data_buffer[(i*32)+17]&0xff,data_buffer[(i*32)+18]&0xff,data_buffer[(i*32)+19]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+20]&0xff,data_buffer[(i*32)+21]&0xff,data_buffer[(i*32)+22]&0xff,data_buffer[(i*32)+23]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+24]&0xff,data_buffer[(i*32)+25]&0xff,data_buffer[(i*32)+26]&0xff,data_buffer[(i*32)+27]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[(i*32)+28]&0xff,data_buffer[(i*32)+29]&0xff,data_buffer[(i*32)+30]&0xff,data_buffer[(i*32)+31]&0xff); 
						}
						
						start_addr[0]= 0x00;
						start_addr[1]= 0xF1;
						start_addr[2]= 0x3F;
						byte_lenght = 256;
						
						page_program(start_addr,data_buffer,byte_lenght,FPGA_ROM_BASE,CS_FPGA_ROM_ENA,PRINT_VALUES);
						
						wait_to_continue();
						break;
						return 0;
						
					}
					
				case 45:
					{	
						data_write[0] = 0x04;
						IOWr(pcie_bar_io[0] + 0x23, data_write, 1, 1, 0, NO_PRINT_VALUES);
						data_write[0] = 0x00;
						IOWr(pcie_bar_io[0] + 0x23, data_write, 1, 1, 0, NO_PRINT_VALUES);
						
						wait_to_continue();
						break;
						return 0;
					} 
				
				case 46:
					{
						printf("-- ========================================\n");
						printf("--       Program Master Key injection      \n");		
						printf("--                                         \n");		
						printf("-- ========================================\n");
						
						IORd(pcie_bar_secs[1] + 0x23, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n\n\n ======================================= \t");
						printf("\n =========== Debug Part =========== \t");
						printf("\n\n == Master Key Ready flag == \t %s",MASTER_KEY_FROM_LP((data_read[0]>>7)&0x01));
						printf("\n == Autentication Status == \t %s",AUTHENTICA((data_read[0]>>6)&0x01));
						printf("\n\n ======================================= \t");
						
						data_write[0]=0x81;
						IOWr(pcie_bar_secs[1] + 0x2B, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x2B, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n AES mode register: %2.2x",data_read[0]);
						
											
						sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0 -i ./../git/libsecuress/bin/x64/masterkey -o ./../git/libsecuress/bin/x64/mastkey_enc > /dev/null");
						sprintf (path, "sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0 -i ./../git/libsecuress/bin/x64/masterkey -o ./../git/libsecuress/bin/x64/mastkey_enc -ow> /dev/null");
						
						system_return = system(path);
						if (system_return != 0)
						{
							printf("system error path %s",path);	
						}
						
						
						data_write[0]=0x00;
						IOWr(pcie_bar_secs[1] + 0x2B, data_write, 1, 1, 0, NO_PRINT_VALUES);
						IORd(pcie_bar_secs[1] + 0x2B, data_read, 1,1, NO_PRINT_VALUES);
						printf("\n AES mode register: %2.2x",data_read[0]); 
						
						printf("\n\n\n ======================================= \t");
						printf("\n == Master Key INJECTED to LP =="); 
						IORd(pcie_bar_secs[1] + 0x8C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_secs[1] + 0x88, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						IORd(pcie_bar_secs[1] + 0x84, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						IORd(pcie_bar_secs[1] + 0x80, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n\n");
						
						
						printf("\n\n\n ======================================= \t");
						printf("\n == unique 17th Key =="); 
						IORd(pcie_bar_secs[1] + 0x9C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_secs[1] + 0x98, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						IORd(pcie_bar_secs[1] + 0x94, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						IORd(pcie_bar_secs[1] + 0x90, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n\n");
						
						
							printf("\n == Readback Memory OTP =="); 
							MRd32(mem_addr_secs, data_buffer, 256, 4, NO_PRINT_VALUES); 
							printf("\n Key 255..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[0]&0xff,data_buffer[1]&0xff,data_buffer[2]&0xff,data_buffer[3]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[4]&0xff,data_buffer[5]&0xff,data_buffer[6]&0xff,data_buffer[7]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[8]&0xff,data_buffer[9]&0xff,data_buffer[10]&0xff,data_buffer[11]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[12]&0xff,data_buffer[13]&0xff,data_buffer[14]&0xff,data_buffer[15]&0xff); 
							printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_buffer[16]&0xff,data_buffer[17]&0xff,data_buffer[18]&0xff,data_buffer[19]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[20]&0xff,data_buffer[21]&0xff,data_buffer[22]&0xff,data_buffer[23]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[24]&0xff,data_buffer[25]&0xff,data_buffer[26]&0xff,data_buffer[27]&0xff); 
							printf(".%2.2x%2.2x%2.2x%2.2x",data_buffer[28]&0xff,data_buffer[29]&0xff,data_buffer[30]&0xff,data_buffer[31]&0xff); 
						
						wait_to_continue();
						break;
						return 0;
						
					}
					
				case 47:
					{
						printf("-- ========================================\n");
						printf("--       Get all AES debug registers       \n");		
						printf("--                                         \n");		
						printf("-- ========================================\n");
						
						IORd(pcie_bar_secs[1] + 0x23, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n\n\n ======================================= \t");
						printf("\n =========== Debug Part =========== \t");
						printf("\n\n == Master Key Ready flag == \t %s",MASTER_KEY_FROM_LP((data_read[0]>>7)&0x01));
						printf("\n == Autentication Status == \t %s",AUTHENTICA((data_read[0]>>6)&0x01));
						printf("\n\n ======================================= \t");
										
						printf("\n\n\n ======================================= \t");
						printf("\n == unique 17th Key =="); 
						IORd(pcie_bar_secs[1] + 0x9C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_secs[1] + 0x98, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						IORd(pcie_bar_secs[1] + 0x94, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						IORd(pcie_bar_secs[1] + 0x90, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n\n");
																
						printf("\n\n\n ======================================= \t");
						printf("\n == MAster Key from LP =="); 
						IORd(pcie_bar_secs[1] + 0x7C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_secs[1] + 0x78, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						IORd(pcie_bar_secs[1] + 0x74, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						IORd(pcie_bar_secs[1] + 0x70, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n\n");
																
						printf("\n\n\n ======================================= \t");
						printf("\n == MAster Key original =="); 
						IORd(pcie_bar_secs[1] + 0x6C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_secs[1] + 0x68, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						IORd(pcie_bar_secs[1] + 0x64, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						IORd(pcie_bar_secs[1] + 0x60, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n\n");
						
						printf("\n\n\n ======================================= \t");
						printf("\n == DW0 KEY0 =="); 
						IORd(pcie_bar_secs[1] + 0x0, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 31..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						//printf("\n ======================================= \t\n\n");
						printf("\n == DW0 KEY1 =="); 
						IORd(pcie_bar_secs[1] + 0x4, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 63..32:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						//printf("\n ======================================= \t\n\n");
						printf("\n == DW0 KEY2 =="); 
						IORd(pcie_bar_secs[1] + 0x8, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 95..64:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						//printf("\n ======================================= \t\n\n");
						printf("\n == DW0 KEY3 =="); 
						IORd(pcie_bar_secs[1] + 0xC, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..96:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						//printf("\n ======================================= \t\n\n");
						printf("\n == DW0 KEY4 =="); 
						IORd(pcie_bar_secs[1] + 0x10, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 159..128:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						//printf("\n ======================================= \t\n\n");
						printf("\n == DW0 KEY5 =="); 
						IORd(pcie_bar_secs[1] + 0x14, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 191..160:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						//printf("\n ======================================= \t\n\n");
						printf("\n == DW0 KEY6 =="); 
						IORd(pcie_bar_secs[1] + 0x18, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 223..192:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						//printf("\n ======================================= \t\n\n");
						printf("\n == DW0 KEY7 =="); 
						IORd(pcie_bar_secs[1] + 0x1C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 255..224:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						
						wait_to_continue();
						break;
						return 0;
						
					}
				case 50: 
					{	
						check_id_error=0;
						//  ** Setup JTAG MAX frequency **
						data_write[1]=0x00;
						data_write[0]=F_1MHz;
						printf("Setup JTAG MAX frequency 1MHz\n");
						IOWr(pcie_bar_sbc[1] + 0x84, data_write, 1, 1, 0,NO_PRINT_VALUES);  
						IOWr(pcie_bar_sbc[1] + 0x85, data_write, 1, 1, 1,NO_PRINT_VALUES);  
						IORd(pcie_bar_sbc[1] + 0x84, data_read, 2, 2, NO_PRINT_VALUES);
						printf("Setup frequency value %2.2x%2.2x\n",data_read[1],data_read[0]); 
						
						printf("\n================\n");	
						printf("== JTAG RESET ==\n");	
						printf("================\n");
						data_write[0]=0x00;
						TAP (mem_addr_sbc, data_write, 0, TEST_LOGIC_RESET, RUN_TEST_IDLE, NO_PRINT_VALUES);
						
						printf("\n** From IDLE to Pause IR **\n");	
						TAP (mem_addr_sbc, data_write, 0, EXIT1_IR, PAUSE_IR, NO_PRINT_VALUES);	
						
						
						printf("\n===============================\n"); 
						printf("==      Check the IDCODE     ==\n");   
						printf("== Shift in IDCODE_PUB(0xE0) ==\n");   
						printf("===============================\n");
						
						/** SIR 8 E0 **/	
						data_write[0]=0xE0;		
						SIR (mem_addr_sbc, data_write, 8, NO_PRINT_VALUES);
						
						/**		SDR 32    **
						** TDO=0x01113043 **/
						data_write[0]=0x00;	
						data_write[1]=0x00;	
						data_write[2]=0x00;	
						data_write[3]=0x00;	
						SDR (mem_addr_sbc, data_write, 32, NO_PRINT_VALUES);

						/**	readback TD0=0x01113043 **/
						printf("\n** readback TD0=0x01113043 **\n"); 
						MRd32(mem_addr_sbc, data_read, 1, 1, 1);//NO_PRINT_VALUES);  
						MRd32(mem_addr_sbc + 0x04, data_read, 1, 1, 1);//NO_PRINT_VALUES);  
						MRd32(mem_addr_sbc + 0x08, data_read, 1, 1, 1);//NO_PRINT_VALUES);  
						MRd32(mem_addr_sbc + 0x0C, data_read, 1, 1, 1);//NO_PRINT_VALUES); 
						
						
						printf("\n===============================\n"); 
						printf("==      Check the TRACE ID   ==\n");   
						printf("== Shift in IDCODE_PUB(0x19) ==\n");   
						printf("===============================\n");
						
						/** SIR 8 E0 **/	
						data_write[0]=0x19;		
						SIR (mem_addr_sbc, data_write, 8, NO_PRINT_VALUES);
						
						/**		SDR 64    **
						** TDO=0xTRACE ID **/
						data_write[0]=0x00;	data_write[1]=0x00;	data_write[2]=0x00;	data_write[3]=0x00;	
						data_write[4]=0x00;	data_write[5]=0x00;	data_write[6]=0x00;	data_write[7]=0x00;	
						SDR (mem_addr_sbc, data_write, 64, NO_PRINT_VALUES);

						/**	readback TD0=0xTRACE ID **/
						trace_id[0]=0x00; trace_id[1]=0x00; trace_id[2]=0x00; trace_id[3]=0x00;
						trace_id[4]=0x00; trace_id[5]=0x00; trace_id[6]=0x00; trace_id[7]=0x00;
						printf("\n** readback TD0=TRACE ID **\n"); 
						MRd32(mem_addr_sbc, data_read, 1, 1, NO_PRINT_VALUES);  
						trace_id[0]=data_read[0];
						MRd32(mem_addr_sbc + 0x04, data_read, 1, 1, NO_PRINT_VALUES);
						trace_id[1]=data_read[0];  
						MRd32(mem_addr_sbc + 0x08, data_read, 1, 1, NO_PRINT_VALUES);  
						trace_id[2]=data_read[0];
						MRd32(mem_addr_sbc + 0x0C, data_read, 1, 1, NO_PRINT_VALUES); 
						trace_id[3]=data_read[0];
						MRd32(mem_addr_sbc + 0x10, data_read, 1, 1, NO_PRINT_VALUES); 
						trace_id[4]=data_read[0];
						MRd32(mem_addr_sbc + 0x14, data_read, 1, 1, NO_PRINT_VALUES); 
						trace_id[5]=data_read[0];
						MRd32(mem_addr_sbc + 0x18, data_read, 1, 1, NO_PRINT_VALUES); 
						trace_id[6]=data_read[0];
						MRd32(mem_addr_sbc + 0x1C, data_read, 1, 1, NO_PRINT_VALUES); 
						trace_id[7]=data_read[0];
						//MRd32(mem_addr_sbc, trace_id, 1, 1, NO_PRINT_VALUES);  
						//MRd32(mem_addr_sbc + 0x04, trace_id, 1, 1, NO_PRINT_VALUES);
						if ((trace_id[0]==0x13) && (trace_id[1]==0x2c) && (trace_id[2]==0x18) && (trace_id[3]==0x47) && (trace_id[4]==0x18) && (trace_id[5]==0x1c) && (trace_id[6]==0x1b) && (trace_id[7]==0x00))
						{
							printf("trace id 0x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x",trace_id[7],trace_id[6],trace_id[5],trace_id[4],trace_id[3],trace_id[2],trace_id[1],trace_id[0]);
						}  
						else
						{
							check_id_error++; 
							printf("Errore numero %d . Expected trace id value 0x1b1c1847182c1300 got 0x %2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x",check_id_error,trace_id[7],trace_id[6],trace_id[5],trace_id[4],trace_id[3],trace_id[2],trace_id[1],trace_id[0]);
						}
								
						wait_to_continue();										
						break;
					}	
					
				case 51: 
					{		
						check_id_error=0;
						//  ** Setup JTAG MAX frequency **
						data_write[1]=0x00;
						data_write[0]=F_1MHz;
						
						//uint8_t c_trace_id[8]=    {0x10,0x16,0x4c,0x20,0x76,0x81,0x1b,0x00};
						uint8_t c_expanded_key[16]= {0xc8,0x2e,0x07,0x14,0xb2,0xaa,0xbd,0x01,0x13,0x3c,0x3e,0x92,0x69,0xb8,0x84,0x87};
						
						printf("\n================\n");	
						printf("== JTAG RESET ==\n");	
						printf("================\n");
							
						
						printf("\n===============================\n"); 
						printf("==      Check the TRACE ID   ==\n");   
						printf("== Shift in IDCODE_PUB(0x19) ==\n");   
						printf("===============================\n");
						
						IORd(pcie_bar_io[0] + 0x80, data_read, 4, 4, 0); 
						printf("\nreg0: %2.2x\n",(data_read[0]&0xff)); 
						printf("\nBusy: %d Tap End Interrupt: %d\n",(data_read[0]&0x02),(data_read[0]&0x01)); 
						printf("\nBitCount %2.2x\n",(data_read[1]&0xff)); 
						printf("\nStateIn & Current_State %2.2x\n",(data_read[2]&0xff)); 
						printf("\nStateJTAGMaster & TMSState %2.2x\n",(data_read[3]&0xff));
						IORd(pcie_bar_io[0] + 0x84, data_read, 2, 2, 0); 
						printf("\nFreq divider %2.2x%2.2x\n",(data_read[1]&0xff),(data_read[0]&0xff)); 
														
							//usleep(1*100*1000);
							/**	readback TD0=0xTRACE ID **/
							expanded_key[0]=0x00; expanded_key[1]=0x00; expanded_key[2]=0x00; expanded_key[3]=0x00;
							expanded_key[4]=0x00; expanded_key[5]=0x00; expanded_key[6]=0x00; expanded_key[7]=0x00;
							//printf("\n** readback TD0=TRACE ID **\n"); 
							IORd(pcie_bar_io[0] + 0x88, data_read, 1, 1, NO_PRINT_VALUES);  
							expanded_key[0]=data_read[0];
							IORd(pcie_bar_io[0] + 0x89, data_read, 1, 1, NO_PRINT_VALUES);  
							expanded_key[1]=data_read[0];  
							IORd(pcie_bar_io[0] + 0x8A, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[2]=data_read[0];
							IORd(pcie_bar_io[0] + 0x8B, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[3]=data_read[0];
							IORd(pcie_bar_io[0] + 0x8C, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[4]=data_read[0];
							IORd(pcie_bar_io[0] + 0x8D, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[5]=data_read[0];
							IORd(pcie_bar_io[0] + 0x8E, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[6]=data_read[0];
							IORd(pcie_bar_io[0] + 0x8F, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[7]=data_read[0]; 
							IORd(pcie_bar_io[0] + 0x90, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[8]=data_read[0]; 
							IORd(pcie_bar_io[0] + 0x91, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[9]=data_read[0]; 
							IORd(pcie_bar_io[0] + 0x92, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[10]=data_read[0]; 
							IORd(pcie_bar_io[0] + 0x93, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[11]=data_read[0]; 
							IORd(pcie_bar_io[0] + 0x94, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[12]=data_read[0]; 
							IORd(pcie_bar_io[0] + 0x95, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[13]=data_read[0]; 
							IORd(pcie_bar_io[0] + 0x96, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[14]=data_read[0]; 
							IORd(pcie_bar_io[0] + 0x97, data_read, 1, 1, NO_PRINT_VALUES);
							expanded_key[15]=data_read[0]; 
							if ((expanded_key[0]==c_expanded_key[0]) && (expanded_key[1]==c_expanded_key[1]) && (expanded_key[2]==c_expanded_key[2]) && (expanded_key[3]==c_expanded_key[3]) && (expanded_key[4]==c_expanded_key[4]) && (expanded_key[5]==c_expanded_key[5]) && (expanded_key[6]==c_expanded_key[6]) && (expanded_key[7]==c_expanded_key[7])) 
							{
								printf("Trace ID read correctly Expected expanded key value 0x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x got 0x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x",c_expanded_key[15],c_expanded_key[14],c_expanded_key[13],c_expanded_key[12],c_expanded_key[11],c_expanded_key[10],c_expanded_key[9],c_expanded_key[8],c_expanded_key[7],c_expanded_key[6],c_expanded_key[5],c_expanded_key[4],c_expanded_key[3],c_expanded_key[2],c_expanded_key[1],c_expanded_key[0],expanded_key[15],expanded_key[14],expanded_key[13],expanded_key[12],expanded_key[11],expanded_key[10],expanded_key[9],expanded_key[8],expanded_key[7],expanded_key[6],expanded_key[5],expanded_key[4],expanded_key[3],expanded_key[2],expanded_key[1],expanded_key[0]);
				
							}  
							else
							{
								check_id_error++; 
								printf("Errore numero %d . Expected  expanded key value 0x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x got 0x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x",check_id_error,c_expanded_key[15],c_expanded_key[14],c_expanded_key[13],c_expanded_key[12],c_expanded_key[11],c_expanded_key[10],c_expanded_key[9],c_expanded_key[8],c_expanded_key[7],c_expanded_key[6],c_expanded_key[5],c_expanded_key[4],c_expanded_key[3],c_expanded_key[2],c_expanded_key[1],c_expanded_key[0],expanded_key[15],expanded_key[14],expanded_key[13],expanded_key[12],expanded_key[11],expanded_key[10],expanded_key[9],expanded_key[8],expanded_key[7],expanded_key[6],expanded_key[5],expanded_key[4],expanded_key[3],expanded_key[2],expanded_key[1],expanded_key[0]);
							}
								
						wait_to_continue();										
						break;
					}	
					
				case 52 : 
					{
						int port_index = 0;
						data_write[0]=0x02;
						//IOWr(pcie_bar_io[0] + 0x1F, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						IOWr(pcie_bar_muart[0] + 0x24, data_write, 1, 1, 0, 1);//NO_PRINT_VALUES); 
			
						for(port_index = 0; port_index < 8; port_index++)
						{
							int reg_index;
							printf("**************************\nCOM%d registers (READ):\n", port_index+1);
							for(reg_index = 0; reg_index < 8; reg_index++)
							{
								memset(data_read, 0, 1);
								IORd(pcie_bar_muart[0] + ((port_index * 8) + reg_index), data_read, 1, 1, NO_PRINT_VALUES); 
								printf("REG%d: 0x%02X\n", reg_index, data_read[0]);

							}
						}
							
						wait_to_continue();
						break;
						return 0;
					}  
					
				case 60 : 
					{
						#if _DEBUG	
						{	
							printf("\n\n === max ram size %d ===",nvramMaxSize);
							printf("\n\n === max ram size %d===",pcie_bar_size_mem[0]);
						}; 
						#endif 
						
						
						printf("\n\n");
						IORd(pcie_bar_mem[1] + 0x14, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ===================================");
						printf("\n === SRAM MODULE (HW parameters) ===");
						printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);
						}; 
						#endif 
						data_write[0]=0x01;
						IOWr(pcie_bar_mem[1] + 0x0C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						// enable all bank
						data_write[0]=0x0F;
						IOWr(pcie_bar_mem[1] + 0x0D, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);//NO_PRINT_VALUES);
						}; 						
						#endif 
						
						uint32_t sram_access_type;
						uint32_t number_of_iteration;
						
						printf("\n\n Insert Type SRAM acces 1 = byte, 2 = word, 4 = dword, 8 = qword: ");
						if ((ret_code=scanf("%d", &sram_access_type))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						switch(sram_access_type)
						{
							case 1:
							case 2:
							case 4:
							case 8:
							{	
								break;
							}
							default:
							{	
								sram_access_type = 8;
								break;
							}  
						} 
						
						printf("Insert number of iteration (0 = infinite time = Icc consumption is chiappa su): ");
						if ((ret_code=scanf("%d", &number_of_iteration))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						uint32_t x;
						uint32_t i;
						uint32_t rpt;
						uint8_t *writeBuffer;
						if (!bufferSize) bufferSize = nvramMaxSize;
						
						writeBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
						printf("Assigning random values to a %u bytes local buffer...\n", bufferSize);
						for (x = 0; x < bufferSize; x++)
							*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);

						printf("Writing local buffer to memory...\n");
						rpt = 0;
						
						
						
						while (rpt < (number_of_iteration+1))
						{	
							i = 0;
							while (i < bufferSize)
								{
									MWr32(mem_addr+i, &writeBuffer[i], sram_access_type, sram_access_type, NO_PRINT_VALUES); 
									i += sram_access_type;		
								}
								
							if (number_of_iteration > 0) rpt++; // check if number of iteration is infinite
								
						}
						IORd(pcie_bar_io[0] + 0x20, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n Authentication status: %s \n",AUTHENTICA(data_read[0]&0x01)); 
						
						free(writeBuffer);
						
						wait_to_continue();
						break;
						return 0;
					}  	
					
				case 61 : 
					{
						#if _DEBUG	
						{	
							printf("\n\n === max ram size %d ===",nvramMaxSize);
							printf("\n\n === max ram size %d===",pcie_bar_size_mem[0]);
						}; 
						#endif 
						
						
						printf("\n\n");
						IORd(pcie_bar_mem[1] + 0x14, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ===================================");
						printf("\n === SRAM MODULE (HW parameters) ===");
						printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);
						}; 
						#endif 
						data_write[0]=0x02;
						IOWr(pcie_bar_mem[1] + 0x0C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						// enable all bank
						data_write[0]=0x0F;
						IOWr(pcie_bar_mem[1] + 0x0D, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);//NO_PRINT_VALUES);
						}; 						
						#endif 
						
						uint32_t sram_access_type;
						uint32_t number_of_iteration;
						uint32_t nchip;
						printf("\n\n Quixant Platform Configuration \n\n");
						printf("   Two SRAM chips   :   Qxi400,Qxi7000,Qxi7000lite,Qxi6000,IQ1,SB7,IQ1-air\n");
						printf("   Four SRAM chips  :   Qx50,Qx60,QMAX-1,Qx70,QMAX-2/A,Qmax3lite\n");
						printf("\n How Many Chips ? [2 = 2 SRAM chip, 4 = 4 SRAM chip] ");
						if ((ret_code=scanf("%d", &nchip))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						switch(nchip)
						{
							case 2:
							case 4:
							{	
								break;
							}
							default:
							{	
								nchip = 4;
								break;
							}  
						} 
							
						printf("\n\n Insert Type SRAM acces 1 = byte, 2 = word, 4 = dword, 8 = qword: ");
						if ((ret_code=scanf("%d", &sram_access_type))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						switch(sram_access_type)
						{
							case 1:
							case 2:
							case 4:
							case 8:
							{	
								break;
							}
							default:
							{	
								sram_access_type = 8;
								break;
							}  
						} 
						
						printf("Insert number of iteration (0 = infinite time = Icc consumption is chiappa su): ");
						if ((ret_code=scanf("%d", &number_of_iteration))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						uint32_t x;
						uint32_t i;
						uint32_t rpt;
						uint8_t *writeBuffer;
						if (!bufferSize) bufferSize = nvramMaxSize/nchip;
						
						writeBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
						printf("Assigning random values to a %u bytes local buffer...\n", bufferSize);
						for (x = 0; x < bufferSize; x++)						
							*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);

						printf("Writing local buffer to memory...\n");
						rpt = 0;
						
						
						
						while (rpt < (number_of_iteration+1))
						{	
							i = 0;
							while (i < (nvramMaxSize/2))
								{
									MWr32(mem_addr+i, &writeBuffer[i], sram_access_type, sram_access_type, NO_PRINT_VALUES); 
									i += sram_access_type;		
								}
								
							if (number_of_iteration > 0) rpt++; // check if number of iteration is infinite
								
						}
						IORd(pcie_bar_io[0] + 0x20, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n Authentication status: %s \n",AUTHENTICA(data_read[0]&0x01)); 
						
						free(writeBuffer);
						
						wait_to_continue();
						break;
						return 0;
					} 
	
				case 62 : 
					{
						#if _DEBUG	
						{	
							printf("\n\n === max ram size %d ===",nvramMaxSize);
							printf("\n\n === max ram size %d===",pcie_bar_size_mem[0]);
						}; 
						#endif 
						
						
						printf("\n\n");
						IORd(pcie_bar_mem[1] + 0x14, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ===================================");
						printf("\n === SRAM MODULE (HW parameters) ===");
						printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);
						}; 
						#endif 
						data_write[0]=NORMAL_MODEx64;
						IOWr(pcie_bar_mem[1] + 0x0C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						// enable all bank
						data_write[0]=0x0F;
						IOWr(pcie_bar_mem[1] + 0x0D, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);//NO_PRINT_VALUES);
						}; 						
						#endif 
						
						uint32_t sram_access_type;
						uint32_t number_of_iteration;
						uint32_t nchip;
						
						printf("\n\n Quixant Platform Configuration \n\n");
						printf("   Two SRAM chips   :   Qxi400,Qxi7000,Qxi7000lite,Qxi6000,IQ1,SB7,IQ1-air\n");
						printf("   Four SRAM chips  :   Qx50,Qx60,QMAX-1,Qx70,QMAX-2/A,Qmax3lite\n");
						printf("\n How Many Chips ? [2 = 2 SRAM chip, 4 = 4 SRAM chip] ");
						if ((ret_code=scanf("%d", &nchip))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						switch(nchip)
						{
							case 2:
							case 4:
							{	
								break;
							}
							default:
							{	
								nchip = 4;
								break;
							}  
						} 
							
						printf("\n\n Insert Type SRAM acces 1 = byte, 2 = word, 4 = dword, 8 = qword: ");
						if ((ret_code=scanf("%d", &sram_access_type))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						switch(sram_access_type)
						{
							case 1:
							case 2:
							case 4:
							case 8:
							{	
								break;
							}
							default:
							{	
								sram_access_type = 8;
								break;
							}  
						} 
						
						printf("Insert number of iteration (0 = infinite time): ");
						if ((ret_code=scanf("%d", &number_of_iteration))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						uint8_t *writeBuffer;
						uint8_t *readBuffer;
						uint32_t sram_test=0;
						
						uint32_t x;
						uint32_t i;
						uint32_t rpt;
						uint32_t j;
						
						// enable all bank
						data_write[0]=0x0F;
						IOWr(pcie_bar_mem[1] + 0x0D, data_write, 1, 1, 0, NO_PRINT_VALUES);
						
						while (sram_test < 4) 
							{
								data_write[0]=NORMAL_MODEx64;
								data_write[1]=NORMAL_MODE;
								data_write[2]=MIRROR_MODEx32;
								data_write[3]=MIRROR_MODE;
								IOWr(pcie_bar_mem[1] + 0x0C, data_write, 1, 1, sram_test, NO_PRINT_VALUES); 
																		
								if (sram_test==0)  
									{
										bufferSize = nvramMaxSize;  
									}	
								else if (sram_test==1)  
									{
										bufferSize = (nvramMaxSize/2);  
									}
								else if (sram_test==2)  
									{
										bufferSize = (nvramMaxSize/2);  
									}
								else  
									{
										bufferSize = (nvramMaxSize/4);  
									}
									
								writeBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
								readBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
								
								
								printf("Assigning random values to a %u bytes local buffer...\n", bufferSize);
								for (x = 0; x < bufferSize; x++)
									//*(writeBuffer + x) = (uint8_t)(x & 0x000000FF);
									*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);
								
																	
								printf("Writing local buffer to memory...\n");
								rpt = 0;
														
								while ((rpt < number_of_iteration) || (number_of_iteration==0))
								{	
									
									#if _DEBUG	
									{	
										printf("\n\n === number of test %d ===",sram_test);
										printf("\n\n === %u bytes buffer ram===", bufferSize);
										printf("\n\n === repetition number %d===",rpt);
										printf("\n\n === request number of iteration %d===\n",number_of_iteration);
									}; 
									#endif 
									i = 0;    //8388608
									while (i < bufferSize)
										{
											MWr32(mem_addr+i, &writeBuffer[i], sram_access_type, sram_access_type, NO_PRINT_VALUES); 
											i += sram_access_type;		
										}
									
									i = 0;
									while (i < bufferSize)
										{
											MRd32(mem_addr+i, &readBuffer[i], sram_access_type, sram_access_type, NO_PRINT_VALUES);
											
											for(j = i; j < (i+sram_access_type); j++)
											{
												//if (j<256) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
												if ((writeBuffer[j]!=readBuffer[j]) && j<256) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
											}
											i += sram_access_type;		
		  
										}
										 
										
		 
									rpt++; // check if number of iteration is infinite
										
								}	
								rpt = 0;
								sram_test++;
							}		
						IORd(pcie_bar_io[0] + 0x20, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n Authentication status: %s \n",AUTHENTICA(data_read[0]&0x01)); 
						
						free(writeBuffer);
						free(readBuffer);
						
						wait_to_continue();
						break;
						return 0;
					}  	

				case 63 : 
					{
						#if _DEBUG	
						{	
							printf("\n\n === max ram size %d ===",nvramMaxSize);
							printf("\n\n === max ram size %d===",pcie_bar_size_mem[0]);
						}; 
						#endif 
						
						printf("\n\n");
						IORd(pcie_bar_mem[1] + 0x14, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ===================================");
						printf("\n === SRAM MODULE (HW parameters) ===");
						printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);
						}; 
						#endif 
						//data_write[0]=NORMAL_MODEx64;
						//IOWr(pcie_bar_mem[1] + 0x0C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						// enable all bank
						data_write[0]=0x0F;
						IOWr(pcie_bar_mem[1] + 0x0D, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);//NO_PRINT_VALUES);
						}; 						
						#endif 
						
						// check Autentication Status			
						IORd(pcie_bar_io[0] + 0x20, data_read, 1, 1, NO_PRINT_VALUES);
						printf("\n Authentication status: %s \n",AUTHENTICA(data_read[0]&0x01)); 
						
						
						// enable all bank
						data_write[0]=0x0F;
						IOWr(pcie_bar_mem[1] + 0x0D, data_write, 1, 1, 0, NO_PRINT_VALUES);
						
						uint32_t sram_waccess_type,sram_raccess_type;
						uint32_t nvram_sel;
						
						uint8_t *writeBuffer;
						uint8_t *readBuffer;
						uint32_t sram_mode_num;
						
						uint32_t x;
						uint32_t i;
						uint32_t test_size;
						uint32_t j;	
						uint32_t data_type;
						
						printf("-- =================================\n");
						printf("--       SRAM command               \n");		
						printf("--  (1) Write Access                \n");
						printf("--  (2) Read Access                 \n");
						printf("--  (3) Change SRAM Mode Access     \n");
						printf("--  (4) Exit                        \n");
						printf("-- =================================\n");
						
						
						printf("Selection [1..4] ? : ");
						if ((ret_code=scanf("%d", &nvram_sel))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						switch(nvram_sel)
						{
							case 1:
								{				
									bufferSize = nvramMaxSize;   // temp	
									
									writeBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
									
									printf("\n\n Insert Type SRAM acces 1 = byte, 2 = word, 4 = dword, 8 = qword: ");
									if ((ret_code=scanf("%d", &sram_waccess_type))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									switch(sram_waccess_type)
									{
										case 1:
										case 2:
										case 4:
										case 8:
										{	
											break;
										}
										default:
										{	
											sram_waccess_type = 8;
											break;
										}  
									} 
									
									printf("\n\n How many byte to write ? [0..%d]: ",bufferSize);
									if ((ret_code=scanf("%d", &test_size))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									// max write byte size is Memory max size
									if (test_size > bufferSize) test_size=bufferSize;
									
									printf("-- =====================\n");
									printf("--     Data Type        \n");		
									printf("--  (1) 0xFF            \n");
									printf("--  (2) 0x00            \n");
									printf("--  (3) Consecutive     \n");
									printf("--  (4) Random          \n");
									printf("-- =====================\n");
						
									printf("\n\n Data Type ? [1..4]: ");
									if ((ret_code=scanf("%d", &data_type))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									switch(data_type)
									{
										case 1:
											{
												printf("Assigning 0xFF values to a %u bytes local buffer...\n", test_size);
												for (x = 0; x < test_size; x++)
												*(writeBuffer + x) = (uint8_t)(0xFF);
												break;
											}
										case 2:
											{
												printf("Assigning 0x00 values to a %u bytes local buffer...\n", test_size);
												for (x = 0; x < test_size; x++)
												*(writeBuffer + x) = (uint8_t)(0x00);
												break;
											}
										case 3:
											{	
												printf("Assigning consecutive values to a %u bytes local buffer...\n", test_size);
												for (x = 0; x < test_size; x++)
												*(writeBuffer + x) = (uint8_t)(x);
												break;
											}  
										case 4:
											{	
												printf("Assigning random values to a %u bytes local buffer...\n", test_size);
												for (x = 0; x < test_size; x++)
												*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);
												break;
											}  
										default:
											{	
												printf("Assigning random values to a %u bytes local buffer...\n", test_size);
												for (x = 0; x < test_size; x++)
												*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);
												break;
											}  
									} 
																		
									#if _DEBUG	
									{	
										printf("\n\n === write access size %d ===",sram_waccess_type);
										printf("\n\n === %u bytes buffer ram===", bufferSize);
										printf("\n\n === %u bytes test size===", test_size);
										printf("\n\n === data type %d===\n",data_type);
									}; 
									#endif 
									
									i = 0;
									while (i < test_size)
										{
											MWr32(mem_addr+i, &writeBuffer[i], sram_waccess_type, sram_waccess_type, NO_PRINT_VALUES); 
											i += sram_waccess_type;		
										}
									
									
								free(writeBuffer);
								break;
								} 
							case 2:
								{
									bufferSize = nvramMaxSize;   // temp
									
									readBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
									
									printf("\n\n Insert Type SRAM acces 1 = byte, 2 = word, 4 = dword, 8 = qword: ");
									if ((ret_code=scanf("%d", &sram_raccess_type))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									switch(sram_raccess_type)
									{
										case 1:
										case 2:
										case 4:
										case 8:
										{	
											break;
										}
										default:
										{	
											sram_raccess_type = 8;
											break;
										}  
									} 
												
									
									printf("\n\n How many byte to read ? [0..%d]: ",bufferSize);
									if ((ret_code=scanf("%d", &test_size))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									// max write byte size is Memory max size
									if (test_size > bufferSize) test_size=bufferSize;
									
									#if 1//_DEBUG	
									{	
										printf("\n\n === read access size %d ===",sram_raccess_type);
										printf("\n\n === %u bytes buffer ram===", bufferSize);
										printf("\n\n === %u bytes test size===", test_size);
										printf("\n\n === data type %d===\n",data_type);
									}; 
									#endif 
								
									i = 0;
									while (i < test_size)
										{
											MRd32(mem_addr+i, &readBuffer[i], sram_raccess_type, sram_raccess_type, 1);//NO_PRINT_VALUES);
									
											/*for(j = i; j < (i+sram_access_type); j++)
											{
												if ((writeBuffer[j]!=readBuffer[j]) && j<256) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
											}*/
											i += sram_raccess_type;		
		  
										}
										 
								free(readBuffer);
								break;
								} 
							case 3:
								{	
									printf("-- =================================\n");
									printf("--       SRAM MODE                  \n");		
									printf("--  (1) NORMAL_MODEx64              \n");
									printf("--  (2) NORMAL_MODE                 \n");
									printf("--  (3) MIRROR_MODEx32              \n");
									printf("--  (4) MIRROR_MODE                 \n");
									printf("-- =================================\n");
						
									printf("\n\n Insert SRAM mode acces [1..4]: ");
									if ((ret_code=scanf("%d", &sram_mode_num))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									data_write[0]=NORMAL_MODEx64;
									data_write[1]=NORMAL_MODE;
									data_write[2]=MIRROR_MODEx32;
									data_write[3]=MIRROR_MODE;
									// write sram mode
									IOWr(pcie_bar_mem[1] + 0x0C, data_write, 1, 1, (sram_mode_num-1), NO_PRINT_VALUES);
								
									switch(sram_mode_num)
									{
										case 1:
											{
												bufferSize = nvramMaxSize; 	
												break;
											}
										case 2:
											{
												bufferSize = (nvramMaxSize/2); 
												break;
											}
										case 3:
											{
												bufferSize = (nvramMaxSize/2); 
												break;
											}
										case 4:
											{	
												bufferSize = (nvramMaxSize/4);  
												break;
											}
										default:
										{	
											break;
										}  
									} 
								break;
								}
							case 4:
								{	
									break;
								}
							default:
								{	
									break;
								}  
						} 						
						
						wait_to_continue();
						break;
						return 0;
					}  	
					
				case 64 : 
					{
						
						printf("-- =======================================\n");
						printf("--       ATS command                      \n");		
						printf("--  (1) ATS Hardware Reset                \n");
						printf("--  (2) Enable ATS + DHCP                 \n");
						printf("--  (3) Enable ATS + HW/SW TRACING        \n");
						printf("--  (4) Enable ATS and Disable HW tracing \n");
						printf("--  (5) Enable ATS and Disable SW tracing \n");
						printf("--  (6) Enable ATS internal loop          \n");
						printf("--  (7) Enable ATS disabled loop          \n");
						printf("--  (8) Set Baudrate                      \n");
						printf("--  (9) Disable All                       \n");
						printf("-- =======================================\n");
						
						
					printf("Selection [1..7] ? : ");
						if ((ret_code=scanf("%d", &ats_sel))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
												
						switch(ats_sel)
							{
							case 1: 
								
								data_write[0]=0x80;
								MWr32(mem_addr_ats + 0x21, data_write, 1, 1, NO_PRINT_VALUES); 	
								usleep(2*100*1000);
								data_write[0]=0x00;										
								MWr32(mem_addr_ats + 0x21, data_write, 1, 1, NO_PRINT_VALUES); 	
								break;

							case 2: 
									
								data_write[0]=0x13;
								MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 
								break;

							case 3: 
													
								data_write[0]=0x01;
								MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 
								data_write[0]=0x00;
								MWr32(mem_addr_ats + 0x21, data_write, 1, 1, NO_PRINT_VALUES); 
								break;

							case 4: 
													
								data_write[0]=0x01;
								MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 
								data_write[0]=0x01; // disabled hw tracing
								MWr32(mem_addr_ats + 0x21, data_write, 1, 1, NO_PRINT_VALUES); 
								break;

							case 5:	
														
								data_write[0]=0x01;
								MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 	
								data_write[0]=0x02; // disabled sw tracing
								MWr32(mem_addr_ats + 0x21, data_write, 1, 1, NO_PRINT_VALUES); 
								break;
								
							case 6:	
												
								data_write[0]=0x21;
								MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 
								break;
								
							case 7:	
							
								data_write[0]=0x01;
								MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 
								break;
								
							case 8:	
								uint32_t baud_sel;
								printf("\n\n Insert new Baudrate [0..4]: ");
								printf("\n 0 = 115200 ");	
								printf("\n 1 = 460800 ");	
								printf("\n 2 = 921600 ");	
								printf("\n 3 = 1843200 ");	
								printf("\n 4 = 3916800 ");	

								if ((ret_code=scanf("%d", &baud_sel))!=1)
								{
									printf("function read error %d\n",ret_code);
								};
								data_write[0]=baud_sel;
								MWr32(mem_addr_ats + 0x23, data_write, 1, 1, NO_PRINT_VALUES); 
								break;
								
							case 9:	
								
								data_write[0]=0x00;
								MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 	
								data_write[0]=0x03; // disabled hw/sw tracing
								MWr32(mem_addr_ats + 0x21, data_write, 1, 1, NO_PRINT_VALUES); 
								break;

								
							default:
								break;
							}
							
						usleep(20*100*1000);

						MRd32(mem_addr_ats + 0x20, data_read, 4, 4, NO_PRINT_VALUES); 
						printf("\n ATS               = %s", ENABLE_ATS(data_read[0]&0x01));
						printf("\n ATS DHCP          = %s", ENABLE_ATS_DHCP((data_read[0]>>1)&0x01));
						printf("\n ATS IPV6          = %s", ENABLE_ATS_IPV6((data_read[0]>>1)&0x02));
						printf("\n ATS VAR PACKET    = %s", ENABLE_ATS_VARIABLE_PACKET((data_read[0]>>3)&0x01));
						printf("\n ATS HW TRACING    = %s", DISABLE_ATS_HW_TRACING(data_read[1]&0x01));
						printf("\n ATS SW TRACING    = %s", DISABLE_ATS_SW_TRACING((data_read[1]>>1)&0x01));
						printf("\n ATS INTERNAL LOOP = %s", ENABLE_ATS_LOOP((data_read[0]>>5)&0x01));
						printf("\n ATS SYNC SIGNAL   = %s", DISABLE_ATS_SYNC((data_read[0]>>6)&0x01));
						
						MRd32(mem_addr_ats + 0x08, data_read, 4, 4, NO_PRINT_VALUES); 
						printf("\n ATS Ip Address   = %d.%d.%d.%d",data_read[3],data_read[2],data_read[1],data_read[0]); 
						
						MRd32(mem_addr_ats + 0x28, data_read, 6, 1, NO_PRINT_VALUES); 
						printf("\n Physical Address = %2.2x-%2.2x-%2.2x-%2.2x-%2.2x-%2.2x",data_read[5],data_read[4],data_read[3],data_read[2],data_read[1],data_read[0]); 
						
						MRd32(mem_addr_ats + 0x48, data_read, 32, 1, NO_PRINT_VALUES); 
						printf("\n Host Name        = %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",data_read[0],data_read[1],data_read[2],data_read[3],data_read[4],data_read[5],data_read[6],data_read[7],data_read[8],data_read[9],data_read[10],data_read[11],data_read[12],data_read[13],data_read[14],data_read[15]); 
						printf("%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",data_read[16],data_read[17],data_read[18],data_read[19],data_read[20],data_read[21],data_read[22],data_read[23],data_read[24],data_read[25],data_read[26],data_read[27],data_read[28],data_read[29],data_read[30],data_read[31]); 
						
						printf("\n\n");
						MRd32(mem_addr_ats + 0x80, data_buffer, 256, 4, NO_PRINT_VALUES);
							
						wait_to_continue();
						break;
						return 0;
					}  	
						
				case 65 : 
					{
						printf("Cmd_65: ATS pcie_bar_ats read\n");
						printf("\n == ATS VERSION read ==\n"); 
						MRd32(mem_addr_ats + 0x24, data_read, 4, 4, NO_PRINT_VALUES); 
						printf("\n version major: %2.2x",(data_read[0]&0xFF)); 
						printf("\n version minor: %2.2x \n",(data_read[1]&0xFF)); 
						printf("\n == ATS Status Enable Register ==\n"); 
						MRd32(mem_addr_ats + 0x20, data_read, 4, 4, NO_PRINT_VALUES); 
						printf("\n ATS               = %s", ENABLE_ATS(data_read[0]&0x01));
						printf("\n ATS DHCP          = %s", ENABLE_ATS_DHCP((data_read[0]>>1)&0x01));
						printf("\n ATS IPV6          = %s", ENABLE_ATS_IPV6((data_read[0]>>1)&0x02));
						printf("\n ATS VAR PACKET    = %s", ENABLE_ATS_VARIABLE_PACKET((data_read[0]>>3)&0x01));
						printf("\n ATS HW TRACING    = %s", DISABLE_ATS_HW_TRACING(data_read[1]&0x01));
						printf("\n ATS SW TRACING    = %s", DISABLE_ATS_SW_TRACING((data_read[1]>>1)&0x01));
						printf("\n ATS INTERNAL LOOP = %s", ENABLE_ATS_LOOP((data_read[0]>>5)&0x01));
						printf("\n ATS SYNC SIGNAL   = %s", DISABLE_ATS_SYNC((data_read[0]>>6)&0x01));
						
						MRd32(mem_addr_ats + 0x08, data_read, 4, 4, NO_PRINT_VALUES); 
						printf("\n ATS Ip Address   = %d.%d.%d.%d",data_read[3],data_read[2],data_read[1],data_read[0]); 
						
						MRd32(mem_addr_ats + 0x28, data_read, 6, 1, NO_PRINT_VALUES); 
						printf("\n Physical Address = %2.2x-%2.2x-%2.2x-%2.2x-%2.2x-%2.2x",data_read[5],data_read[4],data_read[3],data_read[2],data_read[1],data_read[0]); 
						//printf("\n status 0x20: %2.2x \n",(data_read[0]&0xFF)); 
						MRd32(mem_addr_ats + 0x48, data_read, 32, 1, NO_PRINT_VALUES); 
						printf("\n Host Name        = %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",data_read[0],data_read[1],data_read[2],data_read[3],data_read[4],data_read[5],data_read[6],data_read[7],data_read[8],data_read[9],data_read[10],data_read[11],data_read[12],data_read[13],data_read[14],data_read[15]); 
						printf("%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",data_read[16],data_read[17],data_read[18],data_read[19],data_read[20],data_read[21],data_read[22],data_read[23],data_read[24],data_read[25],data_read[26],data_read[27],data_read[28],data_read[29],data_read[30],data_read[31]); 
						
						printf("\n\n");
						MRd32(mem_addr_ats + 0x80, data_buffer, 256, 4, NO_PRINT_VALUES);
						
						wait_to_continue();
						break;
						return 0;
					}  	
					
					
				case 66 : 
					{
						
						
						printf("-- ============================\n");
						printf("-- Serial Port Open /dev/ttyS0 \n");	
						printf("-- ============================\n");
						
						int fd1=serial_open("/dev/ttyS0");
							
						data_write[0] = 0x40; 
												
						while ((ret_code=read(fd1,data_read,sizeof(data_read))) > 0) 
						{
							printf("\n Return code read = %d",ret_code);
							
							printf("\n Status  = ");
							for(int i = 0; i < ret_code; i++)
							{												
								printf("0x%2.2x ",data_read[i]&0xff); 
							};
							printf("\n"); 
						};
								
								if ((ret_code=write(fd1,data_write,1))==-1) 
								{
									perror(NULL);
								};
								printf("\n Return code write = %d",ret_code); 	
								
								while ((ret_code=read(fd1,data_read,sizeof(data_read))) > 0)
								{
									printf("\n Status  = ");
									for(int i = 0; i < ret_code; i++)
									{												
										printf("0x%2.2x ",data_read[i]&0xff); 										
									};
									printf("\n"); 								
								};	
								
								if ((ret_code=read(fd1,data_read,sizeof(data_read)))==-1) 
								{
									perror(NULL);
								}; 
								printf("\n Return code read = %d",ret_code); 						
								
									printf("\n Status  = ");
									for(int i = 0; i < ret_code; i++)
									{												
										printf("0x%2.2x ",data_read[i]&0xff); 
									};
									printf("\n"); 
													
								while ((ret_code=read(fd1,data_read,sizeof(data_read))) > 0) 
								{
									printf("\n Status  = ");
									for(int i = 0; i < ret_code; i++)
									{												
										printf("0x%2.2x ",data_read[i]&0xff); 
									};
									printf("\n"); 	
								};						
						
								
						close(fd1);
						wait_to_continue();
						break;
						return 0;
					}  	
					
				case 67 : 
					{
						printf("-- ===================\n");
						printf("-- Enable ATS and set \n");
						printf("-- internal loop mode \n");	
						printf("-- ===================\n");						
												
						printf("\n*** Enable ATS and set internal loop mode \n");	
						data_write[0]=0x21;
						
						MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 
						
						printf("\n*** Serial Port Open /dev/ttyS0 \n");	
						int fd1=serial_open("/dev/ttyS0");
							
						ats_wait_for_sync(data_read,fd1);		
						
						// STATUS CMD
						data_write[0] = STATUS_CMD; 
							
						if ((ret_code=write(fd1,data_write,1))==-1) 
						{
							perror(NULL);
						};
						printf("\n*** Sent Command 0x%2.2x \n",(data_write[0]>>4)&0xff);		
						
						ats_wait_cmd_ack(data_read,fd1,data_write[0]);						
						read_debug_response(data_read,fd1,60, NO_PRINT_VALUES);//PRINT_VALUES);//NO_PRINT_VALUES);
						
						printf("-- ============================\n");
						printf("*** STAUTS parameter ***\n");
						printf("*** DEBUG Version 0x%2.2x \n",data_read[0]);
						printf("*** DEBUG module Mask 0x%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x",data_read[1],data_read[2],data_read[3],data_read[4],data_read[5],data_read[6],data_read[7],data_read[8]);
						printf(".%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x \n",data_read[9],data_read[10],data_read[11],data_read[12],data_read[13],data_read[14],data_read[15],data_read[16]);
						printf("*** DEBUG level Mask 0x%2.2x \n",data_read[17]&0xf0);
						printf("*** DEBUG status %s \n",TRACING_STATUS((data_read[17]&0x0f)>>3));
						printf("-- ============================\n");
						
						
						// START DEBUG CMD
						data_write[0] = START_CMD; 

						if ((ret_code=write(fd1,data_write,1))==-1) 
						{
							perror(NULL);
						};
						printf("\n*** Sent Command 0x%2.2x \n",(data_write[0]>>4)&0xff);
						
						ats_wait_cmd_ack(data_read,fd1,data_write[0]);
						
						// disable debounce
						data_write[0] = 0x00; 
						data_write[1] = 0x00;  
						data_write[2] = 0x00; 	
						data_write[3] = 0x00;  
							
						IOWr(pcie_bar_io[0] + 0x3C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3D, data_write, 1, 1, 1, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3E, data_write, 1, 1, 2, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3F, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x40, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x41, data_write, 1, 1, 1, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x42, data_write, 1, 1, 2, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x43, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						
						// MOVE DOUT
						// walking 0
						for(i = 0; i < 32; i++)
						{
							
							data_write[0] = (0x01<<i);  	
							data_write[1] = (0x01<<(i-8));  	
							data_write[2] = (0x01<<(i-16));   	
							data_write[3] = (0x01<<(i-24));  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
										
							ats_response_read(fd1, 1, 3, NO_PRINT_VALUES);		
							
							
						};
							ats_response_read(fd1, 3, 13, NO_PRINT_VALUES);
						/* 
						// init 0xFF data
							data_write[0] = 0xff; 	
							data_write[1] = 0xff;  	
							data_write[2] = 0xff; 
							data_write[3] = 0xff; 
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
							
							ats_response_read(fd1, 1, 3, NO_PRINT_VALUES);
						
						// walking 1
						for(i = 0; i < 32; i++)
						{
							data_write[0] = 0xff-(0x01<<i);  	
							data_write[1] = 0xff-(0x01<<(i-8));  	
							data_write[2] = 0xff-(0x01<<(i-16));   	
							data_write[3] = 0xff-(0x01<<(i-24));  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
													
							ats_response_read(fd1, 1, 3, NO_PRINT_VALUES);		
							
							
						};
							ats_response_read(fd1, 3, 13, NO_PRINT_VALUES);
						 		*/	
						// reset data
							data_write[0] = 0x00; 	
							data_write[1] = 0x00;  	
							data_write[2] = 0x00; 
							data_write[3] = 0x00; 
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
								
						
						close(fd1);
						wait_to_continue();
						break;
						return 0;
					}  	
				
				case 68 : 
					{
						printf("-- ===================\n");
						printf("-- Enable ATS and set \n");
						printf("-- internal loop mode \n");	
						printf("-- ===================\n");						
												
						printf("\n*** Enable ATS and set internal loop mode \n");	
						data_write[0]=0x21;
						
						MWr32(mem_addr_ats + 0x20, data_write, 1, 1, NO_PRINT_VALUES); 
						
						printf("\n*** Serial Port Open /dev/ttyS0 \n");	
						int fd1=serial_open("/dev/ttyS0");
							
						ats_wait_for_sync(data_read,fd1);		
						
						// STATUS CMD
						data_write[0] = STATUS_CMD; 
							
						if ((ret_code=write(fd1,data_write,1))==-1) 
						{
							perror(NULL);
						};
						printf("\n*** Sent Command 0x%2.2x \n",(data_write[0]>>4)&0xff);		
						
						ats_wait_cmd_ack(data_read,fd1,data_write[0]);						
						read_debug_response(data_read,fd1,60, NO_PRINT_VALUES);//PRINT_VALUES);//NO_PRINT_VALUES);
						
						printf("-- ============================\n");
						printf("*** STAUTS parameter ***\n");
						printf("*** DEBUG Version 0x%2.2x \n",data_read[0]);
						printf("*** DEBUG module Mask 0x%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x",data_read[1],data_read[2],data_read[3],data_read[4],data_read[5],data_read[6],data_read[7],data_read[8]);
						printf(".%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x.%2.2x \n",data_read[9],data_read[10],data_read[11],data_read[12],data_read[13],data_read[14],data_read[15],data_read[16]);
						printf("*** DEBUG level Mask 0x%2.2x \n",data_read[17]&0xf0);
						printf("*** DEBUG status %s \n",TRACING_STATUS((data_read[17]&0x0f)>>3));
						printf("-- ============================\n");
						
						
						// START DEBUG CMD
						data_write[0] = START_CMD; 

						if ((ret_code=write(fd1,data_write,1))==-1) 
						{
							perror(NULL);
						};
						printf("\n*** Sent Command 0x%2.2x \n",(data_write[0]>>4)&0xff);
						
						ats_wait_cmd_ack(data_read,fd1,data_write[0]);
						
						// wait for LP messages
						ats_response_read(fd1, 3, 32, NO_PRINT_VALUES);		
							
						
						close(fd1);
						wait_to_continue();
						break;
						return 0;
					}  	
					
				case 70 : 
					{
						
						
						printf("-- ============================\n");
						printf("--       Get DIN status        \n");	
						printf("-- ============================\n");
						
						IORd(pcie_bar_io[0] + 0x00, data_read, 4, 1, NO_PRINT_VALUES);      
						printf("\n Din[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_io[0] + 0x04, data_read, 4, 1, NO_PRINT_VALUES);      
						printf("\n Din[63..32] = 0x%2.2x.%2.2x.%2.2x.%2.2x\n",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						
						wait_to_continue();
						break;
						return 0;
					}  	
						
			
				case 71 : 
					{
						// disable debounce
						data_write[0] = 0x00; 
						data_write[1] = 0x00;  
						data_write[2] = 0x00; 	
						data_write[3] = 0x00;  
							
						IOWr(pcie_bar_io[0] + 0x3C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3D, data_write, 1, 1, 1, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3E, data_write, 1, 1, 2, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3F, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x40, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x41, data_write, 1, 1, 1, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x42, data_write, 1, 1, 2, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x43, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						
						// enable interrupt
						data_write[0] = 0xff; 
						IOWr(pcie_bar_io[0] + 0x08, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x09, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x0A, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x0B, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x0C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x0D, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x0E, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x0F, data_write, 1, 1, 0, NO_PRINT_VALUES);
						
						// enable interrupt rising edge
						IOWr(pcie_bar_io[0] + 0x00, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x01, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x02, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x03, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x04, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x05, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x06, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x07, data_write, 1, 1, 0, NO_PRINT_VALUES);
						
						data_write[0] = 0x08; 
						IOWr(pcie_bar_io[0] + 0x28, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						
						// ISR and Interrupt register cleaning up
						IORd(pcie_bar_io[0] + 0x28, data_read, 1, 1, NO_PRINT_VALUES);
						#if _DEBUG	
							{	
								printf("\n Status Register = 0x%2.2x\n",data_read[0]&0xff); 
							}; 
						#endif   
						IORd(pcie_bar_io[0] + 0x08, data_read, 4, 4, NO_PRINT_VALUES);      
						#if _DEBUG	
							{	
								printf("\n Got Dinterrupt[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							}; 
						#endif 		
							
						printf("-- ============================\n");
						printf("--      Dout Walking ON/OFF    \n");
						printf("-- ============================\n");
						printf("\n\n");
						
						printf("-- ============================\n");
						// walking 1
						for(i = 0; i < 32; i++)
						{
							data_write[0] = 0xff-(0x01<<i);  	
							data_write[1] = 0xff-(0x01<<(i-8));  	
							data_write[2] = 0xff-(0x01<<(i-16));   	
							data_write[3] = 0xff-(0x01<<(i-24));  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						
			
							data_write[4] = 0xff; 
							data_write[5] = 0xff; 
							data_write[6] = 0xff; 	
							data_write[7] = 0xff; 
							
							IOWr(pcie_bar_io[0] + 0x1C, data_write, 1, 1, 4, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1D, data_write, 1, 1, 5, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1E, data_write, 1, 1, 6, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1F, data_write, 1, 1, 7, NO_PRINT_VALUES); 
					
							usleep(2*100*1000);
					
							printf("--       Dout: %d OFF       \n",i);
							
							printf("\n Set Dout[31..0] = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_write[3]&0xff,data_write[2]&0xff,data_write[1]&0xff,data_write[0]&0xff); 
							IORd(pcie_bar_io[0] + 0x00, data_read, 4, 1, NO_PRINT_VALUES);      
							printf("\n Got Din[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							expected_data[0]=0xff-data_write[0];
							expected_data[1]=0xff-data_write[1];
							expected_data[2]=0xff-data_write[2];
							expected_data[3]=0xff-data_write[3];
							/*
							IORd(pcie_bar_io[0] + 0x04, data_read, 4, 1, NO_PRINT_VALUES);      
							printf("\n Din[63..32] = 0x%2.2x.%2.2x.%2.2x.%2.2x\n",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							expected_data[4]=0xff-data_write[4];
							expected_data[5]=0xff-data_write[5];
							expected_data[6]=0xff-data_write[6];
							expected_data[7]=0xff-data_write[7];
							*/
							if (data_read[0]!=expected_data[0]) printf("\n Error at byte 0: Expected: 0x%2.2x got: 0x%2.2x",expected_data[0]&0xff,data_read[0]&0xff); 
							if (data_read[1]!=expected_data[1]) printf("\n Error at byte 1: Expected: 0x%2.2x got: 0x%2.2x",expected_data[1]&0xff,data_read[1]&0xff); 
							if (data_read[2]!=expected_data[2]) printf("\n Error at byte 2: Expected: 0x%2.2x got: 0x%2.2x",expected_data[2]&0xff,data_read[2]&0xff); 
							if (data_read[3]!=expected_data[3]) printf("\n Error at byte 3: Expected: 0x%2.2x got: 0x%2.2x",expected_data[3]&0xff,data_read[3]&0xff); 
										
							IORd(pcie_bar_io[0] + 0x28, data_read, 1, 1, NO_PRINT_VALUES);      
							#if _DEBUG	
							{	
								printf("\n Status Register = 0x%2.2x\n",data_read[0]&0xff); 
							}; 
							#endif 	
							if (data_read[0]==0x08) 
								{
									IORd(pcie_bar_io[0] + 0x08, data_read, 4, 4, NO_PRINT_VALUES);      
									printf("\n Got Dinterrupt[31..0] rising edge = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
								}
							else
								printf("\n No Rising Edge Interrupt[31..0] Detected"); 
								
							
							IORd(pcie_bar_io[0] + 0x28, data_read, 1, 1, NO_PRINT_VALUES);      
							#if _DEBUG	
							{	
								printf("\n Status Register = 0x%2.2x\n",data_read[0]&0xff); 
							}; 
							#endif							
											
							printf("\n");
							printf("-- ============================\n");
						}
						
						printf("\n");
						printf("-- ============================\n");
						
						// ISR and Interrupt register cleaning up
						IORd(pcie_bar_io[0] + 0x28, data_read, 1, 1, NO_PRINT_VALUES);
						#if _DEBUG	
							{	
								printf("\n Status Register = 0x%2.2x\n",data_read[0]&0xff); 
							}; 
						#endif   
						IORd(pcie_bar_io[0] + 0x08, data_read, 4, 4, NO_PRINT_VALUES);      
						#if _DEBUG	
							{	
								printf("\n Got Dinterrupt[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							}; 
						#endif 	
						
						// enable interrupt
						data_write[0] = 0x00; 
						
						// enable interrupt falling edge
						IOWr(pcie_bar_io[0] + 0x00, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x01, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x02, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x03, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x04, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x05, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x06, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x07, data_write, 1, 1, 0, NO_PRINT_VALUES);
						
						// walking 0
						for(i = 0; i < 32; i++)
						{
							
							data_write[0] = (0x01<<i);  	
							data_write[1] = (0x01<<(i-8));  	
							data_write[2] = (0x01<<(i-16));   	
							data_write[3] = (0x01<<(i-24));  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						
			
							data_write[4] = 0x00; 
							data_write[5] = 0x00;  
							data_write[6] = 0x00; 	
							data_write[7] = 0x00;  
							
							IOWr(pcie_bar_io[0] + 0x1C, data_write, 1, 1, 4, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1D, data_write, 1, 1, 5, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1E, data_write, 1, 1, 6, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1F, data_write, 1, 1, 7, NO_PRINT_VALUES); 
					
							usleep(2*100*1000);
					
							//printf("\n");
							//printf("-- ============================\n");
							printf("--       Dout: %d ON       \n",i);
							
							printf("\n Set Dout[31..0] = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_write[3]&0xff,data_write[2]&0xff,data_write[1]&0xff,data_write[0]&0xff); 
							IORd(pcie_bar_io[0] + 0x00, data_read, 4, 1, NO_PRINT_VALUES);      
							printf("\n Got Din[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							expected_data[0]=0xff-data_write[0];
							expected_data[1]=0xff-data_write[1];
							expected_data[2]=0xff-data_write[2];
							expected_data[3]=0xff-data_write[3];
							/*
							IORd(pcie_bar_io[0] + 0x04, data_read, 4, 1, NO_PRINT_VALUES);      
							printf("\n Din[63..32] = 0x%2.2x.%2.2x.%2.2x.%2.2x\n",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							expected_data[4]=0xff-data_write[4];
							expected_data[5]=0xff-data_write[5];
							expected_data[6]=0xff-data_write[6];
							expected_data[7]=0xff-data_write[7];
							*/
							if (data_read[0]!=expected_data[0]) printf("\n Error at byte 0: Expected: 0x%2.2x got: 0x%2.2x",expected_data[0]&0xff,data_read[0]&0xff); 
							if (data_read[1]!=expected_data[1]) printf("\n Error at byte 1: Expected: 0x%2.2x got: 0x%2.2x",expected_data[1]&0xff,data_read[1]&0xff); 
							if (data_read[2]!=expected_data[2]) printf("\n Error at byte 2: Expected: 0x%2.2x got: 0x%2.2x",expected_data[2]&0xff,data_read[2]&0xff); 
							if (data_read[3]!=expected_data[3]) printf("\n Error at byte 3: Expected: 0x%2.2x got: 0x%2.2x",expected_data[3]&0xff,data_read[3]&0xff); 
							
							IORd(pcie_bar_io[0] + 0x28, data_read, 1, 1, NO_PRINT_VALUES);      
							#if _DEBUG	
							{	
								printf("\n Status Register = 0x%2.2x\n",data_read[0]&0xff); 
							}; 
							#endif 	
							if (data_read[0]==0x08) 
								{
									IORd(pcie_bar_io[0] + 0x08, data_read, 4, 4, NO_PRINT_VALUES);      
									printf("\n Got Dinterrupt[31..0] falling edge = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
								}
							else
								printf("\n No Falling Edge Interrupt[31..0] Detected"); 
							
							IORd(pcie_bar_io[0] + 0x28, data_read, 1, 1, NO_PRINT_VALUES);      
							#if _DEBUG	
							{	
								printf("\n Status Register = 0x%2.2x\n",data_read[0]&0xff); 
							}; 
							#endif								
							
							printf("\n");
							printf("-- ============================\n");
								
														
						}
						
						
						wait_to_continue();
						break;
						return 0;
					}  	
				
				case 72 : 
					{
						// disable debounce
						data_write[0] = 0x00; 
						data_write[1] = 0x00;  
						data_write[2] = 0x00; 	
						data_write[3] = 0x00;  
							
						IOWr(pcie_bar_io[0] + 0x3C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3D, data_write, 1, 1, 1, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3E, data_write, 1, 1, 2, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3F, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x40, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x41, data_write, 1, 1, 1, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x42, data_write, 1, 1, 2, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x43, data_write, 1, 1, 3, NO_PRINT_VALUES); 
							
							
						printf("-- ============================\n");
						printf("--          Dout All ON        \n");
						printf("-- ============================\n");
						printf("\n\n");
						
						printf("-- ============================\n");
						// all FF
						
							data_write[0] = 0xff;  	
							data_write[1] = 0xff;  	
							data_write[2] = 0xff;   	
							data_write[3] = 0xff;  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						
			
							data_write[4] = 0xff; 
							data_write[5] = 0xff; 
							data_write[6] = 0xff; 	
							data_write[7] = 0xff; 
							
							IOWr(pcie_bar_io[0] + 0x1C, data_write, 1, 1, 4, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1D, data_write, 1, 1, 5, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1E, data_write, 1, 1, 6, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1F, data_write, 1, 1, 7, NO_PRINT_VALUES); 
					
							usleep(4*1000*1000);
					
							
							data_write[0] = 0x00;  	
							data_write[1] = 0x00;  	
							data_write[2] = 0x00;   	
							data_write[3] = 0x00;  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						
			
							data_write[4] = 0x00; 
							data_write[5] = 0x00;  
							data_write[6] = 0x00; 	
							data_write[7] = 0x00;  
							
							IOWr(pcie_bar_io[0] + 0x1C, data_write, 1, 1, 4, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1D, data_write, 1, 1, 5, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1E, data_write, 1, 1, 6, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1F, data_write, 1, 1, 7, NO_PRINT_VALUES); 
					
							usleep(4*1000*1000);
					
							printf("\n");
							printf("-- ============================\n");
								
						
						wait_to_continue();
						break;
						return 0;
					}  	
				
				case 73 : 
					{
						// disable debounce
						data_write[0] = 0x00; 
						data_write[1] = 0x00;  
						data_write[2] = 0x00; 	
						data_write[3] = 0x00;  
							
						IOWr(pcie_bar_io[0] + 0x3C, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3D, data_write, 1, 1, 1, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3E, data_write, 1, 1, 2, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x3F, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x40, data_write, 1, 1, 0, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x41, data_write, 1, 1, 1, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x42, data_write, 1, 1, 2, NO_PRINT_VALUES); 
						IOWr(pcie_bar_io[0] + 0x43, data_write, 1, 1, 3, NO_PRINT_VALUES); 
							
						printf(" Select Dout[0..7] value ? : 0x");
						if ((ret_code=scanf("%2hhx", &data_read[0]))!=1)
						{
							printf("function read error %d\n",ret_code);
						};	
						
						printf("-- ============================\n");
						printf("--          Dout All ON        \n");
						printf("-- ============================\n");
						printf("\n\n");
						
						printf("-- ============================\n");
						// all FF
						
							data_write[0] = data_read[0];//0xff;  	
							data_write[1] = data_read[0];//0xff;  	
							data_write[2] = 0xff;   	
							data_write[3] = 0xff;  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						
			
							data_write[4] = 0xff; 
							data_write[5] = 0xff; 
							data_write[6] = 0xff; 	
							data_write[7] = 0xff; 
							
							IOWr(pcie_bar_io[0] + 0x1C, data_write, 1, 1, 4, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1D, data_write, 1, 1, 5, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1E, data_write, 1, 1, 6, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1F, data_write, 1, 1, 7, NO_PRINT_VALUES); 
					
										
							printf("\n");
							printf("-- ============================\n");
								
						
						wait_to_continue();
						break;
						return 0;
					}  

				case 80:
					{
						printf("-- ==================================================\n");
						printf("--                IO Config Capabilities             \n");	
						printf("--        fb_width[2..0]       |  Dout type sel      \n");	
						printf("-- 000 - 8 bits  100 - 40 bits |    00000 TI         \n");
						printf("-- 001 - 16 bits 101 - 48 bits |    00001 ST         \n");
						printf("-- 010 - 24 bits 110 - 56 bits |    00010 ONSEMI     \n");
						printf("-- 011 - 32 bits 111 - TBD     |                     \n");
						printf("-- ==================================================\n");
						
						printf("Insert Feedback width params [0..6] (0 = 8 bits, .. ,6 = 56 bits): ");
						if ((ret_code=scanf("%d", &fb_width))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						printf("Dout octal buffer: 2 = ONSEMI (feedback); 1 = ST (feedback); 0 = TI (no feedback)\n type ? : ");
						if ((ret_code=scanf("%d", &dout_type))!=1)
						{
							printf("function read error %d\n",ret_code);
						};						
						
						data_write[0]=(  ((dout_type & 0x1F)<<3) | (fb_width & 0x07)  );
						printf("byte26 is %2.2x\n",data_write[0]);
						
						IOWr(pcie_bar_io[0] + 0x26, data_write, 1, 1, 0, NO_PRINT_VALUES); 
												
						wait_to_continue();
						break;
					}
					
						
				case 99 : 
					{
						
						
						printf("-- ============================\n");
						printf("--       Get PCIe status       \n");		
						printf("--  (1) Quixant LSPCI          \n");
						printf("--  (2) tree diagram lspci     \n");
						printf("--  (3) PCIe BAR size/offset   \n");
						printf("--  (4) BAR config register    \n");
						printf("--  (5) Enable PCIe/Set command\n");
						printf("-- ============================\n");
						
						
						printf("Selection :    ? : ");
						if ((ret_code=scanf("%d", &getpci_sel))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
												
						switch(getpci_sel)
							{
							case 1: 
								
								sprintf (path, "sudo lspci -vvv -d 19D4:");
								system_return = system(path);
								break;

							case 2: 
							
								sprintf (path, "sudo lspci -vvv -t");
								system_return = system(path);
								break;

							case 3: 
										
								find_pci_reg(IO_DEVID,		pcie_bar_io,	pcie_bar_size_io,	1);	
								find_pci_reg(NVRAM_DEVID,	pcie_bar_mem,	pcie_bar_size_mem,	1);
								find_pci_reg(SECs_DEVID,	pcie_bar_secs,	pcie_bar_size_secs,	1);	
								find_pci_reg(MUART_DEVID,	pcie_bar_muart,	pcie_bar_size_muart,1);
								find_pci_reg(QLI_DEVID,		pcie_bar_qli,	pcie_bar_size_qli,	1);		
								find_pci_reg(SBC_DEVID,		pcie_bar_sbc,	pcie_bar_size_sbc,	1);		
								find_pci_reg(ATS_DEVID,		pcie_bar_ats,	pcie_bar_size_ats,	1);
								break;

							case 4:	
								
								sprintf (path, "sudo lspci -xxx -d 19D4:");
								system_return = system(path);
								break;

							case 5:	
								
								printf("-- =======================================\n");
								printf("--            Set PCIe command            \n");		
								printf("--  (1) enable IO/MEM/DMA in all function \n");
								printf("--  (2) enable IO/MEM in all function     \n");
								printf("--  (3) enable ATS function               \n");
								printf("--  (4)                                   \n");
								printf("--  (5)                                   \n");
								printf("-- =======================================\n");
									
								printf("Selection :    ? : ");
								if ((ret_code=scanf("%d", &getpci_sel))!=1)
								{
									printf("function read error %d\n",ret_code);
								};
								
									switch(getpci_sel)
									{
									case 1: 
										
										sprintf (path, "sudo setpci -d 19d4: 04.b=07");
										system_return = system(path);
										break;

									case 2: 
									
										sprintf (path, "sudo setpci -d 19d4: 04.b=03");
										system_return = system(path);
										break;

									case 3: 
												 
										//sprintf (path, "sudo setpci -s :07:00.5 04.b=03");	
										sprintf (path, "sudo setpci -d 19d4:0c00 04.b=07");									
										system_return = system(path);
										break;

									default:
										break;
									}
								
							default:
								break;
							}
						
						wait_to_continue();
						break;
						return 0;
					}  	
						
				case 0 : 
					{
						exit=1;	
						break;
						return 0;
					}  
					
				default:
					{
						printf("Selezione errata %d\n",ret_code);
						exit=1;							
						break;
					}  
					
			}
			
		
		if (exit==0) goto SEL_MENU ;
				

		


	free(pcie_bar_io); 
	free(pcie_bar_mem);
	free(pcie_bar_secs);
	free(pcie_bar_muart);
	free(pcie_bar_qli);
	free(pcie_bar_sbc);
	free(pcie_bar_ats);
	
	free(pcie_bar_size_io); 
	free(pcie_bar_size_mem);
	free(pcie_bar_size_secs);
	free(pcie_bar_size_muart);
	free(pcie_bar_size_qli);
	free(pcie_bar_size_sbc);
	free(pcie_bar_size_ats);
	
	//free(dst_ssl);
	
	free(data_read);
	free(data_write);
	free(expected_data);
	
    free(fpga_wdata);
    free(fpga_rdata);

	close(fd);
  }
  
  return 0;
#else
  //TODO: to be implemented
  return 0xFFFFFFFF;
#endif
}


