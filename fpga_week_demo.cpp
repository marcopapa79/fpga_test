
/*
* =====================================================================================
*
*       Filename:  new_fpga_test.c
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
#include <iostream>
#include <chrono>
#include <vector>
using namespace std::chrono;
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
#define TEST_DEVID  "12049c25"
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
uint8_t ver_minor = 15;
char date[]="21/10/2024";

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
uint32_t *pcie_bar_test       = NULL;
uint32_t *pcie_bar_size_io    = NULL;
uint32_t *pcie_bar_size_mem   = NULL;
uint32_t *pcie_bar_size_secs  = NULL;
uint32_t *pcie_bar_size_muart = NULL;
uint32_t *pcie_bar_size_qli   = NULL;
uint32_t *pcie_bar_size_sbc   = NULL;
uint32_t *pcie_bar_size_ats   = NULL;
uint32_t *pcie_bar_size_test  = NULL;
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
	pcie_bar_test = (uint32_t *)calloc (6,sizeof(uint32_t));
	
	pcie_bar_size_io = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_mem = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_secs = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_muart = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_qli = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_sbc = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_ats = (uint32_t *)calloc (6,sizeof(uint32_t));
	pcie_bar_size_test = (uint32_t *)calloc (6,sizeof(uint32_t));
  
	find_pci_reg(IO_DEVID,		pcie_bar_io,	pcie_bar_size_io,	NO_PRINT_VALUES);	
	find_pci_reg(NVRAM_DEVID,	pcie_bar_mem,	pcie_bar_size_mem,	NO_PRINT_VALUES);
	find_pci_reg(SECs_DEVID,	pcie_bar_secs,	pcie_bar_size_secs,	NO_PRINT_VALUES);	
	find_pci_reg(MUART_DEVID,	pcie_bar_muart,	pcie_bar_size_muart,NO_PRINT_VALUES);
	find_pci_reg(QLI_DEVID,		pcie_bar_qli,	pcie_bar_size_qli,	NO_PRINT_VALUES);		
	find_pci_reg(SBC_DEVID,		pcie_bar_sbc,	pcie_bar_size_sbc,	NO_PRINT_VALUES);		
	find_pci_reg(ATS_DEVID,		pcie_bar_ats,	pcie_bar_size_ats,	NO_PRINT_VALUES);	
	find_pci_reg(TEST_DEVID,	pcie_bar_test,	pcie_bar_size_test,	PRINT_VALUES);
	//==========================================

	uint32_t nvramSize = 16384;//pcie_bar_size_test[0];//16384;
	uint32_t nvramMaxSize = pcie_bar_size_mem[0];//262144;

	int fd = open("/dev/mem", O_RDWR);
	//			MEM Map for PCIe mem space qli and nvram 
	if (fd != -1) {
		uint8_t * mem_addr;	
		uint8_t * mem_addr_led;		
		uint8_t * mem_addr_secs;		
		uint8_t * mem_addr_sbc;			
		uint8_t * mem_addr_ats;			
		uint8_t * mem_addr_test;	
		
		mem_addr = (uint8_t*)mmap(0, nvramMaxSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_mem[0]);
		/*mem_addr_led = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_qli[0]);
		mem_addr_sbc = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_sbc[0]);
		*/
		
		mem_addr_secs = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_secs[0]);
		//mem_addr_ats = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_ats[0]);
		mem_addr_test = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_test[0]);
	
		
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
	long long elapsedTime = 0; // Time elapsed in each attept
	long long totalElapsedTime = 0; // Total time elapsed
	high_resolution_clock::time_point start, stop; // start and stop time
	// ==== init array for timing count ====
	const size_t array_size = 100000000; // 100 million elements
    std::vector<int> array(array_size, 0);

    // Fill the array with some values
    for (size_t i = 0; i < array_size; ++i) {
        array[i] = i;
    }
	// ==================================
	
	uint32_t system_return;
	char path [1024]="sudo ./../git/libsecuress/bin/x64/test-libsecs -enc -k 0   -i ./../git/libsecuress/bin/x64/original_key    -o ./../git/libsecuress/bin/x64/key_enc  > /dev/null";
	
	sprintf (path, "sudo setpci -d 19d4: 04.b=07");	
	system_return = system(path);
	sprintf (path, "sudo setpci -d 1204: 04.b=07");	
	system_return = system(path);
	
	printf("\n ====================================");  
	printf("\n ====================================\n");  
	
	
/* 	MWr32(mem_addr_secs+24, data_write, 4, 4, NO_PRINT_VALUES);
	MRd32(mem_addr_secs+24, data_read, 4, 4, NO_PRINT_VALUES); 
	printf("\n SRAM first 4 bytes: byte0: 0x%2.2x byte1: 0x%2.2x byte2: 0x%2.2x byte3: 0x%2.2x\n",data_read[0]&0xff,data_read[1]&0xff, data_read[2]&0xff,data_read[3]&0xff); 
  */
	/**/

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
		// PWM
		uint32_t res_div;
		uint32_t pwm_duty_cyc[9];
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
	
		printf("\n\n"); 
		printf("====================================\n");  
		printf("===                              ===\n");  
		printf("===    FPGA WEEK DEMO UTILITY    ===\n"); 
		printf("===      Ver %d.%d  %s    ===\n",ver_major,ver_minor,date);  
		printf("===                              ===\n");   
		printf("===========  DEMO MENU  ============\n\n");		
		printf("Cmd_1 : xSPI CMD user selection     \n");
		printf("Cmd_2 : xSPI MRAM Write Read access \n\n");	
		printf("=========== PCIe utility ===========\n\n");
		printf("Cmd_99: Get PCIe status/Enable PCIe \n");
		printf("        Set command reg             \n\n");
		printf("====================================\n"); 
		printf("\n"); 
	
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
	
					
				case 1 : 
					{		
						uint32_t xSPI_cmd;
						
						uint8_t *writeCTRL = (uint8_t*)calloc(32, sizeof(uint8_t));
						uint8_t *readSTATUS = (uint8_t *)calloc(4,sizeof(uint8_t));						
									
						printf("-- =================================\n");
						printf("--       xSPI command               \n");		
						printf("--  (0) JESD reset                  \n");
						printf("--  (1) Read MAN ID CMD       (x1)  \n");
						printf("--  (2) Write Enable 0x06     (x1)  \n");
						printf("--  (3) DFIM entry            (x1)  \n");
						printf("--  (4) Init Status Reg 0x01  (x1)  \n");
						printf("--  (5) Init NVR        0xb1  (x1)  \n");
						printf("--  (6) Init OTP        0x42  (x1)  \n");
						printf("--  (7) Init mem Array  0xC7  (x1)  \n");
						printf("--  (8) Exit DFIM             (x1)  \n");
						printf("--  (9) Write NVR to octal    (x1)  \n");
						printf("--  (10) Read NVR             (x1)  \n");
						printf("--  (11) Soft Reset           (x1)  \n");
						printf("--  (12) Write VR to octal    (x1)  \n");
						printf("--  (13) Read NVR             (x8)  \n");
						printf("--  (14) Dev Man ID           (x8)  \n");
						printf("--  (15) Read NV              (x8)  \n");
						printf("--  (18) Wel and SR(0x00)     (x8)  \n");
						printf("--  (19) User mode                  \n");
						printf("--  (20) init (JESD, WEL, SR->00)   \n");
						printf("--  (25) Rst,Wel/SR=(0x00),user mode\n");
						printf("-- =================================\n");
						
						
						printf("Selection [1..15] ? : ");
						if ((ret_code=scanf("%d", &xSPI_cmd))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						switch(xSPI_cmd)
						{
							case 0:
								{	 	
									printf("--   JESD reset          \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES); 
									} while (!(readSTATUS[0] & 0x01));
									
									//*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);
									writeCTRL[0] =0x01;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
																
	/* 								MWr32(mem_addr+i, &writeBuffer[i], sram_access_type, sram_access_type, NO_PRINT_VALUES); 
									i += sram_access_type;	
									
									i = 0;
									while (i < bufferSize)
										{
											MRd32(mem_addr+i, &readBuffer[i], sram_access_type, sram_access_type, NO_PRINT_VALUES); */
										
								break;
								} 
							case 1:
								{
									printf("--   Read MAN ID CMD       (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x02;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 
							case 2:
								{
									printf("--   Write Enable 0x06     (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x10;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 
							case 3:
								{
									printf("--   DFIM entry           (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x20;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 							
							case 4:
								{
									printf("--   Init Status Reg 0x01  (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x40;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 
							case 5:
								{
									printf("--   Init NVR  xb1        (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x80;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								}  
							case 6:
								{
									printf("--   Init OTP x42      (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x20;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 7:
								{
									printf("--   Init mem Array  0xC7  (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));writeCTRL[0] =0x00;	
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x80;		
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
								break;
								}
							case 8:
								{
									printf("--   Exit DFIM        (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x10;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 9:
								{
									printf("--  Write NVR to octal      (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x04;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 
							case 10:
								{
									printf("--   Read NVR      (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x08;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								}
							case 11:
								{
									printf("--   Soft Reset         (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x02;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
																
								break;
								}
							case 13:
								{
									printf("--   Write VR  to octal        (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x04;		
									writeCTRL[2] =0x00;		
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
								break;
								}							
							case 12:
								{
									printf("--   Read NVR          (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x04;		
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								}
							case 14:
								{
									printf("--   Dev Man ID         (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x01;		
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								}
							case 15:
								{
									printf("--   Read VR       (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x20;	
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 18:
								{
									printf("--   Wel and SR=(0x00)    (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x40;	
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 19:
								{
									printf("--   User mode           \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x01;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
								break;
								}
							case 20:
								{
									printf("--   JESD + WEL + SR -> x00 + oSPI + reset        \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES); 
									} while (!(readSTATUS[0] & 0x01));
									
									//*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);
									writeCTRL[0] =0x01;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x10;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x08;		
									writeCTRL[2] =0x00;		
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x08;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000000, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x02;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
								break;
								}
							case 25:
								{
									printf("--   Soft reset + Wel and SR=(0x00) + user mode       \n");
									
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x02;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
									usleep(100*1000);
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x40;	
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_addr+0x10000002, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_addr +0x10000000 + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x01;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_addr+0x10000001, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
									
								break;
								}
							default:
								{	
									break;
								}  
						} 						
						
						wait_to_continue();
						free(writeCTRL);
						free(readSTATUS);
						break;
						return 0;
					}  	
						
				case 2 : 
					{
						#if _DEBUG	
						{	
							printf("\n\n === max ram size %d ===",nvramMaxSize);
							printf("\n\n === max ram size %d ===",pcie_bar_size_mem[0]);
						}; 
						#endif 
						
						printf("\n\n");
						IORd(pcie_bar_mem[1] + 0x14, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ===================================");
						printf("\n === SRAM MODULE (HW parameters) ===");
						printf("\n =====      Version %2.2x.%2.2x      =====\n",data_read[0]&0xff,data_read[1]&0xff); 
						#if _DEBUG	
						{	
							printf("\n\n === MODE REGISTER READ ===");
							IORd(pcie_bar_mem[1] + 0x0C, data_read, 1, 1, 1);
						}; 
						#endif 
						// enable all bank
						data_write[0]=0x0F;
						IOWr(pcie_bar_mem[1] + 0x0D, data_write, 1, 1, 0, NO_PRINT_VALUES);
						
						uint32_t sram_waccess_type,sram_raccess_type;
						uint32_t nvram_sel;
						
						uint8_t *writeBuffer;
						uint8_t *readBuffer;
						uint32_t sram_mode_num;
						uint32_t start_address;
						
						uint32_t x;
						uint32_t i;
						uint32_t test_size;
						uint32_t j;	
						uint32_t data_type;
						
						printf("-- =================================\n");
						printf("--       SRAM command               \n");		
						printf("--  (1) Write Access                \n");
						printf("--  (2) Read Access                 \n");
						printf("--  (3) Test Write Read Access      \n");
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
									
									printf("\n\n Insert Type SRAM acces 1 = byte, 2 = word, 4 = dword, 8 = qword:, 16 = oword:  ");
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
										case 16:
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
									
									printf("\n\n Which address do you want to write to ? [0..%d]: ",bufferSize);
									if ((ret_code=scanf("%d", &start_address))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									
									printf("-- =====================\n");
									printf("--     Data Type        \n");		
									printf("--  (1) 0xFF            \n");
									printf("--  (2) 0x00            \n");
									printf("--  (3) Consecutive     \n");
									printf("--  (4) Dword_type     \n");
									printf("--  (5) Random          \n");
									printf("-- =====================\n");
						
									printf("\n\n Data Type ? [1..5]: ");
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
												for (x = 0; x < test_size/4; x++)
													{
														*(writeBuffer + x*4) = (uint8_t)(x+0xD0);// & 0x000000FF);  
														*(writeBuffer + x*4+1) = (uint8_t)(x+0xD0);// &  0x000000FF);  
														*(writeBuffer + x*4+2) = (uint8_t)(x+0xD0);// & 0x000000FF);  
														*(writeBuffer + x*4+3) = (uint8_t)(x+0xD0);// & 0x000000FF);	
													}
												break;
											}  
										case 5:
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
									
									// Measure the time taken to access an element
									// Start time
									start = std::chrono::high_resolution_clock::now();
									//while (1)
									//{
										i = 0;
										while (i < test_size)//*1000)
											{
												MWr32(mem_addr+i+start_address, &writeBuffer[i], sram_waccess_type, sram_waccess_type, NO_PRINT_VALUES); 
												i += sram_waccess_type;		
											}
									//}
									volatile int value = array[array_size / 2]; // Access the middle element
									// Stop time
									stop = std::chrono::high_resolution_clock::now();

								    // Time calculation
									elapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count();
									// Total Time calculation
									//totalElapsedTime = totalElapsedTime + elapsedTime;
									printf("Time taken for DRAM access = %lld nanoseconds\n", elapsedTime);
									printf("Troughput = %d B/s\n", test_size/elapsedTime);
									double throughput = ((double)test_size/(double)elapsedTime)*1000; 
									printf("Troughput = %f MB/s\n", throughput);
									
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
									
									printf("\n\n Which address do you want to read from ? [0..%d]: ",bufferSize);
									if ((ret_code=scanf("%d", &start_address))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									#if _DEBUG	
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
											MRd32(mem_addr+i+start_address, &readBuffer[i], sram_raccess_type, sram_raccess_type, 1);//NO_PRINT_VALUES);
									
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
									bufferSize = nvramMaxSize; 
									readBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
									writeBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
									
									printf("\n\n How many byte to write ? [0..%d]: ",bufferSize);
									if ((ret_code=scanf("%d", &test_size))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									// max write byte size is Memory max size
									if (test_size > bufferSize) test_size=bufferSize;
									
									printf("\n\n Which address do you want to write to ? [0..%d]: ",bufferSize);
									if ((ret_code=scanf("%d", &start_address))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									
									
									
									printf("-- =================================\n");
									printf("--       SRAM MODE                  \n");		
									printf("--  (1) Consecutive Data            \n");
									printf("--  (2) Random Data                 \n");
									printf("--  (3)                             \n");
									printf("--  (4)                             \n");
									printf("-- =================================\n");
						
									printf("\n\n Data Type ? [1..5]: ");
									if ((ret_code=scanf("%d", &data_type))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									switch(data_type)
									{
										case 1:
											{	
												printf("Assigning consecutive values to a %u bytes local buffer...\n", test_size);
												for (x = 0; x < test_size; x++)
												*(writeBuffer + x) = (uint8_t)(x);
												break;
											} 
										case 2:
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
								
						uint8_t *writeBuffer00;	
						writeBuffer00 = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
						
						uint32_t rpt=0;
						//while (rpt < 12)
						while (rpt < 9)
						{			
									switch(rpt)
									{
										case 0:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 2;  // Word
												break;
											} 
										case 1:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 4;  // Dword
												break;
											} 
										case 2:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 2;  // Word
												break;
											}  
										case 3:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 4;  // Dword
												break;
											}  
										case 4:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												break;
											} 
										case 5:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 4;  // Dword
												break;
											} 
										case 6:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 8;  // Qword
												break;
											}  
										case 7:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 8;  // Qword
												break;
											}  
										case 8:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 8;  // Qword
												break;
											}  
										case 9:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 2;   // Word
												break;
											} 
										case 10:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 4;   // Dword
												break;
											} 
										case 11:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 8;   // Qword
												break;
											} 
										default:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 4;  // Dword
												break;
											}  
									} 
									// erase mem
									i = 0;
									//printf("write buffer...\n");	
									while (i < test_size)
										{
											MWr32(mem_addr+i+start_address, &writeBuffer00[i], 4, 4, NO_PRINT_VALUES); 
											i += 4;		
										}
										
									printf("Write @ %d bytes access type and read @ %d bytes access type...\n", sram_waccess_type,sram_raccess_type);
									i = 0;
									printf("write buffer...\n");												
									while (i < test_size)//*1000)
										{
											MWr32(mem_addr+i+start_address, &writeBuffer[i], sram_waccess_type, sram_waccess_type, NO_PRINT_VALUES); 
											i += sram_waccess_type;		
										}
											
									i = 0;
									printf("read buffer...\n");
									while (i < test_size)
										{
											MRd32(mem_addr+i+start_address, &readBuffer[i], sram_raccess_type, sram_raccess_type, NO_PRINT_VALUES);//1);//NO_PRINT_VALUES);
									
											/**/for(j = i; j < (i+sram_raccess_type); j++)
											{
												if ((writeBuffer[j]!=readBuffer[j]) && j<256) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
											}/**/
											i += sram_raccess_type;		
		  
										}
									
										
								
									rpt++; // check if number of iteration is infinite	
								
									usleep(2*1000*1000);
						}				 
								free(writeBuffer);
								free(readBuffer);
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
				
						
				case 66 : 
					{
						
						
						printf("-- ============================\n");
						printf("-- Serial Port Open /dev/ttyS6 \n");	
						printf("-- ============================\n");
						
						int fd1=serial_open("/dev/ttyS6");
							
						data_write[0] = 0x00; 
						data_write[1] = 0x01; 
						data_write[2] = 0x02; 
						data_write[3] = 0x03; 
						data_write[4] = 0x04; 
						data_write[5] = 0x05; 
						data_write[6] = 0x06; 
						data_write[7] = 0x07; 
						data_write[8] = 0x08; 
												
						while ((ret_code=read(fd1,data_read,sizeof(data_read))) > 0) 
						{
							printf("\n Return code read = %d",ret_code);
							
							printf("\n Read Data svuota fifo  = ");
							for(int i = 0; i < ret_code; i++)
							{												
								printf("0x%2.2x ",data_read[i]&0xff); 
							};
							printf("\n"); 
						};
								
								if ((ret_code=write(fd1,data_write,9))==-1) 
								{
									perror(NULL);
								};
								printf("\n Return code write = %d",ret_code); 	
								
								printf("\n Serial Data Readback "); 
								while ((ret_code=read(fd1,data_read,sizeof(data_read))) > 0)
								{
									printf("\n Data Read  = ");
									for(int i = 0; i < ret_code; i++)
									{												
										printf("0x%2.2x ",data_read[i]&0xff); 										
									};
									printf("\n"); 								
								};	
								
								/* if ((ret_code=read(fd1,data_read,sizeof(data_read)))==-1) 
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
								};	 */					
						
								
						close(fd1);
						wait_to_continue();
						break;
						return 0;
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
								find_pci_reg(ATS_DEVID,		pcie_bar_ats,	pcie_bar_size_ats,	1);	/*	*/	
								find_pci_reg(TEST_DEVID,	pcie_bar_test,	pcie_bar_size_test,	1);
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


