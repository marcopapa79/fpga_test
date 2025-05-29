
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
#define NVRAM_DEVID "19d41000"
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
uint8_t ver_major = 2;
uint8_t ver_minor = 0;
char date[]="08/05/2025";

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
int f_check = 0;                	// Function Check 
int f_verbose = 0;					// Verbosity

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
#define USER(x)      ((x) == 0?"not user mode":"user mode")
#define BIOS(x)      ((x) == 0?"BIOS didn't wrote enc aes keys":"BIOS wrote enc aes keys")
#define M_KEY_RDY(x) ((x) == 0?"LP mast key not ready":"LP mast key ready")
#define EXP17THKEY(x)((x) == 0?"17th key not expanded":"17th key expanded")


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
		else if(strncmp(argv[n], "-check", 6)==0) {
            // Check RAM
            f_check = 1;
			f_verbose = 0;
        }
		else if(strncmp(argv[n], "-v_check", 8)==0) {
            // Check RAM
            f_check = 1;
			f_verbose = 1;
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

	uint32_t nvramMaxSize = pcie_bar_size_mem[0];//262144;
	uint32_t aesMaxSize = pcie_bar_size_mem[2];//262144;

	int fd = open("/dev/mem", O_RDWR);
	//			MEM Map for PCIe mem space qli and nvram 
	if (fd != -1) {
		uint8_t * mem_addr;	
		uint8_t * mem_addr_led;		
		//uint8_t * mem_addr_secs;		
		uint8_t * mem_addr_sbc;			
		uint8_t * mem_addr_ats;			
		//uint8_t * mem_addr_test;	
		uint8_t * mem_ctrl;			
		uint8_t * mem_aes;		
		
		mem_addr = (uint8_t*)mmap(0, nvramMaxSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_mem[0]);
		mem_ctrl = (uint8_t*)mmap(0, 256, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_mem[1]);
		mem_aes = (uint8_t*)mmap(0, aesMaxSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_mem[2]);
		/*mem_addr_led = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_qli[0]);
		mem_addr_sbc = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_sbc[0]);
		*/
		
		//mem_addr_secs = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_secs[0]);
		//mem_addr_ats = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_ats[0]);
		//mem_addr_test = (uint8_t*)mmap(0, nvramSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pcie_bar_test[0]);
	
		
		uint8_t *data_read  = (uint8_t *)calloc (256,sizeof(uint8_t));
		uint8_t *data_write = (uint8_t *)calloc (256,sizeof(uint8_t));
		uint8_t *expected_data = (uint8_t *)calloc (256,sizeof(uint8_t));
		uint8_t *fpga_wdata = (uint8_t *)malloc(mem_size);
		uint8_t *fpga_rdata = (uint8_t *)malloc(mem_size);
		uint8_t uni17key[16];
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
			
			//
        // MEM check DQA write and read code
        //
        if (f_check) 
			{
					if (f_verbose)
					{	
						printf("\n\n");
						MRd32(mem_ctrl + 0x28, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ====================================");
						printf("\n === NVRAM MODULE (HW parameters) ===");
						printf("\n =====      Version %2.2x.%2.2x       =====\n\n",data_read[0]&0xff,data_read[1]&0xff); 
						MRd32(mem_ctrl + 0x3C, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ====================================");
						printf("\n ===       Platform Version       ===");
						printf("\n =====      Version %2.2x.%2.2x       =====\n\n\n\n",data_read[0]&0xff,data_read[1]&0xff); 
						
						MRd32(mem_ctrl + 0x36, data_read, 1, 1, NO_PRINT_VALUES);   
						printf("\n\n === Chip Number  ==="); 
						printf("\n Number of Chip = %d", (pot2(data_read[0]>>4&0x03)));						
							if ( ((data_read[0])&0x0F)==0 )
							{
								printf("\n Chip Size \t= 0 Mbit\n");	
							} 
							else
							{
								printf("\n Chip Size \t= %.0lf Mbit\n", pow(2, ((data_read[0]&0x0F)+3) ));
							} 
					}
				srand(time(0));
				uint32_t sram_waccess_type,sram_raccess_type;
				uint8_t *dword_write,*dword_read;
				uint32_t start_address,test_size;
				printf("mem address is: %8.8x \n",(uint64_t)(mem_addr));
				dword_write = (uint8_t*)calloc(4, sizeof(uint8_t));
				dword_read = (uint8_t*)calloc(4, sizeof(uint8_t));
				sram_waccess_type = 4;
				sram_raccess_type = 4;
				test_size			= 4;
				start_address	= 0;
				uint32_t x;
				uint32_t j;
				for (x = 0; x < 4; x++)
				*(dword_write + x) = (uint8_t)(rand() & 0x000000FF);
												
				MWr32(mem_addr+start_address, dword_write, sram_waccess_type, sram_waccess_type, NO_PRINT_VALUES); 
										
				printf("\n");
				printf("Read Mem Access Type:  Byte\n");
				printf("Address (hex)          Data value (hex)\n");
				printf("                       |          Dword|\n");
									
				MRd32(mem_addr+start_address, dword_read, sram_raccess_type, sram_raccess_type, 1);//NO_PRINT_VALUES);
				
				for(j = 0; j < 4; j++)
				{
					if (dword_write[j]!=dword_read[j]) 
					{
						printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,dword_write[j]&0xff,dword_read[j]&0xff); 
						exit(-1);								
					} 								
				}/**/								
				exit(0);
				free(dword_write);
				free(dword_read);
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
	printf("\n === Quixant Configuration utlity ==="); 
	printf("\n ===      Ver %d.%d  %s     ===",ver_major,ver_minor,date); 
	printf("\n ====================================\n");  
	
	IORd(pcie_bar_io[0] + 0x2A, data_read, 2, 1, NO_PRINT_VALUES); 
	printf("\n FPGA VERSION Maj.Min\t= %2.2x.%2.2x\n\n", data_read[0]&0xff,data_read[1]&0xff);
	 
	
	IORd(pcie_bar_io[0] + 0x44, data_read, 2, 1, NO_PRINT_VALUES); 
	printf("\n ===================================");
	printf("\n ===  IO MODULE (HW parameters)  ===");
	printf("\n =====      Version %2.2x.%2.2x      =====",data_read[0]&0xff,data_read[1]&0xff); 
	
	
	printf("\n ====================================");
	printf("\n ===       Platform Version       ===");
	IORd(pcie_bar_io[0] + 0x24, data_read, 2, 1, NO_PRINT_VALUES);
	printf("\n PLATFORM ID \t= %2.2x\n PLATFORM SP ID\t= %2.2x", data_read[0]&0xff,data_read[1]&0xff);	
	IORd(pcie_bar_io[0] + 0x27, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n CAPABILITIES \t= %2.2x", data_read[0]&0xff);
	
	printf("\n\n === CONFIGURATION ===");
	IORd(pcie_bar_io[0] + 0x26, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n DOUT_TYPE \t= %s", IO_TYPE((data_read[0]>>3)&0x1F));
	printf("\n FEEDBACK WIDTH\t= %d", (((data_read[0]&0x07)+1)*8));	
	IORd(pcie_bar_io[0] + 0x70, data_read, 4, 1, NO_PRINT_VALUES);
	printf("\n Dout[7..0] feedback disable value \t= 0x%2.2x", (data_read[0]&0xff));
	printf("\n Dout[15..8] feedback disable value \t= 0x%2.2x", (data_read[1]&0xff));
	printf("\n Dout[23..16] feedback disable value \t= 0x%2.2x", (data_read[2]&0xff));
	printf("\n Dout[31..24] feedback disable value \t= 0x%2.2x", (data_read[3]&0xff));
	printf("\n ===================================");
	
	IORd(pcie_bar_io[0] + 0x20, data_read, 1, 1, NO_PRINT_VALUES);
	printf("\n Authentication status: %s \n",AUTHENTICA(data_read[0]&0x01)); 
	
	printf("\n\n");
	MRd32(mem_ctrl + 0x28, data_read, 2, 1, NO_PRINT_VALUES); 
	printf("\n ====================================");
	printf("\n === NVRAM MODULE (HW parameters) ===");
	printf("\n =====      Version %2.2x.%2.2x       =====\n\n",data_read[0]&0xff,data_read[1]&0xff); 
					
	MRd32(mem_ctrl + 0x36, data_read, 1, 1, NO_PRINT_VALUES);   
	printf("\n\n === Chip Number/Size  ==="); 
	printf("\n Number of Chip = %d", (pot2(data_read[0]>>4&0x03)));				
		if ( ((data_read[0])&0x0F)==0 )
		{
			printf("\n Chip Size \t= 0 Mbit\n");	
		} 
		else
		{
			printf("\n Chip Size \t= %.0lf Mbit\n", pow(2, ((data_read[0]&0x0F)+3) ));
		} 
	MRd32(mem_ctrl + 0xBC, data_read, 1, 1, NO_PRINT_VALUES); 
	printf("\n ====================================");
	printf("\n ===  SECS MODULE (HW parameters) ===");
	printf("\n =====      Version %2.2x.%2.2x       =====\n\n",data_read[0]&0xff,data_read[1]&0xff); 	
	printf("\n ====================================");
	MRd32(mem_ctrl + 0x3F, data_read, 1, 1, NO_PRINT_VALUES);   
	printf("\n CAPABILITIES \t= %2.2x", data_read[0]&0xff);	
	
	printf("\n KEY17th feature \t= %s", KEY_17th((data_read[0]>>3)&0x01));
	printf("\n Master KEY feature \t= %s", MASTER_KEY((data_read[0]>>4)&0x01));
	MRd32(mem_ctrl + 0xBE, data_read, 1, 1, NO_PRINT_VALUES);//NO_PRINT_VALUES);
	printf("\n User Mode    \t\t= %s", USER(data_read[0]&0x01));
	printf("\n BIOS_Key written    \t= %s", BIOS((data_read[0]>>1)&0x01));
	printf("\n LP master key ready \t= %s", M_KEY_RDY((data_read[0]>>2)&0x01));
	printf("\n expanded 17th key \t= %s", EXP17THKEY((data_read[0]>>3)&0x01));
	
	MRd32(mem_ctrl + 0xAB, data_read, 1,1, NO_PRINT_VALUES);
	printf("\n AES mode register: %2.2x",data_read[0]); 
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
		
		// io variable
		uint32_t fb_width;
		uint32_t dout_type;
		uint8_t fb_mask;
		uint8_t dout_cfg;
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
		printf("===   FPGA CERTUS DEMO UTILITY   ===\n"); 
		printf("===      Ver %d.%d  %s    ===\n",ver_major,ver_minor,date);  
		printf("===                              ===\n");   
		printf("===========  DEMO MENU  ============\n\n");		
		printf("Cmd_1 : xSPI CMD user selection     \n");
		printf("Cmd_2 : xSPI MRAM Write Read access \n");	
		printf("Cmd_3 : Stress A Read ID (IOrd32)   \n");
		printf("Cmd_4 : Stress B Mem Write (MemWr)  \n\n");
		printf("Cmd_10: Advanced Program SRAM CFG nByte \n");
		printf("============ FPGA FLASH CMD ===============        (last fpga byte 0x1d6B99)\n");
		printf("Cmd_20: Get device id and manufacture id (FPGA FLASH) \n");
		printf("Cmd_21: Read Status Register (FPGA FLASH) \n");
		printf("Cmd_22: Read Data nByte (FPGA FLASH) \n");//
		printf("Cmd_23: Program Page nByte (FPGA FLASH)  \n");
		printf("Cmd_24: FPGA program from input file (FPGA FLASH)  \n");
		printf("Cmd_25: FPGA readback and store to output file (FPGA FLASH)  \n");
		printf("Cmd_26: Chip Erase (FPGA FLASH) \n");
		printf("Cmd_27: Sector Erase (FPGA FLASH) \n");	
		printf("Cmd_39: Write SR protection disable ALL (FPGA FLASH) \n");
		printf("=============== SRAM TEST ==================\n");		
		printf("Cmd_40: Program AES encrypted Keys into internal keys memory (4096 bits)\n");	
		printf("Cmd_41: Verify Key0..15 with open SSL\n");	
		printf("Cmd_42: Encrypt AES Keys with 17th key and write in FPGA FLASH sector 1023 0x3FF000\n");	
		printf("Cmd_43: Test_Analyzer\n");	
		printf("Cmd_44: Encrypt AES Keys with Master key and write in FPGA FLASH sector 1023 0x3FF000\n");
		printf("Cmd_45: AES reset\n");
		printf("Cmd_46: Program Master Key injection\n");
		printf("Cmd_60: SRAM Write Test from 0 to MAX size in Normal mode (stress test for consumption)\n");
		printf("Cmd_61: SRAM Write Test from 0 to MAX size in Mirror mode (stress test for consumption)\n");
		printf("Cmd_62: SRAM Write Test and Read back/verify from 0 to MAX size in Normal mode 64\n");
		printf("Cmd_63: SRAM Write Read access\n");	
		printf("=============== DIO TEST ==================\n");
		printf("Cmd_70: DIN status\n");
		printf("Cmd_71: DIN interrupt test\n");
		printf("Cmd_72: Set Dout all ON/OFF\n");
		printf("Cmd_73: Set Dout and readback Feedback\n");
		printf("Cmd_74: Set Dout and read DIN status\n");
		printf("Cmd_75: Check Feedback\n");
		printf("=========== PCIe utility ===========\n\n");
		printf("Cmd_99: Get PCIe status/Enable PCIe \n");
		printf("        Set command reg             \n\n");
		printf("====================================\n"); 
		printf("\n"); 
		
		#define MAX_KEYS 4
		
		typedef struct {
			uint32_t id_31_0;    
			uint32_t id_63_32;
			uint32_t id_95_64;
			uint32_t id_127_96;  
		} UniqueID;
		
		
		const UniqueID expected_keys[MAX_KEYS]= {
		
			{// Chiave 1
				.id_31_0 	= 0x3402a048,
				.id_63_32 	= 0x21b82430,
				.id_95_64 	= 0x04503d7f,
				.id_127_96 = 0x11eab905
			 },
			
			{// Chiave 2
				.id_31_0 	= 0xf8001260,
				.id_63_32 	= 0xedba961a,
				.id_95_64 	= 0x0c503d7f,
				.id_127_96 = 0x19eab905
			},
			
	 		{// Chiave 3
				.id_31_0 	= 0x7402b05f,
				.id_63_32 	= 0x61b83425,
				.id_95_64 	= 0x04503f7f,
				.id_127_96  = 0x11eabb05
			},
			
			{// Chiave 4
				.id_31_0 	= 0xf8003060,
				.id_63_32 	= 0xedbab41a,
				.id_95_64 	= 0x0c503d7f,
				.id_127_96 = 0x19eab905
			}
			
		};
		
		UniqueID read_id;
		uint32_t vld_id = 0;
		
		MRd32(mem_ctrl + 0xD0, data_read, 4, 1, NO_PRINT_VALUES); 
		uni17key[0]=data_read[0];
		uni17key[1]=data_read[1];
		uni17key[2]=data_read[2];
		uni17key[3]=data_read[3];
		
		read_id.id_31_0 = ((data_read[3]&0xff) << 24) | ((data_read[2]&0xff) << 16) | ((data_read[1]&0xff) << 8) | (data_read[0]&0xff);
		
		MRd32(mem_ctrl + 0xD4, data_read, 4, 1, NO_PRINT_VALUES); 
		uni17key[4]=data_read[0];
		uni17key[5]=data_read[1];
		uni17key[6]=data_read[2];
		uni17key[7]=data_read[3];        
		
		read_id.id_63_32 = ((data_read[3]&0xff) << 24) | ((data_read[2]&0xff) << 16) | ((data_read[1]&0xff) << 8) | (data_read[0]&0xff);
		
		
		MRd32(mem_ctrl + 0xD8, data_read, 4, 1, NO_PRINT_VALUES); 
		uni17key[8]=data_read[0];
		uni17key[9]=data_read[1];
		uni17key[10]=data_read[2];
		uni17key[11]=data_read[3];  
		
		read_id.id_95_64 = ((data_read[3]&0xff) << 24) | ((data_read[2]&0xff) << 16) | ((data_read[1]&0xff) << 8) | (data_read[0]&0xff);
		
		MRd32(mem_ctrl + 0xDC, data_read, 4, 1, NO_PRINT_VALUES); 
		uni17key[12]=data_read[0];
		uni17key[13]=data_read[1];
		uni17key[14]=data_read[2];
		uni17key[15]=data_read[3];	
		
		read_id.id_127_96 = ((data_read[3]&0xff) << 24) | ((data_read[2]&0xff) << 16) | ((data_read[1]&0xff) << 8) | (data_read[0]&0xff);  
		
		printf("\n ===      Unique ID key      ===");
		// 
		printf("\n =====    ID31..0: 0x%2.2x%2.2x%2.2x%2.2x      =====\t",uni17key[3]&0xff,uni17key[2]&0xff,uni17key[1]&0xff,uni17key[0]&0xff); 	
		printf("\n =====    ID63..32: 0x%2.2x%2.2x%2.2x%2.2x      =====\t",uni17key[7]&0xff,uni17key[6]&0xff,uni17key[5]&0xff,uni17key[4]&0xff); 	
		printf("\n =====    ID95..64: 0x%2.2x%2.2x%2.2x%2.2x      =====\t",uni17key[11]&0xff,uni17key[10]&0xff,uni17key[9]&0xff,uni17key[8]&0xff); 		
		printf("\n =====    ID127..96: 0x%2.2x%2.2x%2.2x%2.2x      =====\t",uni17key[15]&0xff,uni17key[14]&0xff,uni17key[13]&0xff,uni17key[12]&0xff); 	
		
		
		for (int i = 0; i < MAX_KEYS; i++) {
			( (read_id.id_31_0 == expected_keys[i].id_31_0) & (read_id.id_63_32 == expected_keys[i].id_63_32) & (read_id.id_95_64 == expected_keys[i].id_95_64) & (read_id.id_127_96 == expected_keys[i].id_127_96)  ) ?  vld_id=i : vld_id;  
		}	
		
		vld_id == 0 ? printf("\n\n ERRORE \n\n") : printf(" \n\n SUCCESS"); printf(" Chiave Board N° %d\n\n",++vld_id) ; 
		
		
						printf("\n ======================================= \t");
						printf("\n == Master Key INJECTED to LP =="); 
						MRd32(mem_ctrl + 0xCC, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						MRd32(mem_ctrl + 0xC8, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						MRd32(mem_ctrl + 0xC4, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						MRd32(mem_ctrl + 0xC0, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n");						
						
		
						printf("\n ======================================= \t");
						printf("\n == MAster Key original =="); 
						MRd32(mem_ctrl + 0x6C, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						MRd32(mem_ctrl + 0x68, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						MRd32(mem_ctrl + 0x64, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						MRd32(mem_ctrl + 0x60, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n");
						
		printf("\n\n\n");
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
						uint8_t *nvregister = (uint8_t *)calloc(8,sizeof(uint8_t));						
									
						printf("-- =====================================\n");
						printf("--       xSPI command                   \n");		
						printf("--  (0) JESD reset                      \n");
						printf("--  (1) Read MAN ID CMD           (x1)  \n");
						printf("--  (2) Write Enable 0x06         (x1)  \n");
						printf("--  (3) DFIM entry                (x1)  \n");
						printf("--  (4) Init Status Reg    0x01   (x1)  \n");
						printf("--  (5) Init NVR           0xb1   (x1)  \n");
						printf("--  (6) Init OTP           0x42   (x1)  \n");
						printf("--  (7) Init mem Array     0xC7   (x1)  \n");
						printf("--  (8) Exit DFIM                 (x1)  \n");
						printf("--  (9) Write NVR to octal        (x1)  \n");
						printf("--  (10) Read NVR                 (x1)  \n");
						printf("--  (11) Soft Reset               (x1)  \n");
						printf("--  (12) Write VR to octal        (x1)  \n");
						printf("--  (13) Read NVR                 (x8)  \n");
						printf("--  (14) Dev Man ID               (x8)  \n");
						printf("--  (15) Read VR                  (x8)  \n");
						printf("--  (16) Write NVR                (x8)  \n");
						printf("--  (17) Write VR                 (x8)  \n");
						printf("--  (18) Wel and SR(0x00)         (x8)  \n");
						printf("--  (19) User mode                      \n");
						printf("--  (20) init (JESD, WEL, SR->00) (x1)  \n");
						printf("--  (21) Soft Reset               (x8)  \n");
						printf("--  (24) Rst(x8), Wel/SR=(0),user mode  \n");
						printf("--  (25) Rst(x1), Wel/SR=(0),user mode  \n");
						printf("--  (26) Check Status                   \n");
						printf("--  (27) Enter in command maual mode    \n");
						printf("--  (28) Read NVR content               \n");
						printf("-- =====================================\n");
						
						
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
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES); 
									} while (!(readSTATUS[0] & 0x01));
									
									//*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);
									writeCTRL[0] =0x01;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
																
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
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x02;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 
							case 2:
								{
									printf("--   Write Enable 0x06     (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x10;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 
							case 3:
								{
									printf("--   DFIM entry           (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x20;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 							
							case 4:
								{
									printf("--   Init Status Reg 0x01  (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x40;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 
							case 5:
								{
									printf("--   Init NVR  xb1        (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x80;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								}  
							case 6:
								{
									printf("--   Init OTP x42      (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x20;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 7:
								{
									printf("--   Init mem Array  0xC7  (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));writeCTRL[0] =0x00;	
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x40;		
									writeCTRL[2] =0x00;		
									MWr32(mem_ctrl+0x01, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
								break;
								}
							case 8:
								{
									printf("--   Exit DFIM        (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x10;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 9:
								{
									printf("--  Write NVR to octal      (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x04;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								} 
							case 10:
								{
									printf("--   Read NVR      (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x08;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
								break;
								}
							case 11:
								{
									printf("--   Soft Reset         (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x02;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
																
								break;
								}
							case 12:
								{
									printf("--   Write VR         (x1)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x04;		
									writeCTRL[2] =0x00;		
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
								break;
								}							
							case 13:
								{
									printf("--   Read NVR          (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x04;		
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								}
							case 14:
								{
									printf("--   Dev Man ID         (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x01;		
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								}
							case 15:
								{
									printf("--   Read VR       (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x20;	
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 16:
								{
									printf("--   Write NVR       (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x02;	
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 17:
								{
									printf("--   Write VR       (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x80;	
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 18:
								{
									printf("--   Wel and SR=(0x00)    (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x40;	
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
								break;
								} 
							case 19:
								{
									printf("--   Enter in User mode           \n");
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x01;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(1000);
									if ((readSTATUS[0] & 0x02)) printf("--   SUCCESS          \n");
									
								break;
								}
							case 20:
								{
									printf("--   JESD + WEL + SR -> x00 + oSPI + reset        \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES); 
									} while (!(readSTATUS[0] & 0x01));
									
									//*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);
									writeCTRL[0] =0x01;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x10;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x08;		
									writeCTRL[2] =0x00;		
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x08;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl, &writeCTRL[0], 4, 4, NO_PRINT_VALUES); 
									
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x02;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
								break;
								}
							case 21:
								{
									printf("--   Soft Reset 0x66 -> 0x99  (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));writeCTRL[0] =0x00;	
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x80;		
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
								break;
								}
							case 24:
								{
									printf("--   Soft reset(x8) + Wel and SR=(0x00) + user mode    \n");
									
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));writeCTRL[0] =0x00;	
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x80;		
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
									usleep(100*1000);
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x40;	
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x01;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
									
								break;
								}
							case 25:
								{
									printf("--   Soft reset(x1) + Wel and SR=(0x00) + user mode    \n");
									
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x02;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
									usleep(100*1000);
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x40;	
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;			
									MWr32(mem_ctrl+0x02, &writeCTRL[2], 1, 1, NO_PRINT_VALUES); 
									
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x01;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;			
									MWr32(mem_ctrl+0x01, &writeCTRL[1], 1, 1, NO_PRINT_VALUES); 
									
									
								break;
								}
							case 26:
								{
									printf("--   Check status  \n");
									// check in MANUAL_CMD_DECODE status 
									/* do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01)); */
									MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);
									printf("\n Status is = %2.2x\n", readSTATUS[0]&0xff);
									
									
								break;
								} 
							case 27:
								{
									printf("--   Enter in Manual mode           \n");
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;	
									writeCTRL[3] =0x80;			
									MWr32(mem_ctrl+0x03, &writeCTRL[3], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;	
									writeCTRL[3] =0x00;			
									MWr32(mem_ctrl+0x03, &writeCTRL[3], 1, 1, NO_PRINT_VALUES); 
									usleep(1000);
									if (readSTATUS[0] & 0x01) printf("--   SUCCESS          \n");
									
									
								break;
								}
							case 28:
								{
									printf("--   Read content of NVR  \n");
									// check in MANUAL_CMD_DECODE status 
									/* do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01)); */
									MRd32(mem_ctrl + 0x14, nvregister, 4, 4, NO_PRINT_VALUES);
									printf("\n NVR0 = %2.2x\n NVR1 = %2.2x\n NVR2 = %2.2x\n NVR3 = %2.2x", nvregister[0]&0xff,nvregister[1]&0xff, nvregister[2]&0xff,nvregister[3]&0xff);
									
									MRd32(mem_ctrl + 0x18, nvregister, 4, 4, NO_PRINT_VALUES);
									printf("\n NVR4 = %2.2x\n NVR5 = %2.2x\n NVR6 = %2.2x\n NVR7 = %2.2x\n\n", nvregister[0]&0xff,nvregister[1]&0xff, nvregister[2]&0xff,nvregister[3]&0xff);
									
								break;
								} 
							
							case 29:
								{
									printf("--   Read ISR       (x8)  \n");
									// check in MANUAL_CMD_DECODE status 
									do{ 
										MRd32(mem_ctrl + 0x10, readSTATUS, 1, 1, NO_PRINT_VALUES);      
									} while (!(readSTATUS[0] & 0x01));
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;		
									writeCTRL[2] =0x00;			
									writeCTRL[3] =0x01;
									MWr32(mem_ctrl+0x03, &writeCTRL[3], 1, 1, NO_PRINT_VALUES); 
									usleep(10*1000);
									writeCTRL[0] =0x00;	
									writeCTRL[1] =0x00;	
									writeCTRL[2] =0x00;	
									writeCTRL[3] =0x00;			
									MWr32(mem_ctrl+0x03, &writeCTRL[3], 1, 1, NO_PRINT_VALUES); 
									
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
						MRd32(mem_ctrl + 0x28, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ====================================");
						printf("\n === NVRAM MODULE (HW parameters) ===");
						printf("\n =====      Version %2.2x.%2.2x       =====\n\n",data_read[0]&0xff,data_read[1]&0xff); 
						MRd32(mem_ctrl + 0x3C, data_read, 2, 1, NO_PRINT_VALUES); 
						printf("\n ====================================");
						printf("\n ===       Platform Version       ===");
						printf("\n =====      Version %2.2x.%2.2x       =====",data_read[0]&0xff,data_read[1]&0xff); 
						
						MRd32(mem_ctrl + 0x36, data_read, 1, 1, NO_PRINT_VALUES);   
						printf("\n\n === Chip Number  ==="); 
						printf("\n Number of Chip = %d", (pot2(data_read[0]>>4&0x03)));						
							if ( ((data_read[0])&0x0F)==0 )
							{
								printf("\n Chip Size \t= 0 Mbit\n\n\n");	
							} 
							else
							{
								printf("\n Chip Size \t= %.0lf Mbit\n\n\n", pow(2, ((data_read[0]&0x0F)+3) ));
							} 
													
						
						uint32_t sram_waccess_type,sram_raccess_type;
						uint32_t nvram_sel;
						
						uint8_t *writeBuffer;
						uint8_t *readBuffer;
						uint8_t *ctrlBuffer;
						uint32_t sram_mode_num;
						uint32_t start_address;
						uint32_t wr_offset;
						uint32_t rd_offset;
						
						uint32_t x;
						uint32_t i;
						uint32_t test_size;
						uint32_t j;	
						uint32_t data_type;
						
						printf("-- =================================\n");
						printf("--       SRAM command               \n");
						printf("--  (0) Get MRAM bar 1 info         \n");						
						printf("--  (1) Write Access                \n");
						printf("--  (2) Read Access                 \n");
						printf("--  (3) Test Write Read Access      \n");
						printf("--  (4) Not Alligned chiappa su     \n");
						printf("--  (5) Exit                        \n");
						printf("-- =================================\n");
						
						
						printf("Selection [1..4] ? : ");
						if ((ret_code=scanf("%d", &nvram_sel))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						switch(nvram_sel)
						{
							case 0:
								{	
									bufferSize = 64;  
									sram_raccess_type = 8;									
									
									ctrlBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
									printf("\n\n GET BAR1 image see below:  ");
									i=0;
									while (i < 64)
										{
											MRd32(mem_ctrl+i, &ctrlBuffer[i], sram_raccess_type, sram_raccess_type, 1);//NO_PRINT_VALUES);
									
											/*for(j = i; j < (i+sram_access_type); j++)
											{
												if ((writeBuffer[j]!=ctrlBuffer[j]) && j<256) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
											}*/
											i += sram_raccess_type;		
		  
										}
									
									free(ctrlBuffer);
								break;
								} 
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
									printf("--  (4) Dword_type      \n");
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
									
/* 												// debug
												bufferSize = 16;  
												sram_raccess_type = 4;	
												ctrlBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
												MRd32(mem_ctrl+0x34, &ctrlBuffer[0], sram_raccess_type, sram_raccess_type, 1);//NO_PRINT_VALUES);
												free(ctrlBuffer); */
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
										case 16:
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
									switch(sram_raccess_type)
									{
										case 1: // 1 byte
										break;

										case 2 :	// 2 bytes - 1Word
										break;
										
										case 4:	// 4 bytes - 1DWord
											printf("\n");
											printf("Read Mem Access Type:  Byte\n");
											printf("Address (hex)          Data value (hex)\n");
											printf("                       |          Dword|\n");
										break;
									
										default:
											//printf("Error! wrong byte alligned value: type %d expected 1, 2 or 4 \n",byte_alligned);
										break;	
									};
/* 												// debug
												bufferSize = 4;  
												sram_raccess_type = 4;	
												ctrlBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
												MRd32(mem_ctrl+0x34, &ctrlBuffer[i], sram_raccess_type, sram_raccess_type, 1);//NO_PRINT_VALUES);
												free(ctrlBuffer); */
									
									printf("mem address is: %8.8x \n",(uint64_t)(mem_addr));
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
									if (test_size > bufferSize) test_size=8388608;//bufferSize;

									
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
						uint32_t total_rpt=0;
						//while (rpt < 12)
						//while (rpt < 13)
						while (rpt < 13)
						{			
									switch(rpt)
									{
										case 0:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 2;  // Word
												rpt++; // check if number of iteration is infinite	
												break;
											} 
										case 1:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 4;  // Dword
												rpt++; // check if number of iteration is infinite	
												break;
											} 
										case 2:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 2;  // Word
												rpt++; // check if number of iteration is infinite	
												break;
											}  
										case 3:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 4;  // Dword
												rpt++; // check if number of iteration is infinite	
												break;
											}  
										case 4:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												rpt++; // check if number of iteration is infinite	
												break;
											} 
										case 5:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 4;  // Dword
												rpt++; // check if number of iteration is infinite	
												break;
											} 
										case 6:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 8;  // Qword
												rpt++; // check if number of iteration is infinite	
												break;
											}  
										case 7:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 8;  // Qword
												rpt++; // check if number of iteration is infinite	
												break;
											}  
										case 8:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 8;  // Qword
												rpt++; // check if number of iteration is infinite	
												break;
											}  
										case 9:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 2;   // Word
												rpt++; // check if number of iteration is infinite	
												break;
											} 
										case 10:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 4;   // Dword
												rpt++; // check if number of iteration is infinite	
												break;
											} 
										case 11:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 8;   // Qword
												rpt++; // check if number of iteration is infinite	
												break;
											} 
										default:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 4;  // Dword
												rpt=0; // check if number of iteration is infinite	
												break;
											}  
									} 
									total_rpt++;
									printf("Incremental rpt are: %d.\n", total_rpt);
									
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
												if (writeBuffer[j]!=readBuffer[j]) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
											}/**/
											MRd32(mem_addr+i+start_address, &readBuffer[i], sram_raccess_type, sram_raccess_type, NO_PRINT_VALUES);//1);//NO_PRINT_VALUES);
									
											/**/for(j = i; j < (i+sram_raccess_type); j++)
											{
												if (writeBuffer[j]!=readBuffer[j]) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
											}/**/
											
											i += sram_raccess_type;		
		  
										}
									
										
								
									//wait_to_continue();
									usleep(1*1000*1000);
						}	
						
								free(writeBuffer);
								free(readBuffer);
								break;
								}
							case 4:
								{	
									wr_offset 	= 0;
									rd_offset 	= 0;
									bufferSize  = nvramMaxSize; 
									readBuffer  = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
									writeBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
									
									printf("\n\n How many byte to write ? [0..%d]: ",bufferSize);
									if ((ret_code=scanf("%d", &test_size))!=1)
									{
										printf("function read error %d\n",ret_code);
									};
									
									// max write byte size is Memory max size
									if (test_size > bufferSize) test_size=8388608;//bufferSize;

									
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
						
						uint32_t case_type=0;
						//while (rpt < 12)
						//while (rpt < 13)
						//while (case_type < 24)
							uint32_t rpt=0;
						while (rpt < 24)
						{			
									switch(rpt)//case_type)
									{
										case 0:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 2;  // Word
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											} 
										case 1:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 4;  // Dword
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											} 
										case 2:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 4;  // Dword
												wr_offset		  = 2;
												rd_offset		  = 2;
												break;
											} 
										case 3:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 4;  // Dword
												wr_offset		  = 3;
												rd_offset		  = 3;
												break;
											} 
										case 5:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											}  
										case 6:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 2;
												rd_offset		  = 2;
												break;
											}  
										case 7:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 3;
												rd_offset		  = 3;
												break;
											}  
										case 8:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 4;  // Dword
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											}  
										case 9:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 4;  // Dword
												wr_offset		  = 2;
												rd_offset		  = 2;
												break;
											} 
										case 10:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 4;  // Dword
												wr_offset		  = 3;
												rd_offset		  = 3;
												break;
											} 
										case 11:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											} 
										case 12:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 2;
												rd_offset		  = 2;
												break;
											} 
										case 13:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 3;
												rd_offset		  = 3;
												break;
											} 
										case 14:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 4;
												rd_offset		  = 4;
												break;
											} 
										case 15:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 5;
												rd_offset		  = 5;
												break;
											} 
										case 16:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 6;
												rd_offset		  = 6;
												break;
											} 
										case 17:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 2;  // Word
												wr_offset		  = 7;
												rd_offset		  = 7;
												break;
											} 
										case 18:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 4;  // Dword
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											} 
										case 19:
											{	
												sram_waccess_type = 2;  // Word
												sram_raccess_type = 8;  // Qword
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											}  
										case 20:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 8;  // Qword
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											}  
										case 21:
											{	
												sram_waccess_type = 8;  // Qword
												sram_raccess_type = 8;  // Qword
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											}  
										case 22:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 2;   // Word
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											} 
										case 23:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 4;   // Dword
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											} 
										case 24:
											{	
												sram_waccess_type = 16;  // Oword
												sram_raccess_type = 8;   // Qword
												wr_offset		  = 1;
												rd_offset		  = 1;
												break;
											} 
										default:
											{	
												sram_waccess_type = 4;  // Dword
												sram_raccess_type = 4;  // Dword
												wr_offset		  = 1;
												rd_offset		  = 1;
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
										
									printf("Write @ %d bytes access type and read @ %d bytes access type from/to address %d...\n", sram_waccess_type,sram_raccess_type,wr_offset);
									i = 0;
									printf("write buffer...\n");												
									while (i < test_size)//*1000)
										{
											MWr32(mem_addr+i+start_address+wr_offset, &writeBuffer[i], sram_waccess_type, sram_waccess_type, NO_PRINT_VALUES); 
											i += sram_waccess_type;		
										}
											
									i = 0;
									printf("read buffer...\n");
									while (i < test_size)
										{
											MRd32(mem_addr+i+start_address+rd_offset, &readBuffer[i], sram_raccess_type, sram_raccess_type, NO_PRINT_VALUES);//1);//NO_PRINT_VALUES);
									
											/**/for(j = i; j < (i+sram_raccess_type); j++)
											{
												if (writeBuffer[j]!=readBuffer[j]) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
											}/**/
			//								MRd32(mem_addr+i+start_address, &readBuffer[i], sram_raccess_type, sram_raccess_type, NO_PRINT_VALUES);//1);//NO_PRINT_VALUES);
			//							
			//									/**/for(j = i; j < (i+sram_raccess_type); j++)
			//									{
			//										if (writeBuffer[j]!=readBuffer[j]) printf("\n Error at address %d: Expected: 0x%2.2x got: 0x%2.2x\n",j,writeBuffer[j]&0xff,readBuffer[j]&0xff); 
			//									}/**/
			//									
											i += sram_raccess_type;		
		  
										}
									
										
								
									rpt++; // check if number of iteration is infinite	
								
									//wait_to_continue();
									usleep(1*1000*1000);
						}	
						
								free(writeBuffer);
								free(readBuffer);
								break;
								}
							case 6:
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
			
				case 3 : 
					{
						
						printf("\n");	
						uint32_t rpt=0;
						uint32_t toterror=0;
						while (rpt++ < 5000000)
						{											
							IORd(pcie_bar_io[0] + 0x2A, data_read, 2, 1, NO_PRINT_VALUES); 	
							//
							//usleep(1);
							//usleep(1*1000*1000);	
							if (data_read[1] != 0x02)	
							{
								printf("num: %d FPGA ver.Min\t= %2.2x\n", rpt, data_read[1]&0xff);
								printf("\n chiappa su \n\n\n");	
								toterror++;	
							} 		
						//wait_to_continue();			
								
						} 					
						printf("\n %d \n",toterror);		
						wait_to_continue();
						break;
						return 0;
					} 
						
				case 4 : 
					{
						
						uint32_t sram_waccess_type,sram_raccess_type;
						
						uint8_t *writeBuffer;
						
						uint32_t x;
						uint32_t i;
						uint32_t test_size;
						uint32_t j;	
							
					
							bufferSize = nvramMaxSize;							
							writeBuffer = (uint8_t*)calloc(bufferSize, sizeof(uint8_t));
									
							test_size=8388608;
							printf("Assigning random values to a %u bytes local buffer...\n", test_size);
							for (x = 0; x < test_size; x++)
							*(writeBuffer + x) = (uint8_t)(rand() & 0x000000FF);
								
						while (1)
						{			
							
							sram_waccess_type = 4;  // Dword
							sram_raccess_type = 4;  // Dword
									
										
							printf("Write @ %d bytes access type and read @ %d bytes access type...\n", sram_waccess_type,sram_raccess_type);
							i = 0;
							printf("write buffer...\n");												
							while (i < test_size)//*1000)
								{
									MWr32(mem_addr+i, &writeBuffer[i], sram_waccess_type, sram_waccess_type, NO_PRINT_VALUES); 
									i += sram_waccess_type;		
								}
							usleep(1*1000*1000);
						}	
						
						free(writeBuffer);
						wait_to_continue();
						
						break;
						return 0;
					}  	
					
				case 5 : 
					{
						
						data_write[0] = 0xF1;
						data_write[1] = 0xCA;
						printf("\n");	
						uint32_t rpt=0;
						uint32_t toterror=0;
						while (rpt++ < 5000000)
						{	
							IOWr(pcie_bar_io[0] + 0x38, data_write, 1, 1, 0, NO_PRINT_VALUES);
							IORd(pcie_bar_io[0] + 0x38, data_read, 1, 1, NO_PRINT_VALUES); 	
							//
							//usleep(1);
							//usleep(1*1000*1000);	
						/*
							if (data_read[0] != 0xF1)	
							{
								//printf("num: %d FPGA ver.Min\t= %2.2x\n", rpt, data_read[1]&0xff);
								printf("\n chiappa su \n\n\n");	
								toterror++;	
							} */		
						//wait_to_continue();			
								
						} 					
						printf("\n %d \n",toterror);		
						wait_to_continue();
						break;
						return 0;
					} 
					
				case 10:
					{
						/*
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
						};*/

						printf("\nInsert one of the following Chip size \n");
						printf("   ====================================\n");
						printf("   0  0 Mbit SRAMs    8   TBD           \n");
						printf("   1  16 Mbit SRAMs   9   TBD           \n");
						printf("   2  32 Mbit SRAMs   10  TBD           \n");
						printf("   3  64 Mbit SRAMs   11  TBD           \n");
						printf("   4  128 Mbit SRAMs  12  TBD           \n");
						printf("   5  TBD             13  TBD           \n");
						printf("   6  TBD             14  TBD           \n");
						printf("   7  TBD             15  TBD           \n");
						printf("   ====================================\n");
						printf("Which is the chip size ? [0..15] ");
						if ((ret_code=scanf("%d", &chipsize))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						
						/*
						printf("Which SRAM type is mounted ? [0 - SRAM, 1 - FAST SRAM, 2 - MRAM] ");
						if ((ret_code=scanf("%d", &sramtype))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						*/
						//byte0 = (((sramtype & 0xFF)<<6) | ((chipsize & 0xFF)<<2) | ((nbanks-1) & 0xFF));
						//data_write[0]=(((nbanks) & 0x03)<<4) | (chipsize & 0x0F));
						data_write[0]=(0x10 | (chipsize & 0x0F));
						printf("byte is %2.2x\n",data_write[0]);
						
						
						MWr32(mem_ctrl + 0x36, data_write, 1, 1, NO_PRINT_VALUES); 
						#if _DEBUG
						{
							//printf("nbanks is %2.2x\n",(pot2(nbanks) & 0xFF));
							printf("chipsize is %2.2x\n",(chipsize & 0xFF));
						}; 
						#endif
						
						wait_to_continue();
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
						uint32_t x,i;
						uint32_t key_idx;
						char key_enc_file_name[]="EncKeys.bin";
						int32_t key_file;
						sizeSecsBuffer = 4096;
						uint32_t size;
						uint8_t *writeBuffer;
						uint8_t *key_encr  	= (uint8_t *)calloc (sizeSecsBuffer,sizeof(uint8_t));
						writeBuffer = (uint8_t*)calloc(sizeSecsBuffer, sizeof(uint8_t));
						
						if (key_enc_file_name) {
								key_file = open(key_enc_file_name, O_RDONLY);						
										
								if (key_file >= 0) 
									{					
										size = read(key_file, key_encr, sizeSecsBuffer);
										if (size == 0)
										{
											printf("FAILURE: file  %s. is empty\n", key_enc_file_name);								
										};
										close(key_file);
										
										//printf("\nPROGRAM DONE\n");									
										exit=0;	
									}								
								else 
									{			
										printf("FAILURE: error opening %s. Abort\n", key_enc_file_name);								
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
												printf("%2.2x",key_encr[i]);	
										}
									}
							}; 
						#endif 
						
						MRd32(mem_ctrl + 0xBE, data_read, 1, 1, NO_PRINT_VALUES); 
						//printf("\n User Mode    \t= %s", USER(data_read[0]&0x01));
						printf("\n BIOS_Key written    \t= %s\n", BIOS((data_read[0]>>1)&0x01));
						//printf("\n LP master key ready \t= %s", M_KEY_RDY((data_read[0]>>2)&0x01));
						//printf("\n expanded 17th key \t= %s", EXP17THKEY((data_read[0]>>3)&0x01));
						
						for (x = 0; x < sizeSecsBuffer; x++)
												*(writeBuffer + x) = (uint8_t)(0xFF);
						for (x = 0; x < sizeSecsBuffer/8; x++)
							*(writeBuffer + x) = *(key_encr+x);

						for(i = 0; i < 512; i++)
						{	
							MWr32(mem_aes + i, &writeBuffer[i], 1, 1, NO_PRINT_VALUES); 
							printf("\n address: %d data_wr %2.2x",i,writeBuffer[i]); 
						
						}			
						
						//MWr32(mem_addr_secs, writeBuffer, 512, 1, 0);//NO_PRINT_VALUES); 
						MRd32(mem_ctrl + 0xBE, data_read, 1, 1, NO_PRINT_VALUES); 
						//printf("\n User Mode    \t= %s", USER(data_read[0]&0x01));
						printf("\n BIOS_Key written    \t= %s\n", BIOS((data_read[0]>>1)&0x01));
						//printf("\n LP master key ready \t= %s", M_KEY_RDY((data_read[0]>>2)&0x01));
						//printf("\n expanded 17th key \t= %s", EXP17THKEY((data_read[0]>>3)&0x01));
						
						free(writeBuffer);
						wait_to_continue();
						break;
					}			
					
				case 46:
					{
						uint8_t master_key[32] = "\xBC\x97\x50\xE1\xAA\xC8\x32\x96\xDA\x6F\x68\xE1\x2C\x26\x77\x19";  

						#if _DEBUG	
							{	
								for (int j = 0; j<16; j++)
									{
									printf("\nla chiave %d è: ",j);	
									for (int i = (32*j); i<(32*(j+1)); i++)
										{
												printf("%2.2x",master_key[i]);	
										}
									}	
							}; 
						#endif 
						
						printf("-- ========================================\n");
						printf("--       Program Master Key injection      \n");		
						printf("--                                         \n");		
						printf("-- ========================================\n");
						
						MRd32(mem_ctrl + 0xA3, data_read, 1, 1, NO_PRINT_VALUES); 
						printf("\n\n\n ======================================= \t");
						printf("\n =========== Debug Part =========== \t");
						printf("\n\n == Master Key Ready flag == \t %s",MASTER_KEY_FROM_LP((data_read[0]>>7)&0x01));
						printf("\n == Autentication Status == \t %s",AUTHENTICA((data_read[0]>>6)&0x01));
						printf("\n\n ======================================= \t");
						
						data_write[0]=0x81;
						MWr32(mem_ctrl + 0xAB, data_write, 1, 1, NO_PRINT_VALUES); 
						MRd32(mem_ctrl + 0xAB, data_read, 1, 1, NO_PRINT_VALUES); 
						
						printf("\n AES mode register: %2.2x",data_read[0]);
						
						printf("** Set Expansion Key command **");
						data_write[0]=0x04;
						MWr32(mem_ctrl + 0xA3, data_write, 1, 1, NO_PRINT_VALUES); 
						do{ 
							MRd32(mem_ctrl + 0x22, data_read, 1, 1, 1);//NO_PRINT_VALUES); 
							usleep(15);
							} while (!((data_read[0]>>2) & 0x01));
						data_write[0]=0x00;
						MWr32(mem_ctrl + 0xA3, data_write, 1, 1, NO_PRINT_VALUES); 
						printf("** Expansion Done **");
						MWr32(mem_aes, master_key, 16, 4, 1);//NO_PRINT_VALUES); 
						data_write[0]=0x00;
						MWr32(mem_aes + 2047, data_write, 1, 1, NO_PRINT_VALUES); 
						
						printf("** Set Encrypt command **");
						data_write[0]=0x01;
						MWr32(mem_ctrl + 0xA3, data_write, 1, 1, NO_PRINT_VALUES); 
				   		do{ 
							MRd32(mem_ctrl + 0x22, data_read, 1, 1, 1);//NO_PRINT_VALUES); 
							usleep(150);
							} while (!(data_read[0] & 0x01));
						data_write[0]=0x00;
						MWr32(mem_ctrl + 0xA3, data_write, 1, 1, NO_PRINT_VALUES); 		
						printf("** Encrypt Done **");			
						
						
						data_write[0]=0x00;
						MWr32(mem_ctrl + 0xAB, data_write, 1, 1, NO_PRINT_VALUES); 
						MRd32(mem_ctrl + 0xAB, data_read, 1, 1, NO_PRINT_VALUES); 
						printf("\n AES mode register: %2.2x",data_read[0]); 
					
						printf("\n\n\n ======================================= \t");
						printf("\n == Master Key INJECTED to LP =="); 
						MRd32(mem_ctrl + 0xC0, data_read, 4, 1, NO_PRINT_VALUES);
						printf("\n Key 127..0:\t 0x%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						MRd32(mem_ctrl + 0xC0, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);  
						MRd32(mem_ctrl + 0xC0, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						MRd32(mem_ctrl + 0xC0, data_read, 4, 1, NO_PRINT_VALUES);
						printf(".%2.2x%2.2x%2.2x%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff);
						printf("\n ======================================= \t\n\n");						
							
						
					/* 		
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
						 */
						wait_to_continue();
						break;
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
					
				case 70 : 
					{
						
							
						printf("-- ============================\n");
						printf("--       Get DIN status        \n");	
						printf("-- ============================\n");
						
						for(i = 0; i < 32; i++)
						{
							IORd(pcie_bar_io[0] + 0x00, data_read, 4, 1, NO_PRINT_VALUES);      
							printf("\n Din[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							IORd(pcie_bar_io[0] + 0x04, data_read, 4, 1, NO_PRINT_VALUES);      
							printf("\n Din[63..32] = 0x%2.2x.%2.2x.%2.2x.%2.2x\n",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							
							usleep(1*1000*1000);
						}
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
						printf(" Select Dout[8..15] value ? : 0x");
						if ((ret_code=scanf("%2hhx", &data_read[1]))!=1)
						{
							printf("function read error %d\n",ret_code);
						};	
						printf(" Select Dout[15..23] value ? : 0x");
						if ((ret_code=scanf("%2hhx", &data_read[2]))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf(" Select Dout[24..31] value ? : 0x");
						if ((ret_code=scanf("%2hhx", &data_read[3]))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("-- ============================\n");
						printf("--          Dout Set           \n");
						printf("-- ============================\n");
						printf("\n\n");
						
						printf("-- ============================\n");
						// all FF
						
							data_write[0] = data_read[0];//0xff;  	
							data_write[1] = data_read[1];//0xff;  	
							data_write[2] = data_read[2];//0xff;   	
							data_write[3] = data_read[3];//0xff;  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
			/*
							data_write[4] = data_read[0];//0xff; 
							data_write[5] = data_read[1];//0xff; 
							data_write[6] = data_read[2];//0xff; 	
							data_write[7] = data_read[3];//0xff; 
							
							IOWr(pcie_bar_io[0] + 0x1C, data_write, 1, 1, 4, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1D, data_write, 1, 1, 5, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1E, data_write, 1, 1, 6, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1F, data_write, 1, 1, 7, NO_PRINT_VALUES); 
				*/	
										
							printf("\n");
							printf("-- ============================\n");
								
							usleep(500*1000);
						printf("--     Check Feedback:        \n");
							
						//printf("\n Set Dout[31..0] = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_write[3]&0xff,data_write[2]&0xff,data_write[1]&0xff,data_write[0]&0xff); 
						IORd(pcie_bar_io[0] + 0x18, data_read, 4, 1, NO_PRINT_VALUES);    
						printf("\n Got Fb[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							/*
							expected_data[0]=0xff-data_write[0];
							expected_data[1]=0xff-data_write[1];
							expected_data[2]=0xff-data_write[2];
							expected_data[3]=0xff-data_write[3];*/
							
						IORd(pcie_bar_io[0] + 0x1C, data_read, 4, 1, NO_PRINT_VALUES);      
						printf("\n Got Fb[63..32] = 0x%2.2x.%2.2x.%2.2x.%2.2x\n",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							/*
							expected_data[4]=0xff-data_write[4];
							expected_data[5]=0xff-data_write[5];
							expected_data[6]=0xff-data_write[6];
							expected_data[7]=0xff-data_write[7];
							
							if (data_read[0]!=expected_data[0]) printf("\n Error at byte 0: Expected: 0x%2.2x got: 0x%2.2x",expected_data[0]&0xff,data_read[0]&0xff); 
							if (data_read[1]!=expected_data[1]) printf("\n Error at byte 1: Expected: 0x%2.2x got: 0x%2.2x",expected_data[1]&0xff,data_read[1]&0xff); 
							if (data_read[2]!=expected_data[2]) printf("\n Error at byte 2: Expected: 0x%2.2x got: 0x%2.2x",expected_data[2]&0xff,data_read[2]&0xff); 
							if (data_read[3]!=expected_data[3]) printf("\n Error at byte 3: Expected: 0x%2.2x got: 0x%2.2x",expected_data[3]&0xff,data_read[3]&0xff); 
								*/
						wait_to_continue();
						break;
						return 0;
					}  
					
					
				case 74 : 
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
						printf(" Select Dout[8..15] value ? : 0x");
						if ((ret_code=scanf("%2hhx", &data_read[1]))!=1)
						{
							printf("function read error %d\n",ret_code);
						};	
						printf(" Select Dout[15..23] value ? : 0x");
						if ((ret_code=scanf("%2hhx", &data_read[2]))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf(" Select Dout[24..31] value ? : 0x");
						if ((ret_code=scanf("%2hhx", &data_read[3]))!=1)
						{
							printf("function read error %d\n",ret_code);
						};
						printf("-- ============================\n");
						printf("--          Dout Set           \n");
						printf("-- ============================\n");
						printf("\n\n");
						
						printf("-- ============================\n");
						// all FF
						// walking 1
						
							data_write[0] = data_read[0];//0xff;  	
							data_write[1] = data_read[1];//0xff;  	
							data_write[2] = data_read[2];//0xff;   	
							data_write[3] = data_read[3];//0xff;  		
							IOWr(pcie_bar_io[0] + 0x18, data_write, 1, 1, 0, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x19, data_write, 1, 1, 1, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1A, data_write, 1, 1, 2, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1B, data_write, 1, 1, 3, NO_PRINT_VALUES); 
						
			
							/*data_write[4] = data_read[0];//0xff; 
							data_write[5] = data_read[1];//0xff; 
							data_write[6] = data_read[2];//0xff; 	
							data_write[7] = data_read[3];//0xff; 
							
							IOWr(pcie_bar_io[0] + 0x1C, data_write, 1, 1, 4, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1D, data_write, 1, 1, 5, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1E, data_write, 1, 1, 6, NO_PRINT_VALUES); 
							IOWr(pcie_bar_io[0] + 0x1F, data_write, 1, 1, 7, NO_PRINT_VALUES); 
					*/
										
							printf("\n");
							printf("-- ============================\n");
								
							usleep(500*1000);
						
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
	
				case 75 : 
					{
						
						printf("-- ============================\n");
						printf("--       Check Feedback        \n");	
						printf("-- ============================\n");
						/*
						IORd(pcie_bar_io[0] + 0x00, data_read, 4, 1, NO_PRINT_VALUES);      
						printf("\n Din[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						IORd(pcie_bar_io[0] + 0x04, data_read, 4, 1, NO_PRINT_VALUES);      
						printf("\n Din[63..32] = 0x%2.2x.%2.2x.%2.2x.%2.2x\n",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						*/
						//printf("--     Check Feedback:        \n");
							
						IORd(pcie_bar_io[0] + 0x18, data_read, 4, 1, NO_PRINT_VALUES);    
						printf("\n Got Fb[31..0]  = 0x%2.2x.%2.2x.%2.2x.%2.2x",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
							
						IORd(pcie_bar_io[0] + 0x1C, data_read, 4, 1, NO_PRINT_VALUES);      
						printf("\n Got Fb[63..32] = 0x%2.2x.%2.2x.%2.2x.%2.2x\n",data_read[3]&0xff,data_read[2]&0xff,data_read[1]&0xff,data_read[0]&0xff); 
						
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



			