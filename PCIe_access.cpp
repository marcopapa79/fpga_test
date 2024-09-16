/*
* =====================================================================================
*
*       Filename:  PCIe_access.c
*
*    Description:  FPGA registers access function
*
*        Version:  1.0
*        Created:  22/01/2020
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

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>
#endif


#include <stdio.h>
#include "PCIe_access.h"


	// *******************************
	// ** PCIe Function Declaration **
	// *******************************
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  IOWr
		 *  Description:  IO Write, 32/16/8 bit access, 
		 *  Requiremets:  io_addr must be aligned to 32/16/8 bit and length must be an integer
		 *                multiple of 4/2/1.
		 *                As no check is done, if the requirements are not meet, the result is
		 *                unpredictable!
		 *
		 *				  byte_alligned = is the type of access 1=byte,2=word,4=dword
		 *                debug_print   = 2 print data written and readback from same io_addr
		 *								  1 print data writtten 
		 *                                0 no debug print
		 *                start_index   = index of uint8_t reg_we_value 
		 * =====================================================================================
		 */
		void IOWr (unsigned short int io_addr, uint8_t * reg_wr_value, unsigned int length, unsigned int byte_alligned, unsigned int start_index, unsigned int debug_print)  
		{	

			uint16_t * word_src 	= (uint16_t *)reg_wr_value;
			uint32_t * dword_src 	= (uint32_t *)reg_wr_value;	
			switch(byte_alligned) 
					{	
						case 1: // 1 byte
					  
							for (unsigned int n=0; n<length; n++) 
								{
									outb_p(*(reg_wr_value+start_index+n), io_addr+n);
									//*(mem_addr+n) = *(reg_wr_value+n);									
								}
									switch(debug_print) 
										{
											case 0: // no print
											break;
											
											case 1: // print written data												
												for (unsigned int j=0;j<length;j++) printf("wr_data %2.2x ",*(reg_wr_value+start_index+j));
												printf("\n");
											break;
											case 2: // print written data and read data												
												for (unsigned int j=0;j<length;j++) printf("wr_data %2.2x ",*(reg_wr_value+start_index+j));
												printf("\n");
												for (unsigned int j=0;j<length;j++) printf("rd_data %2.2x ",(io_addr+j));
												printf("\n");
											break;
										}; 					
						break;	
						
						case 2 :	// 2 bytes - 1Word
						
							for (unsigned int n=0; n<(length/2); n++) 
								{
									outw_p(*(reg_wr_value+start_index+n), io_addr+(n*2));
								}

									switch(debug_print) 
										{
											case 0: // no print
											break;
											
											case 1: // print written data	
												for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, reg_wr_value[j]);
												for (unsigned int j=0;j<(length/2);j++) printf("word written: %4.4x \n",*(word_src+j));						
											break;
											case 2: // print written data and read data	
												for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, reg_wr_value[j]);
												for (unsigned int j=0;j<(length/2);j++) printf("word written: %4.4x \n",*(word_src+j));
												for (unsigned int j=0;j<(length/2);j++) printf("readback word: %4.4x \n",(io_addr+(j*2)));
											break;
										}; 									
						break;	
						
						case 4:	// 4 bytes - 1DWord				
							for (unsigned int n=0; n<(length/4); n++) 
								{
									outl_p(*(reg_wr_value+start_index+n), io_addr+(n*4));
								}
							
								switch(debug_print) 
									{	
										case 0: // no print
										break;
										case 1: // print written data	
											for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, reg_wr_value[j]);
											for (unsigned int j=0;j<(length/4);j++) printf("dword written: %8.8x \n",*(dword_src+j));						
										break;
										case 2: // print written data and read data	
											for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, reg_wr_value[j]);
											for (unsigned int j=0;j<(length/4);j++) printf("dword written: %8.8x \n",*(dword_src+j));
											for (unsigned int j=0;j<(length/4);j++) printf("readback dword: %8.8x \n",(io_addr+(j*4)));
										break;
									};
						break;						
				
					};
				
				
					
						
			}
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  IORd
		 *  Description:  IO Read, 32/16/8 bit access, 
		 *  Requiremets:  io_addr must be aligned to 32/16/8 bit and length must be an integer
		 *                multiple of 4/2/1.
		 *                As no check is done, if the requirements are not meet, the result is
		 *                unpredictable!
		 *
		 *				  byte_alligned = is the type of access 1=byte,2=word,4=dword
		 *                debug_print   = 1 print data read, 
		 *								  0 no data printed
		 * =====================================================================================
		 */
		void * IORd(unsigned short int io_addr, uint8_t * reg_value, unsigned int length, unsigned int byte_alligned, unsigned int debug_print)
		{
			uint16_t * word_dst 	= (uint16_t *)reg_value;		
			uint32_t * dword_dst 	= (uint32_t *)reg_value;
			
			switch(byte_alligned) 
				{	
					case 1: // 1 byte
						for (unsigned int n=0; n<length; n++) 
							{
								*(reg_value+n) = inb_p(io_addr+n);
							}
							if (debug_print == 1) 
								{		
									for (unsigned int j=0;j<length;j++) printf(" \nbyte read is: %2.2x \n",*(reg_value+j));
								}; 
					break;	
					case 2 :	// 2 bytes - 1Word
						for (unsigned int n=0; n<(length/2); n++) 
							{
								*(word_dst+n) = inw_p(io_addr+(n*2));
							}
							
							if (debug_print == 1) 
								{		
									for (unsigned int j=0;j<(length/2);j++) printf(" \nword read is: %4.4x \n",*(word_dst+j));
								}; 
							
					break;	
					case 4:	// 4 bytes - 1DWord				
						for (unsigned int n=0; n<(length/4); n++) 
							{
								*(dword_dst+n) = inl_p(io_addr+(n*4));
							}
						
							if (debug_print == 1) 
								{		
									for (unsigned int j=0;j<(length/4);j++) printf(" \ndword read is: %8.8x \n",*(dword_dst+j));
								}; 			
					break;
				};
			return reg_value;	
			
		}
		/*   
       uint8_t inb_p(unsigned short int port);
       unsigned short int inw_p(unsigned short int port);
       unsigned int inl_p(unsigned short int port);

       void outb_p(uint8_t value, unsigned short int port);
       void outw_p(unsigned short int value, unsigned short int port);
       void outl_p(unsigned int value, unsigned short int port);
*/
		
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  MRd32
		 *  Description:  Memory Read, 64/32/16/8 bit access, 
		 *  Requiremets:  mem_addr must be aligned to 64/32/16/8 bit and length must be an integer
		 *                multiple of 8/4/2/1.
		 *                As no check is done, if the requirements are not meet, the result is
		 *                unpredictable!
		 *
		 *				  byte_alligned = is the type of access 1=byte,2=word,4=dword,8=quadword
		 *                debug_print   = 1 print data read, 
		 *								  0 no data printed
		 * =====================================================================================
		 */
		void * MRd32 (uint8_t * mem_addr, uint8_t * read_buffer, unsigned int length, unsigned int byte_alligned, unsigned int debug_print) 
		{
			
			uint16_t * word_dst 	= (uint16_t *)read_buffer;
			uint16_t * word_src 	= (uint16_t *)mem_addr;
			uint32_t * dword_dst 	= (uint32_t *)read_buffer;
			uint32_t * dword_src 	= (uint32_t *)mem_addr;				
			uint64_t * qword_dst 	= (uint64_t *)read_buffer;
			uint64_t * qword_src 	= (uint64_t *)mem_addr;
			
			switch(byte_alligned) 
				{	
					case 1: // 1 byte
				  
						for (unsigned int n=0; n<length; n++) 
							{
								*(read_buffer+n) = *(mem_addr+n);
							}
							if (debug_print == 1) 
								{		
									for (unsigned int j=0;j<length;j++) printf("byte read is: %2.2x \n",*(read_buffer+j));
								}; 
					break;	
					
					case 2 :	// 2 bytes - 1Word
					
						for (unsigned int n=0; n<(length/2); n++) 
							{
								*(word_dst+n) = *(word_src+n);
							}

							if (debug_print == 1) 
								{		
									for (unsigned int j=0;j<(length/2);j++) printf("word read is: %4.4x \n",*(word_dst+j));
								}; 
					break;	
					
					case 4:	// 4 bytes - 1DWord				
						for (unsigned int n=0; n<(length/4); n++) 
							{
								*(dword_dst+n) = *(dword_src+n);
							}
						
							if (debug_print == 1) 
								{		
									for (unsigned int j=0;j<(length/4);j++) printf("dword read is: %8.8x \n",*(dword_dst+j));
								}; 
					break;
					
					case 8:	// 8 bytes - 1QWord				
						for (unsigned int n=0; n<(length/8); n++) 
							{
								*(qword_dst+n) = *(qword_src+n);
							}
							if (debug_print == 1)
								{		
									for (unsigned int j=0;j<(length/8);j++) printf("qword read is: %16.16lx \n",*(qword_dst+j));
								}; 
					break;
			
				};
					
			return read_buffer;	
			
		}
	
		/* 
		 * ===  FUNCTION  ======================================================================
		 *         Name:  MWr32
		 *  Description:  Memory Write, 64/32/16/8 bit access, 
		 *  Requiremets:  mem_addr must be aligned to 64/32/16/8 bit and length must be an integer
		 *                multiple of 8/4/2/1.
		 *                As no check is done, if the requirements are not meet, the result is
		 *                unpredictable!
		 *
		 *				  byte_alligned = is the type of access 1=byte,2=word,4=dword,8=quadword
		 *                debug_print   = 2 print data written and readback from same mem_addr
		 *								  1 print data writtten 
		 *                                0 no debug print
		 * =====================================================================================
		 */
			void MWr32 (uint8_t * mem_addr, uint8_t * write_buffer, unsigned int length, unsigned int byte_alligned, unsigned int debug_print)  
			{	

				uint16_t * word_dst 	= (uint16_t *)mem_addr;
				uint16_t * word_src 	= (uint16_t *)write_buffer;
				uint32_t * dword_dst 	= (uint32_t *)mem_addr;
				uint32_t * dword_src 	= (uint32_t *)write_buffer;				
				uint64_t * qword_dst 	= (uint64_t *)mem_addr;
				uint64_t * qword_src 	= (uint64_t *)write_buffer;
				
				uint128_t *oword_dst 	= (uint128_t *)mem_addr;
				uint128_t *oword_src 	= (uint128_t *)write_buffer;
				uint256_t *hword_dst 	= (uint256_t *)mem_addr;
				uint256_t *hword_src 	= (uint256_t *)write_buffer;
				//uint8_t octword_src[16] = {0};
				//uint8_t oword_src[16] = write_buffer;
				
				switch(byte_alligned) 
					{	
						case 1: // 1 byte
					  
							for (unsigned int n=0; n<length; n++) 
								{
									*(mem_addr+n) = *(write_buffer+n);
								}
									switch(debug_print) 
										{
											case 0: // no print
											break;
											
											case 1: // print written data												
												for (unsigned int j=0;j<length;j++) printf("wr_data %2.2x ",*(write_buffer+j));
												printf("\n");
											break;
											case 2: // print written data and read data												
												for (unsigned int j=0;j<length;j++) printf("wr_data %2.2x ",*(write_buffer+j));
												printf("\n");
												for (unsigned int j=0;j<length;j++) printf("rd_data %2.2x ",*(mem_addr+j));
												printf("\n");
											break;
										}; 					
						break;	
						
						case 2 :	// 2 bytes - 1Word
						
							for (unsigned int n=0; n<(length/2); n++) 
								{
									*(word_dst+n) = *(word_src+n);
								}

									switch(debug_print) 
										{
											case 0: // no print
											break;
											
											case 1: // print written data	
												for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, write_buffer[j]);
												for (unsigned int j=0;j<(length/2);j++) printf("word written: %4.4x \n",*(word_src+j));						
											break;
											case 2: // print written data and read data	
												for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, write_buffer[j]);
												for (unsigned int j=0;j<(length/2);j++) printf("word written: %4.4x \n",*(word_src+j));
												for (unsigned int j=0;j<(length/2);j++) printf("readback word: %4.4x \n",*(word_dst+j));
											break;
										}; 									
						break;	
						
						case 4:	// 4 bytes - 1DWord				
							for (unsigned int n=0; n<(length/4); n++) 
								{
									*(dword_dst+n) = *(dword_src+n);
								}
							
								switch(debug_print) 
									{	
										case 0: // no print
										break;
										case 1: // print written data	
											for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, write_buffer[j]);
											for (unsigned int j=0;j<(length/4);j++) printf("dword written: %8.8x \n",*(dword_src+j));						
										break;
										case 2: // print written data and read data	
											for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, write_buffer[j]);
											for (unsigned int j=0;j<(length/4);j++) printf("dword written: %8.8x \n",*(dword_src+j));
											for (unsigned int j=0;j<(length/4);j++) printf("readback dword: %8.8x \n",*(dword_dst+j));
										break;
									};
						break;
						
						case 8:	// 8 bytes - 1QWord				
							for (unsigned int n=0; n<(length/8); n++) 
								{
									*(qword_dst+n) = *(qword_src+n);
								}
							
								switch(debug_print) 
									{	
										case 0: // no print
										break;
										case 1: // print written data	
											for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, write_buffer[j]);
											for (unsigned int j=0;j<(length/8);j++) printf("qword written: %16.16lx \n",*(qword_src+j));						
										break;
										case 2: // print written data and read data	
											for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, write_buffer[j]);
											for (unsigned int j=0;j<(length/8);j++) printf("qword written: %16.16lx \n",*(qword_src+j));	
											for (unsigned int j=0;j<(length/8);j++) printf("readback qword: %16.16lx \n",*(qword_dst+j));
										break;
									};
						break;						
						
						case 16:	// 16 bytes - 1QWord				
							for (unsigned int n=0; n<(length/16); n++) 
								{
									*(oword_dst+n) = *(oword_src+n);
								}
							
								switch(debug_print) 
									{	
										case 0: // no print
										break;
										case 1: // print written data	
											for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, write_buffer[j]);
											for (unsigned int j=0;j<(length/16);j++) printf("octal written: %16.16lx %16.16lx \n",(oword_src+j)->hi, (oword_src+j)->lo);					
										break;
										case 2: // print written data and read data	
											for (unsigned int j=0;j<length;j++) printf("byte %d: %2.2x \n",j, write_buffer[j]);
											for (unsigned int j=0;j<(length/16);j++) printf("octal written: %16.16lx %16.16lx \n",(oword_src+j)->hi, (oword_src+j)->lo);
											for (unsigned int j=0;j<(length/16);j++) printf("readback octalword: %16.16lx %16.16lx \n",(oword_dst+j)->hi, (oword_dst+j)->lo);
										break;
									};
							#if _DEBUG	
									{		
										
									}; 
							#endif 
						break;				
						
/* 						case 32:	// 8 bytes - 1QWord				
							for (unsigned int n=0; n<(length/32); n++) 
								{
									*(hword_dst+n) = *(hword_src+n);
								}
							#if _DEBUG	
									{		
										for (unsigned int j=0;j<length;j++) printf("%2.2x ",write_buffer[j]);
										printf("\n");
										for (unsigned int j=0;j<(length/32);j++) printf("%16.16lx %16.16lx %16.16lx %16.16lx",(hword_src+j)->hi, (hword_src+j)->hi_m, (hword_src+j)->lo_m, (hword_src+j)->lo);
										printf("\n");						
										for (unsigned int j=0;j<(length/32);j++) printf("%16.16lx %16.16lx %16.16lx %16.16lx",(hword_dst+j)->hi, (hword_dst+j)->hi_m, (hword_dst+j)->lo_m, (hword_dst+j)->lo);
										printf("\n");
									}; 
							#endif 
						break; */
						
				
					};
								
					
						
			}
																
	uint8_t find_pci_reg(const char* devid, uint32_t *pcie_bar_offset, uint32_t *pcie_bar_size, unsigned int debug_print)
	{
		char line[1000];
		uint8_t number_of_bar=0;
		unsigned int reg=0;
		FILE *f=fopen("/proc/bus/pci/devices","rb");
		if (f == NULL) {
			return 0;
		}
		while (fgets(line, sizeof line, f)) {
			
			if (strstr(line,devid))
			{
				int i=0;
				char *ptr=(char*)malloc(sizeof(line));
				ptr=strtok(line,"\t");
				if (debug_print != 0)
					{
						printf(" ** PCIe configuration space function : %s **\n",devid);
					}
				while (NULL!=(ptr=strtok(NULL,"\t")))
				{							
					 //reg = strtol(ptr, NULL, 16);
					if (sscanf(ptr, "%x", &reg) != 1)
						return 0;
					i++;
					if ((i >= 3) & (i <= 8) & (reg!=0)) 
						{	
							number_of_bar++;
									
							if (reg&0x01)//is a io_reg
							{
								pcie_bar_offset[i-3]=reg-1;
							}
							else
							{
								pcie_bar_offset[i-3]=reg;
							}
							if (debug_print != 0)
							{
								printf(" BAR %d: %2.2x\n",i-3,pcie_bar_offset[i-3]);
							}	
						}
					if ((i >= 10) & (reg!=0)) 
						{										
							pcie_bar_size[i-10]=reg;
							
							if (debug_print != 0)
							{
								printf(" BAR %d size is : %2.2x [byte=%d]\n",i-10,pcie_bar_size[i-10],pcie_bar_size[i-10]);
							}	
						}	
					if (i==15) break;
					
				}
			}
			/** /
			else 
				{
					if (debug_print != 0)
					{
						printf(" ** Don't find any PCIe configuration space function : %s **\n",devid);
					}
				}
				/ **/
		}
		fclose(f);
		if (debug_print != 0)
			{
				printf(" total of number of BAR for this function is: %d \n",number_of_bar);
			}
		return number_of_bar;

	}
	

