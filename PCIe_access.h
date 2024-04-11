//#ifndef _PCIe_ACCESS_H
//#define _PCIe_ACCESS_H

			#pragma pack(push, 1)
			typedef struct _uint128_t {
				uint64_t lo;
				uint64_t hi;
			} uint128_t;
			typedef struct _uint256_t {
				uint64_t lo;
				uint64_t lo_m;
				uint64_t hi_m;
				uint64_t hi;
			} uint256_t;
			#pragma pack(pop)
	// ** Function Declaration **
	//		
	
	void * IORd(unsigned short int io_addr, uint8_t * reg_value, unsigned int length, unsigned int byte_alligned, unsigned int debug_print);
	void IOWr (unsigned short int io_addr, uint8_t * reg_wr_value, unsigned int length, unsigned int byte_alligned, unsigned int start_index, unsigned int debug_print);  
	
	void * MRd32 (uint8_t * mem_addr, uint8_t * read_buffer, unsigned int length, unsigned int byte_alligned, unsigned int debug_print);   
	void MWr32 (uint8_t * mem_addr, uint8_t * write_buffer, unsigned int length, unsigned int byte_alligned, unsigned int debug_print);
 
  uint8_t find_pci_reg(const char* devid, uint32_t *pcie_bar_offset, uint32_t *pcie_bar_size, unsigned int debug_print);
	
	//uint8_t find_pci_reg(const char* devid, uint32_t *pcie_bar, unsigned int debug_print);
	

//#endif
