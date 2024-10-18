CFLAGS += -O3 -Wall

all:
	g++ $(CFLAGS) -DDIO_WR_TEST fpga_test_cfg_spi.cpp PCIe_access.cpp -o fpga_test -lssl -lcrypto

clean:
	rm fpga_test

