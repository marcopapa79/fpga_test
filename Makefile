CFLAGS += -O3 -Wall

all:
	g++ $(CFLAGS) -DDIO_WR_TEST new_fpga_test.cpp PCIe_access.cpp -o fpga_test -lssl -lcrypto
	#g++ $(CFLAGS) -DDIO_WR_TEST fpga_week_demo.cpp PCIe_access.cpp -o fpga_test -lssl -lcrypto

clean:
	rm fpga_test

