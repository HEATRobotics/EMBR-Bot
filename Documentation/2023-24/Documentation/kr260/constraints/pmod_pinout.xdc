##################### PMOD 1 Upper ###################################
set_property PACKAGE_PIN H12 [get_ports {kria_uart_rxd}]
set_property IOSTANDARD LVCMOS33 [get_ports {kria_uart_rxd}]

set_property PACKAGE_PIN E10 [get_ports {kria_i2c_sda_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {kria_i2c_sda_io}]
set_property PULLUP TRUE [get_ports {kria_i2c_sda_io}]

set_property PACKAGE_PIN D10 [get_ports {kria_spi_io0_io}]   
set_property IOSTANDARD LVCMOS33 [get_ports {kria_spi_io0_io}] 
                                                         
set_property PACKAGE_PIN C11 [get_ports {kria_spi_ss_io[0]}]   
set_property IOSTANDARD LVCMOS33 [get_ports {kria_spi_ss_io[0]}] 

##################### PMOD 1 Lower ###################################
set_property PACKAGE_PIN B10 [get_ports {kria_uart_txd}]
set_property IOSTANDARD LVCMOS33 [get_ports {kria_uart_txd}]

set_property PACKAGE_PIN E12 [get_ports {kria_i2c_scl_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {kria_i2c_scl_io}]
set_property PULLUP TRUE [get_ports {kria_i2c_scl_io}]

set_property PACKAGE_PIN D11 [get_ports {kria_spi_sck_io}]   
set_property IOSTANDARD LVCMOS33 [get_ports {kria_spi_sck_io}] 
                                                         
set_property PACKAGE_PIN B11 [get_ports {kria_spi_io1_io}]   
set_property IOSTANDARD LVCMOS33 [get_ports {kria_spi_io1_io}] 


##################### PMOD 2 Upper ###################################
set_property PACKAGE_PIN J11 [get_ports {pmod2_io_tri_io[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod2_io_tri_io[0]}]

set_property PACKAGE_PIN J10 [get_ports {pmod2_io_tri_io[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod2_io_tri_io[1]}]

set_property PACKAGE_PIN K13 [get_ports {pmod2_io_tri_io[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod2_io_tri_io[2]}]

set_property PACKAGE_PIN K12 [get_ports {pmod2_io_tri_io[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod2_io_tri_io[3]}]

##################### PMOD 2 Lower ###################################
set_property PACKAGE_PIN H11 [get_ports {pmod2_io_tri_io[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod2_io_tri_io[4]}]

set_property PACKAGE_PIN G10 [get_ports {pmod2_io_tri_io[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod2_io_tri_io[5]}]

set_property PACKAGE_PIN F12 [get_ports {pmod2_io_tri_io[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod2_io_tri_io[6]}]

set_property PACKAGE_PIN F11 [get_ports {pmod2_io_tri_io[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod2_io_tri_io[7]}]


##################### PMOD 3 Upper ###################################
set_property PACKAGE_PIN AE12 [get_ports {pmod3_io_tri_io[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod3_io_tri_io[0]}]

set_property PACKAGE_PIN AF12 [get_ports {pmod3_io_tri_io[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod3_io_tri_io[1]}]

set_property PACKAGE_PIN AG10 [get_ports {pmod3_io_tri_io[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod3_io_tri_io[2]}]

set_property PACKAGE_PIN AH10 [get_ports {pmod3_io_tri_io[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod3_io_tri_io[3]}]

##################### PMOD 3 Lower ###################################
set_property PACKAGE_PIN AF11 [get_ports {pmod3_io_tri_io[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod3_io_tri_io[4]}]

set_property PACKAGE_PIN AG11 [get_ports {pmod3_io_tri_io[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod3_io_tri_io[5]}]

set_property PACKAGE_PIN AH12 [get_ports {pmod3_io_tri_io[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod3_io_tri_io[6]}]

set_property PACKAGE_PIN AH11 [get_ports {pmod3_io_tri_io[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod3_io_tri_io[7]}]


##################### PMOD 4 Upper ###################################
set_property PACKAGE_PIN AC12 [get_ports {pmod4_io_tri_io[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod4_io_tri_io[0]}]

set_property PACKAGE_PIN AD12 [get_ports {pmod4_io_tri_io[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod4_io_tri_io[1]}]

set_property PACKAGE_PIN AE10 [get_ports {pmod4_io_tri_io[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod4_io_tri_io[2]}]

set_property PACKAGE_PIN AF10 [get_ports {pmod4_io_tri_io[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod4_io_tri_io[3]}]

##################### PMOD 3 Lower ###################################
set_property PACKAGE_PIN AD11 [get_ports {pmod4_io_tri_io[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod4_io_tri_io[4]}]

set_property PACKAGE_PIN AD10 [get_ports {pmod4_io_tri_io[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod4_io_tri_io[5]}]

set_property PACKAGE_PIN AA11 [get_ports {pmod4_io_tri_io[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod4_io_tri_io[6]}]

set_property PACKAGE_PIN AA10 [get_ports {pmod4_io_tri_io[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod4_io_tri_io[7]}]