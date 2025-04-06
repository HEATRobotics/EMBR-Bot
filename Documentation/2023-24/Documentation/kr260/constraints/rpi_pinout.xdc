##################### Raspberry Pi GPIO Header #######################
### AXI GPIO ###
set_property PACKAGE_PIN AD15 [get_ports {rpi_gpio_tri_io[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[0]}]

set_property PACKAGE_PIN AD14 [get_ports {rpi_gpio_tri_io[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[1]}]

### I2C ###
set_property PACKAGE_PIN AE15 [get_ports {rpi_i2c_sda_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_i2c_sda_io}]
set_property PULLUP TRUE [get_ports {rpi_i2c_sda_io}]

set_property PACKAGE_PIN AE14 [get_ports {rpi_i2c_scl_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_i2c_scl_io}]
set_property PULLUP TRUE [get_ports {rpi_i2c_scl_io}]


set_property PACKAGE_PIN AG14 [get_ports {rpi_gpio_tri_io[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[2]}]

set_property PACKAGE_PIN AH14 [get_ports {rpi_gpio_tri_io[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[3]}]

set_property PACKAGE_PIN AG13 [get_ports {rpi_gpio_tri_io[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[4]}]

### SPI ###      
set_property PACKAGE_PIN AH13 [get_ports {rpi_spi_ss_io[1]}]   
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_spi_ss_io[1]}]

set_property PACKAGE_PIN AC14 [get_ports {rpi_spi_ss_io[0]}]   
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_spi_ss_io[0]}]  
                                                   
set_property PACKAGE_PIN AC13 [get_ports {rpi_spi_io1_io}]   
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_spi_io1_io}] 

set_property PACKAGE_PIN AE13 [get_ports {rpi_spi_io0_io}]   
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_spi_io0_io}] 

set_property PACKAGE_PIN AF13 [get_ports {rpi_spi_sck_io}]   
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_spi_sck_io}] 



set_property PACKAGE_PIN AA13 [get_ports {rpi_gpio_tri_io[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[5]}]

set_property PACKAGE_PIN AB13 [get_ports {rpi_gpio_tri_io[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[6]}]

### UART ###
set_property PACKAGE_PIN W14 [get_ports {rpi_uart_txd}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_uart_txd}]

set_property PACKAGE_PIN W13 [get_ports {rpi_uart_rxd}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_uart_rxd}]


set_property PACKAGE_PIN AB15 [get_ports {rpi_gpio_tri_io[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[7]}]

set_property PACKAGE_PIN AB14 [get_ports {rpi_gpio_tri_io[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[8]}]

set_property PACKAGE_PIN Y14 [get_ports {rpi_gpio_tri_io[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[9]}]

set_property PACKAGE_PIN Y13 [get_ports {rpi_gpio_tri_io[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[10]}]

set_property PACKAGE_PIN W12 [get_ports {rpi_gpio_tri_io[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[11]}]

set_property PACKAGE_PIN W11 [get_ports {rpi_gpio_tri_io[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[12]}]

set_property PACKAGE_PIN Y12 [get_ports {rpi_gpio_tri_io[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[13]}]

set_property PACKAGE_PIN AA12 [get_ports {rpi_gpio_tri_io[14]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[14]}]

set_property PACKAGE_PIN Y9 [get_ports {rpi_gpio_tri_io[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[15]}]

set_property PACKAGE_PIN AA8 [get_ports {rpi_gpio_tri_io[16]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[16]}]

set_property PACKAGE_PIN AB10 [get_ports {rpi_gpio_tri_io[17]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[17]}]

set_property PACKAGE_PIN AB9 [get_ports {rpi_gpio_tri_io[18]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio_tri_io[18]}]
