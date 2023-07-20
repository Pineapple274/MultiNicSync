nic1 = rx_7_230717_173645;
nic2 = rx_8_230717_173645;

data = sync_nics(nic1, nic2);

data = exp(1i * (data.'));  

radio = 2.68 / 5.147;  

multpi_path = 1;  

angle = -90 : 1 : 90;  

phaser_aoa(data, radio, multpi_path, angle);
