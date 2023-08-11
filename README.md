# LAN9252_spidev

Userspace driver API for LAN9252 Ethercat module using spidev which makes it compatible with the most embedded linux enviroments regardless of the GPIO chip.

The project includes:
- EasyCat inferface
- stub which makes it suitable for cross-platform integration, without having the targer hardware being available
- EasyCat implementation using 32byte input-output by default but it is configurable
- code example
  
For an example how to integrate it to your CMakeProject, please see the *example* folder. 
