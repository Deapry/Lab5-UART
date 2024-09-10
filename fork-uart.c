/**
 * @file    fork-uart.c
 * 
 * @brief fork uart
 * Non Cannonical mode 
 * Sellecting the Serial port Number on Linux   
 * /dev/ttyUSBx - when using USB  to Serial Converter, where x can be 0,1,2...etc  
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc 
 * termios structure -  /usr/include/asm-generic/termbits.h  
 * use "man termios" to get more info about  termios structure 
 * @author  Samy Lao
 * @date    2024-10-08
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions

// device port série à utiliser 
//const char *portTTY = "/dev/ttyGS0"; 
//const char *portTTY = "/dev/ttyS0";
const char *portTTY = "/dev/ttyS1";
//const char *portTTY = "/dev/ttyS2";
//const char *portTTY = "/dev/ttyS3";
//const char *portTTY = "/dev/ttyS4";
//const char *portTTY = "/dev/ttyS5";
//const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter

 int fd;
    





 void vInitialisationUART(void)
{
    
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port 
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed  
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
    SerialPortSettings.c_lflag |= ICANON;
	SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  

	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

	// Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 1; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 0; // Wait 10 sec (0 for indefinetly) 

    	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
		printf("\n  Erreur! configuration des attributs du port serie");
}



void codeDuProcessusParent(void)
{
    
    sleep(1);
    printf("Je suis le Processus Père, j'écris sur la console (terminal) ce que j'entends sur le port série.");
    char write_buffer[] = "";	// Buffer containing characters to write into port
	int  bytes_written  = 0;  	// Value for storing the number of bytes written to the port 

	bytes_written = write(fd, write_buffer, sizeof(write_buffer)); // use write() to send data to port 
										// "fd"                   - file descriptor pointing to the opened serial port
										//	"write_buffer"         - address of the buffer containing data
										// "sizeof(write_buffer)" - No of bytes to write 
	printf("\n Processus Père : nombres d'octets recus : %d --> %s", bytes_written, write_buffer);
	printf("\n");
    if(write_buffer == "!")
    {
        printf("Fin du Père");
        return 0;
    }
	
	close(fd); // Close the Serial port 
}

/// @brief Code exécuté par le processus Fils
/// @param aucun
void codeDuProcessusEnfant(void)
{
    
    sleep(2);
    printf("Je suis le Processus Fils, j'écris sur le port série ce que j'entends sur la console 9terminal).");
    	// Read data from serial port 
	tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
	char read_buffer[32];   // Buffer to store the data received 
	int  bytes_read = 0;    // Number of bytes read by the read() system call 
	int i = 0;

	bytes_read = read(fd, &read_buffer, 32); // Read the data 
	printf(" Bytes Recus : %d --> ", bytes_read); // Print the number of bytes read
	for(i=0; i<bytes_read; i++)	 // printing only the received characters
	printf("%c", read_buffer[i]);
	printf("\n");
        if(read_buffer == "q")
    {
        printf("Fin du Fils");
        return 0;
    }

	close(fd); // Close the serial port
}

/// @brief Exemple de processus Père/Fils avec la fonction 'fork'
/// @return 0


int main() 
{

   

     fd = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
	// Setting the Attributes of the serial port using termios structure 

	
    vInitialisationUART();


    pid_t pid;
    pid = fork();

    if(pid == 0)
    {
    // Appel fonction Enfant
    codeDuProcessusEnfant();

    }
    else
    {
    // Appel fonction Parent
    wait(0);
    codeDuProcessusParent();
    }
    return 0;
}


