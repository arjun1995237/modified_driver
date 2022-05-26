#include <stdio.h>
#include<iostream>
#include<cstring>
#include <fstream>
#include <unistd.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#define window 150
int count=0;
//To write to a serial port, you write to the file. To read from a serial port, you read from the file
using namespace std;
void convert_char_arr(string s,char* p,int& n){
    	for (int i = 0; i < s.length(); i++) {
        	p[i] = s[i];n++;
        	if (p[i]=='\n')break;
    		}
	}
void change_parameter(int serial_port,int value){
	string val=to_string(value)+'\n';
	cout<<"sending the following--->"<<val<<"\n";
	int size=0;char* char_array=new char[val.length()];
	convert_char_arr(val,char_array,size);
	write(serial_port,char_array,size);usleep(100000); 
	delete char_array;
	}
int main(int argc, char *argv[]){
    	int n=0;
//    	ofstream outfile;
//   	outfile.open("example.txt");
    	int serial_port=open(argv[1],O_RDWR);
//        double time=stod(argv[2]);
	cout<<argv[1]<<" ";
    	if(serial_port<0){
        	printf("Error %i from open: %s\n", errno, std::strerror(errno));
    	}
//One of the common errors you might see here is errno = 2, and strerror(errno) returns No such file or directory. Make sure you have the right path to the device and that the device exists!
//
//Another common error you might get here is errno = 13, which is Permission denied. This usually happens because the current user is not part of the dialout group. Add the current user to the dialout group with:
//sudo adduser $USER dialout.

//configurations
        
    	struct termios tty;
    	if(tcgetattr(serial_port, &tty) != 0) {
    		printf("Error %i from tcgetattr: %s\n", errno, std::strerror(errno));
    	}
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
	tty.c_cflag |= CS8; // 5 bits per byte
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
   
	tty.c_cc[VTIME] = 0;   // This will make read() always wait for bytes (exactly how many is determined by VMIN), so read() could block indefinitely.
	tty.c_cc[VMIN] = 1;
//B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
//

	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}		
	char option;int num_bytes;string result="";

	while(1){

	cout<<"Select your options? \n0 - Stop\n1 - M1 (speed & direction set)\n 2 - M2 (speed & direction set)\n3 - Read encoder reading of M1\n4 - Read encoder reading of M2\n5 - \n 5- Read encoder reading of M1(filtered)\n 6- Read encoder reading of M2(filtered)\n Q - Quit\n";cin>>option;
	if(option=='3'|| option=='4'){
        //int k=0;
		//while(k<int(time/0.1)){
			write(serial_port,&option,1);
            result="";
 			char read_buf;
			while(1){
				num_bytes = read(serial_port,&read_buf,1);
 				if (num_bytes < 0) {
      					printf("Error reading: %s", strerror(errno));
      					return 1;
      				 }
      				if(read_buf!='\n')
        				result=result+char(read_buf);
    				else
        				
                        break;
				}// write inputted data into the file.
			cout<<"encoder "<<(option=='3')?"of M1 reads":" of M2 reads"<<result<<"\n";
	
		    

			//}
		}
    if(option=='5'|| option=='6'){
        //int k=0;
		//while(k<int(time/0.1)){
			write(serial_port,&option,1);
            result="";
 			char read_buf;
			while(1){
				num_bytes = read(serial_port,&read_buf,1);
 				if (num_bytes < 0) {
      					printf("Error reading: %s", strerror(errno));
      					return 1;
      				 }
      				if(read_buf!='\n')
        				result=result+char(read_buf);
    				else
        				
                        break;
				}// write inputted data into the file.
			cout<<"encoder(filtered) "<<(option=='5')?"of M1 reads":" of M2 reads"<<result<<"\n";
	
		    

			//}
		}
	else if(option=='1'|| option=='2'){
		int speed_dir;
    		cout<<"Enter speed and direction (say 1000 cps & forward ==>10001)?\n";
    		cin>>speed_dir;
		write(serial_port,&option,1);
		change_parameter(serial_port,speed_dir);
		cout<<"Task Completed\n";
		}

	else if(option=='0'){
		
        cout<<"stopping M! and M2\n";
		write(serial_port,&option,1);
		cout<<"Task accomplished\n";
		}
	else if(option=='Q'){//forward,backward or stop
		
	    return 0;
		}

	else{
		cout<<"Invalid option, try again!!!\n";
		}
	}
return 1;
}

