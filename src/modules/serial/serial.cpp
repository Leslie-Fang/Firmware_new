#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/vision_position_estimate.h>

#include <mavlink/mavlink_log.h>
#include <px4_posix.h>

#define SERIAL_PORT	"/dev/ttyS6"
#define MAXSIZE 	50
#define BAUDRATE	57600

static bool thread_should_exit = false;		
static bool thread_running = false;		
static int serial_task;				

int _serial_fd;
int ret;

unsigned char ringbuf[MAXSIZE];
unsigned char data_buf[14];
char readbuf[25];
int read_addr = 0;  
int write_addr = 0;  
short  data_transformed[6];
unsigned short crc_data = 0;
unsigned short crc_data_last = 0;
bool valid = true;
bool read_valid = false;

/**
 *  management function.
 */
 extern "C" __EXPORT int serial_main(int argc, char *argv[]);

/**
 * Mainloop .
 */
int serial_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop) ;
void serial_init() ;
int next_data_handle(int addr) ;  
int next_data_handle(int addr , int count) ;
void write_data(char data) ;
unsigned short crc_update(unsigned short  crc ,  unsigned char data) ;
unsigned short crc(void* data, unsigned short count) ;

/**
 * The app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int serial_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("serial already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		serial_task = px4_task_spawn_cmd("serial",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 serial_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int serial_thread_main(int argc, char *argv[])
{
	int mavlink_fd;
	mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

	warnx("[serial] starting\n");
	thread_running = true;

	/* advertise attitude topic */
	struct vision_position_estimate_s vicon;
	memset(&vicon, 0, sizeof(vicon));
	orb_advert_t vicon_pub = orb_advertise(ORB_ID(vision_position_estimate), &vicon);

	serial_init();
	ret = set_serial( _serial_fd, BAUDRATE, 8, 'N', 1 );
        	if(ret == -1)
	{
		warnx("set error: %d\n", ret);
		exit(0);
	}

	while (!thread_should_exit) {
		//warnx("Hello serial!\n");

		
		ret = read(_serial_fd, readbuf,20);
		if( ret <= 0 )
		{
			warnx("read err: %d\n", ret);

		}else
		{
			for(int i = 0 ; i < 20 ; i++)
			{
				write_data(readbuf[i]) ;
			}
		}

		for(int i = 0 ; i < MAXSIZE ; i++)
		{
			if((ringbuf[read_addr] == '>') && (ringbuf[next_data_handle(read_addr)] == '*') 
			    && (ringbuf[next_data_handle(read_addr,2)] == '>') && (ringbuf[next_data_handle(read_addr,17)] == '<') 
			    && (ringbuf[next_data_handle(read_addr,18)] == '#') && (ringbuf[next_data_handle(read_addr,19)] == '<'))  
			{
				read_addr = next_data_handle(read_addr,3) ; 
				for(int j = 0 ; j < 14 ; j++)
				{
					data_buf[j] = ringbuf[read_addr] ;
					read_addr = next_data_handle(read_addr) ;
				}
				read_valid = true;
				break;
			}else
			{
				read_addr = next_data_handle(read_addr) ;
			}
		}
		read_addr = next_data_handle(read_addr, 3);
		data_transformed[0] = (data_buf[1]<<8) | data_buf[0] ;
		data_transformed[1] = (data_buf[3]<<8) | data_buf[2] ; 
		data_transformed[2] = (data_buf[5]<<8) | data_buf[4] ; 
		data_transformed[3] = (data_buf[7]<<8) | data_buf[6] ; 
		data_transformed[4] = (data_buf[9]<<8) | data_buf[8] ; 
		data_transformed[5] = (data_buf[11]<<8) | data_buf[10] ; 
		crc_data_last = crc_data;
		crc_data = (data_buf[13]<<8) | data_buf[12] ;
		if(crc(data_transformed,12) == crc_data)
		{
			vicon.timestamp_boot = hrt_absolute_time(); 
			vicon.x = data_transformed[0]/1000.0f;
			vicon.y = data_transformed[1]/1000.0f;
			vicon.z = data_transformed[2]/1000.0f;
			vicon.vx = data_transformed[3]/1000.0f;
			vicon.vy = data_transformed[4]/1000.0f;
			vicon.vz = data_transformed[5]/1000.0f;

			if(crc_data == crc_data_last){
				valid = false;
			}
			if(valid && read_valid){
				if (vicon_pub == nullptr) {
					vicon_pub = orb_advertise(ORB_ID(vision_position_estimate), &vicon);
				} else {
					orb_publish(ORB_ID(vision_position_estimate), vicon_pub, &vicon);
					mavlink_log_info(mavlink_fd, "[vicon] position: %d", (double)vicon.x);
				}	
			}else{
				valid = true;
			}
			
		}

		usleep(90000);
	}

	warnx("[serial] exiting.\n");

	thread_running = false;

	return 0;
}

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: serial {start|stop|status} [-p <additional params>]\n\n");
}


int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
	struct termios newtio,oldtio;  
	if  ( tcgetattr( fd,&oldtio)  !=  0) {   
		perror("SetupSerial 1");  
		return -1;  
	}  
	bzero( &newtio, sizeof( newtio ) );  
	newtio.c_cflag  |=  CLOCAL | CREAD;  
	newtio.c_cflag &= ~CSIZE;  

	switch( nBits )  
	{  
		case 7:  
		newtio.c_cflag |= CS7;  
		break;  

		case 8:  
		newtio.c_cflag |= CS8;  
		break;  
	}  

	switch( nEvent )  
	{  
		case 'O':  
		newtio.c_cflag |= PARENB;  
		newtio.c_cflag |= PARODD;  
		newtio.c_iflag |= (INPCK | ISTRIP);  
		break;  

		case 'E':   
		newtio.c_iflag |= (INPCK | ISTRIP);  
		newtio.c_cflag |= PARENB;  
		newtio.c_cflag &= ~PARODD;  
		break;  

		case 'N':    
		newtio.c_cflag &= ~PARENB;  
		break;  
	}  

	switch( nSpeed )  
	{  
		case 2400:  
		cfsetispeed(&newtio, B2400);  
		cfsetospeed(&newtio, B2400);  
		break;  

		case 4800:  
		cfsetispeed(&newtio, B4800);  
		cfsetospeed(&newtio, B4800);  
		break;

		case 9600:  
		cfsetispeed(&newtio, B9600);  
		cfsetospeed(&newtio, B9600);  
		break;  

		case 19200:  
		cfsetispeed(&newtio, B19200);  
		cfsetospeed(&newtio, B19200);  
		break; 

		case 57600:  
		cfsetispeed(&newtio, B57600);  
		cfsetospeed(&newtio, B57600);  
		break; 

		case 115200:  
		cfsetispeed(&newtio, B115200);  
		cfsetospeed(&newtio, B115200);  
		break;  

		case 460800:  
		cfsetispeed(&newtio, B460800);  
		cfsetospeed(&newtio, B460800);  
		break;  

		default:  
		cfsetispeed(&newtio, B9600);  
		cfsetospeed(&newtio, B9600);  
		break;  
	}  

	if( nStop == 1 )  
	{
		newtio.c_cflag &=  ~CSTOPB;  
	}
	else if ( nStop == 2 )
	{  
		newtio.c_cflag |=  CSTOPB;  
	} 

	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);  
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
	{  
		perror("com set error");  
		return -1;  
	}  
	return 0;  
}  

void serial_init()
{
	_serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
	warnx("serial open: %d",_serial_fd);
	if (_serial_fd < 0) {
		warnx("FAIL: serial fd");
		exit(0);
	}
}

int next_data_handle(int addr)     
{     
  	return (addr + 1) == MAXSIZE ?  0 : (addr + 1) ;     
}
     
int next_data_handle(int addr , int count)     
{     
  	int a;
  	a = addr;
  	for(int i = 0; i < count ; i++)
  	{ 
    		a = ( (a + 1)  == MAXSIZE ?  0 : ( a + 1 ) ) ;   
  	}
  	return a;  
}
 
void write_data(char data)  
{  
	*(ringbuf+write_addr) = (unsigned char)data;  
	write_addr = next_data_handle(write_addr);  
}  

/*crc*/
unsigned short crc_update(unsigned short  crc ,  unsigned char data)
{
	data ^= (crc & 0xff) ;
	data ^= data << 4;
	return ((((unsigned short)data << 8) | ((crc >> 8) & 0xff) )^ (unsigned char)(data >> 4) ^ ((unsigned short)data << 3)) ;
}

unsigned short crc(void* data, unsigned short count)
{
	unsigned short crc = 0xff;
	unsigned char *ptr = (unsigned char*)data;
	for(int i = 0 ; i < count ; i++)
	{
		crc = crc_update(crc , *ptr);
		ptr++;
	}
	return crc;
}
