#include <stdio.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> 
#include <signal.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/pipe.h>
#include <native/sem.h>
#include <rtdm/rtcan.h>

#define MAX_REGESTERED_CALBACKS 512

#define SOUND_PORT_INTERN 0x61
#define SOUND_PORT 0x378
#define SOUND_BIT_INTERN 2
#define SOUND_BIT 64
#define SAMPLE_RATE 44100
#define BUFFER_SECONDS 200

#define POOLSIZE_IN_KB 32
#define AUDIOBUFFER_SIZE_IN_KB 2
#define BUFFER_COUNT 6

RT_TASK beep_task;
RT_TASK fill_buffer_task;


int taskHasBeenStarted = 0;
RT_TASK read_task;
int can_reading_socket;
struct sockaddr_can can_receiver_addres;
int CAN_INTERFACE_INDEX = 1;
long initial_time_offset = 0;
int regelgroesse = 0; //in der Formel m
//VC(t) = C(t) + T_L(t)
//T_L(t)*m+n
//Pi(v)^t=K_p(v_i^t-v_i^t)+k_i/n sum(n <- i=0)(v_j^(k-i)-v_i^(t-i))
long last_timestamp=0;
long pre_last_timestamp=0;
typedef void(*FunctionPointer)();
void time_signal();
void start_music();
FunctionPointer registered_callbacks[MAX_REGESTERED_CALBACKS] = {0};

long start_music_time=0;

void cleanup(int blubb){
	printf("\nexit me\n");/*{{{*/

	if(taskHasBeenStarted)
		rt_task_delete(
				&read_task
		);
	
	_exit(0);
}/*}}}*/

void prepareReadOnCanBus(){
/*{{{*/
	//STEP 1: Create Socket
	printf("Create Socket\n");
	int protocol_family = PF_CAN;
	int socket_type = SOCK_RAW;
	int protocol = CAN_RAW;
	can_reading_socket =  rt_dev_socket(protocol_family, socket_type, protocol);
	if(can_reading_socket < 0){
		printf("Socket error");
		cleanup(0);
	}
	
	//STEP 2: Configure CAN-Card
	struct ifreq intefaceIndexStruct;
			//Device
	strncpy(intefaceIndexStruct.ifr_name, "rtcan0", IFNAMSIZ);
	if(rt_dev_ioctl(can_reading_socket, SIOCGIFINDEX, &intefaceIndexStruct)<0){
		fprintf(stderr, "ioctl device: %s\n", strerror(-errno));
	}
	
	//Baudrate
	can_baudrate_t *baudrate = (can_baudrate_t *)&intefaceIndexStruct.ifr_ifru;
	*baudrate = 250000;
	int ret = 0;
	if(ret = rt_dev_ioctl(can_reading_socket, SIOCSCANBAUDRATE, &intefaceIndexStruct)<0){
		fprintf(stderr, "ioctl baudrate: %s\n", strerror(-ret));
	}
	
	//CAN_STATE
	can_ctrlmode_t *can_mode = (can_ctrlmode_t *)&intefaceIndexStruct.ifr_ifru;
	*can_mode = CAN_MODE_START;
	if(rt_dev_ioctl(can_reading_socket, SIOCSCANMODE, &intefaceIndexStruct)<0){
		fprintf(stderr, "ioctl can mode: %s\n", strerror(-errno));
	}

	//Enable timestamps
  if(rt_dev_ioctl(can_reading_socket, RTCAN_RTIOC_TAKE_TIMESTAMP, RTCAN_TAKE_TIMESTAMPS)<0){
		fprintf(stderr, "rt_dev_ioctl TAKE_TIMESTAMP: %s\n", strerror(-errno));
	}
	//TODO: ASK SIOCGCANSTATE if CAN_STATE_ACTIVE ... can take some time (128 occurrences of 11 consecutive recessive bits)



	//STEP 2: Bind Socket
	printf("Bind Socket\n");
	can_receiver_addres.can_family = AF_CAN;
	can_receiver_addres.can_ifindex = CAN_INTERFACE_INDEX;
	//TODO: Close Socket
	int bind_return = rt_dev_bind(
			can_reading_socket,
			(struct sockaddr *) &can_receiver_addres,
			sizeof(struct sockaddr_can)
	);
	if(bind_return < 0){
		printf("Can't bind socket");
		cleanup(0);
	}
}/*}}}*/

#define COUNT_LAST_OFFSET_STORE 5
int counter_last_offsets = 0;
long lastOffsets[COUNT_LAST_OFFSET_STORE];
long lastTimestamps[COUNT_LAST_OFFSET_STORE];
long calculatedTime=0;
int preparation_count_down = COUNT_LAST_OFFSET_STORE+2;

//Calculates the drifft as average diff of the last offsets
double meanOfDiffsBetweenLastOffsets(){
	double diffs=0;
	int diff_nr;
	for(diff_nr = 0; diff_nr < (COUNT_LAST_OFFSET_STORE-1); diff_nr++){
		diffs += (1.0*(lastOffsets[((diff_nr+counter_last_offsets)+1)%COUNT_LAST_OFFSET_STORE]-lastOffsets[(diff_nr+counter_last_offsets)%COUNT_LAST_OFFSET_STORE]))/(lastTimestamps[((diff_nr+counter_last_offsets)+1)%COUNT_LAST_OFFSET_STORE]-lastTimestamps[(diff_nr+counter_last_offsets)%COUNT_LAST_OFFSET_STORE]);
	}
	
	return diffs/(COUNT_LAST_OFFSET_STORE-1);
}

long getGlobalTime(long timestamp){
	return calculatedTime+(timestamp-last_timestamp)*(1+meanOfDiffsBetweenLastOffsets());
}

long getLocalTime(long global_timestamp){
	return last_timestamp+((global_timestamp-calculatedTime)/(1+meanOfDiffsBetweenLastOffsets()));
}

void time_signal(struct can_frame* frame, long timestamp){

	//Error Recoginiton
	if (frame->can_dlc != 8){ //expected size is 8 byte/*{{{*/
		printf("The package is too long or too short! %i\n",frame->can_dlc);
		return;
	}

	//Timestamp when the Server received its last Message
	int dataIndex = 0;
	long receivedTime = 0;
	for(dataIndex=0; dataIndex < frame->can_dlc; dataIndex++){
		receivedTime =((long*)(frame->data))[0];
	}

	//Store the initial offset
	if(initial_time_offset == 0 && last_timestamp != 0 && preparation_count_down==1){
		initial_time_offset = receivedTime-last_timestamp;
	}
	
	long actual_offset = receivedTime-last_timestamp;

	//Store the last 5 offsets
	counter_last_offsets = (counter_last_offsets +1)%COUNT_LAST_OFFSET_STORE;
	lastOffsets[counter_last_offsets] = actual_offset;
	lastTimestamps[counter_last_offsets] = last_timestamp;

	if(preparation_count_down == 0){
		//printf("lastCalc - lastServer = %ld\n", calculatedTime-receivedTime);

		calculatedTime-=(calculatedTime-receivedTime);
		calculatedTime += (timestamp-last_timestamp)*(1.0+meanOfDiffsBetweenLastOffsets());
	}else{
		printf("Please wait: %i\n",preparation_count_down--);
		if(preparation_count_down == 0){
			initial_time_offset = receivedTime-last_timestamp;
			calculatedTime = timestamp+initial_time_offset;
		}
	}


	//Remember the last timestamp with Time_Signal
	pre_last_timestamp = last_timestamp;
	last_timestamp = timestamp;
}/*}}}*/

void getSyncFromCanBus(){
	//STEP 3: prepare Message/*{{{*/
	
	struct can_frame frame;
	int RECEIVE_FLAGS = 0;
	long timestamp;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	struct iovec iov;

	struct msghdr msg;
	msg.msg_iov = (void*)&iov;	
	msg.msg_iovlen = 1;
	iov.iov_base = (void *)&frame;
	iov.iov_len = sizeof(can_frame_t);
	msg.msg_name = (void *)&addr;
	msg.msg_namelen = sizeof(struct sockaddr_can);
	msg.msg_control = (void *)&timestamp;
	msg.msg_controllen = sizeof(nanosecs_abs_t);

	//STEP 4: receive Message
	int received_bytes = rt_dev_recvmsg(
		can_reading_socket,
		&msg, 
		RECEIVE_FLAGS
	);
	if(received_bytes < 0){
		printf("Error while receiving a Frame. ErrorNo: %s\n", strerror(-received_bytes));
		return;
	}

	//DeMask left bit. The ID is only 31bit
	frame.can_id &= 0x8fff;

	//STEP 5: handle callbacks
	if(frame.can_id >0 
			&& frame.can_id < MAX_REGESTERED_CALBACKS 
			&& registered_callbacks[frame.can_id] != 0){
		registered_callbacks[frame.can_id](&frame, timestamp);
	}
	return;
}/*}}}*/

void readOnCanBus(){
	while(1){
 		getSyncFromCanBus();
	}
}

typedef struct {
	unsigned short buf [ 1024*AUDIOBUFFER_SIZE_IN_KB ];
} audiobuf;

audiobuf ring[BUFFER_COUNT];
RT_PIPE xenomaiPipe;
RT_SEM audiobuffer_semaphore_read;
RT_SEM audiobuffer_semaphore_write;
int next_write_area = 0;
int next_read_area = 0;

void beep(){
  /*{{{*/
	if(ioperm(SOUND_PORT,1,1)){
		printf("NOT ALLOWED\n");
	}

	rt_sem_p(&audiobuffer_semaphore_read, TM_INFINITE);
	RTIME start_time = start_music_time;
	printf("Starte Musik in: %ldns\n",getLocalTime(start_time)-rt_timer_read());
	rt_task_sleep_until(getLocalTime(start_time));
	rt_sem_v(&audiobuffer_semaphore_read);
	
	int sampleInNS = 1000000000/SAMPLE_RATE;
	
	while(1){
		rt_sem_p(&audiobuffer_semaphore_read, TM_INFINITE);
		//printf("<play buffer %i>",next_read_area);

	  int counter = 0;
		while(counter<sizeof(ring[next_read_area].buf)/sizeof(ring[next_read_area].buf[0])){/*{{{*/
		
			//ON
			outb(inb(SOUND_PORT)^SOUND_BIT,SOUND_PORT);
			rt_task_sleep_until(
					getLocalTime(start_time) +rt_timer_ns2ticks(
						(sampleInNS*ring[next_read_area].buf[counter])>>16
					)
			);

			//OFF
			outb(inb(SOUND_PORT)^SOUND_BIT,SOUND_PORT);
			start_time += rt_timer_ns2ticks(sampleInNS);
			rt_task_sleep_until(getLocalTime(start_time));
			counter++;
		}/*}}}*/
		next_read_area=(next_read_area+1)%BUFFER_COUNT;

		//printf("</play>",next_read_area);
		rt_sem_v(&audiobuffer_semaphore_write);
	}

	return;
}/*}}}*/

void fillBuffer(){
		while(1){/*{{{*/
			rt_sem_p(&audiobuffer_semaphore_write,TM_INFINITE);
			//printf("< write in buffer %i.>",next_write_area);

			rt_pipe_read(&xenomaiPipe, ring[next_write_area].buf, sizeof(ring[next_write_area].buf), TM_INFINITE);
			next_write_area = (next_write_area+1)%BUFFER_COUNT;

			//printf("</write>",next_write_area);
			rt_sem_v(&audiobuffer_semaphore_read);
		}
}/*}}}*/


int main () {
	signal(SIGINT,cleanup);
	signal(SIGPIPE,cleanup);
	signal(SIGQUIT,cleanup);
	signal(SIGHUP,cleanup);
	signal(SIGTERM,cleanup);

	prepareReadOnCanBus();

	//printf("Get shure, that we will not beeing paged\n");
	mlockall(MCL_CURRENT|MCL_FUTURE);

	//Register events
	int i = 0;
	for(i=0; i<MAX_REGESTERED_CALBACKS; i++){
		registered_callbacks[i] = 0;
	}
	registered_callbacks[1] = time_signal;
	registered_callbacks[256] = start_music;

	//create pipe
	int error = rt_pipe_create(&xenomaiPipe,"speakerpipe",P_MINOR_AUTO,POOLSIZE_IN_KB*1024);
	if(error){
		cleanup(error);
		return;
	}

	//create semaphore
	//TODO: Cleanup Semaphore
	error = rt_sem_create(
		&audiobuffer_semaphore_read,
		"Read buffer semaphore",
		0,
		S_FIFO
	);
	error = rt_sem_create(
		&audiobuffer_semaphore_write,
		"write buffer semaphore",
		BUFFER_COUNT,
		S_FIFO
	);
	if(error){
		cleanup(error);
		return;
	}

	rt_task_create(
		&fill_buffer_task,    // Task "object"
		"Fill Buffer Task",  // Taskname
		0, 	      		// stacksize
		10, 	      	// priority
		0             // option-flags
	);
	rt_task_create(
		&beep_task,    // Task "object"
		"Beep Task",  // Taskname
		0, 	      		// stacksize
		20, 	      	// priority
		0             // option-flags
	);

	rt_task_create(
		&read_task,    // Task "object"
		"CAN Read Task",  // Taskname
		0, 	      		// stacksize
		99, 	      	// priority
		0             // option-flags
	);
	taskHasBeenStarted = 1;
	rt_task_start(
			&read_task,	// Taski
			readOnCanBus, 			// fuction pointer
			NULL				// Data
	);

	pause();

	rt_task_delete(
			&read_task
	);
	
	return 0;
}

void start_music(struct can_frame* frame, long timestamp){

	int dataIndex = 0;
	long globalStartTime = 0;
	for(dataIndex=0; dataIndex < frame->can_dlc; dataIndex++){
		globalStartTime =((long*)(frame->data))[0];
	}

	start_music_time = globalStartTime;

	rt_task_start(
			&fill_buffer_task,	// Task 
			fillBuffer, 			// fuction pointer
			NULL				// Data
	);

	rt_task_start(
			&beep_task,	// Task 
			beep, 			// fuction pointer
			NULL				// Data
	);

}
