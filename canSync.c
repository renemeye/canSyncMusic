#include <stdio.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> 
#include <signal.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdm/rtcan.h>

#define MAX_REGESTERED_CALBACKS 512

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
	//return diffs;
}

long getGlobalTime(long timestamp){
	return calculatedTime+(timestamp-last_timestamp)*(1+meanOfDiffsBetweenLastOffsets());
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
		printf("lastCalc - lastServer = %ld\n", calculatedTime-receivedTime);

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

void start_music(struct can_frame* frame, long timestamp){
	printf("Leck mich einer fett\n");
}

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

