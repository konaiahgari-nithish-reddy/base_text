#include<wiringPi.h>
#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<string.h>
#include<math.h>
#include <wiringSerial.h>
#include<stdio_ext.h>
#include<ctype.h>
#include<mosquitto.h>

void UART_Tx(void);


//************************************
//********Global Variables************
//************************************
int serial_port=0,cnt=0;

//_____Display Messages_____

char welcome[]="\n***************Welcome To Smarttrak Solar Panel Controller***************\n";
char display[]="\nDisplaying The Available Zigbees Please Choose one\n";
char devices[]="-------1.R1 2.R2 3.R3 4.R4 5.R5--------\n\n";
char sel[]="Select the Device:";	
char dev_error[]="\nplease choose devices availbale above\n";
char config[]="\nConfiguring Zigbee Address Please Wait.............\n";
char config_addr[70]="Configured Zigbee Address:";
char Read_cont[]="do you wanna continue or not :";
char comd_opt[]="\n\nSelect The Option :";
char resp[]="No Response From Devie\n";
char exit_dev[]="#) Exit Device";
char rep_null[]="Please Select Proper Option Or press ENTER\n";
char prev_xbee[]="Previous Zigbee Selected\n";
char cont[]="do you wanna continue press y or n :";


//zigbee address structure
struct add
{
	char name[5];
	char addr[20];
};
struct add address[]={{"R1","41A44C3B"},
			{"R2","41898F46"},
			   {"R3","4104AB9B"}};
//CloudMQTT.com Credentials
	char *host = "soldier.cloudmqtt.com";
	char *username = "cbocdpsu";
	char *passwd = "3_UFu7oaad-8";
	int port = 14035;
	int keepalive = 60;
	bool clean_session = true;
	struct mosquitto *mosq = NULL;
	char *topic = "zigbee"; //Publish topic
	char *sub_topic="reply";//Subscribe topic


int j=0,f=0;
int count=0;
unsigned char rx_pkt[10000],cmd_cpy[15],sub_temp[10],add_chk[10];
int pkt_len,data_flag=0,null_flag1=0,adflag=0,ext=0,ss=0,chk_flag=0,sub_flag=0,msg_flag=0;
unsigned char data[10],ptr[50][200];
char xbeeaddr[20];

//*****************************************************************
//*****************Mosquitto Callback Functions********************
//*****************************************************************
void publish(char *dat,int pkt_len,char *top,struct mosquitto *mosq)
{
		mosquitto_publish(mosq,NULL,top,pkt_len,dat,0,true);
		printf("\n------------------------------------------------\n");
		printf("%s\n",dat);
}

void my_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
	if(message->payloadlen){
		printf("%s %s\n", message->topic, message->payload);
		strcpy(sub_temp,message->payload);
		sub_flag=1;
	}else{
		publish(rep_null,strlen(rep_null),topic,mosq);
		printf("%s (null)\n", message->topic);

	}
	fflush(stdout);
}

void my_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
	int i;
	printf("\n----------------------------------------------\n");
	printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
	for(i=1; i<qos_count; i++){
		printf(", %d", granted_qos[i]);
	}
	printf("\n");
}

//*********************************************************************
//**************************UART FUNCTIONS*****************************
//*********************************************************************

int uart_init(void)
{
	if ((serial_port = serialOpen ("/dev/ttyAMA0", 9600)) < 0)	/* open serial port */
  	{
		perror("serialopen");
    		return 0;
  	}
}

int UART_Rx(void)
{
	int i=0;
	char ch;
	memset(rx_pkt,0,sizeof(rx_pkt));
	while(serialDataAvail(serial_port)>0)
	{
		chk_flag=1;//Sets When data receives else no response form device
		rx_pkt[i]=serialGetchar(serial_port); //receiving byte
			
		if(rx_pkt[i-2]=='~' && rx_pkt[i-1]=='\n' && rx_pkt[i]=='!') //Keep this to set identification for Values
			f=0;

		if(rx_pkt[i]=='!' )//Checking For end of Line
		{
			rx_pkt[i]='\0';
			strcpy(ptr[j],rx_pkt);		

			if(j>45)//Checking for Buffer Overflow
			{
				memset(ptr,0,50*200*sizeof(ptr[0][0]));
				memset(rx_pkt,0,sizeof(rx_pkt));
				j=0;
				ss=1;
			}
			else
			{
				rx_pkt[i]='\0';
				strcpy(ptr[j],rx_pkt);
				if(strstr(ptr[j],"Not a valid"))//For Invalid data entry
				{
					publish(ptr[j],strlen(ptr[j]),topic,mosq);//Publishing error and reseting menu
					//printf("\n%s\n\n",ptr[j]);
				}
				else
				{
					publish(ptr[j],strlen(ptr[j]),topic,mosq);//Publishing Menu line by line
				//	printf("%s\n",ptr[j]);
				}

				if(strchr(ptr[j],'@'))//For Identification of Data entering
				{
					data_flag=1;//Setting data flag
					f=1;
				}
				j++;
					
					memset(rx_pkt,0,sizeof(rx_pkt));//Clearing receive buffer
					i=0;
//					printf("exit:%d\n",flag);
			}
		}
		else if(rx_pkt[i]=='~')//For identifying end of menu
		{
			i++;
			if(i>3)
			{
			     f=1;
			}
			j=0;
		}
		else i++;
			
	}
	if(i==1)
		f=1;//Setting flag for transmit function
	if(i>0) return 1;//Menu not yet completed
	
	return 0;
}

//Used Only for Zigbee init function AT commands
int rx_data(char *dat1)
{
	int i,len=0;
	if((len=serialDataAvail(serial_port))>0)
	{
		for(i=0;i<len;i++)
			dat1[i]=serialGetchar(serial_port);
	}
	else
		return 0;
	dat1[i]='\0';
	return 1;
}

//***********************************************************
//******************Zigbee AT COMMANDS***********************
//***********************************************************

int xbee_init(void)
{
	char data[8000],addr[20];
//	static char compare[20],c;
	memset(data,0,sizeof(data));
	memset(addr,0,sizeof(addr));
	
	strcpy(addr,"ATDL");
	strcat(addr,xbeeaddr);
	strcat(addr,"\r");
	
	//checks for no data coming in serial port
	serialPuts(serial_port,"\n\n\r");
	delay(2000);
	//enabling AT mode
	serialPuts(serial_port,"+++");//enter into command mode
	delay(2000);
	if(rx_data(data))
	{}
	//	printf("+++:%s\n",data);
	else printf("+++:error\n");
	serialPuts(serial_port,addr);//set dest low addr
	delay(2000);
	if(rx_data(data))
	{}
	//	printf("ATDL:%s\n",data);
	else printf("ATDL:error\n");
	serialPuts(serial_port,"ATWR\r");//writes into xbee
	delay(2000);
	if(rx_data(data))
	{}
	//	printf("ATWR:%s\n",data);
	else printf("ATWR:error\n");
	serialPuts(serial_port,"ATCN\r");//close at mode
	delay(2000);
	if(rx_data(data))
	{}
	//	printf("ATCN:%s\n",data);
	else printf("ATCN:error\n");
	memset(data,0,sizeof(data));
	strcat(data,config_addr);
	strcat(data,addr);
	publish(data,strlen(data),topic,mosq);
	memset(data,0,sizeof(data));
	memset(add_chk,0,sizeof(add_chk));
	strcpy(add_chk,xbeeaddr);
}

void UART_Tx()
{
	int i=0;
	if(data_flag==1)//Used for setting rtc and other parameters
	{
		data_flag=0;
		if(null_flag1==1)//Sends carriage return to go back to menu
		{
			serialPutchar(serial_port,'\r');
			null_flag1=0;
		}
		else
		{
			for(i=0;cmd_cpy[i]!='\0';i++)//Setting Parameters
			{
					serialPutchar(serial_port,cmd_cpy[i]);
			}
		}
	}
	else if(ss==1) //For Read Values Continously
	{
		char chh[5];
	//	printf("do you wanna continue or not :");
		publish(Read_cont,strlen(Read_cont),topic,mosq);
		//chh=getchar();
		while(sub_flag!=1);
		if(sub_flag==1)
		{
			strcpy(chh,sub_temp);
			memset(sub_temp,0,sizeof(sub_temp));
			sub_flag=0;
		}
		if(chh[0]=='n')//If no pressed exits the read values
			serialPutchar(serial_port,chh[0]);
		ss=0;//else clears flag and continues printing
	}
	else
	{
	//	__fpurge(stdin);
	//	printf("\n\nSelect The Option :");
		publish(comd_opt,strlen(comd_opt),topic,mosq);
		while(sub_flag!=1);
		if(sub_flag==1)
		{
			strcpy(data,sub_temp);
			memset(sub_temp,0,sizeof(sub_temp));
			sub_flag=0;
		}
		if(data[0]=='#')//Checks for exit device
		{
			serialPutchar(serial_port,'r');
			ext=1;
		}
		else//sends command
			serialPutchar(serial_port,data[0]);
	}
}

//*************************************************************
//***************************MENU******************************
//*************************************************************

void MENU()
{
	int m=0,l=0,naflag=0;
	char ch[5];
	

		//Publishing welcome messages and devices
loop:		publish(welcome,strlen(welcome),topic,mosq);
		publish(display,strlen(display),topic,mosq);
		publish(devices,strlen(devices),topic,mosq);
		publish(sel,strlen(sel),topic,mosq);	
	//	scanf("%s",ch);
		while(sub_flag!=1); //Waits for input
		if(sub_flag==1)
		{
			strcpy(ch,sub_temp);
			memset(sub_temp,0,sizeof(sub_temp));
			sub_flag=0;
		}
		//Lower case to upper case
		for(m=0; m<strlen(ch); m++)
		{
			if(islower(ch[m]))
			{
				ch[m]=toupper(ch[m]);
			}
		}
		//Comparing device Name
		for(l=0;l<10;l++)
		{
			if(strcmp(address[l].name,ch)==0)
			{
				strcpy(xbeeaddr,address[l].addr);
				naflag=1;
				break;
			}
		}
		if(naflag==0) //Name Not match Sends error and continues
		{
			publish(dev_error,strlen(dev_error),topic,mosq);
			naflag=0;
			goto loop;
		}
		else if(strcmp(add_chk,xbeeaddr)==0)
			adflag=1;
		//publish(config,strlen(config),topic,mosq);
		//setting the device destination address
		if(adflag==0)
		{	
			publish(config,strlen(config),topic,mosq);
			xbee_init();
		}
		else
		{
			publish(prev_xbee,strlen(prev_xbee),topic,mosq);
			adflag=0;
		}

		//Clearing all buffers
		memset(ptr,0,50*200*sizeof(ptr[0][0]));
		memset(rx_pkt,0,sizeof(rx_pkt));
		memset(cmd_cpy,0,sizeof(cmd_cpy));
		pkt_len=0,data_flag=0,null_flag1=0,adflag=0,j=0,f=0;
		delay(1000);
		//Sending new line as start byte
		serialPutchar(serial_port,'\n');
//		printf("Menu\n");
	__fpurge(stdin);
}

//********************************************************************
//*********************************MAIN*******************************
//********************************************************************

int main()
{
	unsigned char command[10],ch[3],val[2];
	int k,ii=0,jj=0,cnt=0,i=0,l,m,qos=0,cout=0;
	static int var;
	
	//INITIALIZATIONS
	
	if(uart_init()==0)
		return 0;


	if ( wiringPiSetup() == -1 )
		exit( 1 );
	
	mosquitto_lib_init();
	mosq = mosquitto_new(NULL, clean_session, NULL);
	if(!mosq){
		fprintf(stderr, "Error: Out of memory.\n");
		return 1;
	}
	mosquitto_message_callback_set(mosq, my_message_callback);
	mosquitto_subscribe_callback_set(mosq, my_subscribe_callback);
		
	mosquitto_username_pw_set(mosq,username,passwd);

	if(mosquitto_connect_async(mosq, host, port, keepalive)){
		fprintf(stderr, "Unable to connect.\n");
		return 1;
		}
	mosquitto_loop_start(mosq);
	mosquitto_subscribe(mosq, NULL,sub_topic,qos);

	//Main While LOOP Should Not break
	while(1)
	{
		//Sending Device Menu
		MENU();

	//Second While Loop Breaks when exit from device	
	while(1){
			//Flag checking for reception Completion
			if(f==0)
			{		
				if(ext==1)
				{
					ext=0;
					publish("exiting....",13,topic,mosq);
					break;
				}
				k=UART_Rx();
				if(chk_flag==0)
				{
					if(cout>5)
					{
						publish(resp,strlen(resp),topic,mosq);
						publish(exit_dev,strlen(exit_dev),topic,mosq);
						UART_Tx();
					}
					else
					{
						cout++;
						delay(400);
					}
				}
				else if(ss==1)
				{
				//	printf("do you wanna continue press y or n :");
					publish(cont,strlen(cont),topic,mosq);
					//__fpurge(stdin);
					//fgets(chh,sizeof(chh),stdin);

					while(sub_flag!=1);//Waiting for subscribe function
					if(sub_flag==1)
					{
						strcpy(val,sub_temp);
						memset(sub_temp,0,sizeof(sub_temp));
						sub_flag=0;
					}
					if(val[0]=='n') //For exiting from read data
					{
						serialPuts(serial_port,"sssssssssss\n");
					}
					ss=0;
				}
//				printf("in main\n");
				delay(400);
			}
			else if(f==1) //Sets when Reception completes and waits for input
			{
				//Clearing receive buffer	
				for(i=0;i<50;i++)
				{
					memset(ptr[i],0,sizeof(ptr[i]));				
				}
				f=0;//Clearing the flag
				if(data_flag==1)//For setting Values(RTC,Directions,LAT,LONG)
				{
				//	__fpurge(stdin);
					publish("Enter Ur Data: ",16,topic,mosq);
					while(sub_flag!=1);
					if(sub_flag==1)
					{
						strcpy(command,sub_temp);
				//		printf("%s\n",sub_temp);
						memset(sub_temp,0,sizeof(sub_temp));
						sub_flag=0;
					}
					//fgets(command,sizeof(command),stdin);
					if(command[0]=='\n') //if new line or null given go back to menu
					{
						null_flag1=1;
						UART_Tx();
					}
					else
					{
						//Setting the values
						for(ii=0,jj=0; ii<strlen(command); ii++,jj++)
						{
							cmd_cpy[jj]=command[ii];
							cmd_cpy[++jj]= ' ';
						}
					cmd_cpy[jj]='\r';
					cmd_cpy[++jj]='\n';
					cmd_cpy[++jj]='\0';

			//	for(ii=0; ii<strlen(cmd_cpy); ii++,jj++)
			//		printf("cmd:%c %x\n",cmd_cpy[ii],cmd_cpy[ii]);
					UART_Tx();
					}
					//UART_Tx();
				}
				else{
					//Call For Normal menu command
					UART_Tx();
				}
			}
		}
	//After exiting Device Clearing Buffers and Flags
		memset(ptr,0,50*200*sizeof(ptr[0][0]));
		memset(rx_pkt,0,sizeof(rx_pkt));
		memset(cmd_cpy,0,sizeof(cmd_cpy));
		pkt_len=0,sub_flag=0,null_flag1=0,adflag=0,j=0,f=0,ext=0,ss=0,chk_flag=0,msg_flag=0;
		memset(xbeeaddr,0,sizeof(xbeeaddr));

	}
	//Mosquitto Close Functions Should not get into This
	mosquitto_loop_stop(mosq,true);
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();
	
return(0);
}
