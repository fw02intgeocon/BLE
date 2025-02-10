/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : serial_port.c
* Author             : AMS - RF  Application team
* Version            : V1.1.0
* Date               : 24-August-2022
* Description        : This file handles bytes received from USB and the 
*                      serial port device configuration, connection..... 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Include -------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "app_state.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "osal.h"
#include "gatt_db.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* discovery procedure mode context */
typedef struct discoveryContext_s {
  uint8_t check_disc_proc_timer;
  uint8_t check_disc_mode_timer;
  uint8_t is_device_found; 
  uint8_t do_connect;
  tClockTime startTime;
  uint8_t device_found_address_type;
  uint8_t device_found_address[6];
  uint16_t device_state;
} discoveryContext_t;

/* Private defines -----------------------------------------------------------*/
#define BLE_NEW_SERIALPORT_COMPLETE_LOCAL_NAME_SIZE 12
#define CMD_BUFF_SIZE 3000

#define DISCOVERY_TIMEOUT 3000 /* at least 3 seconds */

#define DISCOVERY_PROC_SCAN_INT 0x4000 
#define DISCOVERY_PROC_SCAN_WIN 0x4000
#define ADV_INT_MIN 0x90
#define ADV_INT_MAX 0x90 
//AK
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DATA_STORAGE_PAGE       (N_PAGES-8)
#define DATA_STORAGE_ADDR       (((N_PAGES-8)*N_BYTES_PAGE) + FLASH_START)


/* Private macros ------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
static discoveryContext_t discovery;
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;
extern uint16_t SerialPortServHandle, TXCharHandle, RXCharHandle; 

/* Private function prototypes -----------------------------------------------*/
void SdkDelayMs(volatile uint32_t lTimeMs);

extern uint32_t baud_rate;
/* UUIDs */
UUID_t UUID_Tx;
UUID_t UUID_Rx;
uint32_t baud_rate=0;
uint16_t tx_handle=0, rx_handle=0;
uint16_t discovery_time = 0;
uint8_t device_role = 0xFF;
uint8_t counter = 0;
uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'S','P','o','r','t','_','1','2','_','M','S'};

uint8_t AT_CMD[9]={"AT"};
uint8_t AT_NAME[9]={"AT+NAME="};
uint8_t AT_NAME_READ[9]={"AT+NAME?"};
uint8_t AT_NODE[14]={"AT+NODEID="};
uint8_t AT_NODE_READ[14]={"AT+NODEID?"};
uint8_t AT_MAC[14]={"AT+MAC="};
uint8_t AT_MAC_READ[14]={"AT+MAC?"};
uint8_t AT_UART1[9]={"AT+UART="};
uint8_t AT_UART2[9]={"AT+UART?"};
uint8_t AT_UART3[9]={"AT+UART=?"};

uint8_t AT_OK[9]={"\r\nOK\r\n"};
uint8_t slave_name[80]={0};
uint8_t read_name[80]={0};
uint8_t node_ID[4]={0};
tBDAddr bdaddr;// = {SERVER_ADDRESS};

static char cmd[CMD_BUFF_SIZE];
 int cmd_buff_end = 0, cmd_buff_start = 0;

uint8_t i=0;
uint32_t memory_word=0;
void name_write(char slave_name[],uint8_t len){


    FLASH_ErasePage(DATA_STORAGE_PAGE);
    //FLASH_ErasePage(1);aci_gap_start_general_di

    /* Wait for the end of erase operation */
    while(FLASH_GetFlagStatus(Flash_CMDDONE) != SET);

    // Name Flash Write Logic
    for(i = 0; i <= len; i+=4) {
      memory_word = 0;
      for(uint8_t j = 0; j < 4; j++) {
          memory_word |= (uint32_t)(slave_name[i+j]<<(8*j));
      }
      /* Program the word */
      FLASH_ProgramWord(DATA_STORAGE_ADDR + (i*4), memory_word);
    }
}

void node_id_write(char node_ID[]){
    FLASH_ErasePage(121);
    //FLASH_ErasePage(1);aci_gap_start_general_di

    /* Wait for the end of erase operation */
    while(FLASH_GetFlagStatus(Flash_CMDDONE) != SET);

    // Node ID Flash Write Logic
      memory_word = 0;
      for(uint8_t j = 0; j < 4; j++) {
          memory_word |= (uint32_t)(node_ID[j]<<(8*j));
      }
      /* Program the word */
      FLASH_ProgramWord(0x1007c800, memory_word);

}

void baud_write(uint32_t baud_rate){
    FLASH_ErasePage(122);
    //FLASH_ErasePage(1);aci_gap_start_general_di

    /* Wait for the end of erase operation */
    while(FLASH_GetFlagStatus(Flash_CMDDONE) != SET);

    //baud rate flash write
	FLASH_ProgramWord(0x1007D000,(uint32_t)baud_rate);
}

void mac_write(char mac[]){
    FLASH_ErasePage(123);
    //FLASH_ErasePage(1);aci_gap_start_general_di

    /* Wait for the end of erase operation */
    while(FLASH_GetFlagStatus(Flash_CMDDONE) != SET);

    // Name Flash Write Logic
    for(i = 0; i <= 7; i+=4) {
      memory_word = 0;
      for(uint8_t j = 0; j < 4; j++) {
          memory_word |= (uint32_t)(mac[i+j]<<(8*j));
      }
      /* Program the word */
      FLASH_ProgramWord(0x1007D800 + (i*4), memory_word);
    }
//      /* Program the word */
//      FLASH_ProgramWord(0x1007D800, memory_word);

}

void flash_read(void){
	for(i = 0; i < 20; i+=4) {
	      memory_word = FLASH_ReadWord(DATA_STORAGE_ADDR + (i*4));
	      for(uint8_t j = 0; j < 4; j++) {
	    	  slave_name[i+j] =(memory_word >> (8*j) & 0xFF);
	      }
	}
	slave_name[i]=0;
	//Node_ID READ
	memory_word =0;
	     memory_word = FLASH_ReadWord(0x1007c800);
	      for(uint8_t j = 0; j < 4; j++) {
	    	  node_ID[j] =(memory_word >> (8*j) & 0xFF);
	      }
	      node_ID[2]=0;
	  	//MAC addr READ
	      memory_word=0;
	  	for(i = 0; i < 7; i+=4) {
	  	      memory_word = FLASH_ReadWord(0x1007D800 + (i*4));
	  	      for(uint8_t j = 0; j < 4; j++) {
	  	    	bdaddr[i+j] =(memory_word >> (8*j) & 0xFF);
	  	      }
	  	}
	  	bdaddr[6]=0x00;
	      ///////////////

//	  	     memory_word = FLASH_ReadWord(0x1007c600);
//	  	      for(uint8_t j = 0; j < 4; j++) {
//	  	    	bdaddr[j] =(memory_word >> (8*j) & 0xFF);
//	  	      }
//		      memory_word=0;
//		  	  memory_word = FLASH_ReadWord(0x1007C800);
//		  	  bdaddr[4] = (memory_word >> (0) & 0xFF);
//		  	  bdaddr[5] = (memory_word >> (8) & 0xFF);


//baud rate read
	baud_rate = FLASH_ReadWord(0x1007D000);
}
void clean_name(void){
	int i=0;
	for(i=0;i<80;i++){
		slave_name[i]=0x00;
	}

}

void cmd_clean(){
	int i=0;
	for(i=0;i<CMD_BUFF_SIZE;i++){

		cmd[i]=0;
	}
}

void delay(void){
	int k=0;

	for(k=0;k<200000;k++){

	}
}

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Process_InputData.
* Description    : Process a command. It should be called when data are received.
* Input          : data_buffer: data address.
*	           Nb_bytes: number of received bytes.
* Return         : none.
*******************************************************************************/

  uint8_t binary_M = 1;
  uint32_t nodeid = 02;
uint8_t start=1;
uint8_t exp=0;
uint8_t cnt$=0;

  void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
  {
/*// default setting //
	  uint8_t i;

	  for (i = 0; i < Nb_bytes; i++)
	  {
	    if(cmd_buff_end >= CMD_BUFF_SIZE-1){
	      cmd_buff_end = 0;
	    }

	    cmd[cmd_buff_end] = data_buffer[i];
	    SdkEvalComIOSendData(data_buffer[i]);
	    cmd_buff_end++;
	    PRINTF("default = %d\n",cmd_buff_end);
	    if(cmd_buff_end==3){
	  	if(cmd[0] == '@' &&cmd[1] == '#' &&cmd[2] == '$' ){
	  		cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.
	  		//cmd_buff_end=0;
	  	}

	  }

	    if((cmd[cmd_buff_end-1] == '\n') || (cmd[cmd_buff_end-1] == '\r')){
	      if(cmd_buff_end != 1){

	        cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.

	        // Set flag to send data. Disable UART IRQ to avoid overwriting buffer with new incoming data
	        APP_FLAG_SET(SEND_DATA);
	        NVIC_DisableIRQ(UART_IRQn);

	        cmd_buff_start = 0;

	      }
	      else {
	        cmd_buff_end = 0; // Discard
	      }
	    }
	  }
*/


    uint8_t idx=0;
    uint32_t memory_word2=0;
    char *token=0;
    for (i = 0; i < Nb_bytes; i++) {
      if(cmd_buff_end >= CMD_BUFF_SIZE-1)
        cmd_buff_end = 0;

	  	 if(binary_M && APP_FLAG(CONNECTED)){
	  	  	  if(start==1){
	  			cmd[0]= node_ID[0];
	  			cmd[1]= node_ID[1];
	  			cmd[2]= 0x2C;

	  			cmd_buff_end=3;
	  			start=0;
	  			//cmd[cmd_buff_end] = data_buffer[i];

	  			//SdkEvalComIOSendData(0x00);
	  	  	  }
		  	 cmd[cmd_buff_end] = data_buffer[i];
		  	 cmd_buff_end++;

		 //escape sequence
       	if(cmd[cmd_buff_end-1] == '$' && cmd[cmd_buff_end-2] == '#' && cmd[cmd_buff_end-3] == '@'){
       		cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.
       		cmd_buff_end=0;
       		binary_M=0;
       		delay();
       		PRINTF("%s",AT_OK);
       		cmd_buff_end = 0;
       	}

       	if(!exp){

       	if(node_ID[0] == 0x30 && node_ID[1] == 0x32){ //this device only send 0x0D
           	if(cmd[cmd_buff_end-1] == '\r'){
      	      if(cmd_buff_end != 1){
      	    	start=1;
    		  	 cmd[cmd_buff_end++] = 0x03;
    		  	cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.
    		  	 //cmd_buff_end =0;

    			// Set flag to send data. Disable UART IRQ to avoid overwriting buffer with new incoming data
    			APP_FLAG_SET(SEND_DATA);
    			NVIC_DisableIRQ(UART_IRQn);

    			cmd_buff_start = 0;
      	      }else{
      	    	cmd_buff_end = 0; // Discard
      	      }
           	}
       	}else{

       	if((cmd[cmd_buff_end-1] == '\n') && (cmd[cmd_buff_end-2] == '\r')){
  	      if(cmd_buff_end != 1){
  	    	start=1;
		  	 cmd[cmd_buff_end++] = 0x03;
		  	cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.
		  	 //cmd_buff_end =0;

			// Set flag to send data. Disable UART IRQ to avoid overwriting buffer with new incoming data
			APP_FLAG_SET(SEND_DATA);
			NVIC_DisableIRQ(UART_IRQn);

			cmd_buff_start = 0;
  	      }else{
  	    	cmd_buff_end = 0; // Discard
  	      }
       	}
       	}

	  	 }else {

	  		 if(cmd[cmd_buff_end-1] == 0x24){
	  			cnt$++;
	  			if(cnt$ >=18){
	  				cnt$=0;
	  	  	      if(cmd_buff_end != 1){
	  	  	    	start=1;
	  			  	 cmd[cmd_buff_end++] = 0x03;
	  			  	cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.
	  			  	 //cmd_buff_end =0;

	  				// Set flag to send data. Disable UART IRQ to avoid overwriting buffer with new incoming data
	  				APP_FLAG_SET(SEND_DATA);
	  				NVIC_DisableIRQ(UART_IRQn);
	  				exp=0;
	  				cmd_buff_start = 0;
	  	  	      }else{
	  	  	    	cmd_buff_end = 0; // Discard
	  	  	      }


	  			}


	  		 }


	  	 }
	  	//PRINTF("DEFAULT = %d",cmd_buff_end);
	  	 }else{
		  	 cmd[cmd_buff_end] = data_buffer[i];
		  	 cmd_buff_end++;

  		if((cmd[cmd_buff_end-1] == '\n') || (cmd[cmd_buff_end-1] == '\r')){
  		  if(cmd_buff_end != 1) {

  			cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.


  			if(!strncmp(AT_NAME,cmd,strlen(AT_NAME))){

  				token = strtok(cmd,"\"");
  				while(token != NULL){
  					idx++;
  					if(idx==2){
  						idx=0;
  						clean_name();
  						slave_name[0]=0x09;
  						memcpy(&slave_name[1],token,strlen(token));
  						name_write(slave_name,strlen(slave_name));
  						break;
  					}

  				token = strtok(0,"\"");
  				}

  				 PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;


  			}else if(!strncmp(AT_NAME_READ,cmd,strlen(AT_NAME_READ))){
  				flash_read();
  				PRINTF("\r\n+NAME:\"%s\"\r\n",&slave_name[1]);
  				PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;

  			}else if(!strncmp(AT_NODE,cmd,strlen(AT_NODE))){
  				token = strtok(cmd,"\"");
  				while(token != NULL){
  					idx++;
  					if(idx==2){
  						idx=0;
  						node_ID[0]=0;
  						node_ID[1]=0;
  						node_ID[2]=0;
  						node_ID[3]=0;
  						memcpy(node_ID,token,strlen(token));
  						//name_write(slave_name,node_ID,strlen(slave_name),baud_rate);
  						node_id_write(node_ID);
  						break;
  					}

  				token = strtok(0,"\"");
  				}

  				PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;

  			}else if(!strncmp(AT_NODE_READ,cmd,strlen(AT_NODE_READ))){
  				flash_read();
  				PRINTF("\r\n+NODEID:\"%s\"\r\n",node_ID);
  				PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;

  			}else if(!strncmp(AT_MAC,cmd,strlen(AT_MAC))){ //MAC Address

  				bdaddr[0] = cmd[7];
  				bdaddr[1] = cmd[9];
  				bdaddr[2] = cmd[11];
  				bdaddr[3] = cmd[13];
  				bdaddr[4] = cmd[15];
  				bdaddr[5] = cmd[17];

  				mac_write(bdaddr);
  				delay();
  				PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;

  			}else if(!strncmp(AT_MAC_READ,cmd,strlen(AT_MAC_READ))){
  				flash_read();
  				PRINTF("\r\n+MAC:\"%s\"\r\n",bdaddr);
  				PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;

  			}else if(!strncmp(AT_UART3,cmd,strlen(AT_UART3))){
  				PRINTF("\r\n+UART=<1-8>\r\n");
  				delay();
  				PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;

  			}else if(!strncmp(AT_UART1,cmd,strlen(AT_UART1))){

  				if(cmd[8]==0x31){
  					PRINTF("%s",AT_OK);
  					cmd_buff_end = 0;
  					delay();
  					baud_write(115200);
  					UART_Cmd(DISABLE);
  					SdkEvalComUartInit(115200);

  				}else if(cmd[8]==0x32){

  					PRINTF("%s",AT_OK);
  					cmd_buff_end = 0;
  					delay();
  					baud_write(57600);
  					UART_Cmd(DISABLE);
  					SdkEvalComUartInit(57600);
  				}else if(cmd[8]==0x33){
  					PRINTF("%s",AT_OK);
  					cmd_buff_end = 0;
  					delay();
  					baud_write(28800);
  					UART_Cmd(DISABLE);
  					SdkEvalComUartInit(28800);

  				}else if(cmd[8]==0x34){
  					PRINTF("%s",AT_OK);
  					cmd_buff_end = 0;
  					delay();
  					baud_write(19200);
  					UART_Cmd(DISABLE);
  					SdkEvalComUartInit(19200);

  				}else if(cmd[8]==0x35){
  					PRINTF("%s",AT_OK);
  					cmd_buff_end = 0;
  					delay();
  					baud_write(9600);
  					UART_Cmd(DISABLE);
  					SdkEvalComUartInit(9600);

  				}else if(cmd[8]==0x36){
  					PRINTF("%s",AT_OK);
  					cmd_buff_end = 0;
  					delay();
  					baud_write(4800);
  					UART_Cmd(DISABLE);
  					SdkEvalComUartInit(4800);

  				}else if(cmd[8]==0x37){
  					PRINTF("%s",AT_OK);
  					cmd_buff_end = 0;
  					delay();
  					baud_write(2400);
  					UART_Cmd(DISABLE);
  					SdkEvalComUartInit(2400);

  				}else if(cmd[8]==0x38){
  					PRINTF("%s",AT_OK);
  					cmd_buff_end = 0;
  					delay();
  					baud_write(1200);
  					UART_Cmd(DISABLE);
  					SdkEvalComUartInit(1200);

  				}

  			}else if(!strncmp(AT_UART2,cmd,strlen(AT_UART2))){
  				//baud_rate = FLASH_ReadWord(0x1007c900);
  				flash_read();

  				if(baud_rate==115200){
  	  				PRINTF("\r\n+UART=1\r\n");
  					delay();

  				}else if(baud_rate==57600){
  					PRINTF("\r\n+UART=2\r\n");
  					delay();

  				}else if(baud_rate==28800){
  					PRINTF("\r\n+UART=3\r\n");
  					delay();

  				}else if(baud_rate==19200){
  					PRINTF("\r\n+UART=4\r\n");
  					delay();

  				}else if(baud_rate==9600){
  					PRINTF("\r\n+UART=5\r\n");
  					delay();

  				}else if(baud_rate==4800){
  					PRINTF("\r\n+UART=6\r\n");
  					delay();

  				}else if(baud_rate==2400){
  					PRINTF("\r\n+UART=7\r\n");
  					delay();

  				}else if(baud_rate==1200){
  					PRINTF("\r\n+UART=8\r\n");
  					delay();

  				}

  				PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;

  			}else if(!strncmp(AT_CMD,cmd,strlen(AT_CMD))){
  				delay();
  				PRINTF("%s",AT_OK);
  				cmd_buff_end = 0;

  			}


  			// Set flag to send data. Disable UART IRQ to avoid overwriting buffer with new incoming data
  			//APP_FLAG_SET(SEND_DATA); //AK check
  			//NVIC_DisableIRQ(UART_IRQn);

  			cmd_buff_start = 0;
  		  }
  		  else {
  			cmd_buff_end = 0; // Discard
  		  }
  		}
    }
    }
  }

/*******************************************************************************
* Function Name  : Reset_DiscoveryContext.
* Description    : Reset the discovery context.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Reset_DiscoveryContext(void)
{
  discovery.check_disc_proc_timer = FALSE;
  discovery.check_disc_mode_timer = FALSE;
  discovery.is_device_found = FALSE; 
  discovery.do_connect = FALSE;
  discovery.startTime = 0;
  discovery.device_state = INIT;
  Osal_MemSet(&discovery.device_found_address[0], 0, 6);
  device_role = 0xFF;
}

/*******************************************************************************
* Function Name  : Setup_DeviceAddress.
* Description    : Setup the device address.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Setup_DeviceAddress(void)
{
  tBleStatus ret;
 // uint8_t bdaddr[] = {0x00, 0x00, 0x00, 0xE1, 0x80, 0x02};
  //uint8_t random_number[8];
  flash_read();
  /* get a random number from BlueNRG */ 
//  ret = hci_le_rand(random_number);
//  if(ret != BLE_STATUS_SUCCESS)
//     PRINTF("hci_le_rand() call failed: 0x%02x\n", ret);
//
//
//  discovery_time = DISCOVERY_TIMEOUT;
//
//  /* setup discovery time with random number */
//  for (uint8_t i=0; i<8; i++) {
//    discovery_time += (2*random_number[i]);
//  }
//
//  /* Setup last 3 bytes of public address with random number */
//  bdaddr[0] = (uint8_t) (random_number[0]);
//  bdaddr[1] = (uint8_t) (random_number[3]);
//  bdaddr[2] = (uint8_t) (random_number[6]);
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS) {
      //PRINTF("Setting BD_ADDR failed 0x%02x\n", ret);
  } else {
    //PRINTF("Public address: ");
    for (uint8_t i=5; i>0; i--) {
      //PRINTF("%02X-", bdaddr[i]);
    }
    //PRINTF("%02X\n", bdaddr[0]);
  }
}

/*******************************************************************************
* Function Name  : Find_DeviceName.
* Description    : Extracts the device name.
* Input          : Data length.
*                  Data value
* Return         : TRUE if the local name found is the expected one, FALSE otherwise.
*******************************************************************************/
uint8_t Find_DeviceName(uint8_t data_length, uint8_t *data_value)
{
  uint8_t index = 0;
  
  while (index < data_length) {
    /* Advertising data fields: len, type, values */
    /* Check if field is complete local name and the lenght is the expected one for serial port  */
    if (data_value[index+1] == AD_TYPE_COMPLETE_LOCAL_NAME) {
      /* check if found device name is the expected one: local_name */ 
      if (memcmp(&data_value[index+1], &local_name[0], BLE_NEW_SERIALPORT_COMPLETE_LOCAL_NAME_SIZE) == 0)
        return TRUE;
      else
        return FALSE;
    } else {
      /* move to next advertising field */
      index += (data_value[index] +1); 
    }
  }
  
  return FALSE;
}

/*******************************************************************************
* Function Name  : SerialPort_DeviceInit.
* Description    : Init Serial Port device.
* Input          : None.
* Return         : Status.
*******************************************************************************/
uint8_t SerialPort_DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  /* Setup the device address */
  Setup_DeviceAddress();

  /* Set the TX power to -2 dBm */
  aci_hal_set_tx_power_level(1, 7);

  /* GATT Init */
  ret = aci_gatt_init();    
  if(ret != BLE_STATUS_SUCCESS) {
      //PRINTF("GATT_Init failed: 0x%02x\n", ret);
      return ret;
  }

  /* GAP Init */
  ret = aci_gap_init(/*GAP_CENTRAL_ROLE|*/GAP_PERIPHERAL_ROLE, 0,0x07, &service_handle,
                     &dev_name_char_handle, &appearance_char_handle);
  if(ret != BLE_STATUS_SUCCESS) {
    //PRINTF("GAP_Init failed: 0x%02x\n", ret);
    return ret;
  }

  /* Add Device Service & Characteristics */
  ret = Add_SerialPort_Service();
  if(ret != BLE_STATUS_SUCCESS) {
    //PRINTF("Error while adding service: 0x%02x\n", ret);
    return ret;
  }

  /* Reset the discovery context */
  Reset_DiscoveryContext();

  return BLE_STATUS_SUCCESS;
}
//AK
void Send_Data_Over_BLE(void)
{
  if(!APP_FLAG(SEND_DATA) || APP_FLAG(TX_BUFFER_FULL))
    return;
  
  while(cmd_buff_start < cmd_buff_end){
    uint32_t len = MIN(20, cmd_buff_end - cmd_buff_start);
    aci_gatt_update_char_value(connection_handle,TXCharHandle,0,len,(uint8_t *)cmd+cmd_buff_start);
      if(aci_gatt_update_char_value_ext(connection_handle,SerialPortServHandle,TXCharHandle,1,len,0, len,(uint8_t *)cmd+cmd_buff_start)==BLE_STATUS_INSUFFICIENT_RESOURCES){

        APP_FLAG_SET(TX_BUFFER_FULL);
        return;
      }

    cmd_buff_start += len;
  }
  
  cmd_clean();
  // All data from buffer have been sent.
  APP_FLAG_CLEAR(SEND_DATA);
  cmd_buff_end = 0;
  NVIC_EnableIRQ(UART_IRQn);
}

/*******************************************************************************
* Function Name  : Connection_StateMachine.
* Description    : Connection state machine.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Connection_StateMachine(void)
{  
  uint8_t ret;
   
  switch (discovery.device_state) {
  case (INIT):
    {
      Reset_DiscoveryContext();
      discovery.device_state = START_DISCOVERY_PROC;
    }
    break; /* end case (INIT) */
  case (START_DISCOVERY_PROC):
    {
      ret = aci_gap_start_general_discovery_proc(DISCOVERY_PROC_SCAN_INT, DISCOVERY_PROC_SCAN_WIN, PUBLIC_ADDR, 0x00); 
      if (ret != BLE_STATUS_SUCCESS) {
        //PRINTF("aci_gap_start_general_discovery_proc() failed: %02X\n",ret);
        discovery.device_state = DISCOVERY_ERROR; 
      } else {
        //PRINTF("aci_gap_start_general_discovery_proc OK\n");
        discovery.startTime = Clock_Time();
        discovery.check_disc_proc_timer = TRUE; 
        discovery.check_disc_mode_timer = FALSE; 
        discovery.device_state = WAIT_TIMER_EXPIRED; 
      }
    }
    break;/* end case (START_DISCOVERY_PROC) */
  case (WAIT_TIMER_EXPIRED):
    {
      /* Verify if startTime check has to be done  since discovery procedure is ongoing */
      if (discovery.check_disc_proc_timer == TRUE) {
        /* check startTime value */
        if (Clock_Time() - discovery.startTime > discovery_time) {  
          discovery.check_disc_proc_timer = FALSE; 
          discovery.startTime = 0;
          discovery.device_state = DO_TERMINATE_GAP_PROC; 
          
        }/* if (Clock_Time() - discovery.startTime > discovery_time) */
      }  
      /* Verify if startTime check has to be done  since discovery mode is ongoing */
      else if (discovery.check_disc_mode_timer == TRUE) {
        /* check startTime value */
        if (Clock_Time() - discovery.startTime > discovery_time) {  
          discovery.check_disc_mode_timer = FALSE; 
          discovery.startTime = 0;
          
          /* Discovery mode is ongoing: set non discoverable mode */
          discovery.device_state = DO_NON_DISCOVERABLE_MODE; 
          
        }/* else if (discovery.check_disc_mode_timer == TRUE) */
      }/* if ((discovery.check_disc_proc_timer == TRUE) */
    }
    break; /* end case (WAIT_TIMER_EXPIRED) */
  case (DO_NON_DISCOVERABLE_MODE):
    {
      ret = aci_gap_set_non_discoverable();
      if (ret != BLE_STATUS_SUCCESS) {
        //PRINTF("aci_gap_set_non_discoverable() failed: 0x%02x\n", ret);
        discovery.device_state = DISCOVERY_ERROR; 
      } else {
        //PRINTF("aci_gap_set_non_discoverable() OK\n");
        /* Restart Central discovery procedure */
        discovery.device_state = INIT; 
      }
    }
    break; /* end case (DO_NON_DISCOVERABLE_MODE) */
  case (DO_TERMINATE_GAP_PROC):
    {
      /* terminate gap procedure */
      ret = aci_gap_terminate_gap_proc(0x02); // GENERAL_DISCOVERY_PROCEDURE
      if (ret != BLE_STATUS_SUCCESS) 
      {
       //PRINTF("aci_gap_terminate_gap_procedure() failed: 0x%02x\n", ret);
        discovery.device_state = DISCOVERY_ERROR; 
        break;
      }
      else 
      {
        //PRINTF("aci_gap_terminate_gap_procedure() OK\n");
        discovery.device_state = WAIT_EVENT; /* wait for GAP procedure complete */
      }
    }
    break; /* end case (DO_TERMINATE_GAP_PROC) */
  case (DO_DIRECT_CONNECTION_PROC):
    {
     // PRINTF("Device Found with address: ");
      for (uint8_t i=5; i>0; i--) {
        //PRINTF("%02X-", discovery.device_found_address[i]);
      }
      //PRINTF("%02X\n", discovery.device_found_address[0]);
      /* Do connection with first discovered device */ 
      ret = aci_gap_create_connection(DISCOVERY_PROC_SCAN_INT, DISCOVERY_PROC_SCAN_WIN,
                                      discovery.device_found_address_type, discovery.device_found_address,
                                      PUBLIC_ADDR, 40, 40, 0, 60, 2000 , 2000); 
      if (ret != BLE_STATUS_SUCCESS) {
        //PRINTF("aci_gap_create_connection() failed: 0x%02x\n", ret);
        discovery.device_state = DISCOVERY_ERROR; 
      } else {
        //PRINTF("aci_gap_create_connection() OK\n");
        discovery.device_state = WAIT_EVENT;
      }
    }
    break; /* end case (DO_DIRECT_CONNECTION_PROC) */
  case (WAIT_EVENT):
    {
      discovery.device_state = WAIT_EVENT;
    }
    break; /* end case (WAIT_EVENT) */
  case (ENTER_DISCOVERY_MODE):
    {
      /* Put Peripheral device in discoverable mode */
	  char len;

      /* disable scan response */
      hci_le_set_scan_response_data(0,NULL);
      flash_read();
      //memcpy(local_name,slave_name,strlen(slave_name));
      ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                     strlen(slave_name), slave_name, 0, NULL, 0, 0);
      if (ret != BLE_STATUS_SUCCESS) {
        //PRINTF("aci_gap_set_discoverable() failed: 0x%02x\n", ret);
        discovery.device_state = DISCOVERY_ERROR; 
      } else {
       // PRINTF("aci_gap_set_discoverable() OK\n");
        discovery.device_state = WAIT_EVENT;
        //discovery.startTime = Clock_Time();
        //discovery.check_disc_mode_timer = TRUE;
        //discovery.check_disc_proc_timer = FALSE;
        //discovery.device_state = WAIT_TIMER_EXPIRED;
      }
    }
    break; /* end case (ENTER_DISCOVERY_MODE) */
  case (DISCOVERY_ERROR):
    {
      //Reset_DiscoveryContext();
      discovery.device_state = ENTER_DISCOVERY_MODE;
    }
    break; /* end case (DISCOVERY_ERROR) */
  default:
    break;
   }/* end switch */

}/* end Connection_StateMachine() */

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Application tick to run the state machine.
* Input          : None.
* Return         : None.
*******************************************************************************/
void APP_Tick(void)
{
  if(APP_FLAG(SET_CONNECTABLE)) {
    Connection_StateMachine();
  }
  if(APP_FLAG(CONNECTED)){
	  SdkEvalLedOn(LED1);

  }else {
	  SdkEvalLedOff(LED1);

  }
  
  Send_Data_Over_BLE();
        
  if (device_role == MASTER_ROLE) {
    /* Start TX handle Characteristic discovery if not yet done */
    if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_TX_CHAR_HANDLE)) {
      if (!APP_FLAG(START_READ_TX_CHAR_HANDLE)) {
        /* Discovery TX characteristic handle by UUID 128 bits */
        //const uint8_t charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
        const uint8_t charUuid128_TX[16] = {0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e};
        Osal_MemCpy(&UUID_Tx.UUID_16, charUuid128_TX, 16);
        aci_gatt_disc_char_by_uuid(connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,&UUID_Tx);
        APP_FLAG_SET(START_READ_TX_CHAR_HANDLE);
      }
    }
    /* Start RX handle Characteristic discovery if not yet done */
    else if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_RX_CHAR_HANDLE)) {
      /* Discovery RX characteristic handle by UUID 128 bits */
      if (!APP_FLAG(START_READ_RX_CHAR_HANDLE)) {
        /* Discovery RX characteristic handle by UUID 128 bits */
        //const uint8_t charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
        const uint8_t charUuid128_RX[16] = {0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e};
        
        Osal_MemCpy(&UUID_Rx.UUID_16, charUuid128_RX, 16);
        aci_gatt_disc_char_by_uuid(connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,&UUID_Rx);
        APP_FLAG_SET(START_READ_RX_CHAR_HANDLE);
      }
    }
      
    if(APP_FLAG(CONNECTED) && APP_FLAG(END_READ_TX_CHAR_HANDLE) && APP_FLAG(END_READ_RX_CHAR_HANDLE) && !APP_FLAG(NOTIFICATIONS_ENABLED)) {
      uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
      struct timer t;
      Timer_Set(&t, CLOCK_SECOND*10);
          
      while(aci_gatt_write_char_desc(connection_handle, tx_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED) { 
        // Radio is busy.
        if(Timer_Expired(&t)) break;
      }
      APP_FLAG_SET(NOTIFICATIONS_ENABLED);
    }
  }/* if (device_role == MASTER_ROLE) */      
}


/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : aci_gap_proc_complete_event.
 * Description    : This event indicates the end of a GAP procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gap_proc_complete_event(uint8_t Procedure_Code,
                                 uint8_t Status,
                                 uint8_t Data_Length,
                                 uint8_t Data[])
{
  if (Procedure_Code == GAP_GENERAL_DISCOVERY_PROC) { 
    /* gap procedure complete has been raised as consequence of a GAP 
       terminate procedure done after a device found event during the discovery procedure */
    if (discovery.do_connect == TRUE) {
      discovery.do_connect = FALSE;
      discovery.check_disc_proc_timer = FALSE;
      discovery.startTime = 0; 
      /* discovery procedure has been completed and device found:
         go to connection mode */
      discovery.device_state = DO_DIRECT_CONNECTION_PROC;
    } else {
      /* discovery procedure has been completed and no device found:
         go to discovery mode */
      discovery.check_disc_proc_timer = FALSE;
      discovery.startTime = 0; 
      discovery.device_state = ENTER_DISCOVERY_MODE;
    }
  }
}

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates the end of a connection procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{ 
  /* Set the exit state for the Connection state machine: APP_FLAG_CLEAR(SET_CONNECTABLE); */
  APP_FLAG_CLEAR(SET_CONNECTABLE); 
  discovery.check_disc_proc_timer = FALSE;
  discovery.check_disc_mode_timer = FALSE;
  discovery.startTime = 0; 
  
  connection_handle = Connection_Handle;
  
  APP_FLAG_SET(CONNECTED); 
  discovery.device_state = INIT;
  
  /* store device role */
  device_role = Role;
  
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event indicates the discconnection from a peer device.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  APP_FLAG_CLEAR(CONNECTED);
  /* Make the device connectable again. */
  APP_FLAG_SET(SET_CONNECTABLE);
  APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
  
  APP_FLAG_CLEAR(START_READ_TX_CHAR_HANDLE);
  APP_FLAG_CLEAR(END_READ_TX_CHAR_HANDLE);
  APP_FLAG_CLEAR(START_READ_RX_CHAR_HANDLE); 
  APP_FLAG_CLEAR(END_READ_RX_CHAR_HANDLE); 
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
  
  Reset_DiscoveryContext();

}/* end hci_disconnection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_le_advertising_report_event.
 * Description    : An advertising report is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_advertising_report_event(uint8_t Num_Reports,
                                     Advertising_Report_t Advertising_Report[])
{
  /* Advertising_Report contains all the expected parameters */
  uint8_t evt_type = Advertising_Report[0].Event_Type ;
  uint8_t data_length = Advertising_Report[0].Length_Data;
  uint8_t bdaddr_type = Advertising_Report[0].Address_Type;
  uint8_t bdaddr[7];

  Osal_MemCpy(bdaddr, Advertising_Report[0].Address,6);
      
  /* BLE Serial port device not yet found: check current device found */
  if (!(discovery.is_device_found)) { 
    /* BLE Serial port device not yet found: check current device found */
    if ((evt_type == ADV_IND) && Find_DeviceName(data_length, Advertising_Report[0].Data)) {
      discovery.is_device_found = TRUE; 
      discovery.do_connect = TRUE;  
      discovery.check_disc_proc_timer = FALSE;
      discovery.check_disc_mode_timer = FALSE;
      /* store first device found:  address type and address value */
      discovery.device_found_address_type = bdaddr_type;
      Osal_MemCpy(discovery.device_found_address, bdaddr, 6);
      /* device is found: terminate discovery procedure */
      discovery.device_state = DO_TERMINATE_GAP_PROC;      
    }
  }  
} /* hci_le_advertising_report_event() */


/*******************************************************************************
* Function Name  : Attribute_Modified_CB
* Description    : Attribute modified callback.
* Input          : Attribute handle modified.
*                  Length of the data.
*                  Attribute data.
* Return         : None.
*******************************************************************************/
int chk=1,chk2=0;

char node_chk[24]={0};
//AK receives master data
void Attribute_Modified_CB(uint16_t handle, uint16_t data_length, uint8_t *att_data)
{
  if(handle == RXCharHandle + 1)
  {
	if(chk && att_data[0] == node_ID[0] && att_data[1] == node_ID[1]){

		chk2=1;
		chk=0;

		if(att_data[4] == 0xf0 || att_data[4] == 0xf1 || att_data[4] == 0xf2){
			exp=1;

		}

    for(int i = 3; i < data_length; i++){

    	if(att_data[i]==0x03){
    		chk=1;
    		chk2=0;
    	}
    	if(chk2){
    		PRINTF("%c", att_data[i]);
    	}
    }
  }else {

	    for(int i = 0; i < data_length; i++){

	    	if(att_data[i]==0x03){
	    		chk=1;
	    		chk2=0;
	    	}
	    	if(chk2){
	    		PRINTF("%c", att_data[i]);
	    	}
	    }

  }

  }
  else if(handle == TXCharHandle + 2)
  {
    if(att_data[0] == 0x01)
      APP_FLAG_SET(NOTIFICATIONS_ENABLED);
  }
}

/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : Attribute modified from a peer device.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  Attribute_Modified_CB(Attr_Handle, Attr_Data_Length, Attr_Data);      
} /* end aci_gatt_attribute_modified_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_notification_event.
 * Description    : Notification received from a peer device.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_notification_event(uint16_t Connection_Handle,
                                 uint16_t Attribute_Handle,
                                 uint8_t Attribute_Value_Length,
                                 uint8_t Attribute_Value[])
{ 
	//PRINTF(Attribute_Value); //AK
  if(Attribute_Handle == tx_handle+1) {
    for(volatile uint8_t i = 0; i < Attribute_Value_Length; i++)
    	printf("%c", Attribute_Value[i]);
  }
} /* end aci_gatt_notification_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_disc_read_char_by_uuid_resp_event.
 * Description    : Read characteristic by UUID from a peer device.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_disc_read_char_by_uuid_resp_event(uint16_t Connection_Handle,
                                                uint16_t Attribute_Handle,
                                                uint8_t Attribute_Value_Length,
                                                uint8_t Attribute_Value[])
{
 // printf("aci_gatt_disc_read_char_by_uuid_resp_event, Connection Handle: 0x%04X\n", Connection_Handle);
  if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE)) {
    tx_handle = Attribute_Handle; 
    //printf("TX Char Handle 0x%04X\n", tx_handle);
  } else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE)) {
    rx_handle = Attribute_Handle;
    //printf("RX Char Handle 0x%04X\n", rx_handle);
  }
} /* end aci_gatt_disc_read_char_by_uuid_resp_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_proc_complete_event.
 * Description    : GATT procedure complet event.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{ 
  if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE)) {
    APP_FLAG_SET(END_READ_TX_CHAR_HANDLE);
  } else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE)) {
    APP_FLAG_SET(END_READ_RX_CHAR_HANDLE);
  }
} /* end aci_gatt_proc_complete_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : GATT TX pool available event.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{      
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
} /* end aci_gatt_tx_pool_available_event() */
