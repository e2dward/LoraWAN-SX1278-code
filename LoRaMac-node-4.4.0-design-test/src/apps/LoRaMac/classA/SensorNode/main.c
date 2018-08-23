/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

/*! \file classA/SensorNode/main.c */

#include <string.h>
#include <math.h>
#include "board.h"

#include "LoRaMac.h"
#include "Region.h"
#include "Commissioning.h"

#include "usart.h"//串口头文件



/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 * 定义应用程序数据传输占空比。 5s，以[ms]为单位的值。
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].  定义应用程序数据传输占空比的随机延迟。 1s，以[ms]为单位的值。
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate  默认数据速率 0？
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 * LoRaWAN自适应数据速率
 *
 * \ remark请注意，启用ADR后，终端设备应为静态
 */
#define LORAWAN_ADR_ON                              1

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

/*!
 * LoRaWAN application port  LoRaWAN应用程序端口
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size  用户应用数据缓冲区大小
 */
#if defined( REGION_CN470 ) || defined( REGION_CN779 ) || defined( REGION_EU433 ) || defined( REGION_EU868 ) || defined( REGION_IN865 ) || defined( REGION_KR920 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( REGION_AS923 ) || defined( REGION_AU915 ) || defined( REGION_US915 ) || defined( REGION_US915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#else

#error "Please define a region in the compiler options."

#endif

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;//空中激活 OTAA 使用以下参数
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )//不使用OTAA 则需要 DevAddr NwkSKey AppSKey

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;//DevAddr 定义为0 则根据芯片号生成

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];


/*!
 * Received data
 */
//static uint8_t ReceData[LORAWAN_APP_DATA_MAX_SIZE];
//static int ReceLen=0;


/*!
 * Indicates if the node is sending confirmed or unconfirmed messages 指示节点是否正在发送已确认或未确认的消息
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle  定义应用程序数据传输占空比
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle  定时器处理应用程序数据传输占空比
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED  指定应用程序LED的状态
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the state of LED1   LED_R G B  PA0 PA1 PB12  board.h
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED4
 */
static TimerEvent_t Led4Timer;

/*!
 * Indicates if a new packet can be sent  指示是否可以发送新数据包
 */
static bool NextTx = true;

/*!
 * Device states  设备状态
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data  LoRaWAN一致性测试支持数据
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
 * \brief   Prepares the payload of the frame   准备 帧 的 有效负载  
 * 上行数据 数据头MHDR + 载荷MACPayload(帧头FHDR(4字节终端地址(DevAddr)  1字节帧控制字节 (FCtrl) 2字节帧计数器 (FCnt)
 * 和用来传输 MAC 命令的帧选项 (FOpts，最多 15 个字节 ))) + 校验MIC
 */
static void PrepareTxFrame( uint8_t port )
{
	  uint8_t len=1;	
	uint16_t times;
	  int i;
	  uart_init(115200);
    switch( port )
    {
    case 2:
        {
#if defined( REGION_CN470 ) || defined( REGION_CN779 ) || defined( REGION_EU433 ) || defined( REGION_EU868 ) || defined( REGION_IN865 ) || defined( REGION_KR920 )
            uint16_t pressure = 0;
            int16_t altitudeBar = 0;
            int16_t temperature = 0;
            int32_t latitude, longitude = 0;
            int16_t altitudeGps = 0xFFFF;//海拔高度 等一系列设定的需要发送的帧
            uint8_t batteryLevel = 0;
					
//					  printf("\r\n请输入要发送的消息:\r\n");
//					  HAL_UART_Receive(&UART1_Handler,(uint8_t *)&AppData,16,3000);//在这个语句停留5000ms内等待接收16个字节数据，把数据存放在 AppData 中

//            pressure = ( uint16_t )( MPL3115ReadPressure( ) / 10 );             // in hPa / 10
//            temperature = ( int16_t )( MPL3115ReadTemperature( ) * 100 );       // in 癈 * 100
//            altitudeBar = ( int16_t )( MPL3115ReadAltitude( ) * 10 );           // in m * 10
//            batteryLevel = BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
//            GpsGetLatestGpsPositionBinary( &latitude, &longitude );
//            altitudeGps = GpsGetLatestGpsAltitude( );                           // in m

//            AppData[0] = AppLedStateOn;//false
//            AppData[1] = ( pressure >> 8 ) & 0xFF;//0x00
//            AppData[2] = pressure & 0xFF;
//            AppData[3] = ( temperature >> 8 ) & 0xFF;
//            AppData[4] = temperature & 0xFF;
//            AppData[5] = ( altitudeBar >> 8 ) & 0xFF;
//            AppData[6] = altitudeBar & 0xFF;
//            AppData[7] = batteryLevel;
//            AppData[8] = ( latitude >> 16 ) & 0xFF;
//            AppData[9] = ( latitude >> 8 ) & 0xFF;
//            AppData[10] = latitude & 0xFF;
//            AppData[11] = ( longitude >> 16 ) & 0xFF;
//            AppData[12] = ( longitude >> 8 ) & 0xFF;
//            AppData[13] = longitude & 0xFF;
//            AppData[14] = ( altitudeGps >> 8 ) & 0xFF;
//            AppData[15] = altitudeGps & 0xFF;

            AppData[0] = 0x11;
            AppData[1] = 0x22;
            AppData[2] = 0x33;
            AppData[3] = 0x44;
            AppData[4] = 0x55;
            AppData[5] = 0x66;
            AppData[6] = 0x77;
            AppData[7] = batteryLevel;
            AppData[8] = ( latitude >> 16 ) & 0xFF;
            AppData[9] = ( latitude >> 8 ) & 0xFF;
            AppData[10] = latitude & 0xFF;
            AppData[11] = ( longitude >> 16 ) & 0xFF;
            AppData[12] = ( longitude >> 8 ) & 0xFF;
            AppData[13] = longitude & 0xFF;
            AppData[14] = ( altitudeGps >> 8 ) & 0xFF;
            AppData[15] = altitudeGps & 0xFF;
						
						//USART_RX_BUF[0]=3;
						printf("请输入要发送的消息数,以回车键结束\r\n"); 
						__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
						//HAL_UART_Receive(&huart1,(uint8_t *)&AppData,16,3000);//在这个语句停留5000ms内等待接收16个字节数据，把数据存放在 AppData 中
						//for(i=0;i<16;i++){printf("%x\n",AppData[i]); }
										//HAL_UART_Receive(&huart1, (uint8_t *)aRxBuffer, 1,2000);
						
		
						//HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);
						//printf("%x\n",aRxBuffer[0]); 
						
//						if(USART_RX_STA&0x8000)
//						{					   
//							len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
//							printf("\r\n您发送的消息为:\r\n");
//							HAL_UART_Transmit(&huart1,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
//							while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发送结束
//							printf("\r\n\r\n");//插入换行
//							USART_RX_STA=0;
//						}else
//						{
//							times++;
//							if(times%200==0)printf("请输入数据,以回车键结束\r\n");   
//						} 
						
						if(USART_RX_CNT>0){	   
							len=USART_RX_CNT;//得到此次接收到的数据长度
							printf("\r\n您发送的消息为:\r\n");
							for(i=0;i<USART_RX_CNT;i++){
							    printf("%x",USART_RX_BUF[i]); 
							}
							//HAL_UART_Transmit(&huart1,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
							while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发送结束
							printf("\r\n\r\n");//插入换行
							
							
							for(i=0;i<USART_RX_CNT;i++)
							{
									AppData[i]=USART_RX_BUF[i];//获得接收数据
							}
							
							USART_RX_CNT=0;
					}
						//printf("\r\n请输入要发送的消息:\r\n");
					  //HAL_UART_Receive(&huart1,(uint8_t *)&AppData,16,3000);//在这个语句停留5000ms内等待接收16个字节数据，把数据存放在 AppData 中

						
#elif defined( REGION_AS923 ) || defined( REGION_AU915 ) || defined( REGION_US915 ) || defined( REGION_US915_HYBRID )
            int16_t temperature = 0;
            int32_t latitude, longitude = 0;
            uint16_t altitudeGps = 0xFFFF;
            uint8_t batteryLevel = 0;

            temperature = ( int16_t )( MPL3115ReadTemperature( ) * 100 );       // in 癈 * 100

            batteryLevel = BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
            GpsGetLatestGpsPositionBinary( &latitude, &longitude );
            altitudeGps = GpsGetLatestGpsAltitude( );                           // in m

            AppData[0] = AppLedStateOn;
            AppData[1] = temperature;                                           // Signed degrees celsius in half degree units. So,  +/-63 C
            AppData[2] = batteryLevel;                                          // Per LoRaWAN spec; 0=Charging; 1...254 = level, 255 = N/A
            AppData[3] = ( latitude >> 16 ) & 0xFF;
            AppData[4] = ( latitude >> 8 ) & 0xFF;
            AppData[5] = latitude & 0xFF;
            AppData[6] = ( longitude >> 16 ) & 0xFF;
            AppData[7] = ( longitude >> 8 ) & 0xFF;
            AppData[8] = longitude & 0xFF;
            AppData[9] = ( altitudeGps >> 8 ) & 0xFF;
            AppData[10] = altitudeGps & 0xFF;
#endif
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]  [0：帧可以发送，1：错误]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;  //LoRaMAC MCPS-请求结构  Mcps_t Type + union uMcpsParam
    LoRaMacTxInfo_t txInfo;//LoRaMAC tx信息  uint8_t MaxPossiblePayload + uint8_t CurrentPayloadSize;可以处理的应用有效载荷的大小+当前有效负载大小

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )//Service not started successfully
    {
        // Send empty frame in order to flush MAC commands  发送空帧以刷新MAC命令
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else//Service started successfully
    {
        if( IsTxConfirmed == false )//指示节点是否正在发送已确认或未确认的消息
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;//fport=2
            mcpsReq.Req.Confirmed.fBuffer = AppData;//发送的数据
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )//发送帧
    {
        return false;//false=0 可以发送
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event  在 TxNextPacket Timeout 事件上执行的函数
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
    GpioWrite( &Led1, 1 );
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
    GpioWrite( &Led2, 1 );
}

/*!
 * \brief Function executed on Led 4 Timeout event
 */
static void OnLed4TimerEvent( void )
{
    TimerStop( &Led4Timer );
    // Switch LED 4 OFF
    GpioWrite( &Led4, 1 );
}

/*!
 * \brief   MCPS-Confirm event function   MCPS-Confirm 事件功能
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,  mcpsConfirm - 指向确认结构的指针，包含确认属性。
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
        GpioWrite( &Led1, 0 );
        TimerStart( &Led1Timer );
    }
    NextTx = true;
}




/*!
 * \brief   MCPS-Indication event function  MCPS-Indication事件功能  接收

 *   Mcps_t McpsIndication;MCPS-指示类型
 *   LoRaMacEventInfoStatus_t Status;操作状态
 *   uint8_t Multicast;组播
 *   uint8_t Port;应用程序端口
 *   uint8_t RxDatarate;下行数据速率
 *   uint8_t FramePending;帧挂起状态
 *   uint8_t *Buffer;指向接收数据流的指针
 *   uint8_t BufferSize;接收数据流的大小
 *   bool RxData;表示数据是否可用
 *   int16_t Rssi;收到的数据包的Rssi
 *   uint8_t Snr;收到的数据包的Snr
 *   uint8_t RxSlot;接收窗口 [0：Rx窗口1,  1：Rx窗口2]
 *   bool AckReceived;设置是否收到确认
 *   uint32_t DownLinkCounter;接收帧的下行计数器值
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,  指示结构的指针，包含指示属性。
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
	  int i;
  	uart_init(115200);//波特率115200
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
//					    printf("\r\n接收到:\r\n");
//              for(i=0;i<mcpsIndication->BufferSize;i++)
//              {
//							    printf("%x",mcpsIndication->Buffer[i]);//获得接收数据
//              }
//							printf("\r\n \r\n");
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
							  switch(mcpsIndication->Buffer[0])
								{
									case 0x01:
								  	GpioWrite(&Led_R,0);                  //打开LED_R
									  printf("\r\n接收到:\r\n");
    								printf("%x\r\n",mcpsIndication->Buffer[0]);
									  break;
									case 0x02:
										GpioWrite(&Led_G,0);                  //打开LED_R
									  printf("\r\n接收到:\r\n");
    								printf("%x\r\n",mcpsIndication->Buffer[0]);
									  break;
									case 0x03:
										GpioWrite(&Led_B,0);                  //打开LED_R
									  printf("\r\n接收到:\r\n");
    								printf("%x\r\n",mcpsIndication->Buffer[0]);
									  break;
									case 0x04:
										GpioWrite(&Led_R,1);                  //
									  GpioWrite(&Led_G,1); 
									  GpioWrite(&Led_B,1); 
									  printf("\r\n接收到:\r\n");
    								printf("%x\r\n",mcpsIndication->Buffer[0]);
									  break;
									default:
										break;
								}
            }
						else{
						   printf("\r\n接收到:\r\n");
               for(i=0;i<mcpsIndication->BufferSize;i++)
               {
							    printf("%x",mcpsIndication->Buffer[i]);//获得接收数据
               }
							 printf("\r\n \r\n");
						}

//							  ReceLen=mcpsIndication->BufferSize;//获取接收数据长度
//							  for(i=0;i<ReceLen;i++)
//							  {
//								    ReceData[i]=mcpsIndication->Buffer[i];//获得接收数据
//								}
//								printf("\r\n接收到:\r\n");
//								printf("%x\r\n",ReceData);
							  //HAL_UART_Transmit(&huart1,ReceData,ReceLen,1000);	//发送接收到的数据

            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                    GpsStop( );
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    GpsStart( );
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < MIN( AppDataSize, LORAWAN_APP_DATA_MAX_SIZE ); i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                        GpsStart( );

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
    GpioWrite( &Led2, 0 );
    TimerStart( &Led2Timer );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/**
 * Main application entry point.
 */
int main( void )
{
	  //SystemInitialize();//波特率115200
    LoRaMacPrimitives_t LoRaMacPrimitives;//LoRaMAC事件结构  用于通知MAC事件的上层
    LoRaMacCallback_t LoRaMacCallbacks;// LoRaMAC回调结构  测量电池电量
    MibRequestConfirm_t mibReq;//Mib_t Type + MibParam_t Param

    BoardInitMcu( );//HAL库初始化  系统时钟配置 RTC GPIO  spi初始化(与1278连接)   1278IO口初始化
    BoardInitPeriph( );//GPIO口配置

    DeviceState = DEVICE_STATE_INIT;//设备状态 初始化

    while( 1 )
    {
			  //printf("start cycle! \r\n");
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;//用于保存MCPS的结构确认数据  LoRaMac.c
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;//用于保存MCPS指示数据的结构 接收数据保存
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;// 保持MLME确认数据的结构
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;//获取当前的电池电量 0：USB， 1：最低等级， x：等级 254：充满电， 255：错误
#if defined( REGION_AS923 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923 );
#elif defined( REGION_AU915 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915 );
#elif defined( REGION_CN470 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );
#elif defined( REGION_CN779 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779 );
#elif defined( REGION_EU433 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433 );//初始化LoRaMAC 设置终端类型等参数 ， MCPS和MLME服务的回调函数
#elif defined( REGION_EU868 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );
#elif defined( REGION_IN865 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865 );
#elif defined( REGION_KR920 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920 );
#elif defined( REGION_US915 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915 );
#elif defined( REGION_US915_HYBRID )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915_HYBRID );
#else
    #error "Please define a region in the compiler options."
#endif
                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                TimerInit( &Led1Timer, OnLed1TimerEvent );
                TimerSetValue( &Led1Timer, 25 );

                TimerInit( &Led2Timer, OnLed2TimerEvent );
                TimerSetValue( &Led2Timer, 25 );

                TimerInit( &Led4Timer, OnLed4TimerEvent );
                TimerSetValue( &Led4Timer, 25 );

                mibReq.Type = MIB_ADR;//ADR 自适应速率调节
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;//设置public 或是private网络
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;//public
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
                LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
                LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
                LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
                LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
                LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
                LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
                LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );

                mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
                mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_RX2_CHANNEL;
                mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
                LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif
                DeviceState = DEVICE_STATE_JOIN;//状态变为入网
                break;
            }
            case DEVICE_STATE_JOIN:
            {
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
                BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 3;

                if( NextTx == true )
                {
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                DeviceState = DEVICE_STATE_SLEEP;
#else
                // Choose a random device address if not already defined in Commissioning.h
                if( DevAddr == 0 )
                {
                    // Random seed initialization
                    srand1( BoardGetRandomSeed( ) );

                    // Choose a random device address
                    DevAddr = randr( 0, 0x01FFFFFF );
                }

                mibReq.Type = MIB_NET_ID;//设置网络标识符
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;//0
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;//设置 DevAddr
                mibReq.Param.DevAddr = DevAddr;//根据芯片ID随机 或者 设置LORAWAN_DEVICE_ADDRESS
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;//设置NwkSKey
                mibReq.Param.NwkSKey = NwkSKey;//LORAWAN_NWKSKEY
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;//设置AppSKey
                mibReq.Param.AppSKey = AppSKey;//LORAWAN_APPSKEY
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;//设置 LoRaWAN网络加入了属性
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;//状态变为发送
#endif
                break;
            }
            case DEVICE_STATE_SEND:
            {
                if( NextTx == true )//指示是否可以发送新数据包 默认true
                {
                    PrepareTxFrame( AppPort );//LoRaWAN应用程序端口2  准备发送的数据

                    NextTx = SendFrame( );
                }
                if( ComplianceTest.Running == true )//LoRaWAN一致性测试支持数据
                {
                    // Schedule next packet transmission  安排下一个数据包传输 占空比？ 间隔5s
                    TxDutyCycleTime = 10000; // 5000 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                }
                DeviceState = DEVICE_STATE_CYCLE;//进入循环状态
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;//进入睡眠状态

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );//设置计时器新的超时值
                TimerStart( &TxNextPacketTimer );//启动并将计时器对象添加到计时器事件列表中
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                TimerLowPowerHandler( );//进入ARM-cortex深度睡眠模式
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
        if( GpsGetPpsDetectedState( ) == true )
        {
            // Switch LED 4 ON
            GpioWrite( &Led4, 0 );
            TimerStart( &Led4Timer );
        }
    }
}
