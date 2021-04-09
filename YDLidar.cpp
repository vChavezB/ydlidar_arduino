/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR Arduino
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.eaibot.com
 * 
 */
/*
  INCLUDES
*/
#include "YDLidar.h"
#include <stdint.h>

/*
  LOCAL DEFINITIONS
*/
#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x8000

#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81

#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT    8


#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D
#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1

#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define PackageSampleBytes 2

#define Node_Default_Quality (10<<2)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
//Packet header constant
#define SIZE_PACKET_HEADER 2
#define PH_LSB 0xAA
#define PH 0x55AA

#define SCAN_METADATA_SIZE 8
#define SCAN_SAMPLE_SIZE 2
#define SCAN_SAMPLE_OFFSET SCAN_METADATA_SIZE+SIZE_PACKET_HEADER


uint8_t scanpackage_buffer[SCAN_SAMPLE_OFFSET+(MAX_SAMPLES*SCAN_SAMPLE_SIZE)];


typedef enum
{
	metadata_ok=0,
	metadata_packagetpe_error
}metadata_error_et;


typedef enum ydlidar_baudrates_e
{
	baudrate_tx8=115200,
	baudrate_x4=128000
}ydlidar_baudrates_et;

uint32_t ydlidar_baudrates[YDLidar::ydlidar_total_types]=
{
	ydlidar_baudrates_et::baudrate_tx8,
	ydlidar_baudrates_et::baudrate_x4
};
/*
  LOCAL FUNCTIONS
*/

//Unit mm
float angle_conversion(uint16_t raw_angle)
{
	return (raw_angle>>1)/64.0;
}
metadata_error_et parse_metadata(uint8_t * pData,scan_content_metadata_t * metadata)
{
	metadata_error_et result = metadata_ok;
	do
	{
		metadata->package_type	=	(CT)pData[0];
		metadata->sample_qty	=	pData[1];
		metadata->FirstSampleAngle 	=	pData[3]<<8 | pData[2];
		metadata->LastSampleAngle		=	pData[5]<<8 | pData[4];
		metadata->checksum		= 	pData[7]<<8 | pData[6];

		if(metadata->package_type >CT_valid_types)
		{
			result=metadata_packagetpe_error;
			break;
		}
	}while(0);
	return result;
}


//Unit °
float intermediate_angle(float angle_diff,uint16_t sample_qty,float start_angle,uint8_t sample)
{
	float angle=  ((angle_diff/(sample_qty-1))*(sample-1))+start_angle;
	if (angle>360.0f)
	{
		angle-=360.0f;
	}
	return angle;
}

/*
  CLASS IMPLEMENTATION
*/

YDLidar::YDLidar()
    : _bined_serialdev(NULL)
{
    point.distance = 0;
    point.angle = 0;
    point.quality = 0;
}


YDLidar::~YDLidar()
{
    end();
}

//Unit mm
uint16_t YDLidar::get_distance(uint16_t raw_distance)
{
	uint16_t distance=0xFFFF;
	if(_type==TX8)
	{
		uint8_t lsb=raw_distance & 0xFF;
		uint8_t msb=raw_distance >>8;
		distance= lsb + (msb*256);
	}
	if(_type==X4)
	{
		distance=raw_distance/4;
	}
	return distance;
}


// open the given serial interface and try to connect to the YDLIDAR
ydlidar_result_et YDLidar::begin(HardwareSerial &serialobj,ydlidar_types_et ydlidar_type)
{
	ydlidar_result_et result=RESULT_OK;
	do
	{
		//Not a valid ydlidar type
		if(ydlidar_type>=ydlidar_total_types)
		{
			result=ydlidar_wrong_type;
			break;
		}
		if(serialobj==NULL)
		{
			result=ydlidar_null_serial;
			break;
		}
		if (isOpen()) {
		  end(); 
		}
		_type=ydlidar_type;
		_bined_serialdev = &serialobj;
		//_bined_serialdev->end();
		//_bined_serialdev->begin(ydlidar_baudrates[(int)ydlidar_type]);

	}while(0);
	return result;
}

// close the currently opened serial interface
void YDLidar::end(void)
{
    if (isOpen())
	{
       _bined_serialdev->end();
       _bined_serialdev = NULL;
    }
}


// check whether the serial interface is opened
bool YDLidar::isOpen(void)
{
    return _bined_serialdev?true:false; 
}

// ask the YDLIDAR for its device health

ydlidar_result_et YDLidar::getHealth(device_health & health, uint32_t timeout){
	ydlidar_result_et  ans=RESULT_OK;
	do
	{
		uint8_t  recvPos = 0;
		uint32_t currentTs = millis();
		uint32_t remainingtime;
		uint8_t *infobuf = (uint8_t*)&health;
		lidar_ans_header response_header;
		//TX8 does not support this feature
		if (!isOpen()) {
			ans= ydlidar_serial_closed;
			break;
		}

		ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH,NULL,0);
		if (ans != RESULT_OK)
		{
			break;
		}

		ans = waitResponseHeader(&response_header, timeout);
		if (ans != RESULT_OK)
		{
			break;
		}

		if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH)
		{
			ans= ydlidar_wrong_header_type;
			break;
		}

		if (response_header.size < sizeof(device_health))
		{
			ans= ydlidar_wrong_header_size;
			break;
		}
		bool rsp_recieved=false;
		while (!rsp_recieved) 
		{
			if((remainingtime=millis() - currentTs) <= timeout)
			{
				rsp_recieved=true;
				ans=RESULT_TIMEOUT;
			}
			else
			{
				int currentbyte = _bined_serialdev->read();
				if (currentbyte<0)
				{
					continue;   
				}				
				infobuf[recvPos++] = currentbyte;
				if (recvPos == sizeof(device_health))
				{
					rsp_recieved=true;
				}
			}
		}
	}while(0);
	return ans;
}


// ask the YDLIDAR for its device info 
ydlidar_result_et YDLidar::getDeviceInfo(device_info & info, uint32_t timeout) {
    ydlidar_result_et  ans=RESULT_OK;
	do
	{
		uint8_t  recvPos = 0;
		uint32_t currentTs = millis();
		uint32_t remainingtime;
		uint8_t *infobuf = (uint8_t*)&info;
		lidar_ans_header response_header;

		if (!isOpen())
		{
			ans= ydlidar_serial_closed;
			break;
		}
		ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO,NULL,0);
		if (ans != RESULT_OK)
		{
			break;
		}
		ans = waitResponseHeader(&response_header, timeout);
		if ( ans != RESULT_OK)
		{
			break;
		}

		if (response_header.type != LIDAR_ANS_TYPE_DEVINFO)
		{
			ans = ydlidar_wrong_header_type;
			break;
		}

		if (response_header.size < sizeof(lidar_ans_header))
		{
			ans = ydlidar_wrong_header_size;
			break;
		}
		bool rsp_recieved=false;
		while (!rsp_recieved)
		{
			if ((remainingtime=millis() - currentTs) <= timeout)
			{
				ans=RESULT_TIMEOUT;
				rsp_recieved=true;
			}
			else
			{
				int currentbyte = _bined_serialdev->read();
				if (currentbyte<0)
				{
					continue;  
				}					
				infobuf[recvPos++] = currentbyte;

				if (recvPos == sizeof(device_info))
				{
					rsp_recieved=true;
				}			
			}
		}
	}while(0);
	return ans;
}

// stop the scanPoint operation
ydlidar_result_et YDLidar::stop(void)
{
	ydlidar_result_et result=RESULT_OK;
	do
	{
		if (!isOpen())
		{
			result=RESULT_FAIL;
			break; 
		}
		result = sendCommand(LIDAR_CMD_FORCE_STOP,NULL,0);
	}while(0);

    return result;
}

// start the scanPoint operation
ydlidar_result_et YDLidar::startScan(bool force, uint32_t timeout ) {
    ydlidar_result_et result=RESULT_OK;
	do
	{
		if (!isOpen())
		{
			result=ydlidar_serial_closed;
			break;
		}

		stop(); //force the previous operation to stop
		result = sendCommand(force?LIDAR_CMD_FORCE_SCAN:LIDAR_CMD_SCAN, NULL, 0);		   
		if (result != RESULT_OK)
		{
			break;
		}
		
		lidar_ans_header response_header;
		result = waitResponseHeader(&response_header, timeout);
		if (result != RESULT_OK)
		{
			break;
		}
		if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT)
		{
			result= ydlidar_wrong_header_type;
			break;
		}

		if (response_header.size < sizeof(node_info))
		{
			result= ydlidar_wrong_header_size;
			break;
		}		
	}while(0);
   
    return result;
}


scan_packet_error_et YDLidar::get_scan_packet(scan_content_metadata_t * metadata,unsigned long long timeout)
{
	scan_packet_error_et result=scanpacket_ok;
	do
	{
		uint16_t buffer_idx = 0;
		bool scan_packet_complete=false;
		uint16_t expected_bytes=0;
		unsigned long long startTs=millis();
		while (!scan_packet_complete)
		{
			if( (millis() - startTs) > timeout)
			{
				result=scanpacket_timeout;
				break;
			}
			int currentByte = _bined_serialdev->read();
        	if (currentByte<0)
			{
				continue;
			}
			//Store rx buffer until possible ph_lsb is recieved
			if (buffer_idx == 0 && currentByte!=PH_LSB)
			{
				continue;
			}
			scanpackage_buffer[buffer_idx++] = currentByte;
			if (buffer_idx == SIZE_PACKET_HEADER)
			{
				uint16_t recv_ph= scanpackage_buffer[1]<<8 | scanpackage_buffer[0];
				if(recv_ph!=PH)
				{
					buffer_idx=0;
					continue;
				}
			}
			if (buffer_idx == SIZE_PACKET_HEADER+SCAN_METADATA_SIZE)
			{
				metadata_error_et metadata_error=parse_metadata(&scanpackage_buffer[SIZE_PACKET_HEADER],
																metadata);
				
				if(metadata_error!=metadata_ok)
				{
					buffer_idx=0;
				}
				else
				{
					expected_bytes = SCAN_SAMPLE_OFFSET+(metadata->sample_qty * SCAN_SAMPLE_SIZE);
				}
			}
			else if(buffer_idx==expected_bytes)
			{
				scan_packet_complete=true;
			}
		}
		if(scan_packet_complete)
		{
			//Seed value checksum is PH constant
			uint16_t checksum=PH; 
			checksum ^= metadata->FirstSampleAngle;
			for(uint8_t sample_no=0;sample_no < metadata->sample_qty; sample_no++)
			{
				uint16_t index=SCAN_SAMPLE_OFFSET + (SCAN_SAMPLE_SIZE*sample_no);
				uint16_t sample_i= (scanpackage_buffer[index+1]<<8) | scanpackage_buffer[index];
				checksum ^= sample_i;
			}
			checksum ^= (((uint8_t) metadata->sample_qty  <<8) |metadata->package_type);
			checksum ^= metadata->LastSampleAngle;
			if(checksum != metadata->checksum)
			{
				result=scanpacket_checksum_mismatch;
				break;
			}
		}
	}while(0);
	return result;
}

// wait scan data
ydlidar_result_et YDLidar::waitScanDot( uint32_t timeout) 
{
	ydlidar_result_et result=RESULT_OK;
	scan_content_metadata_t metadata;
	do
	{
		scan_packet_error_et error=get_scan_packet(&metadata,timeout);
		if(error!=scanpacket_ok)
		{
			result=ydlidar_packet_error;
			break;
		}
		scan_data.samples=metadata.sample_qty;
		uint8_t * sample_data=&scanpackage_buffer[SCAN_SAMPLE_OFFSET];

		float start_angle=angle_conversion(metadata.FirstSampleAngle);
		float end_angle=angle_conversion(metadata.LastSampleAngle);
		float angle_diff;
		if(end_angle<start_angle)
		{
			angle_diff=(360.0f - start_angle)+end_angle;
		}
		else
		{
			angle_diff=end_angle-start_angle;
		}
			
		/*
		Serial.print("Samples: ");
		Serial.print(metadata.sample_qty);
		Serial.print(" FSA:");
		Serial.print(start_angle);
		Serial.print(" LSA:");
		Serial.print(end_angle);
		Serial.print(" Angle Diff:");
		Serial.println(angle_diff);
		*/
		scan_data.start_angle=start_angle;
		scan_data.end_angle=end_angle;
		for(uint8_t sample=0;sample < metadata.sample_qty; sample++)
		{
			uint16_t raw_distance=sample_data[0] | sample_data[1]<<8;
			if(sample > 0 && sample < (metadata.sample_qty-1))
			{
				scan_data.points[sample].angle=intermediate_angle(angle_diff,
															metadata.sample_qty,
															start_angle,sample+1);
			}
			else if(sample==0)
			{
				scan_data.points[sample].angle=start_angle;
			}
			else
			{
				scan_data.points[sample].angle=end_angle;
			}
			scan_data.points[sample].distance=get_distance(raw_distance);	
			sample_data+=SCAN_SAMPLE_SIZE;
		}
		/*
		for(uint8_t sample=0;sample < metadata.sample_qty; sample++)
		{
			Serial.print(scan_data.points[sample].angle);
			Serial.print("\t");
		}
		Serial.println();
		for(uint8_t sample=0;sample < metadata.sample_qty; sample++)
		{
			Serial.print(scan_data.points[sample].distance);
			Serial.print("\t");
		}
		Serial.println();
		*/
		
	}while(0);
	return result;
}


//send data to serial
ydlidar_result_et YDLidar::sendCommand(uint8_t cmd, const void * payload, size_t payloadsize) {
	ydlidar_result_et result=RESULT_OK;
	do
	{
		cmd_packet pkt_header;
		cmd_packet * header = &pkt_header;
		uint8_t checksum = 0;
		//TX8 does not support this feature
		if (_type==TX8)
		{
			result=ydlidar_wrong_type;
			break;
		}
		if (payloadsize && payload)
		{	
			cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
		}

		header->syncByte = LIDAR_CMD_SYNC_BYTE;
		header->cmd_flag = cmd&0xff;

		_bined_serialdev->write((uint8_t *)header, 2) ;
		if((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD))
		{
			checksum ^= LIDAR_CMD_SYNC_BYTE;
			checksum ^= (cmd&0xff);
			checksum ^= (payloadsize & 0xFF);
			for (size_t pos = 0; pos < payloadsize; ++pos)
			{
				checksum ^= ((uint8_t *)payload)[pos];
			}

			uint8_t sizebyte = payloadsize;
			_bined_serialdev->write(&sizebyte, 1);
			_bined_serialdev->write((const uint8_t *)payload, sizebyte);
			_bined_serialdev->write(&checksum, 1);
		}
	}while(0);
	
	return result;
}


// wait response header
ydlidar_result_et YDLidar::waitResponseHeader(lidar_ans_header * header, uint32_t timeout) {
	int  recvPos = 0;
	uint32_t startTs = millis();
	uint8_t  *headerBuffer = (uint8_t *)(header);
	uint32_t waitTime;

	while ((waitTime=millis() - startTs) <= timeout)
	{
		int currentbyte = _bined_serialdev->read();
       	if (currentbyte<0)
		{
			continue;
		}
		switch (recvPos)
		{
			case 0:
				if (currentbyte != LIDAR_ANS_SYNC_BYTE1)
				{
					continue;
				}
				break;
			case 1:
				if (currentbyte != LIDAR_ANS_SYNC_BYTE2)
				{
					recvPos = 0;
					continue;
				}
				break;
		}
		headerBuffer[recvPos++] = currentbyte;

		if (recvPos == sizeof(lidar_ans_header))
		{
			return RESULT_OK;
		}
	}
    return RESULT_TIMEOUT;
}

