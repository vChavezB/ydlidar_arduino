/*
 * YDLIDAR Driver for Arduino
 * eaibot.com
 * 
 * Copyright (c) 2018, EAI 
 * All rights reserved.
 */

#pragma once

#include "Arduino.h"

#define PackageSampleMaxLngth 0x80
#define DEFAULT_TIMEOUT 500
//Max samples limited to 1 byte according to LSN size
#define MAX_SAMPLES 255

struct node_info {
	uint8_t    sync_quality;
	uint16_t   angle_q6_checkbit;
	uint16_t   distance_q2;
} __attribute__((packed)) ;

struct node_package {
	uint16_t  package_Head;
	uint8_t   package_CT;
	uint8_t   nowPackageNum;
	uint16_t  packageFirstSampleAngle;
	uint16_t  packageLastSampleAngle;
	uint16_t  checkSum;
	uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed)) ;


struct device_info{
	uint8_t   model;
	uint16_t  firmware_version;
	uint8_t   hardware_version;
	uint8_t   serialnum[16];
} __attribute__((packed)) ;

struct device_health {
	uint8_t   status;
	uint16_t  error_code;
} __attribute__((packed))  ;

struct sampling_rate {
	uint8_t rate;
} __attribute__((packed))  ;

struct scan_frequency {
	uint32_t frequency;
} __attribute__((packed))  ;

struct scan_rotation {
	uint8_t rotation;
} __attribute__((packed))  ;

struct cmd_packet {
	uint8_t syncByte;
	uint8_t cmd_flag;
	uint8_t size;
	uint8_t data;
} __attribute__((packed)) ;

struct lidar_ans_header {
	uint8_t  syncByte1;
	uint8_t  syncByte2;
	uint32_t size:30;
	uint32_t subType:2;
	uint8_t  type;
} __attribute__((packed));

struct scanPoint {
	uint8_t quality;
	float 	angle;
	float 	distance;
	bool    startBit;
};

typedef struct  {
	float angle;
	uint16_t distance;
} scan_point_t;

typedef struct  {
	scan_point_t points[MAX_SAMPLES];
	uint8_t samples;
} scan_data_t;

typedef enum 
{
	CT_Normal = 0,
	CT_RingStart  = 1,
	CT_valid_types
}CT;
	

typedef struct  {
	CT   package_type;
	uint8_t   sample_qty;
	uint16_t  FirstSampleAngle;
	uint16_t  LastSampleAngle;
	uint16_t  checksum;
} scan_content_metadata_t;


typedef enum
{
	scanpacket_ok=0,
	scanpacket_timeout,
	scanpacket_checksum_mismatch
}scan_packet_error_et;



typedef enum ydlidar_result_e
{
	RESULT_OK=0,
	RESULT_TIMEOUT,
	RESULT_FAIL,
	ydlidar_null_serial,
	ydlidar_wrong_type,
	ydlidar_serial_closed,
	ydlidar_wrong_header_type,
	ydlidar_wrong_header_size,
	ydlidar_packet_error
}ydlidar_result_et;

//YDLidar class
class YDLidar
{
public:
	typedef enum ydlidar_types_e
	{
		TX8=0,
		X4=1,
		ydlidar_total_types
	}ydlidar_types_et;
	
    //construct
    YDLidar();  
    //destructor
    ~YDLidar();

    // open the given serial interface and try to connect to the YDLIDAR
    ydlidar_result_et begin(HardwareSerial &serialobj, ydlidar_types_et ydlidar_type=X4);

    // close the currently opened serial interface
    void end(void);
  
    // check whether the serial interface is opened
    bool isOpen(void); 

    // ask the YDLIDAR for its health info
    ydlidar_result_et getHealth(device_health & health, uint32_t timeout = DEFAULT_TIMEOUT);
    
    // ask the YDLIDAR for its device info like the serial number
    ydlidar_result_et getDeviceInfo(device_info & info, uint32_t timeout = DEFAULT_TIMEOUT);

    // stop the scanPoint operation
    ydlidar_result_et stop(void);

    // start the scanPoint operation
    ydlidar_result_et startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT*2);

    // wait for one sample package to arrive
    ydlidar_result_et waitScanDot(uint32_t timeout = DEFAULT_TIMEOUT);
    
    // retrieve currently received sample point
    const scanPoint & getCurrentScanPoint(void)
    {
        return point;
    }
	scan_data_t scan_data;
protected:
	
	scan_packet_error_et get_scan_packet(scan_content_metadata_t * metadata,unsigned long long timeout);
	ydlidar_types_et _type;
    // send ask commond to YDLIDAR
    ydlidar_result_et sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);
    //wait for response header to arrive
    ydlidar_result_et waitResponseHeader(lidar_ans_header * header, uint32_t timeout = DEFAULT_TIMEOUT);
    HardwareSerial * _bined_serialdev;  
    scanPoint point;
};
