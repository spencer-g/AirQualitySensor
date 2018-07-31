#include "pms5003.h"

void pms5003::setPins(SoftwareSerial* ms){
	this->mySerial = ms;
}

int8_t pms5003::sleep(){
	byte cmd[] = {0xe4, 0x00, 0x00};
	int sent = this->mySerial->write(cmd, 3);
	if(sent != 3){
		return PMSLIB_ERROR_CHECKSUM;
	}
	return PMSLIB_OK;
}

int8_t pms5003::wakeup(){
	byte cmd[] = {0xe4, 0x00, 0x01};
	int sent = this->mySerial->write(cmd, 3);
	if(sent != 3){
		return PMSLIB_ERROR_CHECKSUM;
	}
	return PMSLIB_OK;
}

int8_t pms5003::activeRead(){
	byte cmd[] = {0xe1, 0x00, 0x01};
	int sent = this->mySerial->write(cmd, 3);
	if(sent != 3){
		return PMSLIB_ERROR_CHECKSUM;
	}
	return PMSLIB_OK;
}

int8_t pms5003::passiveRead(){
	byte cmd[] = {0xe1, 0x00, 0x00};
	int sent = this->mySerial->write(cmd, 3);
	if(sent != 3){
		return PMSLIB_ERROR_CHECKSUM;
	}
	return PMSLIB_OK;
}

int8_t pms5003::read(){
	unsigned long timer;
	int check;
	if(! this->mySerial->isListening()){
		this->mySerial->listen();
	}
	timer = 1000000;
	while(this->mySerial->available() > 0){
		this->mySerial->read();
		if(--timer <= 0){
			return PMSLIB_ERROR_TIMEOUT;
		}
	}
	byte startChar = 0;
	timer = 1000000;
	while(startChar != 0x42){
		if(--timer <= 0){
			return PMSLIB_ERROR_TIMEOUT;
		}
		if(this->mySerial->available() > 1){
			startChar = this->mySerial->read();
		}
		if(startChar == 0x42){
			if(this->mySerial->peek() != 0x4d){
				startChar = 0;
			}
			else{
				check = startChar + this->mySerial->read();
			}
		}
	}
	
	timer = 1000000;
	while(this->mySerial->available() < 30){
		if(--timer <= 0){
			return PMSLIB_ERROR_TIMEOUT;
		}
	}
	
	int data[15];
	byte hi, lo;
	for(int8_t i = 0; i < 15; i++){
		hi = this->mySerial->read();
		lo = this->mySerial->read();
		data[i] = intConcat(hi, lo);
		if(i < 14){
			check += hi + lo;
		}
	}
	
	pm_1_0 = data[1];
	pm_2_5 = data[2];
	pm_10 = data[3];
	pm_1_0_ae = data[4];
	pm_2_5_ae = data[5];
	pm_10_ae = data[6];
	bin_0_3 = data[7];
	bin_0_5 = data[8];
	bin_1_0 = data[9];
	bin_2_5 = data[10];
	bin_5_0 = data[11];
	bin_10 = data[12];
	
	if(data[14] != check){
		return PMSLIB_ERROR_CHECKSUM;
	}
	return PMSLIB_OK;
}

int pms5003::intConcat(byte hi, byte lo){
	int result = ((int)hi) << 8;
	result |= lo;
	return result;
}