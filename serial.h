/*
 * serial.h
 *
 *  Created on: Feb 12, 2017
 *      Author: srazak
 */

#ifndef SERIAL_H_
#define SERIAL_H_

void SetupSerial();
void SerialWrite(char*);
void SerialWriteInt(int);
void SerialWriteLine(char*);
void SerialWriteHex(unsigned int n);
#endif /* SERIAL_H_ */
