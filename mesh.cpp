/*
The MIT License (MIT)

Copyright (c) 2017 Wassim Filali

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
___________________________________________________________________________________
 dependencies :
 - 
___________________________________________________________________________________

creation date : 01.11.2017

mesh protocol construction


*/

#include "mesh.hpp"

#include <iostream>

static uint8_t Source_Node_Id = 255;

void mesh::set_source_nodId(uint8_t Id)
{
    Source_Node_Id = Id;
}


// msg_type , size   , data , crc
// 1 BYTE   , 1 BYTE , ...
// message type :
// 'b'  : binary             => size defined
// 'c'  : binary with crc    => size defined
// other: text or unknown without size, timeout limit
void mesh::raw::send(Serial &l_str,uint8_t *buffer)
{
	uint8_t size = buffer[0];
	crc::set(buffer);//already have size @[0] used by crc
	char format = 'b';
	l_str.send(&format,1);
	l_str.send(&buffer,size+2);// +2 for 16 bit crc
}

// msg_type , ...
// 1 BYTE
// message type :
// other such as 't' : text or unknown without size, timeout limit
void mesh::raw::send_txt(Serial &l_str,std::string &message)
{
	char text[128];//uC buffer is also 128
    int nbWrite = sprintf(text,"tmsg %s\r",message.c_str());
	l_str.send(text,nbWrite);
	std::string s(text);
	Log::cout << "rgb> " << s << Log::Info();
}

void mesh::msg::color_txt(Serial &l_str,uint8_t TargetNodeId,uint8_t R,uint8_t G,uint8_t B)
{
	char text[128];//uC buffer is also 128
    int nbWrite = sprintf(text,"trgb 0x%02x 0x%02x 0x%02x 0x%02x\r",TargetNodeId,R,G,B);
	l_str.send(text,nbWrite);
	std::string s(text);
	Log::cout << "rgb> " << s << Log::Info();
}

void mesh::msg::dimmer::all(Serial &l_str,uint8_t TargetNodeId,uint16_t light)
{
	uint8_t buffer[36];
				//p2p:0 , msg:1 , msg:1 , ack:1 , ttl:2
	buffer[1] = 		(1<<6)  | (1<<5)| (1<<4)| 2;
	buffer[2] = 0x07;//light both sensor and order - TODO add dimmer id
	buffer[3] = Source_Node_Id;
	buffer[4] = TargetNodeId;
	buffer[5] = 0xFF & (light>>8);
	buffer[6] = 0xFF & light;
	buffer[0] = 7;
	
	mesh::raw::send(buffer);
	Log::cout << "dimmer> NodeId:" << TargetNodeId << " ; value:" <<light<< Log::Info();
}