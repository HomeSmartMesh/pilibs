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
 - bme_280 with datasheet's associated rights see bme280_server.hpp
___________________________________________________________________________________

 start date : 16.07.2016

 serial port cpp wrapper
 * update() to get data from the serial port buffer
 * collect data into lines and parses each completed line
 * calls the bme_280 as main sensors are calibrated from within that library
 * transforms the registers values into ready to use sensors map Nodes.Sensors.Values,Timestamp
 
*/


//for ios::out,... #issue once placed in the end of the includes, it does not recognise cout part of std::
#include <iostream>
#include <fstream>
#include <string>

#include "serial.hpp"
//for getTime
#include "utils.hpp"

#include "bme280_server.hpp"

#include <stdio.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "log.hpp"

#ifdef CUSTOM_SERIAL_PORT_SPEED_UNDER_INVESTIGATION
//for serial_struct
#include <serial.h>
//for warnx()
#include <err.h>



using namespace std;

void set_custom_speed(int fd,int rate)
{
	struct serial_struct serinfo;
	/* Custom divisor */
	serinfo.reserved_char[0] = 0;
	if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
		return -1;
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = (serinfo.baud_base + (rate / 2)) / rate;
	if (serinfo.custom_divisor < 1) 
		serinfo.custom_divisor = 1;
	if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
		return -1;
	if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
		return -1;
	if (serinfo.custom_divisor * rate != serinfo.baud_base) {
		warnx("actual baudrate is %d / %d = %f",
			  serinfo.baud_base, serinfo.custom_divisor,
			  (float)serinfo.baud_base / serinfo.custom_divisor);
	}	
}
#endif /*CUSTOM_SERIAL_PORT_SPEED_UNDER_INVESTIGATION*/

int set_interface_attribs (int fd, speed_t baudrate, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, baudrate);
        cfsetispeed (&tty, baudrate);
		
		cfmakeraw(&tty);						//very important to avoid swapping CR to LF

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 0;            // timeout : 100 ms per unit

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}


LogBuffer_c::LogBuffer_c()
{
	newLine = true;//must start with a timestamp on the first write;
	plinebuf = linebuf;//points on the beginning of the line buffer
}

Serial::Serial()
{
	isReady = false;
}
Serial::Serial(json &conf,json &calib)
{
	isReady = config(conf,calib);
}

bool Serial::config(json &conf,json &calib)
{
	bool success = false;
	//serial port config------------------------------------------------------------
    if(( conf.find("enable") != conf.end() ) && conf["enable"] )
    {
		std::string portName = conf["portname"];
		std::string port_baud = "115200";
		if( conf.find("portbaud") != conf.end() )
		{
			port_baud = conf["portbaud"];
		}
		if(start(portName,port_baud))
		{
			for(json::iterator node = calib.begin(); node != calib.end(); ++node)
			{
				Log::cout << "str\tloading calib node: " << node.key() << Log::Info();
				int l_Id = std::stoi((std::string)node.key());
				NodesMeasures[l_Id].load_calib_data(node.value());
			}
			success = true;
		}
	}
	else
	{
		Log::cout << "str\tX :Serial Port is not enabled, will not be used" << Log::Info();
	}
	
	return success;
}

bool Serial::start(std::string port_name,std::string baudrate)
{
	bool success = false;
	
	isLogFile = true;
	isLogOut = true;
	
	fd = open (port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (fd >= 0)
	{
		std::string strlog;
		strlog+= "str\tport "+port_name+" is open @";
		if( utl::compare(baudrate,"500000") )
		{
			set_interface_attribs (fd, B500000, 0);
			strlog+="B500000";
		}
		else if( utl::compare(baudrate,"115200") )
		{
			set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
			strlog+="B115200";
		}
		else if( utl::compare(baudrate,"9600") )
		{
			set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
			strlog+="B9600";
		}
		else
		{
			set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
			strlog+="B115200";
		}
		Log::cout << strlog << Log::Info();
		success = true;
	}
	else
	{
		
		Log::cout << "str\tError "+std::to_string(errno)+" opening "+port_name+" : "+strerror(errno) << Log::Error();
	}
	return success;
}

bool LogBuffer_c::update(int fd)
{
	bool res = false;
	n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
	if(n > 0)
	{
		res = true;
		//as we want to print it here, we make sure it is null terminated
		if(n < sizeof buf)
		{
			buf[n] = '\0';//null terminated string
		}
		else
		{
			buf[(sizeof buf)-1] = '\0';//must insert a null terminated string, otherwise not safe to print nor search,...
			Log::cout << "str\tSlow app Max Buffer reached, loss of data !!!"<< Log::Error();
		}
	}
	else
	{
		//Log::cout << "Nothing" << std::endl;
	}
	return res;
}

bool Serial::update()
{
	if(isReady)
	{
		return logbuf.update(fd);//update the content of the local buffer from the serial device
	}
	else
	{
		return false;
	}
}

void Serial::logBuffer()
{
	//Log::cout << "[nb lines]" << logbuf.currentlines.size() << std::endl;
	for(std::string cl : logbuf.currentlines)
	{
		//log(cl);
	}
}

void Serial::clearBuffer()
{
	logbuf.currentlines.clear();
}

void LogBuffer_c::lastLinesDiscardOld()
{
	const int nbSecondsToRetain = 2;
	std::time_t right_now;
	std::time(&right_now);
	std::time_t oldTime = right_now - nbSecondsToRetain;
	bool isDone = false;
	while(!lastLines.empty() && !isDone)
	{
		auto &vLine = lastLines.front();
		if(vLine.time<=oldTime)
		{
			//Log::cout << "lastline_Remove>" << vLine.line << std::endl;
			lastLines.pop_front();
		}
		else
		{
			isDone = true;
		}
	}
}

bool LogBuffer_c::lastLinesCheck(std::string &line)
{
	bool isFound = false;	
	//-------------------check duplicate
	if(!lastLines.empty())
	{
		//more likely to be in the back
		for(std::list<sensor_line_t>::reverse_iterator rit=lastLines.rbegin();
			(rit!=lastLines.rend() && !isFound);
			++rit)
		{
			if(utl::compare(line,(*rit).line))
			{
				isFound = true;
				//Log::cout << "lastline_Found>" << line << std::endl;
			}
		}
	}
	return isFound;
}

void LogBuffer_c::lastLinesAdd(std::string &line)
{
	lastLines.push_back({time_now,line});
	//Log::cout << "lastline_add>" << line << std::endl;
}

void handle_float(const std::string &sensorname,strmap &notif_map,NodeMap_t&nodes,const int node_id,const std::time_t &ts,float coeff=1.0)
{
	sensor_measure_t sensor_measure;
	sensor_measure.time = ts;

	std::string sensor_value_text = notif_map[sensorname];
	float sensor_value_int = std::stof(sensor_value_text);
	sensor_measure.value = sensor_value_int * coeff;
	nodes[node_id][sensorname].push_back(sensor_measure);
}

void Serial::processLine(NodeMap_t &nodes)
{
	//replace end of line by end of string
	(*logbuf.plinebuf) = '\0';
	std::string logline(logbuf.linebuf);
	utl::replace(logline,',',';');
	//reset the line buffer pointer to the beginning of the line
	logbuf.plinebuf = logbuf.linebuf;
	
	
	strmap notif_map;
	utl::str2map( logline, notif_map);
	if(utl::exists(notif_map,"NodeId"))
	{
		std::string t_Id = notif_map["NodeId"];
		int l_Id = std::stoi(t_Id);

		#ifdef OLD_RETRANSMISSION_CHECK
			if(utl::exists(notif_map,"RTX"))//If this is a retransmitted frame
			{
				utl::TakeParseTo(logline,';');//remove the first section "RTX:ttl;"
				
				logbuf.lastLinesDiscardOld();//remove from lastLines[] what is loder than 2 sec
				bool isDuplicate = logbuf.lastLinesCheck(logline);//if the line was available in lastLines[]
				//Here should be taken out the duplicaes,
				// and release it for normal processing otherwise
				if(isDuplicate)
				{
					Log::cout << "str\tDiscarded Duplicate: "<< logline << Log::Debug();
					return;
				}
			}
		#else
			logbuf.lastLinesDiscardOld();//remove from lastLines[] what is loder than 2 sec
			bool isDuplicate = logbuf.lastLinesCheck(logline);//if the line was available in lastLines[]
			// GO OUT Completely "return" so "logline" will not be further processed
			if(isDuplicate)
			{
				Log::cout << "str\tDiscarded Duplicate: "<< logline << Log::Debug();
				return;
			}
		#endif
		
		//if we reached it here, that means the log line is not duplicate with the previous 2 sec
		logbuf.lastLinesAdd(logline);
		bool is_partly_handled = false;
		if(utl::exists(notif_map,"bme280"))
		{
			if( (NodesMeasures.find(l_Id) != NodesMeasures.end()) &&
				(NodesMeasures[l_Id].isReady) )
			{
				NodesMeasures[l_Id].set_all_measures_Text(notif_map["bme280"]);
				
				sensor_measure_t temperature,humidity,pressure;
				temperature.time = logbuf.time_now;
				humidity.time = logbuf.time_now;
				pressure.time = logbuf.time_now;

				temperature.value = NodesMeasures[l_Id].get_float_temperature();
				humidity.value = NodesMeasures[l_Id].get_float_humidity();
				pressure.value = NodesMeasures[l_Id].get_float_pressure();
				
				nodes[l_Id]["temperature"].push_back(temperature);
				nodes[l_Id]["humidity"].push_back(humidity);
				nodes[l_Id]["pressure"].push_back(pressure);
				
				logbuf.currentlines.push_back(	logbuf.day + "\t" + logbuf.time + "\t" 
										+ "NodeId:" + std::to_string(l_Id)
										+ ";temperature:" + NodesMeasures[l_Id].get_temperature());
										
				logbuf.currentlines.push_back(	logbuf.day + "\t" + logbuf.time + "\t" 
										+ "NodeId:" + std::to_string(l_Id)
										+ ";humidity:" + NodesMeasures[l_Id].get_humidity());
				logbuf.currentlines.push_back(	logbuf.day + "\t" + logbuf.time + "\t" 
										+ "NodeId:" + std::to_string(l_Id)
										+ ";pressure:" + NodesMeasures[l_Id].get_pressure());
			}
			else
			{
				Log::cout << "str\tSensorId"<<l_Id<<" calib files not loaded" << Log::Error();
			}
			is_partly_handled = true;
		}
		else
		{
			//generic log for all the others
			logbuf.currentlines.push_back(	logbuf.day + "\t" + logbuf.time + "\t" + logline);
		}
		// ------------------------------ TODO ------------------------------ 
		//could handle all of these as configurable abilities
		if(utl::exists(notif_map,"light"))
		{
			handle_float("light",notif_map,nodes,l_Id,logbuf.time_now);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"red"))
		{
			handle_float("red",notif_map,nodes,l_Id,logbuf.time_now);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"green"))
		{
			handle_float("green",notif_map,nodes,l_Id,logbuf.time_now);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"blue"))
		{
			handle_float("blue",notif_map,nodes,l_Id,logbuf.time_now);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"proximity"))
		{
			handle_float("proximity",notif_map,nodes,l_Id,logbuf.time_now);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"temperature"))
		{
			handle_float("temperature",notif_map,nodes,l_Id,logbuf.time_now,0.01);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"humidity"))
		{
			handle_float("humidity",notif_map,nodes,l_Id,logbuf.time_now,0.01);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"pressure"))
		{
			handle_float("pressure",notif_map,nodes,l_Id,logbuf.time_now,0.01);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"heat"))
		{
			handle_float("heat",notif_map,nodes,l_Id,logbuf.time_now);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"button"))
		{
			handle_float("button",notif_map,nodes,l_Id,logbuf.time_now);
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"event"))//events
		{
			sensor_measure_t reset_evt;
			reset_evt.time = logbuf.time_now;

			if(utl::compare(notif_map["event"],"Reset"))
			{
				reset_evt.value = 1;
				nodes[l_Id]["Reset"].push_back(reset_evt);
			}
			is_partly_handled = true;
		}
		if(utl::exists(notif_map,"status"))//current states
		{
			sensor_measure_t state;
			state.time = logbuf.time_now;

			if(utl::compare(notif_map["status"],"Alive"))
			{
				state.value = 1;
				nodes[l_Id]["Alive"].push_back(state);
			}

			is_partly_handled = true;
		}
		if(!is_partly_handled)
		{
			Log::cout << "stm32\t"<<logline << Log::Info();
		}
	}
}

//we use Serial::buf for data and Serial::n for data size
//isReady is protected by the update that has to change the .n value
NodeMap_t Serial::processBuffer()
{
	NodeMap_t nodes;
	
	clearBuffer();//return only the last gathered data
	
	if(logbuf.n>0)
	{
		char * buf_w = logbuf.buf;
		char * buf_end = logbuf.buf + logbuf.n;
		
		while(buf_w != buf_end)
		{
			bool isp = isprint(*buf_w);

			//Timestamping : avoid empty lines do not create a new timestamp if the char is a line ending
			if(logbuf.newLine && isp)
			{
				time(&logbuf.time_now);//take a timestamp
				logbuf.day = utl::getDay(logbuf.time_now);
				logbuf.time = utl::getTime(logbuf.time_now);
				logbuf.newLine = false;
			}

			//Process characters
			if( ((*buf_w) == '\n') || ((*buf_w) == 10) || ((*buf_w) == 13))//only allowed printable character
			{
				logbuf.newLine = true;
				(*logbuf.plinebuf) = (*buf_w);
				processLine(nodes);
			}
			else if(isp)//skip the CR and any other control
			{
				(*logbuf.plinebuf++) = (*buf_w);
			}
			//else non printable characters other than '\n' are discarded
			buf_w++;
		}
	}
	if(!logbuf.currentlines.empty())
	{
		Log::cout << "str\tProcessed " << logbuf.currentlines.size() << " Line(s)" << Log::Debug();
		for(int i=0;i<logbuf.currentlines.size();i++)
		{
			Log::cout << "str\t"<< logbuf.currentlines[i] << Log::Verbose();
		}
	}
	return nodes;
}

void Serial::send(char* buffer,int size)
{
	if(isReady)
	{
		write(fd,buffer,size);
		//TODO add test for log level that avoids serialising the string
		std::string line(buffer);
		Log::cout << "str\t" << line << Log::Verbose();
	}
	else
	{
		Log::cout << "str\t not enabled" << Log::Verbose();
	}
}
