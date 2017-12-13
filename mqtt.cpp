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
 - sudo apt-get install libmosquitto-dev
 - sudo apt-get install libmosquittopp-dev
___________________________________________________________________________________

 start date : 04.03.2017

 mqtt application wrapper
 
*/

#include "mqtt.hpp"

#include "mesh.hpp"

//for printf
#include <stdio.h>
//for stdout
#include <iostream>

#include <string>
#include <cstring>

//for config
#include "utils.hpp"
//for Log::cout
#include "log.hpp"

#include "json.hpp"
using json = nlohmann::json;

//one per app 
mqtt_c::mqtt_c(json &conf,Serial &l_rfcom) : mosquittopp("streamer"),rfcom(l_rfcom)
{
    isReady = false;
    rgb.sendCount = 0;
	//logfile : log into a file------------------------------------------------------
    if(( conf.find("disable") != conf.end() ) && !conf["disable"] )
    {
        if( conf.find("host") != conf.end() )
        {
            if( conf.find("port") != conf.end() )
            {
                mosqpp::lib_init();
                std::string id = conf["id"];
                reinitialise(id.c_str(), true);
                isReady = true;
                int keepalive = 60;
                int port = conf["port"];
                std::string host = conf["host"];
                int res = connect(host.c_str(), port, keepalive);
                if(res == MOSQ_ERR_SUCCESS)
                {
                    Log::cout << "mqtt"<<"\t"<<"connecting to " << conf["host"] << " : " << port << Log::Info();
                }
                else
                {
                    Log::cout << "mqtt"<<"\t"<<"X Failed to connect" << Log::Error();
                }
            }
        }
        if(!isReady)
        {
            Log::cout << "mqtt"<<"\t"<<"X Not Configured, will not be used" << Log::Info();
        }
    }
    else
    {
        Log::cout << "mqtt"<<"\t"<<"X disabled, will not be used" << Log::Info();
    }

};

void mqtt_send_HEAT_Value(Serial &l_str,int TargetNodeId,int val)
{
	char text[31];
	int nbWrite = sprintf(text,"heat 0x%02x 0x%02x\r",TargetNodeId,val);
	l_str.send(text,nbWrite);
	std::string s(text);
	Log::cout << "ser\t" << s << Log::Debug();
}


void mqtt_c::run()
{
    if(isReady)
    {
        int status = loop(0);//immediate return, 
        if(status == MOSQ_ERR_CONN_LOST)
        {
            Log::cout << "mqtt"<<"\t"<<"error connection lost, reconnecting" << Log::Error();
            reconnect();
        }
        else if(status != MOSQ_ERR_SUCCESS)
        {
            Log::cout << "mqtt"<<"\t"<<"unhandled error status("<<status << ")" << Log::Error();
            //issues on auto boot, reconnect in this case as well
            reconnect();
        }
    }
}

void mqtt_c::on_connect(int rc)
{
    Log::cout << "mqtt"<<"\t"<<"connected id(" << rc << ")" << Log::Info();

    subscribe(NULL,"Nodes/+/RGB");
    subscribe(NULL,"Nodes/+/Heat");
}

void mqtt_c::on_message(const struct mosquitto_message *message)
{
    std::string msg(static_cast<const char*>(message->payload) );
    std::string topic(message->topic);
    if(topic.find("Nodes/") == 0)
    {
        std::string Text = topic;
        utl::TakeParseTo(Text,'/');//remove first section "Nodes/"
        std::string Id = utl::TakeParseTo(Text,'/');//take the second element
        if(Text.find("RGB")==0)
        {
            if(msg.find("#")==0)
            {
                //the topic is "Node/6/RGB"
                rgb.NodeId = std::stoi(Id);
                utl::TakeParseTo(msg,'#');
                unsigned int hxVal = std::stoul(msg, nullptr, 16);
                rgb.R = ((hxVal >> 16) & 0xFF);
                rgb.G = ((hxVal >> 8) & 0xFF); 
                rgb.B = ((hxVal) & 0xFF);
                Log::cout << "mqtt"<<"\t"<<"=> NodeId:"<< rgb.NodeId << " RGB: ("<< rgb.R <<","<< rgb.G <<","<< rgb.B <<")"<< Log::Debug();
                mesh::msg::color_txt(rfcom,rgb.NodeId,rgb.R,rgb.G,rgb.B);
            }
        }
        else if(Text.find("Heat")==0)
        {
            int NodeId = std::stoi(Id);
            int heat_val = std::stoi(msg);
            if((heat_val >= 0) && (heat_val<=10))
            {
                Log::cout << "mqtt"<<"\t"<<"=> NodeId:"<< NodeId << " Heat: ("<< heat_val <<")"<< Log::Debug();
                mqtt_send_HEAT_Value(rfcom,NodeId,heat_val);
            }
        }
    }
    else
    {
        Log::cout << "mqtt"<<"\t"<<"unexpected Topic : "<< topic << Log::Debug();
        Log::cout << "mqtt"<<"\t"<<"=> "<< msg<< Log::Debug();
    }
}

void decode_json_color(std::string topic,std::string message,Serial &rfcom)
{
    bool found = false;
    json jMsg = json::parse(message);
    if(jMsg.find("Red") != jMsg.end())
    if(jMsg.find("Green") != jMsg.end())
    if(jMsg.find("Blue") != jMsg.end())
    {
        found = true;
    }
    if(found)
    {
        if(topic.find("Nodes/") == 0)
        {
            std::string Text = topic;
            //the topic is "Node/6/RGB"
            utl::TakeParseTo(Text,'/');//remove first section 
            std::string Id = utl::TakeParseTo(Text,'/');//take the second element
            int NodeId = std::stoi(Id);
            int R = std::stoi(jMsg["Red"].dump());
            int G = std::stoi(jMsg["Green"].dump());
            int B = std::stoi(jMsg["Blue"].dump());
            Log::cout << "mqtt"<<"\t"<<"=> NodeId:"<< NodeId << " RGB: ("<< R <<","<< G <<","<< B <<")"<< Log::Debug();
            mesh::msg::color_txt(rfcom,NodeId,R,G,B);
        }
        else
        {
            Log::cout << "mqtt"<<"\t"<<"unexpected Topic : "<< topic << Log::Debug();
        }
    }
    else
    {
        Log::cout << "mqtt"<<"\t"<<"=> "<< message<< Log::Debug();
    }
}

void mqtt_c::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
    Log::cout << "mqtt"<<"\t"<<"subscribed to message id(" << mid << ")" << Log::Info();
}

void mqtt_c::publish_measures(NodeMap_t &NodesSensorsVals)
{
	if(!isReady)
	{
		return;
	}
	for(auto const& sensorsTables : NodesSensorsVals) 
	{
		int NodeId = sensorsTables.first;
		std::string Node = std::to_string(NodeId);
		for(auto const& Table : sensorsTables.second) 
		{
			std::string SensorName = Table.first;
			for(auto const& Measure : Table.second) 
			{
                std::string topic = "Nodes/" + Node + "/" + SensorName;
                std::string Value = std::to_string(Measure.value);
                int status = publish(NULL,topic.c_str(),Value.size(),Value.c_str());
                if(status == MOSQ_ERR_SUCCESS)
                {
                    Log::cout << "mqtt" << "\t" << "publish OK" << Log::Debug();
                }
                else
                {
                    Log::cout << "mqtt" << "\t" << "publish Fail" << Log::Error();
                }
			}
		}
	}
}