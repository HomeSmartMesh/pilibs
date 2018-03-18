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
 renamed mqtt_rf on 14.12.2017
 mqtt application wrapper
 
*/

#include "mqtt_rf.hpp"

//for gethostname
#include <unistd.h>

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
mqtt_rf_c::mqtt_rf_c(json &v_conf,Serial &l_rfcom) : mosquittopp("rf_gateway"),rfcom(l_rfcom)
{
    conf = v_conf;
    isConnected = false;
    shouldBeConnected = false;
    shouldPublish = false;
    rgb.sendCount = 0;
	//logfile : log into a file------------------------------------------------------
    if(( conf.find("enable_connect") != conf.end() ) && conf["enable_connect"] )
    {
        if( conf.find("host") != conf.end() )
        {
            if( conf.find("port") != conf.end() )
            {
                mosqpp::lib_init();
                if(conf.find("client_id") != conf.end())
                {
                    char hostname[15];
                    gethostname(hostname,15);
                    std::string this_host(hostname);
                    std::string id = conf["client_id"];
                    id = id + "_" +this_host;
                    reinitialise(id.c_str(), true);
                    Log::cout << "mqtt"<<"\t"<<"client id: " << id << Log::Info();
                }
                int keepalive = 60;
                int port = conf["port"];
                std::string host = conf["host"];
                shouldBeConnected = true;//independent of failure or success
                int res = connect(host.c_str(), port, keepalive);
                if(res == MOSQ_ERR_SUCCESS)
                {
                    isConnected = true;
                }
                else
                {
                    Log::cout << "mqtt"<<"\t"<<"X Failed to connect" << Log::Error();
                }
            }
        }
        if(isConnected)
        {
            Log::cout << "mqtt"<<"\t"<<"connected to " << conf["host"] << " : " << conf["port"] << Log::Info();
        }
        else
        {
            Log::cout << "mqtt"<<"\t"<<"X Connection failed, will not be used" << Log::Error();
        }
        if(( conf.find("enable_publish") != conf.end() ) && conf["enable_publish"] )
        {
            shouldPublish = true;
            Log::cout << "mqtt"<<"\t"<<"publish enabled - as long as connected"<< Log::Info();
        }
        else
        {
            Log::cout << "mqtt"<<"\t"<<"publish not enabled"<< Log::Info();
        }
    }
    else
    {
        Log::cout << "mqtt"<<"\t"<<"X Connection not enabled" << Log::Info();
    }

};

void mqtt_rf_c::handle_dimmer(int TargetNodeId,json &jMsg)
{
    bool found = false;
    if(jMsg.find("All") != jMsg.end())
    {
        uint16_t light = jMsg["All"];
        mesh::msg::dimmer::all(rfcom,TargetNodeId,light);
    }
    else if(    (jMsg.find("Channel") != jMsg.end()) &&
                (jMsg.find("Value") != jMsg.end()) )
    {
        Log::cout << "mqtt> dimmer Channel not yet implemented"<< Log::Warning();
    }
    else if(jMsg.find("Array") != jMsg.end())
    {
        Log::cout << "mqtt> dimmer Array not yet implemented"<< Log::Warning();
    }
    else if(jMsg.find("List") != jMsg.end())
    {
        Log::cout << "mqtt> dimmer List not yet implemented"<< Log::Warning();
    }
    else
    {
        Log::cout << "mqtt>\tUnknown dimmer format"<< Log::Error();
    }
}

void mqtt_rf_c::handle_hexRGB(int TargetNodeId,std::string &message)
{
    if(message.find("#")==0)
    {
        //the topic is "NodesActions/6/RGB"
        rgb.NodeId = TargetNodeId;
        utl::TakeParseTo(message,'#');
        unsigned int hxVal = std::stoul(message, nullptr, 16);
        rgb.R = ((hxVal >> 16) & 0xFF);
        rgb.G = ((hxVal >> 8) & 0xFF); 
        rgb.B = ((hxVal) & 0xFF);
        Log::cout << "mqtt"<<"\t"<<"=> NodeId:"<< rgb.NodeId << " RGB: ("<< rgb.R <<","<< rgb.G <<","<< rgb.B <<")"<< Log::Debug();
        mesh::msg::color_txt(rfcom,rgb.NodeId,rgb.R,rgb.G,rgb.B);
    }
}

void mqtt_rf_c::handle_RGB(int  NodeId,json &jMsg)
{
    bool found = false;
    if(jMsg.find("Red") != jMsg.end())
    if(jMsg.find("Green") != jMsg.end())
    if(jMsg.find("Blue") != jMsg.end())
    {
        found = true;
    }
    if(found)
    {
        int R = std::stoi(jMsg["Red"].dump());
        int G = std::stoi(jMsg["Green"].dump());
        int B = std::stoi(jMsg["Blue"].dump());
        Log::cout << "mqtt"<<"\t"<<"=> NodeId:"<< NodeId << " RGB: ("<< R <<","<< G <<","<< B <<")"<< Log::Debug();
        mesh::msg::color_txt(rfcom,NodeId,R,G,B);
    }
    else
    {
        Log::cout << "mqtt>\twrong RGB format"<< Log::Error();
    }
}

void mqtt_rf_c::handle_heat(int TargetNodeId,int heat_val)
{
    char text[31];
    int nbWrite = sprintf(text,"theat 0x%02x 0x%02x\r",TargetNodeId,heat_val);
    rfcom.send(text,nbWrite);
    std::string s(text);
    Log::cout << "ser\t" << s << Log::Debug();
    Log::cout << "mqtt"<<"\t"<<"=> NodeId:"<< TargetNodeId << " Heat: ("<< heat_val <<")"<< Log::Debug();
}

void mqtt_rf_c::handle_MeshRF(int NodeId,json &jMsg)
{

}

void mqtt_rf_c::handle_RawRF(std::string &message)
{
    Log::cout << "mqtt"<<"\t"<<"sending:" << message << Log::Info();
    mesh::raw::send_txt(rfcom,message);
}

void mqtt_rf_c::run()
{
    static int cycle = 0;
    if(shouldBeConnected)
    {
        int status = loop(0);//immediate return, 
        if((cycle % 500) == 0)
        {
            if(   (status == MOSQ_ERR_CONN_LOST)  || (status != MOSQ_ERR_SUCCESS) )
            {
                Log::cout << "mqtt"<<"\t"<<"error status("<<status << "), reconnecting" << Log::Error();
                int res = reconnect();
                if(res == MOSQ_ERR_SUCCESS)
                {
                    isConnected = true;
                }
                else
                {
                    Log::cout << "mqtt"<<"\t"<<"X Failed to reconnect" << Log::Error();
                }
            }
        }
        cycle++;
    }
}

void mqtt_rf_c::on_connect(int rc)
{
    Log::cout << "mqtt"<<"\t"<<"connected id(" << rc << ")" << Log::Info();

    valueActions = "Nodes/";
    jsonActions  = "jNodes/";
    std::string Subscribe1 = valueActions+"#";
    std::string Subscribe2 = jsonActions +"#";

    if(( conf.find("enable_subscribe") != conf.end() ) && conf["enable_subscribe"] )
    {
        //TODO rather subscribe to the "actions" list
        //for .. subscribe(NULL,"Actions/+/{action1}");,...
        subscribe(NULL,Subscribe1.c_str());
        Log::cout << "mqtt\tsubscribing to: " << Subscribe1 << Log::Info();
        subscribe(NULL,Subscribe2.c_str());
        Log::cout << "mqtt\tsubscribing to: " << Subscribe2 << Log::Info();
    }
    else
    {
        Log::cout << "mqtt\tconnected but subscribe is not enabled: " << Log::Info();
    }
}

void mqtt_rf_c::on_message(const struct mosquitto_message *message)
{
    std::string msg(static_cast<const char*>(message->payload) );
    std::string topic(message->topic);
    //Log::cout << "mqtt"<<"\t"<<"Topic : "<< topic <<" ; Msg : "<< msg << Log::Debug();
    std::string Action = topic;
    utl::TakeParseTo(Action,'/');//remove first section "NodesActions/"
    std::string Id = utl::TakeParseTo(Action,'/');//take the second element
    int  NodeId = (uint8_t)std::stoi(Id);
    if(topic.find(valueActions) == 0)
    {
        if(Action.find("dimmer")==0)
        {
            uint16_t light = std::stoi(msg);
            mesh::msg::dimmer::all(rfcom,NodeId,light);
        }
        else if(Action.find("heat")==0)
        {
            uint16_t heat = std::stoi(msg);
            handle_heat(NodeId,heat);
        }
    }
    else if(topic.find(jsonActions) == 0)
    {
        if(Action.find("RawRF")==0)
        {
            handle_RawRF(msg);//exception not json
        }
        else if(Action.find("hexRGB")==0)
        {
            handle_hexRGB(NodeId,msg);//exception not json
        }
        else
        {
            json jMsg = json::parse(msg);
            if(Action.find("dimmer")==0)
            {
                handle_dimmer(NodeId,jMsg);
            }
            else if(Action.find("RGB")==0)
            {
                handle_RGB(NodeId,jMsg);
            }
            else if(Action.find("MeshRF")==0)
            {
                handle_MeshRF(NodeId,jMsg);
            }
        }
    }
    else
    {
        Log::cout << "mqtt"<<"\t"<<"unexpected Topic : "<< topic << Log::Debug();
        Log::cout << "mqtt"<<"\t"<<"=> "<< msg<< Log::Debug();
    }
}

void mqtt_rf_c::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
    Log::cout << "mqtt"<<"\t"<<"subscribed to message id(" << mid << ")" << Log::Info();
}

void mqtt_rf_c::publish_measures(NodeMap_t &NodesSensorsVals)
{
	if(isConnected && shouldPublish)
	{
        for(auto const& sensorsTables : NodesSensorsVals) 
        {
            int NodeId = sensorsTables.first;
            std::string Node = std::to_string(NodeId);
            for(auto const& Table : sensorsTables.second) 
            {
                std::string SensorName = Table.first;
                for(auto const& Measure : Table.second) 
                {
                    //TODO should make the publish adress configurable
                    std::string topic = "Nodes/" + Node + "/" + SensorName;
                    std::string Value = std::to_string(Measure.value);
                    int status = publish(NULL,topic.c_str(),Value.size(),Value.c_str());
                    if(status == MOSQ_ERR_SUCCESS)
                    {
                        Log::cout << "mqtt" << "\t" << "published @ "<<topic << Log::Debug();
                    }
                    else
                    {
                        Log::cout << "mqtt" << "\t" << "publish Fail" << Log::Error();
                    }
                }
            }
        }
	}
}