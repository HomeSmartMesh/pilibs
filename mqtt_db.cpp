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
switched to mqtt_db on 13.12.2017

 mqtt application wrapper
 
*/

#include "mqtt_db.hpp"

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
mqtt_db_c::mqtt_db_c(json &conf) : mosquittopp("iot_db")
{
    isReady = false;
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

void mqtt_db_c::run()
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

void mqtt_db_c::on_connect(int rc)
{
    Log::cout << "mqtt"<<"\t"<<"connected id(" << rc << ")" << Log::Info();

    subscribe(NULL,"Nodes/#");//get all Nodes traffic
}

void mqtt_db_c::on_message(const struct mosquitto_message *message)
{
    std::string msg(static_cast<const char*>(message->payload) );
    std::string topic(message->topic);
    if(topic.find("Nodes/") == 0)
    {
        notifications.push(topic,msg);
    }
    else
    {
        Log::cout << "mqtt"<<"\t"<<"unexpected Topic : "<< topic << Log::Debug();
        Log::cout << "mqtt"<<"\t"<<"=> "<< msg<< Log::Debug();
    }
}

void mqtt_db_c::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
    Log::cout << "mqtt"<<"\t"<<"subscribed to message id(" << mid << ")" << Log::Info();
}

bool mqtt_db_c::getMeasures(NodeMap_t& measures)
{
    std::string topic,msg;
    while(notifications.poll_any(topic,msg))
    {
        utl::TakeParseTo(topic,'/');//remove first section "Nodes/"
        std::string Id = utl::TakeParseTo(topic,'/');//take the second element
        std::string Sensor = utl::TakeParseTo(topic,'/');//take the sensor
        bool isVerifOK = false;
        int NodeId;
        float value;
        try
        {
            int NodeId = std::stoi(Id);
            float value = std::stof(msg);
        }
        catch(const std::exception& ex)
        {
            std::cout << "dbm> !!! Caught exception \"" << ex.what() << "\"!!!\n";
            isVerifOK = false;
        }
        if(isVerifOK)
        {
            sensor_measure_t measure;
            measure.value = value;
            time(&measure.time);//take a timestamp here
            measures[NodeId][Sensor].push_back(measure);
        }
    }
}
