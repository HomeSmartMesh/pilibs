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

#include <mosquittopp.h>

#include "utils.hpp"
#include "safe_msg.hpp"

#include "json.hpp"
using json = nlohmann::json;

class mqtt_db_c : public mosqpp::mosquittopp
{
	public:
		mqtt_db_c(json &conf);
        void run();
		bool getMeasures(NodeMap_t& measures);
		void on_connect(int rc);
		void on_message(const struct mosquitto_message *message);
		void on_subscribe(int mid, int qos_count, const int *granted_qos);
        bool isReady;
	private:
	SafeMessaging_c notifications;
};
