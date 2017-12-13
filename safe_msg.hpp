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
 - Poco is using The Boost Software License 1.0
___________________________________________________________________________________

 start date : 20.01.2017
 split from webserver on 13.12.2017

*/


//for strmap
#include "utils.hpp"
#include <string>
#include <vector>
#include <mutex>
#include <list>

#include "json.hpp"
using json = nlohmann::json;

typedef std::list<std::string> Messages_t;
typedef std::map<std::string,Messages_t> MessagesMap_t;


class SafeMessaging_c
{
public:
	void push(const std::string &Key, const std::string &message);
	void pull(const std::string &Key, std::string &message);
	bool poll_any(std::string &Key, std::string &message);
	void push_for_all(const std::string &message);
	void remove(const std::string &Key);
private:	
	MessagesMap_t 	Messages;
	std::mutex 		messages_mutex;
};
