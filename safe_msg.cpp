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
#include "safe_msg.hpp"

//for sleep
#include <unistd.h>

#include <map>
#include <string>
#include <mutex>

#include <iostream>


void SafeMessaging_c::push(const std::string &Key, const std::string &message)
{
	std::lock_guard<std::mutex> guard(messages_mutex);
	Messages[Key].push_back(message);
}
void SafeMessaging_c::pull(const std::string &Key, std::string &message)
{
	std::lock_guard<std::mutex> guard(messages_mutex);
	auto &messagelist = Messages[Key];
	if(!messagelist.empty())
	{
		message = messagelist.front();
		messagelist.pop_front();
	}
}
void SafeMessaging_c::remove(const std::string &Key)
{
	std::lock_guard<std::mutex> guard(messages_mutex);
	auto it = Messages.find(Key);
	if(it != Messages.end())
	{
		Messages.erase(it);
	}
	else
	{
		std::cout << "wbs> Client died without requests  " << std::endl;
	}
}

bool SafeMessaging_c::poll_any(std::string &Key, std::string &message)
{
	std::lock_guard<std::mutex> guard(messages_mutex);
	for(auto &msg : Messages)
	{
		if(!msg.second.empty())
		{
			Key = msg.first;
			message = msg.second.front();
			msg.second.pop_front();
			//break; but return rather
			return true;
		}
	}
	return false;
}

void SafeMessaging_c::push_for_all(const std::string &message)
{
	std::lock_guard<std::mutex> guard(messages_mutex);
	for(auto &msg : Messages)
	{
		//do not care who the first is, push for all
		msg.second.push_back(message);
	}
}
