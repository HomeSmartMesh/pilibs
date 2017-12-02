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
 - Boost Filesystem : The Boost Software License 1.0
___________________________________________________________________________________

date : 02.01.2017

database manager : RAM and Files mirroring

 - on startup loads the db files into memory (memory usage management)
 - on sensors string parse:
   - add entry in memory structure
   - create new directory path, and write entry in file

*/


#include "db_mgr.hpp"

//for stdout
#include <iostream>
#include <exception>
#include <stdexcept>

//Linux dependency
#include <sys/stat.h>

#include <ctime>

#include <boost/filesystem.hpp>
using namespace boost::filesystem;
using std::cout;

#include "json.hpp"
using json = nlohmann::json;

#include "log.hpp"

db_manager_c::db_manager_c()
{

}

db_manager_c::db_manager_c(json &v_conf)
{
	config(v_conf);
}

bool db_manager_c::config(json &v_conf)
{
	conf = v_conf;
	if( conf.find("path") != conf.end() )
	{
		dbpath = conf["path"];
	}
	return true;
}

bool db_manager_c::splitPath2Names(std::string path,int &year,int &month,int &NodeId,std::string &SensorName)
{
	bool res = false;
	strvect texts = utl::split(path,'/');
	int length = texts.size();
	if(length > 3)
	{
		year = std::stoi(texts[length-3]);
		month = std::stoi(texts[length-2]);
		if((month >= 1) || (month <= 12))
		{
			strvect node_sensor = utl::split(texts[length-1],'_');
			if(node_sensor.size() == 2)
			{
				std::string NodeName = node_sensor[0];
				if(NodeName.find("NodeId") == 0)
				{
					std::string name = "NodeId";
					utl::remove(name,NodeName);
					NodeId = std::stoi(NodeName);

					strvect file_ext = utl::split(node_sensor[1],'.');
					if(file_ext.size() == 2)
					{
						if(file_ext[1].find("txt") == 0)
						{
							SensorName = file_ext[0];
							res = true;
						}
					}
				}
			}
		}
		
	}
	return res;
}

void db_manager_c::load()
{
	utl::time_u	load_start = utl::get_start();
	long long nbLoadedSamples = 0; 
    if(( conf.find("disable") != conf.end() ) && !conf["disable"] )
    {
		if( conf.find("loadpaths") != conf.end() )
		{
			cout << "dbm>loading files " << std::endl;
			std::string loadpaths = conf["loadpaths"];
			path p(loadpaths);
			
			try
			{
				if (exists(p) && is_directory(p))
				{
					for (directory_entry& x : directory_iterator(p))
					{
						cout << "    " << x.path() << '\n'; 
						if(is_directory(x.path()))
							for (directory_entry& f : directory_iterator(x.path()))
							{
								std::string filename = f.path().string();
								cout << "        " << filename << std::endl;
								//get Node Id and params
								int year,month;
								std::string SensorName;
								int NodeId;
								if (splitPath2Names(filename,year,month,NodeId,SensorName))
								{
									std::tm timeinfo;
									timeinfo.tm_year = year - 1900;//standard say so
									timeinfo.tm_mon = month-1;
									timeinfo.tm_isdst = 0;//false as if it applies, it will take one hour off time-=1h
									
									std::ifstream ifile;
									ifile.open(filename.c_str(), std::ios::in );
									std::string line;
									while (std::getline(ifile, line))
									{
										strvect cells = utl::split(line,'\t');
										if(cells.size() == 3)//3 columns expected
										{
											std::string &day_txt = cells[0];
											std::string &time_txt = cells[1];
											std::string &value_txt = cells[2];

											sensor_measure_t Measure;
											Measure.value = std::stof(value_txt);

											timeinfo.tm_mday = std::stoi(day_txt);
											strvect timevals = utl::split(time_txt,':');
											if(timevals.size() == 3)
											{
												timeinfo.tm_hour = std::stoi(timevals[0]);
												timeinfo.tm_min = std::stoi(timevals[1]);
												timeinfo.tm_sec = std::stoi(timevals[2]);
											}
											else
											{
												std::cout << "Error: unexpected time format" << std::endl;
											}
											Measure.time = std::mktime(&timeinfo);

											Nodes[NodeId][SensorName].push_back(Measure);
											nbLoadedSamples++;
										}
										else
										{
											std::cout << "Error: 3 columns expected" << std::endl;
										}
									}
								}
								
							}
					}
				}
				else
				cout << p << " does not exist\n";
			}

			catch (const filesystem_error& ex)
			{
				cout << ex.what() << '\n';
			}
			if(nbLoadedSamples > 0)
			{
				std::cout << "dbm> loaded " << nbLoadedSamples<< " Measures in " << utl::get_stop(load_start) << std::endl;
			}
		}
		else
		{
			std::cout << "dbm> X :'loadpaths' parameter not available, databse will not be used" << std::endl;
		}
	}
	else
	{
			std::cout << "dbm> X :database disabled, will not be loaded" << std::endl;
	}
		
}

void db_manager_c::print()
{
	for(auto const& sensorsTables : Nodes)
	{
		int NodeId = sensorsTables.first;
		std::string NodeName = "NodeId" + std::to_string(NodeId);
		std::cout << NodeName << std::endl;
		for(auto const& Table : sensorsTables.second) 
		{
			std::string SensorName = Table.first;
			std::cout << "\tSensor: " << SensorName << std::endl;
			for(auto const& Measure : Table.second) 
			{
				std::cout << "\t\ttime: " << utl::getTime(Measure.time) << std::endl;
				std::cout << "\t\tval: " << Measure.value << std::endl;
			}
		}
	}
}

//The year folder should be manually created !!!!
//"dbpath/year/month/NodeName_SensorName.txt"
void db_manager_c::addMeasures(NodeMap_t &NodesSensorsVals)
{
	if(dbpath.empty())
	{
		return;
	}
	for(auto const& sensorsTables : NodesSensorsVals) 
	{
		int NodeId = sensorsTables.first;
		std::string NodeName = "NodeId" + std::to_string(NodeId);
		Log::cout << "dbm\t" << NodeName << Log::Info();
		for(auto const& Table : sensorsTables.second) 
		{
			std::string SensorName = Table.first;
			Log::cout << "dbm" << "\tSensor: " << SensorName << Log::Debug();
			for(auto const& Measure : Table.second) 
			{
				//--------------------------first add it to the memory DB--------------------------
				Nodes[NodeId][SensorName].push_back(Measure);
				//--------------------------then to cout--------------------------
				Log::cout << "dbm" << "\t\ttime: " << utl::getTime(Measure.time) << Log::Debug();
				Log::cout << "dbm" << "\t\tval: " << Measure.value << Log::Debug();
				//--------------------------then save it to the db files--------------------------
				//TODO this year month setting could be triggered on event to update it once.
				std::string text_year,text_month,text_day;
				utl::getYearMonthDay(Measure.time,text_year,text_month,text_day);
				std::string filepath = dbpath + text_year + "/" + text_month;
				std::string filename = filepath + "/" + NodeName + "_" + SensorName + ".txt";
				std::ofstream &ofile = Files[filename];
				//minimal test to evaluate file open only once on usual loop
				if(ofile.is_open())
				{
					std::string text_time = utl::getTime(Measure.time);
					ofile << text_day << "\t" << text_time << "\t" << Measure.value << std::endl;
				}
				else//come here only on exceptions for first time call
				{
					Log::cout << "dbm\t" << ">>> Opening file: " << filename << Log::Info();
					ofile.open(filename.c_str(), (std::ios::out|std::ios::app) );
					if(!ofile.is_open())
					{
						Log::cout << "dbm\t" << "could not open sensor file: " << filename << Log::Error();
						struct stat buffer;
						if(stat(filepath.c_str(), &buffer) != 0)//then Month directory does not exist
						{
							Log::cout << "dbm\t" << ">>> Directory does not exist: " << filepath << Log::Warning();
							mkdir(filepath.c_str(),ACCESSPERMS);
							if(stat(filepath.c_str(), &buffer) == 0)
							{
								Log::cout << "dbm>" << ">>> Directory created,retry open file" << Log::Info();
								ofile.open(filename.c_str(), (std::ios::out|std::ios::app) );
								if(ofile.is_open())
								{
									std::string text_time = utl::getTime(Measure.time);
									ofile << text_day << "\t" << text_time << "\t" << Measure.value << std::endl;
								}
								else
								{
									Log::cout << "dbm\t" << ">>> Still could not open sensor file !!!: " << filename << Log::Error();
									//=> Error don't know what's goig on ?
								}
							}
							else
							{
								Log::cout << "dbm>" << ">>> Directory still does not exist !!!! : " << filepath << Log::Error();
							}
						}
					}
				}
			}
		}
	}
}

void db_manager_c::getMeasures(int NodeId,std::string SensorName, time_t start, time_t stop,NodeMap_t &ResVals)
{
	std::cout << "dbm> get> " << NodeId << " " <<SensorName	<<" from("  << utl::getDay(start)<<" "<< utl::getTime(start)
															<<") till(" << utl::getDay(stop)<<" "<< utl::getTime(stop)<< ")" <<std::endl;
	sensor_measures_table_t &db_measures 	= Nodes[NodeId][SensorName];
	sensor_measures_table_t &resp_measures 	= ResVals[NodeId][SensorName];
	int count = 0;
	int found = 0;
	for(auto const& Measure : db_measures)
	{
		count++;
		if((Measure.time >= start) && (Measure.time <= stop) )
		{
			found++;
			resp_measures.push_back(Measure);
		}
	}
	std::cout << "dbm> counts/found : " << count << "/" << found << std::endl;
}

//get the last measures of all nodes and sensors
void db_manager_c::getUpdate(NodeMap_t &ResVals)
{
	std::cout << "dbm> get update" << std::endl;
	int count = 0;
	for(auto const& node : Nodes)
	{
		int NodeId = node.first;
		for(auto const& sensor: node.second)
		{
			std::string sensorName = sensor.first;
			ResVals[NodeId][sensorName].push_back(sensor.second.back());
			count++;
		}
	}
	std::cout << "dbm> update with " << count << " measures" << std::endl;
}

void db_manager_c::handle_request(const std::string &request,std::string &response)
{
	utl::start();
	if(!request.empty())
	{
		
		std::cout << "dbm> request>" << request << std::endl;
		json jReq = json::parse(request);
		std::string reqType;
		if(jReq.find("request") != jReq.end())
		{
			if(jReq["request"].find("type") != jReq["request"].end())
			{
				reqType = jReq["request"]["type"];
				std::cout << "dbm> req Type>" << reqType << std::endl;
			}
		}
		if(reqType.find("Duration") == 0)
		{
			bool isVerifOK = true;
			//std::exception_ptr eptr;
			time_t start,stop;
			int NodeId;
			std::string SensorName;
			try
			{
				start 		= std::stoll(jReq["request"]["start"].dump())/1000;
				stop 		= std::stoll(jReq["request"]["stop"].dump())/1000;
				NodeId 		= std::stoi(jReq["request"]["NodeId"].dump());
				SensorName 	= jReq["request"]["SensorName"];
			}
			catch(const std::exception& ex)
			{
				std::cout << "dbm> !!! Caught exception \"" << ex.what() << "\"!!!\n";
				isVerifOK = false;
			}

			if(isVerifOK)
			{
				NodeMap_t ResVals;
				getMeasures(NodeId,SensorName,start,stop,ResVals);
				json jResp;
				utl::make_json_resp(NodeId,SensorName,ResVals,jResp,"response");
				jResp["response"]["id"] = jReq["request"]["id"];
				jResp["response"]["type"] = "Duration";
				jResp["response"]["NodeId"] = NodeId;
				jResp["response"]["SensorName"] = SensorName;
				
				response = jResp.dump();
				std::cout << "dbm> response is an update> " << response.length() << " Bytes, prepared in "<< utl::stop() << std::endl;
			}
			else
			{
				std::cout << "dbm> request parameters verification Failed "<< utl::stop() << std::endl;
			}
		}
		else if(reqType.find("update") == 0)
		{
			NodeMap_t ResVals;
			getUpdate(ResVals);
			response = utl::stringify(ResVals,"update");
			std::cout << "dbm> response is an update> " << response.length() << " Bytes, prepared in "<< utl::stop() << std::endl;
		}
		else
		{
			std::cout << "dbm> Error : undefined request 'type' " << utl::stop() << std::endl;
		}
		
	}
}
