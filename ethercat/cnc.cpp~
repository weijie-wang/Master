#include "cnc.hpp"
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>

#define RECAT_DEBUG_ERROR_DATAGRAM

#ifdef RECAT_DEBUG_ERROR_DATAGRAM
#include <stdarg.h>
#endif

CNC::CNC(int n) {
	//resize vector size
	axis_cmds.resize(n);
	slave_data.resize(n);
	last_slave_data.resize(n);
	slave_index.resize(n);
	error = SUCCESS;
	//map motor number to axis number
	for(int i = 0; i < 2; i++)
		slave_index[i] = i;
	
	master = NULL;
	this->point_integral = 0;
	this->p = 1;
	this->i = 0.00001;
	
}

void CNC::setup()
{
    uint8_t data = 0;
    uint8_t ldata = 1;
    /*master->write(0, 0x9060, 1, &data, 1);
    master->write(0, 0x9060, 2, &ldata, 1);
    master->write(0, 0x9060, 3, &ldata, 1);
    master->write(1, 0x9060, 1, &data, 1);
    master->write(1, 0x9060, 2, &ldata, 1);
    master->write(1, 0x9060, 3, &ldata, 1);*/
    //master->write(2, 0x9060, 1, &data, 1);
    //master->write(2, 0x9060, 2, &ldata, 1);
    //master->write(2, 0x9060, 3, &ldata, 1);
}

CNC::~CNC() {
	
}

int CNC::front(int n, CmdData* cmd) {
	this->axis_cmds_lock.lock();
	if(n < 0 || n > this->axis_cmds.size() || this->axis_cmds[n].empty() ){
        this->slave_data_lock.lock();
		(*cmd).time = this->slave_data[n].time;
		(*cmd).target = this->slave_data[n].target;
        this->slave_data_lock.unlock();
    }
    else{
        (*cmd) = this->axis_cmds[n].front();
    }
    this->axis_cmds_lock.unlock();
    return this->error = SUCCESS;
}

int CNC::set_cmd(Cmd& cmd) {
	switch(cmd.type) {
	case ENABLE:
	{
		this->cmds_lock.lock();
		this->cmds.push(cmd);
		this->cmds_lock.unlock();
	
		struct CmdData cmd_data;
		cmd_data.enable();
		this->axis_cmds_lock.lock();
		for(int i = 0; i < this->slave_index.size(); i++) {
			this->axis_cmds[i].push(cmd_data);
		}
		this->axis_cmds_lock.unlock();
		return this->error = SUCCESS;
	}	
	case DISABLE:
	{	
		this->cmds_lock.lock();
		while(true != (this->cmds.empty()))
			this->cmds.pop();
		for(int i = 0; i < this->axis_cmds.size(); i++) {
			while(true != (this->axis_cmds[i].empty()))
				this->axis_cmds[i].pop();
		}
		this->cmds.push(cmd);
		this->cmds_lock.unlock();
	
		CmdData cmd_data;
		cmd_data.disable();
		this->axis_cmds_lock.lock();
		for(int i = 0; i < this->axis_cmds.size(); i++) {
			this->axis_cmds[i].push(cmd_data);
		}
		this->axis_cmds_lock.unlock();
		return this->error = SUCCESS;
	}	
	case POINT:
	{
		this->cmds_lock.lock();
		this->cmds.push(cmd);
		this->cmds_lock.unlock();
		CmdData cmd_data;
    		double Length = 0;
    		double axis_now;
 
    		this->cmds_lock.lock();
    		this->slave_data_lock.lock();
    		for(int i = 0; i < this->slave_index.size(); i++){
        		axis_now = cmd.curve.point.end[i] - this->slave_data[ slave_index[i] ].current;
        		Length += pow(axis_now , 2);
    		}   
    		Length = sqrt(Length);

    		if(Length <= 10000)
		{
			this->point_integral += Length;
		}

    		for(int i = 0; i < this->slave_index.size(); i++){
                        axis_now = cmd.curve.point.end[i] - this->slave_data[ slave_index[i] ].current;
			if(axis_now <= 200)
			{
				cmd_data.stop();
				this->axis_cmds[i].push(cmd_data);
			}
			else
			{
				int speed_temp = (int)(Length * (this->p) + this->point_integral  * (this->i));	
				int output = (int)((axis_now / Length) * (speed_temp >= cmd.curve.point.speed ? cmd.curve.point.speed : speed_temp));
				cmd_data.setspeed(output);
				this->axis_cmds[slave_index[i]].push(cmd_data);
			}
    		}
		cmd_data.setspeed(cmd.curve.point.end[2]);
		this->axis_cmds[2].push(cmd_data);
    		this->slave_data_lock.unlock();
    		this->cmds_lock.unlock();
		return this->error = SUCCESS;
	}
	case TEST:
	{
		this->cmds_lock.lock();
		this->cmds.push(cmd);
		this->cmds_lock.unlock();
		CmdData cmd_data;
		for(int i = 0; i < this->slave_data.size(); i++) {
			
			cmd_data.time = 5;
			cmd_data.target = 200;
			this->axis_cmds[i].push(cmd_data);
					
		}
		return this->error = SUCCESS;
	}
		
		
	default:
	{
#ifdef RECAT_DEBUG_ERROR_DATAGRAM
		printf("[Error] set_cmd(): no cmd is set! \n");
#endif
		break;
	}
	}
}

int CNC::wait_cmd() {
	if(this->cmds.empty())
	{
		CmdData cmd_data;
		cmd_data.stop();
		for(int i = 0; i < this->slave_index.size(); i++) {
			this->axis_cmds[i].push(cmd_data);
		}
		return error = SUCCESS;
	}
	Cmd cmd = this->cmds.front();
	switch(cmd.type) {
	case ENABLE:
	{
		if(cmd.curve.enable.repeat_times == 0)
		{
			std::vector< int > wrong_num;
			for(int i = 0; i < this->slave_data.size(); i++)
			{
				if((this->slave_data[i].status & 0x00001000) == 0)
				{
					wrong_num.push_back(i);
				}
			}
			std::cout<<"[ERROR]: Slave ";
			for(std::vector< int >::iterator m = wrong_num.begin() ; m != wrong_num.end() ; m++ ){
				std::cout<<*m<<" ";
			}
			std::cout<<"can not enable "<<std::endl;
			this->cmds.pop();
			return this->error = ENABLE_ERROR;
		}
		else{
			CmdData cmd_data;
			this->slave_data_lock.lock();
			this->axis_cmds_lock.lock();
			bool flag =true;
			for(int i = 0; i < this->slave_data.size(); i++) {
				if((this->slave_data[i].status & 0x00001000) == 0)
				{
					
					cmd_data.enable();
					this->axis_cmds[i].push(cmd_data);
					flag = false;
				}
				else
				{
					cmd_data.stop();
					this->axis_cmds[i].push(cmd_data);
				}
					
			}
			this->slave_data_lock.unlock();
			this->axis_cmds_lock.unlock();
			if(flag == true) {
				std::cout<<"[ok]: Enable"<<std::endl;
				this->cmds.pop();
			}
			else {
				this->cmds.front().curve.enable.repeat_times--;
			}
			return this->error = SUCCESS;
		}
	}	
	case DISABLE:
	{
		if(cmd.curve.disable.repeat_times == 0)
		{
			std::vector< int > wrong_num;
			for(int i = 0; i < this->slave_data.size(); i++)
			{
				if((this->slave_data[i].status & 0x00001000) != 0)
				{
					wrong_num.push_back(i);
				}
			}
			std::cout<<"[ERROR]: Slave ";
			for(std::vector< int >::iterator m = wrong_num.begin() ; m != wrong_num.end() ; m++ ){
				std::cout<<*m<<" ";
			}
			std::cout<<"can not disable "<<std::endl;
			this->cmds.pop();
			return this->error = ENABLE_ERROR;
		}
		else{
			CmdData cmd_data;
			this->slave_data_lock.lock();
			this->axis_cmds_lock.lock();
			bool flag =true;
			for(int i = 0; i < this->slave_data.size(); i++) {
				if((this->slave_data[i].status & 0x00001000) != 0)
				{
					cmd_data.disable();
					this->axis_cmds[i].push(cmd_data);
					flag = false;
				}
				else
				{
					cmd_data.stop();
					this->axis_cmds[i].push(cmd_data);
				}
					
			}
			this->slave_data_lock.unlock();
			this->axis_cmds_lock.unlock();
			if(flag == true) {
				std::cout<<"[ok]: Disable"<<std::endl;
				this->cmds.pop();
			}
			else {
				this->cmds.front().curve.disable.repeat_times--;
			}
			return this->error = SUCCESS;
		}
	}
	case POINT:
	{
		CmdData cmd_data;
    		double Length = 0;
    		double axis_now;
    		Cmd cmd = this->cmds.front();
    
    		this->cmds_lock.lock();
    		this->slave_data_lock.lock();
		bool flag =true;
    		for(int i = 0; i < this->slave_index.size(); i++){
        		axis_now = cmd.curve.point.end[i] - this->slave_data[ slave_index[i] ].current;
        		Length += pow(axis_now , 2);
    		}   
    		Length = sqrt(Length);
		if(Length <= 10000)
		{
			this->point_integral += Length;
		}

    		for(int i = 0; i < this->slave_index.size(); i++){
                        axis_now = cmd.curve.point.end[i] - this->slave_data[ slave_index[i] ].current;
			if(axis_now <= 200)
			{
				cmd_data.stop();
				this->axis_cmds[i].push(cmd_data);
			}
			else
			{
				int speed_temp = (int)(Length * (this->p) + this->point_integral  * (this->p));	
				int output = (int)((axis_now / Length) * (speed_temp >= cmd.curve.point.speed ? cmd.curve.point.speed : speed_temp));
				cmd_data.setspeed(output);
				this->axis_cmds[slave_index[i]].push(cmd_data);
				flag =false;
			}
    		}
		if(flag == true) {
			std::cout<<"[ok]: Piont"<<std::endl;
			this->cmds.pop();
		}
		cmd_data.setspeed(cmd.curve.point.end[2]);
		this->axis_cmds[2].push(cmd_data);
    		this->slave_data_lock.unlock();
    		this->cmds_lock.unlock();
		return this->error = SUCCESS;
	}
	case TEST:
	{
		CmdData cmd_data;
		for(int i = 0; i < this->slave_data.size(); i++) {
			cmd_data.time = 5;
			cmd_data.target = 200;
			this->axis_cmds[i].push(cmd_data);
					
		}
		return this->error = SUCCESS;
	}
		
	default:
	{
#ifdef RECAT_DEBUG_ERROR_DATAGRAM
		printf("[Error] set_cmd(): no cmd is set! \n");
#endif
		break;
	}
	}
}

int CNC::slave_cmds_pop(int n) {
	this->axis_cmds[n].pop();
	return this->error = SUCCESS;
}

int CNC::set_master(RECAT* t){
    this->master = t;
	return this->error = SUCCESS;
}

int CNC::slave_data_push(int index, Data *data){
    this->last_slave_data[index] = this->slave_data[index];
			this->slave_data[index] = *data;
}

    

