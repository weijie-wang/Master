// test.cpp : ¶šÒå¿ØÖÆÌšÓŠÓÃ³ÌÐòµÄÈë¿Úµã¡£
//System Includes
#include <vector>
#include <iostream>

//app Includes
#include "cnc.hpp"
#include "recat.h"
#include <string>

//Modbus Inludes
#include "modbus.h"
#include "modbus-rtu.h"
#include "modbus-tcp.h"
#include "modbus-version.h"

#include <pthread.h>
using namespace std;

int flag = 0;
int speed[3];
#define SLAVE_CNT 3 //slave number
/** declaration of slave */
#define __PDO_UNIT__(ret_type, type, name)    \
    public:type* _##name;                      \
    public: inline ret_type name() const{       \
        return this->_##name->data;             \
    }

const std::string computer_mac = "";//master mac
#ifdef _DEBUG
std::string mode;
#endif

class Slave
{
public:
    __PDO_UNIT__( uint32_t, PDO_U32, times);
    __PDO_UNIT__(  int32_t, PDO_S32, target);

    __PDO_UNIT__( uint32_t, PDO_U32, status);
    __PDO_UNIT__(  int32_t, PDO_S32, current);

    std::vector<PDO_Data*> pdos_in;
    std::vector<PDO_Data*> pdos_out;
public:
    Slave(int index = 0);
    ~Slave();
    void process (uint8_t * input, uint8_t * output);
};

Slave::Slave(int index)
{
#define RECAT_MOTOR_VENDER_ID 0x0000059D, 0x54490001
    _times  = new PDO_U32 ( 0, index, RECAT_MOTOR_VENDER_ID, 0x7010, 1 );
    _target = new PDO_S32 ( 0, index, RECAT_MOTOR_VENDER_ID, 0x7010, 2 );

    _status   = new PDO_U32 ( 0, index, RECAT_MOTOR_VENDER_ID, 0x6000, 1 );
    _current  = new PDO_S32 ( 0, index, RECAT_MOTOR_VENDER_ID, 0x6000, 2 );
    
    _times->data = 0;
    _target->data = 0;
}

Slave::~Slave()
{
    delete _times  ;
    delete _target ;
    delete _status ;
    delete _current;
}

void Slave::process(uint8_t *input, uint8_t *output)
{
    _times->write(output);
    _target->write(output);

    _status->read(input);
    _current->read(input);
}
/** declaration of controller, maintance some slaves */
class Controller:public RECAT
{
private:
	
public: 
    int send_flag;
    CNC cnc;
    std::vector<Slave*> motors;
    Controller(const std::string& mac, int slave_num = 1);
    ~Controller();

    inline int slave_size()const
    {
        return motors.size();
    }
	void seconds ();
	void process (uint8_t *input, uint8_t *output);
};
Controller lenovo(computer_mac, SLAVE_CNT);

Controller::Controller(const std::string &mac, int slave_num)
    : RECAT(mac), cnc( slave_num )
{
	this->cnc.set_master(this);
    for( int i = 0; i < slave_num; i++)
    {
        Slave* motor = new Slave(i);

        PDO_CONFIG* in  = pdo_config( 0x1A00, motor->_status, motor->_current, NULL);
        PDO_CONFIG* out = pdo_config( 0x1601, motor->_times,  motor->_target, NULL);


        ec_sync_info_t sjtu_syncs_out[] = {
            {2, EC_DIR_OUTPUT,1, out->info},
            {3, EC_DIR_INPUT},
            {0xff} };

        ec_sync_info_t sjtu_syncs_in[] = {
            {2, EC_DIR_OUTPUT},
            {3, EC_DIR_INPUT, 1, in->info},
            {0xff} };

        /** configure the position of the slave*/
        this->slave(0, i, RECAT_MOTOR_VENDER_ID);
        /** configure the SM of the slave which charge the pdo config */
        this->sync_manager(i, sjtu_syncs_out);
        this->sync_manager(i, sjtu_syncs_in);
        this->pdo(in->reg, out->reg);

        pdo_free(in);
        pdo_free(out);
        
        motors.push_back(motor);
		
    }
}

Controller::~Controller()
{
    for(int index = 0; index < SLAVE_CNT; index ++)
        if(motors[index] != NULL)
            delete motors[index];
}

void Controller::seconds()
{
    /*for(int index = 0; index < SLAVE_CNT; index ++)
        if(motors[index] != NULL)
        {
            printf("index=%d times=%d target=%d ", index, motors[index]->times(), motors[index]->target() );
            printf("status=%d current=%d \n", motors[index]->status(), motors[index]->current() );
        }*/
}

void Controller::process(uint8_t *input, uint8_t *output)
{
    if(flag == 1)	
		{
		    for(int index = 0; index < 3; index ++)
    		    {
			uint32_t _status = (motors[index]->_status->data) & 0x80;
			if(_status != 0)
			{
				motors[index]->_times->data = 5;
				motors[index]->_target->data = speed[index];
			}	
			_status = (motors[index]->_status->data) & 0x40;
			if(_status != 0)
			{
				motors[index]->_times->data = 5;
				motors[index]->_target->data = -speed[index];
			}
		    }
		}
		
    for(int index = 0; index < SLAVE_CNT; index ++)
    {
        if(motors[index] != NULL)
		{
			Data data;
			data.time = motors[index]->_times->data;
			data.target = motors[index]->_target->data;
			data.status = motors[index]->_status->data;
			data.current = motors[index]->_current->data;
			//std::cout<<data.status<<"   ";
            		motors[index]->process(input, output);
		}
    }
    //std::cout<<std::endl;	
}

void* thread(void*) {
	std::string mode;
	int motor_num = 0;
	while(true)
	{
		std::cin>>mode;
		if("enable" == mode)
		{
			lenovo.motors[0]->_times->data = 15;
			lenovo.motors[0]->_target->data = 1;
			lenovo.motors[1]->_times->data = 15;
			lenovo.motors[1]->_target->data = 1;
			lenovo.motors[2]->_times->data = 15;
			lenovo.motors[2]->_target->data = 1;	
		}
		else if("set" == mode)
		{
			int times, target;
			std::cin>>times;
			std::cin>>target;
			lenovo.motors[motor_num]->_times->data = times;
			lenovo.motors[motor_num]->_target->data = target;	
		}
		else if("select" == mode)
		{
			std::cin>>motor_num;
			if(motor_num >= SLAVE_CNT || motor_num < 0)
			{
				std::cout<<"error;"<<std::endl;
				motor_num = 0;
			}
		}
		else if("disable" == mode)
		{
			lenovo.motors[0]->_times->data = 15;
			lenovo.motors[0]->_target->data = 0;
			lenovo.motors[1]->_times->data = 15;
			lenovo.motors[1]->_target->data = 0;
			lenovo.motors[2]->_times->data = 15;
			lenovo.motors[2]->_target->data = 0;	
		}
		else if("p" == mode)
		{
			std::cout<<"times = "<<lenovo.motors[motor_num]->_times->data<<"   ";
			std::cout<<"target = "<<lenovo.motors[motor_num]->_target->data<<"   ";
			std::cout<<"status = "<<lenovo.motors[motor_num]->_status->data<<"   ";
			std::cout<<"current = "<<lenovo.motors[motor_num]->_current->data<<"   "<<std::endl;
		}
		else if("speed" == mode)
		{
			int temp_speed;
			std::cin>>temp_speed;
			speed[motor_num] = temp_speed;
			
		}			
		else
		{
			for(int index = 0; index < SLAVE_CNT; index ++)
    		    	{
				lenovo.motors[index]->_times->data = 5;
				lenovo.motors[index]->_target->data = 0;	
		    	}
		}
		
	}
}

int main(int argc, char** argv)
{
	speed[0] = speed[1] = 800;
	speed[2] = 800;
	modbus_t *mb;
    	uint16_t tab_reg[32];
    	uint8_t stop_flag =0;
	std::string command;
    	printf("1 Link...\r\n");
    	mb = modbus_new_tcp("192.168.1.61", 502);
    	int error = -1;
    	printf("Start Link...\r\n");
    	while(error != 0)
    	{
        	printf("Linking...\r\n");
        	error = modbus_connect(mb);
    	}
    	modbus_set_slave(mb, 1);
    	modbus_write_bit(mb, 37, 0);
    	tab_reg[0] = tab_reg[1] = tab_reg[2] = tab_reg[3] = 0;

	
	usleep(1000);
	lenovo.start();
	//usleep(5000000);
	//lenovo.cnc.setup();

	pthread_t id;
	int ret;
	ret = pthread_create(&id, NULL, &thread, NULL);
	if(ret != 0) {
		std::cout<<"Create pthread error!\n\r"<<std::endl;
		exit(1);
	}
	while(true)
	{
		if(0 <= modbus_read_registers(mb, 0, 1, tab_reg))
		{
			if(tab_reg[0] != tab_reg[1])
                	{
				printf("%d\r\n",tab_reg[0]);
                        	modbus_write_bit(mb, tab_reg[0], 1);
				modbus_write_bit(mb, tab_reg[1], 0);
				tab_reg[1] = tab_reg[0];
				if(tab_reg[0]==1){
					speed[0] += 100;
				}
				if(tab_reg[0]==2){
					speed[0] -= 100;
				}
				if(tab_reg[0]==3){
					speed[1] += 100;
				}
				if(tab_reg[0]==4){
					speed[1] -= 100;
				}
				if(tab_reg[0]==5){
					speed[2] += 100;
				}
				if(tab_reg[0]==6){
					speed[2] -= 100;
				}
			std::cout<<speed[0]<<"   "<<speed[1]<<"   "<<speed[2]<<"   "<<std::endl;
			}
		}
		if(0 <= modbus_read_bits(mb, 0, 1, &stop_flag))
		{
	    		tab_reg[2] = stop_flag;
	    		if(tab_reg[2] != tab_reg[3])
	    		{
                                lenovo.send_flag = 1;
				if(tab_reg[2] == 1)
				{
					printf("Start\r\n");
					
					lenovo.motors[0]->_times->data = 15;
					lenovo.motors[0]->_target->data = 1;
					lenovo.motors[1]->_times->data = 15;
					lenovo.motors[1]->_target->data = 1;
					lenovo.motors[2]->_times->data = 15;
					lenovo.motors[2]->_target->data = 1;
					usleep(1000000);
					flag = 1;
					for(int index = 0; index < SLAVE_CNT; index ++)
    		    			{
						lenovo.motors[index]->_times->data = 5;
						lenovo.motors[index]->_target->data = speed[index];	
		    			}
					
				}
				else
				{
					printf("Stop\r\n");
					lenovo.motors[0]->_times->data = 15;
					lenovo.motors[0]->_target->data = 0;
					lenovo.motors[1]->_times->data = 15;
					lenovo.motors[1]->_target->data = 0;
					lenovo.motors[2]->_times->data = 15;
					lenovo.motors[2]->_target->data = 0;
					flag = 0;
				}
                		tab_reg[3] = tab_reg[2];	
	    		}		
		}
		
	}
	return 0;
}

