// test.cpp : ¶šÒå¿ØÖÆÌšÓŠÓÃ³ÌÐòµÄÈë¿Úµã¡£
//System Includes
#include <vector>
#include <iostream>

//app Includes
#include "cnc.h"
#include "recat.h"

//Modbus Inludes
#include "modbus.h"
#include "modbus-rtu.h"
#include "modbus-tcp.h"
#include "modbus-version.h"
using namespace std;

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

Controller::Controller(const std::string &mac, int slave_num)
    : RECAT(mac)
{
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
    static unsigned int tick = 0;
    for(int index = 0; index < SLAVE_CNT; index ++)
    {
        if(motors[index] != NULL)
	{

            motors[index]->process(input, output);
	}
    }

}

int main(int argc, char** argv)
{
	modbus_t *mb;
    	uint16_t tab_reg[32];
    	uint8_t stop_flag =0;
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

	Controller lenovo(computer_mac, SLAVE_CNT);
	usleep(1000);
	lenovo.start();
	//lenovo.parser.setup();
	usleep(1000);
	//lenovo.parser.enable();
	PaserCMD cmd;
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
						lenovo.motors[0]->_times->data = 15;
						lenovo.motors[0]->_target->data = 1;
					}
			}
		}
		if(0 <= modbus_read_bits(mb, 0, 1, &stop_flag))
		{
	    		tab_reg[2] = stop_flag;
	    		if(tab_reg[2] != tab_reg[3])
	    		{
				if(tab_reg[2] == 1)
				{
					printf("Start\r\n");
					if(tab_reg[0]==1){
						lenovo.motors[0]->_times->data = 5;
						lenovo.motors[0]->_target->data = 2000;
					}
					if(tab_reg[0]==2){
						lenovo.motors[1]->_times->data = 5;
						lenovo.motors[1]->_target->data = 2000;
					}
					if(tab_reg[0]==3){
						lenovo.motors[2]->_times->data = 5;
						lenovo.motors[2]->_target->data = 2000;
					}
				}
				else
				{
					printf("Stop\r\n");
					if(tab_reg[0]==1){
						lenovo.motors[0]->_times->data = 5;
						lenovo.motors[0]->_target->data = 0;
					}
				}
                		tab_reg[3] = tab_reg[2];	
	    		}		
		}
		if(0 <= modbus_read_bits(mb, 37, 1, &stop_flag))
		{
	    		if(stop_flag == 1)
	    		{
                		modbus_write_bit(mb, tab_reg[1], 0);
				system("reboot");
	    		}		
		}
		/*cmd.type = LINE;
        	cmd.para.line.endx = 300000;
        	cmd.para.line.endy = 450000;
        	cmd.para.line.speed = 150000;
        	cmd.para.line.endz = 0;
        	lenovo.parser.SetCMD(cmd);
        	lenovo.parser.WaitCMD();*/
		
	}
	return 0;
}

