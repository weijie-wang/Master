#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <stdlib.h>


#include <sys/mman.h>
#include <string.h>
#include <math.h>

#include "parse.h"
#include "timer.h"
#include "recat.h"

#include <vector>

/** declaration of slave */
#define __PDO_UNIT__(ret_type, type, name)    \
    public:type* _##name;                     \
    public: inline ret_type name() const{     \
        return this->_##name->data;           \
    }
class Slave{
public:
    __PDO_UNIT__( uint32_t, PDO_U32, times);
    __PDO_UNIT__(  int32_t, PDO_S32, target);
    //PDO_S32* _user_cmd;

    __PDO_UNIT__( uint32_t, PDO_U32, status);
    __PDO_UNIT__(  int32_t, PDO_S32, current);
    //PDO_S32* _feedback;

    std::vector<PDO_Data*> pdos_in;
    std::vector<PDO_Data*> pdos_out;
public:
    Slave(int index = 0);
    ~Slave();
    void process (uint8_t * input, uint8_t * output);
};

Slave::Slave(int index){
#define RECAT_MOTOR_VENDER_ID 0x0000059D, 0x54490001
    _times  = new PDO_U32 ( 0, index, RECAT_MOTOR_VENDER_ID, 0x7010, 1 );
    _target = new PDO_S32 ( 0, index, RECAT_MOTOR_VENDER_ID, 0x7010, 2 );
    //_user_cmd = new PDO_S32( 0, index, SJTU_SERVO_DRIVE, 0x7010, 3 );

    _status   = new PDO_U32 ( 0, index, RECAT_MOTOR_VENDER_ID, 0x6000, 1 );
    _current  = new PDO_S32 ( 0, index, RECAT_MOTOR_VENDER_ID, 0x6000, 2 );
    //_feedback = new PDO_S32 ( 0, index, SJTU_SERVO_DRIVE, 0x6000, 3 );
    _times->data = 0;
    _target->data = 0;
    //_user_cmd->data = 0;

    print_pdo(_times);
    print_pdo(_target);
    //print_pdo(_user_cmd);

    print_pdo(_status);
    print_pdo(_current);
    //print_pdo(_feedback);
}
Slave::~Slave(){
    delete _times  ;
    delete _target ;
    delete _status ;
    delete _current;
}
void Slave::process(uint8_t *input, uint8_t *output){
    _times->write(output);
    _target->write(output);
    //_user_cmd->write(output);

    _status->read(input);
    _current->read(input);
    //_feedback->read(input);

}


/** declaration of controller, maintance some slaves */
class Controller:public RECAT{
private:
    std::vector<Slave*> motors;
    StopWatch watch;
public:
    CmdPaser parser;
    Controller(const std::string& mac, int slave_num = 1 );
    ~Controller();

    inline int slave_size()const
    {
        return motors.size();
    }
    void seconds ();
    void process (uint8_t * input, uint8_t * output);

    int get_limit(int slave_index, uint8_t& flag);
    int set_limit(int slave_index, uint8_t flag);
    int get_pid(int slave_index, double &p, double& i, double& d);
    int set_pid(int slave_index, double  p, double  i, double  d);
    int save_eeprom(int slave_index);
};
int Controller::get_limit(int slave_index, uint8_t& flag){
    return this->read(slave_index, 0x9150, 1, &flag, 1);
}
int Controller::set_limit(int slave_index, uint8_t flag){
    return this->write(slave_index, 0x9060, 1, &flag, 1);
}
int Controller::get_pid(int slave_index, double &p, double& i, double& d){
    uint8_t data[4];
    return this->write(slave_index, 0x9060, 1, data, 1);
}
int Controller::set_pid(int slave_index, double p, double i, double d){
    uint8_t data[4];
    return this->write(slave_index, 0x9060, 4, data, 1);
}
int Controller::save_eeprom(int slave_index){
    uint8_t data = 1;
    return this->write(slave_index, 0x9060, 1, &data, 1);
}

Controller::Controller(const std::string &mac, int slave_num)
    : RECAT(mac), parser( slave_num ){
    this->parser.set_master(this);
    for( int i = 0; i < slave_num; i++){
        Slave* motor = new Slave(i);

        PDO_CONFIG* in  = pdo_config( 0x1A00, motor->_status, motor->_current, NULL);//motor->_feedback ,NULL);
        PDO_CONFIG* out = pdo_config( 0x1601, motor->_times,  motor->_target, NULL);//motor->_user_cmd ,NULL);


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
    //this->config_dc( slave_num - 1, 0x0700, 1000000000L/1000, 4400000, 0, 0);
    watch.start();
}
Controller::~Controller(){
    for(int index = 0; index < motors.size(); index ++)
        if(motors[index] != NULL)
            delete motors[index];
}
void Controller::seconds(){
    /*for(int index = 0; index < motors.size(); index ++)
        if(motors[index] != NULL)
        {
            printf("%d t=%x tag=%x ", index, motors[index]->times(), motors[index]->target() );
            printf("stat=%x cur=%x \n", motors[index]->status(), motors[index]->current() );
        }
    */
}
void Controller::process(uint8_t *input, uint8_t *output){
    static unsigned int tick = 0;
    for(int index = 0; index < motors.size(); index ++)
        if(motors[index] != NULL)
        {
            data_t data;
            data.tick += tick++;
            data.time = motors[index]->_times->data;
            data.target = motors[index]->_target->data;
            data.status = motors[index]->_status->data;
            data.current = motors[index]->_current->data;
            parser.push(index, data);
            
            cmd_t cmd = parser.front(index);
            if (cmd.type == OPENED_LOOP){
                motors[index]->_times->data = cmd.time;
                motors[index]->_target->data = cmd.target;
                parser.pop(index);
            }
            else if(cmd.type == CLOSED_POSITION_LOOP){
                motors[index]->_times->data = cmd.time;
                motors[index]->_target->data = cmd.target;//error * parser.kp() +;
                if( abs(motors[index]->_current->data - cmd.target) < 2 )
                    parser.pop(index);
            }
            else if(cmd.type == CLOSED_CIRCLE_LOOP){
                motors[index]->_times->data = 5;
                motors[index]->_target->data = this->parser.circle(index);
                //parser.pop(index);
            }
            else if(cmd.type == CLOSED_SYNC_LOOP)
            {
                motors[index]->_times->data = 5;
                motors[index]->_target->data = this->parser.sync(index);
                //parser.pop(index);
            }
            else if(cmd.type == CLOSED_SYNC_HOME)
            {
                motors[index]->_times->data = 5;
                motors[index]->_target->data = this->parser.sync_home(index);
                if(this->parser.homeX && this->parser.homeY)
                    parser.pop(index);
            }

            motors[index]->process(input, output);
            
            
        }
    
    
    //this->set_time(watch.get_time());
    //this->dc_refresh();
}

#include "tclap/CmdLine.h"
int main(int argc, char** argv){
    struct sched_param param;

    /* Declare ourself as a real time task */
    param.sched_priority = 80;
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
        exit(-1);
    }

    /* Lock memory */
    mlockall(MCL_CURRENT);
    //const int stack_size = 1024*1024*96;
    //if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
    //    perror("mlockall failed");
    //    exit(-2);
    //}
   // unsigned char dummy[stack_size];
   // memset(dummy, 0, stack_size);

    int slave_num = 1;
    try {
        TCLAP::CmdLine args("Command description message", ' ', "1.0");

        TCLAP::ValueArg<int> _slave_num("s", "salve_num", "number of slave",
                                        false, 1, "");
        args.add( _slave_num );
        args.parse( argc, argv );
        slave_num = _slave_num.getValue();

    } catch (TCLAP::ArgException &e)
    {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }

    Controller c("", slave_num);
    c.start(2000);
    
    while(1)
    {
        fprintf(stderr,"RECAT MASTER# ");
        c.parser.parse();
    }
}
