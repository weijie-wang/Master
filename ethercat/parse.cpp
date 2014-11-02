#include "parse.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include "tclap/CmdLine.h"
#include <stdlib.h>
#include <time.h> 

std::vector<std:: string> split(std:: string str,std:: string pattern)
{
    std:: string::size_type pos;
    std::vector<std:: string> result;
    str+=pattern; // 扩展字符串以方便操作
    int size=str.size();

    for( int i= 0; i<size; i++)
    {
        pos=str.find(pattern,i);
        if(pos<size)
        {
            std:: string s=str.substr(i,pos-i);
            result.push_back(s);
            i=pos+pattern.size()- 1;
        }
    }
    return result;
}
void CmdPaser::trigger_empty(int n){
    if(this->repeat_times <= 0)
        return;
    /* locked in the called function*/
    this->cmds[n] = this->last_cmds[n];
    this->repeat_times--;
}
CmdPaser::CmdPaser(int n){
    cmds.resize(n);
    io_datas.resize(n);
    last_cmds.resize(n);
    for(int i = 0; i < n; i++)
        slave_index.push_back(i);
    x_index = 0;
    y_index = 1;
    z_index = 2;
    max_data_size = 100;
    
    repeat_times = 0;

    _kp = 1.0;
    _ki = 0;
    _kd = 0;
    RECAT* master = NULL;


    radius = 100000;
    circle_x = 300000;
    circle_y = 300000;
    velocity = 1;


    Axis_point Setaxis_star_end;
    long long Length = 0;
    for(int i = 0; i < this->slave_index.size(); i++){
        Setaxis_star_end.start = 0;
        Setaxis_star_end.end = 0;
        Setaxis_star_end.Vector = Setaxis_star_end.end - Setaxis_star_end.start;
        Length += pow(Setaxis_star_end.Vector , 2);
        this->line_axis.push_back(Setaxis_star_end);
    }
    Length = sqrt(Length);
    Setaxis_star_end.end = Setaxis_star_end.Vector = Setaxis_star_end.start = (int)Length;
    this->line_axis.push_back(Setaxis_star_end);

    Length = 0;
    int velocity_now = 0;
    for(int i = 0; i < this->slave_index.size(); i++){
        velocity_now = 0;
        Length += pow(velocity_now , 2);
        this->axis_velocity.push_back(velocity_now);
    }
    Length = sqrt(Length);
    this->axis_velocity.push_back(Length);


    lastX = 0;
    lastY = 0;
    homeX = false;
    homeY = false;

}
void CmdPaser::setup()
{
    uint8_t data = 0;
    uint8_t ldata = 1;
    master->write(0, 0x9060, 1, &data, 1);
    master->write(0, 0x9060, 2, &ldata, 1);
    master->write(0, 0x9060, 3, &ldata, 1);
    master->write(1, 0x9060, 2, &ldata, 1);
    master->write(1, 0x9060, 3, &ldata, 1);
    master->write(2, 0x9060, 2, &ldata, 1);
    master->write(2, 0x9060, 3, &ldata, 1);
}
int  CmdPaser::enable()
{
    std::vector< int > wrong_num;
    this->repeat_times = 0;
    std::queue< cmd_t > cmd_queue;
    cmd_queue.push(cmd_t::enable());
    this->cmds_lock.lock();
    this->last_cmds_lock.lock();
    for(int i = 0; i < this->slave_index.size(); i++){
        this->cmds[ this->slave_index[i] ] = cmd_queue;
        this->last_cmds[ this->slave_index[i] ] = cmd_queue;
    }
    this->last_cmds_lock.unlock();
    this->cmds_lock.unlock();
    for(int time_num = 0 ; time_num <=3000 ;time_num++ )
    {
        bool flag = true;
        wrong_num.clear();
        this->io_datas_lock.lock();
        for(int i = 0 ; i < this->slave_index.size(); i++){
            if(this->io_datas[i].empty() )
                flag = false;
            else if( (this->io_datas[i].back().status & 0x00001000) == 0){
                wrong_num.push_back(i);
                flag = false;
            }
        }
        this->io_datas_lock.unlock();
        if(flag == true){
            std::cout<<"[ok]: Enable at time = "<< this->slave_index.size() << time_num<<std::endl;
            return 0;
        }
        usleep(1000);

    }
    std::cout<<"[ERROR]: Slave ";
    for(std::vector< int >::iterator m = wrong_num.begin() ; m != wrong_num.end() ; m++ ){
        std::cout<<*m<<" ";
    }
    std::cout<<"can not enable "<<std::endl;
    return -1;
}
int  CmdPaser::disable()
{
    std::vector< int > wrong_num;
    this->repeat_times = 0;
    std::queue< cmd_t > cmd_queue;
    cmd_queue.push(cmd_t::disable());
    this->cmds_lock.lock();
    this->last_cmds_lock.lock();
    for(int i = 0; i < this->slave_index.size(); i++){
        this->cmds[ this->slave_index[i] ] = cmd_queue;
        this->last_cmds[ this->slave_index[i] ] = cmd_queue;
    }
    this->last_cmds_lock.unlock();
    this->cmds_lock.unlock();

    for(int time_num = 0 ; time_num <=10000 ;time_num++ )
    {
        bool flag = true;
        wrong_num.clear();

        this->io_datas_lock.lock();
        for(int i = 0 ; i < this->slave_index.size(); i++){
            if(this->io_datas[i].empty() )
                flag = false;
            else if((this->io_datas[i].back().status & 0x00001000) != 0){
                wrong_num.push_back(i);
                flag = false;
            }
        }
        this->io_datas_lock.unlock();
        if(flag == true){
            std::cout<<"[ok]: Disable"<<std::endl;
            return 0;
        }
        usleep(100);
    }
    std::cout<<"[ERROR]: Slave ";
    for(std::vector< int >::iterator m = wrong_num.begin() ; m != wrong_num.end() ; m++ ){
        std::cout<<*m<<" ";
    }
    std::cout<<"can not disable "<<std::endl;
    return -1;
}
void CmdPaser::demo()
{
    long long time = 0;
    while(1)
    {
        if(time == 3000)
        {
            std::queue< cmd_t > x_queue, y_queue;
            cmd_t cmd;
            cmd.type = CLOSED_CIRCLE_LOOP;
            x_queue.push(cmd);
            cmd.type = OPENED_LOOP;
            x_queue.push(cmd);
            y_queue = x_queue;

            this->cmds_lock.lock();
            this->last_cmds_lock.lock();
            this->cmds[this->x_index] = x_queue;
            this->last_cmds[this->x_index] = x_queue;
            this->cmds[this->y_index] = y_queue;
            this->last_cmds[this->y_index] = y_queue;
            this->cmds_lock.unlock();
            this->last_cmds_lock.unlock();
        }
        else if(time == 0)
        {
            cmd_t cmd;
            std::queue< cmd_t > cmd_queue;
            cmd.time = 5;
            cmd.target = 10;
            cmd_queue.push(cmd);

            this->cmds_lock.lock();
            this->last_cmds_lock.lock();
            for(int i = 0; i < this->slave_index.size(); i++){
               this->cmds[ this->slave_index[i] ] = cmd_queue;
               this->last_cmds[ this->slave_index[i] ] = cmd_queue;
            }
            this->last_cmds_lock.unlock();
            this->cmds_lock.unlock();
        }
        usleep(1000);
        time ++;
        if((time % 100) == 0)
            std::cout<<time<<"ms"<<std::endl;
    }

}
void CmdPaser::demo_line(){
    line_setstart_now();
    int end[3] = {100000,100000,40000};
    line_setend( end );

    std::queue< cmd_t > x_queue, y_queue,z_queue;
    cmd_t cmd;
    cmd.type = CLOSED_LINE_LOOP;
    x_queue.push(cmd);
    y_queue = x_queue;
    z_queue = x_queue;

    this->cmds_lock.lock();
    this->last_cmds_lock.lock();
    this->cmds[this->x_index] = x_queue;
    this->last_cmds[this->x_index] = x_queue;
    this->cmds[this->y_index] = y_queue;
    this->last_cmds[this->y_index] = y_queue;
    this->cmds[this->z_index] = z_queue;
    this->last_cmds[this->z_index] = z_queue;
    this->cmds_lock.unlock();
    this->last_cmds_lock.unlock();
}
CmdPaser::~CmdPaser(){
}
void CmdPaser::parse(){
    std::string cmdline;
    std::getline(std::cin, cmdline);


    std::queue< cmd_t > cmd_queue;

#define CMD_SET(x) else if( cmdline.find(#x)==0 ) { this->repeat_times = 0; cmd_queue.push(cmd_t::x()); }
#define PARA_BEGIN TCLAP::CmdLine parser("", ' ', "",false)
#define PARA(type, name, c, req) TCLAP::ValueArg<type> name##_arg(#c, #name, #name, req, name, #type ); parser.add(name##_arg)
#define PARA_MULTI(type, name, c, req) TCLAP::MultiArg<type> name##_arg(#c,#name, #name, req, #type ); parser.add(name##_arg)
#define PARA_SWITCH(name,c, req) TCLAP::SwitchArg      name##_arg(#c, #name, #name, req); parser.add(name##_arg)
#define PARA_END std::vector<std::string> argvs = split(cmdline, " "); parser.parse(argvs)
#define PARA_GET(name) name = name##_arg.getValue()

    if(0);
    /* pdo data */
    CMD_SET(disable)
    CMD_SET(enable)
    CMD_SET(disable)
    CMD_SET(enable)
    CMD_SET(stop)
    CMD_SET(emergency)
    CMD_SET(cw_home)
    CMD_SET(cww_home)
    CMD_SET(clear)
    else if(cmdline.find("pos")==0){
        this->repeat_times = 0;
        cmd_t cmd;
        cmd.time = 3;
        if( sscanf(cmdline.c_str(), "%*s %i", &cmd.target) != 1){
            std::cout<<"[ERROR]: pdo command should with target (%i)"<<std::endl;
            return;
        }
        cmd_queue.push(cmd);
    }
    else if(cmdline.find("xy")==0){
        int x,y;
        cmd_t cmd;
        cmd.time = 3;
        if( sscanf(cmdline.c_str(), "%*s %i %i", &x, &y) != 2){
            std::cout<<"[ERROR]: pdo command should with target (%i %i)"<<std::endl;
            return;
        }
        cmd.target = x;
        cmd_queue.push(cmd);

        this->cmds_lock.lock();
        this->cmds[ this->x_index ] = cmd_queue;
        cmd_queue.pop();
        cmd.target = y;
        cmd_queue.push(cmd);
        this->cmds[ this->y_index ] = cmd_queue;
        this->cmds_lock.unlock();
        return;
    }
    else if( cmdline.find("pdo")==0 ){
        this->repeat_times = 0;
        cmd_t cmd;
        if( sscanf(cmdline.c_str(), "%*s %i %i", &cmd.time, &cmd.target) != 2){
            std::cout<<"[ERROR]: pdo command should with time and target (%i %i)"<<std::endl;
            return;
        }
        cmd_queue.push(cmd);
    }
    /* sdo data */
    else if( cmdline.find("scale")==0){
        bool reverse = false;
        PARA_BEGIN;
        PARA_SWITCH(reverse, r, false);
        PARA_END;
        PARA_GET(reverse);

        for(int i = 0; i < this->slave_index.size(); i++){
            int index = this->slave_index[i];
            uint8_t data;
            int ret = this->master->read( index, 0x9060, 3, &data, 1);
            if(ret != 1)std::cerr<<"[ERROR]: slave "<<index<<" scale read error!"<<std::endl;
            else{
                if(reverse){
                    data = !data;
                    this->master->write( index, 0x9060, 3, &data, 1);
                }
                std::cout<<"[RESULT]: slave "<<index<<" scale is "
                                <<((unsigned int)data)<<std::endl;
            }
        }
        return;
    }
    else if( cmdline.find("encoder")==0){
        bool reverse = false;
        PARA_BEGIN;
        PARA_SWITCH(reverse, r, false);
        PARA_END;
        PARA_GET(reverse);

        for(int i = 0; i < this->slave_index.size(); i++){
            int index = this->slave_index[i];
            uint8_t data;
            int ret = this->master->read( index, 0x9150, 2, &data, 1);
            if(ret != 1)std::cerr<<"[ERROR]: slave "<<index<<" encoder read error!"<<std::endl;
            else{
                if(reverse){
                    data = !data;
                    this->master->write( index, 0x9050, 2, &data, 1);
                }
                std::cout<<"[RESULT]: slave "<<index<<" encoder is "
                                <<((unsigned int)data)<<std::endl;
            }
        }
        return;
    }
    else if( cmdline.find("limit")==0){
        bool level = false;
        bool reverse = false;
        PARA_BEGIN;
        PARA_SWITCH(reverse, r, false);
        PARA_SWITCH(level, l, false);
        PARA_END;
        PARA_GET(reverse);
        PARA_GET(level);

        for(int i = 0; i < this->slave_index.size(); i++){
            int index = this->slave_index[i];
            uint8_t data;
            uint8_t ldata;
            int ret = this->master->read( index, 0x9060, 1, &data, 1);
            ret = this->master->read( index, 0x9060, 2, &ldata, 1);
            if(ret != 1)std::cerr<<"[ERROR]: slave "<<index<<" limit read error!"<<std::endl;
            else{
                if(reverse){
                    data = !data;
                    this->master->write( index, 0x9060, 1, &data, 1);
                }
                if(level){
                    ldata = !ldata;
                    this->master->write( index, 0x9060, 2, &ldata, 1);
                }
                std::cout<<"[RESULT]: slave "<<index<<" limit is "
                                <<((unsigned int)data) << " level is "<<(int)ldata<<std::endl;
            }
        }
        return;
    }
    else if( cmdline.find("save")==0){
        for(int i = 0; i < this->slave_index.size(); i++){
            int index = this->slave_index[i];
            uint8_t data = 1;
            int ret = this->master->write( index, 0x9070, 1, &data, 1);
            if(ret != 1)std::cerr<<"[ERROR]: slave "<<index<<" save error!"<<std::endl;
        }
        return;
    }
    else if( cmdline.find("read")==0){
        int index, subindex, data_size;
        if( sscanf(cmdline.c_str(), "%*s %i %i %i", &index, &subindex, &data_size) != 3){
            std::cout<<"[ERROR]: read command should with index, subindex and data_size (%i %i %i)"<<std::endl;
            return;
        }

        for(int i = 0; i < this->slave_index.size(); i++){
            int slave = this->slave_index[i];
            uint8_t* data = new uint8_t[data_size];
            int ret = this->master->read( slave, index, subindex, data, data_size);
            if(ret != data_size)std::cerr<<"[ERROR]: slave "<<slave<<" read error!"<<std::endl;
            else{
                std::cout<<"[RESULT]: slave "<<slave<<
                           " read at "<<std::hex<<index<<":"<<subindex<<":";
                for(int j = 0; j < data_size; j++)
                    fprintf(stdout, " 0X%02x", data[j]);
                std::cout<<std::endl<<std::dec;
            }
            delete[] data;
        }
        return;
    }
    else if( cmdline.find("write")==0){
        int index, subindex, data_size = 0;
        uint8_t data[128];
        char data_assci[256];
        if( sscanf(cmdline.c_str(), "%*s %i %i %s", &index, &subindex, data_assci) != 3){
            std::cout<<"[ERROR]: write command should with index, subindex and data in hex(%i %i %[^\n])"<<std::endl;
            return;
        }

        for(int j = 0; j < strnlen(data_assci, 128) - 1; j+=2, data_size++){
            data[data_size] = 16 * (data_assci[j] - '0') + (data_assci[j + 1] - '0');
        }
        for(int i = 0; i < this->slave_index.size(); i++){
            int slave = this->slave_index[i];

            int ret = this->master->write( slave, index, subindex, data, data_size);
            if(ret != data_size)std::cerr<<"[ERROR]: slave "<<slave<<" write error!"<<std::endl;
            else{
                std::cout<<"[RESULT]: slave "<<slave<<" write successfully"<<std::endl;
            }
        }
        return;
    }
    /* master parameter setup */
    else if( cmdline.find("set ")==0){
        std::vector<int> selected;
        PARA_BEGIN;
        PARA_MULTI(int, selected, s, false);
        PARA(int, x_index, x, false);
        PARA(int, y_index, y, false);
        PARA(int, max_data_size, m, false);
        PARA_END;
        PARA_GET(selected);
        PARA_GET(x_index);
        PARA_GET(y_index);
        PARA_GET(max_data_size);
        
        if(selected.empty())
            return;
            
        std::cout<<"[RESULT]: slaves ";
        this->slave_index.clear();
        for( int i = 0; i < selected.size(); i++){
            int index = selected[i];
            if (index >= 0 && index < this->cmds.size()){
                this->slave_index.push_back(index);
                std::cout<<index<<" ";
            }
        }
        std::cout<<"selected"<<std::endl;
        return;
    }
    else if( cmdline.find("exit")==0){
        exit(0);
        return;
    }
    else if( cmdline.find("display")==0 || cmdline.find("p")==0){
        std::string filename;
        bool history = false;
        PARA_BEGIN;
        PARA(std::string, filename, f, false);
        PARA_SWITCH( history, t, false);
        PARA_END;
        PARA_GET(filename);
        PARA_GET(history);

        std::vector< std::queue<data_t> > output;
        output.resize(this->io_datas.size());

        this->io_datas_lock.lock();
        for(int i = 0; i < this->slave_index.size(); i++){
            int index = this->slave_index[i];
            if(history)
                output[index] = this->io_datas[index];
            else
                output[index].push(this->io_datas[index].back());
        }
        this->io_datas_lock.unlock();

        std::ostream* os = &std::cout;
        if(!filename.empty())
            os = new std::ofstream(filename.c_str());
        for(int i = 0; i < this->slave_index.size(); i++){
            int index = this->slave_index[i];

            *os<<"[RESULT]: slave "<<index<<" IO DATAS:"<<std::endl;
            while(!output[index].empty()){
                *os<<"T="<<output[index].front().tick<<"\t"
                         <<"time="<<output[index].front().time << "\t"
                         <<"target="<<output[index].front().target << "\t"
                         <<"status="<<output[index].front().status << "\t"
                         <<"current="<<output[index].front().current << std::endl;
                output[index].pop();
            }
        }
        if(!filename.empty())
            delete os;
        return;
    }
    /* demostation */
    else if( cmdline.find("stype")==0){
        double acc_acc = 1;
        repeat_times = 0;
        PARA_BEGIN;
        PARA( double, acc_acc, a, false);
        PARA( int, repeat_times, p, false);
        PARA_END;
        PARA_GET(acc_acc);
        PARA_GET(repeat_times);

        std::cout<<"repated "<<repeat_times<<" times, with acc_acc equal to "<<acc_acc<<std::endl;
        cmd_t cmd;
        cmd.time = 5;
        double region[7] = {acc_acc, 0, -acc_acc, 0, -acc_acc, 0, acc_acc};
        double a = 0, v = 0;
        for(int t = 0; t < 7000; t++){
            double aa = region[ t/1000 ];
            a += aa;
            v += a;
            cmd.target = v/2000;
            cmd_queue.push(cmd);
        }
        a = 0; v = 0;
        for(int t = 0; t < 7000; t++){
            double aa = -region[ t/1000 ];
            a += aa;
            v += a;
            cmd.target = v/2000;
            cmd_queue.push(cmd);
        }
    }
    else if( cmdline.find("circle")==0 ){
        if(this->x_index >= this->cmds.size() || this->y_index >= this->cmds.size()){
            std::cerr<<"[ERROR]: to run circle, the x and y axis should be specified to right slave"<<std::endl;
            return;
        }
        repeat_times = 0;
        PARA_BEGIN;
        PARA( int, radius, r, false );
        PARA( int, circle_x, x, false );
        PARA( int, circle_y, y, false );
        PARA( int, repeat_times, p, false );
        PARA( double, velocity, w, false);
        PARA_END;
        PARA_GET(radius);
        PARA_GET(circle_x);
        PARA_GET(circle_y);
        PARA_GET(repeat_times);
        PARA_GET(velocity);

        std::queue< cmd_t > x_queue, y_queue;
        cmd_t cmd;
        cmd.type = CLOSED_CIRCLE_LOOP;
        x_queue.push(cmd);
        cmd.type = OPENED_LOOP;
        x_queue.push(cmd);
        y_queue = x_queue;

        this->cmds_lock.lock();
        this->last_cmds_lock.lock();
        this->cmds[this->x_index] = x_queue;
        this->last_cmds[this->x_index] = x_queue;
        this->cmds[this->y_index] = y_queue;
        this->last_cmds[this->y_index] = y_queue;
        this->cmds_lock.unlock();
        this->last_cmds_lock.unlock();
        return;
    }
    else if( cmdline.find("sync")==0 ){
        if(this->x_index >= this->cmds.size() || this->y_index >= this->cmds.size()){
            std::cerr<<"[ERROR]: to run circle, the x and y axis should be specified to right slave"
                     <<std::endl;
            return;
        }
        repeat_times = 0;
        velocity = 100;
        PARA_BEGIN;
        PARA( int, repeat_times, p, false );
        PARA( double, velocity, w, false);
        PARA_END;
        PARA_GET(repeat_times);
        PARA_GET(velocity);
        std::cerr<< "velocity: " << velocity<<std::endl;

        std::queue< cmd_t > x_queue, y_queue;
        cmd_t cmd;
        cmd.type = CLOSED_SYNC_LOOP;
        x_queue.push(cmd);
        cmd.type = OPENED_LOOP;
        x_queue.push(cmd);
        y_queue = x_queue;

        this->cmds_lock.lock();
        this->last_cmds_lock.lock();
        this->cmds[this->x_index] = x_queue;
        this->last_cmds[this->x_index] = x_queue;
        this->cmds[this->y_index] = y_queue;
        this->last_cmds[this->y_index] = y_queue;
        this->cmds_lock.unlock();
        this->last_cmds_lock.unlock();
        return;
    }
    else if( cmdline.find("home")==0 ){
        if(this->x_index >= this->cmds.size() || this->y_index >= this->cmds.size()){
            std::cerr<<"[ERROR]: to run circle, the x and y axis should be specified to right slave"<<std::endl;
            return;
        }

        std::queue< cmd_t > x_queue, y_queue;
        cmd_t cmd;
        cmd.type = CLOSED_SYNC_HOME;
        x_queue.push(cmd);
        y_queue = x_queue;

        this->cmds_lock.lock();
        this->last_cmds_lock.lock();
        this->cmds[this->x_index] = x_queue;
        this->last_cmds[this->x_index] = x_queue;
        this->cmds[this->y_index] = y_queue;
        this->last_cmds[this->y_index] = y_queue;
        this->cmds_lock.unlock();
        this->last_cmds_lock.unlock();
        return;
    }
    else if(cmdline.find("cfg")==0)
    {
        setup();
        return;
    }
    else{
        std::cerr<<"[ERROR]: input the right command, stop all the motor"<<std::endl;
        this->repeat_times = 0; cmd_queue.push(cmd_t::stop());
        this->cmds_lock.lock();
        this->last_cmds_lock.lock();
        for(int i = 0; i < this->cmds.size(); i++){
            this->cmds[ this->slave_index[i] ] = cmd_queue;
            this->last_cmds[ this->slave_index[i] ] = cmd_queue;
        }
        this->last_cmds_lock.unlock();
        this->cmds_lock.unlock();
        return;
    }

    this->cmds_lock.lock();
    this->last_cmds_lock.lock();
    for(int i = 0; i < this->slave_index.size(); i++){
        this->cmds[ this->slave_index[i] ] = cmd_queue;
        this->last_cmds[ this->slave_index[i] ] = cmd_queue;
    }
    this->last_cmds_lock.unlock();
    this->cmds_lock.unlock();
}
cmd_t CmdPaser::front(int n){
    cmd_t cmd;
    this->cmds_lock.lock();
    if(n < 0 || n > this->cmds.size() || this->cmds[n].empty() ){
        this->io_datas_lock.lock();
        if(!this->io_datas[n].empty()){
            cmd.time = this->io_datas[n].back().time;
            cmd.target = this->io_datas[n].back().target;
        }
        this->io_datas_lock.unlock();
    }
    else{
        cmd = this->cmds[n].front();
    }
    this->cmds_lock.unlock();
    return cmd;
}
void CmdPaser::pop(int n){
    this->cmds_lock.lock();
    //for(int n = 0; n < this->cmds.size(); n ++)
    if(this->cmds[n].empty()){
        this->last_cmds_lock.lock();
        this->trigger_empty(n);
        this->last_cmds_lock.unlock();
    }
    else
        this->cmds[n].pop();
    this->cmds_lock.unlock();
}
void CmdPaser::push(int n, data_t data){
    this->io_datas_lock.lock();
    while(this->io_datas[n].size() > this->max_data_size)
        this->io_datas[n].pop();
    this->io_datas[n].push(data);
    this->io_datas_lock.unlock();
}
void CmdPaser::set_master(RECAT* t){
    this->master = t;
}
int CmdPaser::line(int index){
    velocity = 50000;
    double Length;
    std::vector<double> Axis_now;
    double axis_now;
    std::vector< axis_point >::iterator m = this->line_axis.begin();
    std::vector< double >::iterator n = Axis_now.begin();
    this->io_datas_lock.lock();
    for(int i = 0; i < this->slave_index.size(); i++){
        axis_now = (*m).Vector- this->io_datas[i].back().current;
        Length += pow(axis_now , 2);
        Axis_now.push_back(axis_now);
        m++;
    }
    this->io_datas_lock.unlock();
    Length = sqrt(Length);
    n = Axis_now.begin() + index;
    m = this->line_axis.begin() + index;
    double x = (velocity / Length) * (*n);
    if( abs( this->io_datas[index].back().current - (*m).end) <= 1000)
        x = ( abs( this->io_datas[index].back().current - (*m).end) / velocity ) * x;
    x = x/20;
    std::vector< int >::iterator l = this->axis_velocity.begin() + index;
    (*l) = (int)x;
    Length = 0;
    for(std::vector< int >::iterator i = this->axis_velocity.begin(); i != this->axis_velocity.end() - 1; i++){
        Length += pow((*i) , 2);
    }
    Length = sqrt(Length);
    *this->axis_velocity.end() = Length;
    int x_data = (int)x;
    return x_data;

}
void CmdPaser::line_print(){
    this->io_datas_lock.lock();
    std::cout<<"Position now: [ ";
    for(int i = 0; i < this->slave_index.size(); i++){
        std::cout<<this->io_datas[i].back().current<<" ";
    }
    std::cout<<"]";
    this->io_datas_lock.unlock();
    std::cout<<"  Destination: [ ";
    for(std::vector< axis_point >::iterator m = this->line_axis.begin(); m != this->line_axis.end() - 1; m++){
        std::cout<<(*m).end<<" ";
    }
    std::cout<<"]";
    std::cout<<"  Axis speed: [ ";
    for(std::vector< int >::iterator m = this->axis_velocity.begin(); m != this->axis_velocity.end() - 1; m++){
        std::cout<<(*m)<<" ";
    }
    std::cout<<*axis_velocity.end()<<" ]"<<std::endl;
}
int CmdPaser::line_setstart(int start[]){
    Axis_point Setaxis_star_end;
    long long Length = 0;
    std::vector< axis_point >::iterator m = this->line_axis.begin();
    for(int i = 0; i < this->slave_index.size(); i++){
        Setaxis_star_end.start = start[i];
        Setaxis_star_end.Vector = Setaxis_star_end.end - Setaxis_star_end.start;
        Length += pow(Setaxis_star_end.Vector , 2);
        (*m) = Setaxis_star_end;
        m++;
    }
    Length = sqrt(Length);
    Setaxis_star_end.end = Setaxis_star_end.Vector = Setaxis_star_end.start = (int)Length;
    *this->line_axis.end() = Setaxis_star_end;
    std::cout<<"[ok] Set start: [";
    for(std::vector< axis_point >::iterator m = this->line_axis.begin(); m != this->line_axis.end() - 1; m++){
        std::cout<<(*m).start<<" ";
    }
    std::cout<<"]"<<std::endl;
}
int CmdPaser::line_setend(int end[]){
    Axis_point Setaxis_star_end;
    long long Length = 0;
    std::vector< axis_point >::iterator m = this->line_axis.begin();
    for(int i = 0; i < this->slave_index.size(); i++){
        Setaxis_star_end.end = end[i];
        Setaxis_star_end.Vector = Setaxis_star_end.end - Setaxis_star_end.start;
        Length += pow(Setaxis_star_end.Vector , 2);
        (*m) = Setaxis_star_end;
        m++;
    }
    Length = sqrt(Length);
    Setaxis_star_end.end = Setaxis_star_end.Vector = Setaxis_star_end.start = (int)Length;
    *this->line_axis.end() = Setaxis_star_end;
    std::cout<<"[ok] Set end: [";
    for(std::vector< axis_point >::iterator m = this->line_axis.begin(); m != this->line_axis.end() - 1; m++){
        std::cout<<(*m).end<<" ";
    }
    std::cout<<"]"<<std::endl;
}
int CmdPaser::line_setstart_now(){
    Axis_point Setaxis_star_end;
    long long Length = 0;
    this->io_datas_lock.lock();
    std::vector< axis_point >::iterator m = this->line_axis.begin();
    for(int i = 0; i < this->slave_index.size(); i++){
        Setaxis_star_end.start = this->io_datas[i].back().current;
        Setaxis_star_end.Vector = Setaxis_star_end.end - Setaxis_star_end.start;
        Length += pow(Setaxis_star_end.Vector , 2);
        (*m) = Setaxis_star_end;
        m++;
    }
    this->io_datas_lock.unlock();
    Length = sqrt(Length);
    Setaxis_star_end.end = Setaxis_star_end.Vector = Setaxis_star_end.start = (int)Length;
    *this->line_axis.end() = Setaxis_star_end;
    std::cout<<"[ok] Set start: [";
    for(std::vector< axis_point >::iterator m = this->line_axis.begin(); m != this->line_axis.end() - 1; m++){
        std::cout<<(*m).start<<" ";
    }
    std::cout<<"]"<<std::endl;
}
int CmdPaser::line_setend_now(){
    Axis_point Setaxis_star_end;
    long long Length = 0;
    std::vector< axis_point >::iterator m = this->line_axis.begin();
    for(int i = 0; i < this->slave_index.size(); i++){
        Setaxis_star_end.end = this->io_datas[i].back().current ;
        Setaxis_star_end.Vector = Setaxis_star_end.end - Setaxis_star_end.start;
        Length += pow(Setaxis_star_end.Vector , 2);
        (*m) = Setaxis_star_end;
        m++;
    }
    Length = sqrt(Length);
    Setaxis_star_end.end = Setaxis_star_end.Vector = Setaxis_star_end.start = (int)Length;
    *this->line_axis.end() = Setaxis_star_end;
    std::cout<<"[ok] Set end: [";
    for(std::vector< axis_point >::iterator m = this->line_axis.begin(); m != this->line_axis.end() - 1; m++){
        std::cout<<(*m).end<<" ";
    }
    std::cout<<"]"<<std::endl;
}
int CmdPaser::circle(int index){
    double x = this->io_datas[x_index].back().current - this->circle_x;
    double y = this->io_datas[y_index].back().current - this->circle_y;
    double t = atan2(y,x);
    double e = hypot(x,y) - this->radius;
    e = e/this->radius;

    double vx = -sin(t + e) * this->radius;
    double vy = cos(t + e) * this->radius;
    if(index == this->x_index){
        return vx/20;
    }
    else if(index == this->y_index){
        return vy/20;
    }
    else
        return 0;
}
int CmdPaser::sync(int index){
    static int tick = 0;
    
    if(tick++ >= 1000)
    {
        tick = 0;
        static int lastDelta = 0;
        int delta = lastDelta * 0.9 + (rand() % 200 - 100) * 10;
        lastDelta = delta;
        velocity += delta;
        if(velocity > 16000 )
        {
            velocity = 16000;
            lastDelta = - 500;
        }
        else if(velocity < -16000 )
        {
            velocity = -16000;
            lastDelta = 500;
        }
    }    
    double x = this->io_datas[x_index].back().current;
    double y = this->io_datas[y_index].back().current;
	x = x - 2135 + 869;
    x = x * 2 * 3.141592653 / 10000.0;
    y = y * 2 * 3.141592653 / 10000.0;
    y =  - y;
    double e = (x - y) ;//rad
	while(e > 3.141592653)
		e -= 2 * 3.141592653;
	while(e < -3.141592653)
		e += 2 * 3.141592653;
    double deltaW = e / 2.0 / (0.0005 * 2 * 110);
    deltaW /= (0.005*3.141592653);
    
    if(index == this->x_index){
        int v = velocity - deltaW;
        if(v > 20000) v = 20000;
        if(v < -20000) v = -20000;
        return v;
    }
    else if(index == this->y_index){
        int v = -(velocity + deltaW);
        if(v > 20000) v = 20000;
        if(v < -20000) v = -20000;
        return v;
    }
    else
        return 0;
}
int CmdPaser::sync_home(int index)
{
    double deltaW = 0 / 2.0 / 0.001 / 2 / 8;
    deltaW /= (0.005*3.141592653);
    
    if(index == this->x_index){
        return 100 - deltaW;
    }
    else if(index == this->y_index){
        return -(100 + deltaW);
    }
    else
        return 0;
}
