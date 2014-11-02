#ifndef __PARSE_H__
#define __PARSE_H__
#include <vector>
#include <string>
#include <queue>
#include "recat.h"
#include "thread.hpp"
enum cmd_type{
    OPENED_LOOP,
    CLOSED_VELOCITY_LOOP,
    CLOSED_POSITION_LOOP,
    CLOSED_CIRCLE_LOOP,
    CLOSED_SYNC_LOOP,
    CLOSED_SYNC_HOME,
    CLOSED_LINE_LOOP
};

#define CMD_TYPE(t, t1, t2)     \
    inline static cmd_t t(){    \
    cmd_t ret;                  \
    ret.time = t1;              \
    ret.target = t2;            \
    ret.type = OPENED_LOOP;     \
    return ret;                 \
    }

struct data_t{
    unsigned int tick;
    unsigned int time;
    signed int   target;
    unsigned int status;
    signed int   current;
};
typedef struct axis_point{
    int start;
    int end; 
    int Vector;
}Axis_point;

struct cmd_t{
    unsigned int time;
    int target;
    cmd_type type;
    CMD_TYPE(disable,   15, 0)
    CMD_TYPE( enable,   15, 1)
    CMD_TYPE(   stop,    0, 1)
    CMD_TYPE(emergency,  0, 0)
    CMD_TYPE(cw_home,    1, 0)
    CMD_TYPE(cww_home,   2, 0)
    CMD_TYPE(clear, 14, 0x073F)
    inline cmd_t(){
        time = 0;
        target = 0;
        type = OPENED_LOOP;
    }
};

enum PaserCMDType{
    NOP,
    LINE,
    CIRCLE
};
struct PaserCMD
{
    PaserCMDType type;
    union
    {
        struct
        {
            int32_t endx;
            int32_t endy;
            int32_t endz;
            uint32_t speed;
        }line;
        struct
        {
            uint32_t radius;
            uint32_t speed;
            int32_t centerx;
            int32_t centery;
            int32_t angle;
        }circle;
    }para;
};

class CmdPaser
{
private:
    BMutex cmds_lock;
    std::vector< std::queue<cmd_t> > cmds;
    BMutex io_datas_lock;
    std::vector< std::queue<data_t> > io_datas;
    std::vector<int> slave_index;
    int x_index;
    int y_index;
    int z_index;
    int speed;
    int max_data_size;
    BMutex last_cmds_lock;
    std::vector< std::queue<cmd_t> > last_cmds; 
    int repeat_times;
    void trigger_empty(int);

    double _kp;
    double _ki;
    double _kd;
    int cmd_index;
    RECAT* master;
    double velocity;

    int radius, circle_x, circle_y;
    std::vector< axis_point > line_axis;
    std::vector< int > axis_velocity;

    PaserCMD* current_cmd;

public:
    void SetCMD(PaserCMD& cmd);
    int WaitCMD();
    inline double kp(){ return this->_kp; }
    inline double ki(){ return this->_ki; }
    inline double kd(){ return this->_kd; }
    int circle(int index);
    int sync(int index);
    int line(int index);
    int line_setstart(int start[]);
    int line_setend(int end[]);
    int line_setstart_now();
    int line_setend_now();
    void line_print();
private:
    int lastX, lastY;
public:    
    bool homeX;
    bool homeY;
    int sync_home(int index);
    
    CmdPaser(int n);
    ~CmdPaser();
    void parse();
    
    cmd_t front(int n);
    void pop(int n);
    
    void push(int n, data_t data);

    void set_master(RECAT*);
    void setup();
    void demo();
    void demo_line();
    int  enable();
    int  disable();
};

#endif
