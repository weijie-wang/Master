#ifndef __CNC_H__
#define __CNC_H__

#include <vector>
#include <queue>
#include "recat.h"
#include "thread.hpp"

#define CMD_TYPE(name, t1, t2)     \
    inline void name(){    \
    time = t1;              \
    target = t2;            \
    }
	
#define	SUCCESS			 	0	/* no error */
#define	ERROR      	 		-1	/* unspecified error */
#define	ENABLE_ERROR 	 	-2	/* enable error */
#define	PM_NORM_ERR 	 	-3	/* arg should have been norm*/
#define	PM_DIV_ERR  	 	-4	/* divide by zero error */
#define PM_POINTER_ERR		-5	/* pointer is NULL */	



enum CmdType {
	ENABLE,
	DISABLE,
	OPEN_LOOP,
	POINT,
	TEST,
	STOP
};

struct Data {	//slave's data
	unsigned int 	time;
	signed int 	 	target;
	unsigned int 	status;
	signed int 		current;
};

struct CmdData {	//cmd to one axis
	unsigned int time;
	int target;
	CMD_TYPE(disable, 15, 0)
    CMD_TYPE(enable, 15, 1)
    CMD_TYPE(   stop,    0, 1)
    CMD_TYPE(emergency,  0, 0)
    CMD_TYPE(cw_home,    1, 0)
    CMD_TYPE(cww_home,   2, 0)
    CMD_TYPE(clear, 14, 0x073F)
	inline void setspeed(int n){
		time = 5;
		target = n;
	}
	inline CmdData(){
		time = 0;
		target = 0;
	}
};

struct Cmd {	
		CmdType type;
		union {
			struct {
				int32_t end[3];
				uint32_t speed;
			}point;
			struct {
				uint32_t repeat_times;
			}enable;
			struct {
				uint32_t repeat_times;
			}disable;
		}curve;
};

class CNC {
private:
	int error;
	
	BMutex cmds_lock;
	std::queue<Cmd> cmds;						//cmds queue(from user)
	Cmd last_cmd;								//last cmd(from user)
	Cmd current_cmd;							//current cmd(from user)
	
	BMutex axis_cmds_lock;
	std::vector< std::queue<CmdData> > axis_cmds;	//axis cmds queue(from cmds)
	CmdData last_axis_cmd;						//last axis cmd(from cmds)
	
	BMutex slave_data_lock;
	std::vector<Data> slave_data;				//data from slaves (queue)
	std::vector<Data> last_slave_data;			//last data from slaves (queue)
	
	std::vector<unsigned int> slave_index;		//motor number mapping to axis number
public:
	RECAT* master;
	CNC(int n);
	~CNC();
	void setup();
	int set_master(RECAT*);
	int front(int n, CmdData* cmd);
	int set_cmd(Cmd& cmd);
	int wait_cmd();
	int slave_cmds_pop(int n);
	int slave_data_push(int index, Data *data);
	
	long long point_integral;
	float p;
	float i;
};



#endif
