#ifndef __TIMER_H__
#define __TIMER_H__
#include "platform.h"
#ifdef POSIX
#include <signal.h>
#include <time.h>
#include <pthread.h>
#endif

struct Callback
{
    virtual void run() = 0;
};
class EXPORT Timer
{
public:
    Callback* callback_obj;
private:
#ifdef POSIX
    timer_t timer_handle; /* Contains the ID of the created timer */
#endif
public:
    Timer();
    ~Timer();
    void set_callback(Callback* callback_obj);
    void set_interval(int us, int last_count);
    void start();
    void stop();
    void pause();
};

class EXPORT StopWatch
{
public:
    StopWatch(void);
    ~StopWatch(void);
private:
#ifdef POSIX
    struct timespec begin_ticks;
    struct timespec end_ticks;
#else
    LARGE_INTEGER beginticks;
    LARGE_INTEGER endticks ;
    LARGE_INTEGER  frequency;
#endif
public:
    void start();
    void stop();
    long long get_time();//return in ns
};

/*
void CALLBACK __callback__(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dwl, DWORD dw2);

BOOL Timer::start( UINT cycTime, Callback* callback )
{
    TIMECAPS   tc;

    if(timeGetDevCaps(&tc,sizeof(TIMECAPS))!=TIMERR_NOERROR)
        return false;

    u_TimerRes = min(max(tc.wPeriodMin,1),tc.wPeriodMax);

    timeBeginPeriod(u_TimerRes);

    u_TimerID = timeSetEvent(cycTime,  u_TimerRes, (LPTIMECALLBACK)__callback__, (DWORD_PTR) callback, TIME_PERIODIC);

    if(u_TimerID == 0)
        return FALSE;

    return TRUE;
}
Timer::Timer()
{
    u_TimerID = NULL;
}
Timer::~Timer()
{
    if(u_TimerID )
    {
        timeKillEvent(u_TimerID);
        timeEndPeriod(u_TimerRes);
    }
}

void CALLBACK __callback__(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dwl, DWORD dw2)
{
    Callback* obj = (Callback*) dwUser;
    obj->run();
}
*/
#endif
