

#include "timer.h"

StopWatch::~StopWatch(){
}
StopWatch::StopWatch(){
    this->start();
}
void StopWatch::start(){
    clock_gettime(CLOCK_REALTIME, & (this->begin_ticks)) ;
}
void StopWatch::stop(){
}
long long StopWatch::get_time(){
    clock_gettime(CLOCK_REALTIME, & (this->end_ticks)) ;
    return (this->end_ticks.tv_sec -  this->begin_ticks.tv_sec) * 1000 *1000*1000 +
           (this->end_ticks.tv_nsec - this->begin_ticks.tv_nsec);
}


static void callback(union sigval in)
{    
    Timer* ptr = (Timer*) (in.sival_ptr);
    ptr->callback_obj->run();
}

Timer::Timer()
{
    struct sigevent sig;

    sig.sigev_value.sival_ptr = this;

    sig.sigev_notify = SIGEV_THREAD;//;
    sig.sigev_signo = SIGRTMIN;//RTL_SIGUSR1;//;

    sig.sigev_notify_function = callback;
    sig.sigev_notify_attributes = NULL;

    timer_create(CLOCK_REALTIME, &sig, &this->timer_handle); //RTL_POSIX_TIMERS
}
Timer::~Timer()
{
}
void Timer::set_interval(int us, int last_count)
{
    struct itimerspec new_setting, old_setting;
    new_setting.it_value.tv_sec = 1;
    new_setting.it_value.tv_nsec = 0;
    new_setting.it_interval.tv_sec=0;
    new_setting.it_interval.tv_nsec=1000 * us;
    timer_settime(this->timer_handle, 0, &new_setting, &old_setting);
}
void Timer::set_callback(Callback *callback_obj)
{
    this->callback_obj = callback_obj;
}
void Timer::start(  )
{
}
void Timer::stop()
{
}

