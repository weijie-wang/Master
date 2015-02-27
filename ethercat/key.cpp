#include <stdio.h>  
#include <termios.h>  
#include <unistd.h>  
#include <fcntl.h>  

#include "parse.h"
int kbhit(void)  
{  
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}
int getkey(int timeout)//in ms
{
    fflush(stdin);
    for(; (!kbhit()) && (timeout > 0); timeout -= 10)
        usleep(10000);//puts("Press a key!");
    if(timeout <= 0)
        return 0x00ffff;
    return getchar();
}
