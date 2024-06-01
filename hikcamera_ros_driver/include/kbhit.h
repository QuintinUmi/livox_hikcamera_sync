#ifndef _KBHIT_H_
#define _KBHIT_H

#include <iostream>
// #include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
using namespace std;

bool kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}
// ————————————————
// 版权声明：本文为CSDN博主「MOLWH」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
// 原文链接：https://blog.csdn.net/weixin_38369492/article/details/121284590

#endif