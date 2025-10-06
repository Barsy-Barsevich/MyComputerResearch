/* lib.c */
//#include "lib.h"

volatile char data;

/*void LED_ON()
{
    char* anm;
    anm = (char*)0x1234;
    *anm = 0x55;
    data = 234;
    data -= 11;
}*/

void delay()
{
    for(int i=0; i<1000; i++);
}
