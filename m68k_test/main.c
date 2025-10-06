/* main */
//#include "lib.h" 

#define LED_REG     (char*)0x1234

void main();
void delay();

void main()
{
    while (1)
    {
        *LED_REG = 0x10;
        delay();
        *LED_REG = 0x00;
        delay();
    }
}
