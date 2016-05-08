#include <breezystm32.h>

void setup(void)
{
    i2cInit(I2CDEV_2);
}

void loop(void)
{
    printf("%d\n", mb1242_poll());
}
