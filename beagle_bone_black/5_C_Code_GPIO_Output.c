#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

int main() {
    int fd;
    int i;

    // This part can be used for GPIO Pins, here the User LEDs are configured to output and there is no LED direction file.

    fd = open("/sys/class/gpio/gpio7/direction", O_WRONLY);
    if (fd < 0) {
        perror("/sys/class/gpio/gpio7/direction");
        return 1;
    }
	write(fd, "0", 1);
	close(fd);

    // infinite while
    while (1)
        {
                fd = open("/sys/class/gpio/gpio7/value", O_WRONLY);
                if (fd < 0)
                {
                    perror("Error opening brightness file");
                        return 1;
                }

                write(fd, "1", 1);  // LED ON
                usleep(250000);     // Half sec delay

                write(fd, "0", 1);  // LED OFF
                close(fd);

                usleep(250000);
    }
    return 0;
}
