#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

int main() {
    int fd;
    int i;
    char char_read;
    // This part can be used for GPIO Pins.

    fd = open("/sys/class/gpio/gpio7/direction", O_WRONLY);
    if (fd < 0) {
        perror("/sys/class/gpio/gpio7/direction");
        return 1;
    }
        write(fd, "in", 2);
        close(fd);

    fd = open("/sys/class/gpio/gpio7/value", O_RDONLY);
        if (fd < 0)
    {
        perror("Error opening brightness file");
        return 1;
    }

while(1){

        lseek(fd, 0, SEEK_SET);  // rewind to start of file before each read
        if (read(fd, &char_read, 1) > 0 && char_read == '1') {
            printf("GPIO value: %c\n", char_read);
        }

        usleep(250000); // 250ms
    }
	close(fd);
    return 0;
}
