#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

int main() {
    const char *device = "/dev/spidev1.0";
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Failed to open SPI device");
        return 1;
    }

    // SPI settings
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 500000;

    // Set SPI mode, bits per word, speed
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    // Data to send
    uint8_t tx[] = {0xAA, 0xBB, 0xCC};
    uint8_t rx[sizeof(tx)] = {0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = sizeof(tx),
        .speed_hz = speed,
        .bits_per_word = bits,
        .delay_usecs = 0,
    };

    // Perform SPI transaction
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("SPI transfer failed");
        close(fd);
        return 1;
    }

    // Print received data
    printf("Received: ");
    for (int i = 0; i < sizeof(rx); i++) {
        printf("0x%02X ", rx[i]);
    }
    printf("\n");

    close(fd);
    return 0;
}
