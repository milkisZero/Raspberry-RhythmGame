#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/timeb.h>
#include <time.h>

#define SPI_PATH "/dev/spidev0.0"

#define IN 0
#define OUT 1
#define LOW 0
#define HIGH 1

#define VALUE_MAX 40
#define DIRECTION_MAX 40
#define BUFFER_MSG 32

#define AIN 5
#define AOUT 6

#define BIN 20
#define BOUT 21

#define SENSOR_PIN 7 // 진동을 위한 핀의 위치
#define BuzzerPIN 0

#define INPUT_BOUND 100
#define GAMETIME 1000 * 12
#define TURN 3

int spi_fd;
int sock;
long long start_time;
int start_check = 0;
int end_check = 0;

int state, prev_state, state2, prev_state2;

int jazz[192] = {
    1046,
    0,
    0,
    1046,
    0,
    0,
    932,
    0,
    0,
    1046,
    0,
    0,
    0,
    0,
    0,
    784,
    0,
    0,
    0,
    0,
    0,
    784,
    0,
    0,
    1046,
    0,
    0,
    1397,
    0,
    0,
    1318,
    0,
    0,
    1046,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,

    1046,
    0,
    0,
    1046,
    0,
    0,
    932,
    0,
    0,
    1046,
    0,
    0,
    0,
    0,
    0,
    784,
    0,
    0,
    0,
    0,
    0,
    784,
    0,
    0,
    1046,
    0,
    0,
    1397,
    0,
    0,
    1318,
    0,
    0,
    1046,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,

    523,
    0,
    0,
    0,
    0,
    0,
    523,
    0,
    0,
    523,
    0,
    0,
    659,
    0,
    0,
    0,
    0,
    0,
    659,
    0,
    0,
    0,
    0,
    0,
    784,
    0,
    0,
    784,
    0,
    0,
    784,
    0,
    0,
    1318,
    0,
    0,
    0,
    0,
    0,
    1175,
    0,
    0,
    1046,
    0,
    0,
    0,
    0,
    0,

    1046,
    0,
    0,
    1046,
    0,
    0,
    932,
    0,
    0,
    1046,
    0,
    0,
    0,
    0,
    0,
    784,
    0,
    0,
    0,
    0,
    0,
    784,
    0,
    0,
    1046,
    0,
    0,
    1397,
    0,
    0,
    1318,
    0,
    0,
    1046,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

void error_handling(char *message) {
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

// SPI 초기화 함수
void initializeSPI() {
    spi_fd = open(SPI_PATH, O_RDWR);
    if (spi_fd < 0) {
        perror("Error - could not open SPI device");
        return;
    }

    // Set SPI mode
    uint8_t mode = SPI_MODE_0;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);

    // Set bits per word
    uint8_t bits = 8;
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);

    // Set max speed in Hz
    uint32_t speed = 1000000;
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

// ADC를 통해 아날로그 값을 변환
uint16_t readADC(uint8_t channel) {
    uint8_t tx_buffer[3] = {1, (8 + channel) << 4, 0};
    uint8_t rx_buffer[3] = {0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = sizeof(tx_buffer),
        .delay_usecs = 0,
        .speed_hz = 1000000,
        .bits_per_word = 8,
    };

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("Error - problem transmitting spi data");
        return 0;
    }

    uint16_t value = ((rx_buffer[1] & 3) << 8) | rx_buffer[2];
    return value;
}

// 자이로센서의 값을 정밀화
float convertToAcceleration(uint16_t value) {
    float voltage = (value * 3.3) / 1023.0;
    float acceleration = ((voltage - 1.65) / 0.330);
    return acceleration;
}

static int GPIOExport(int pin) {
#define BUFFER_MAX 3
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open export for writing!\n");
        return (-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return (0);
}

static int GPIOUnexport(int pin) {
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open unexport for writing!\n");
        return (-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return (0);
}

static int GPIODirection(int pin, int dir) {
    static const char s_directions_str[] = "in\0out";

    char path[DIRECTION_MAX];
    int fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio direction for writing!\n");
        return (-1);
    }

    if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
        fprintf(stderr, "Failed to set direction!\n");
        return (-1);
    }

    close(fd);
    return (0);
}

static int GPIORead(int pin) {
    char path[VALUE_MAX];
    char value_str[3];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio value for reading!\n");
        return (-1);
    }

    if (-1 == read(fd, value_str, 3)) {
        fprintf(stderr, "Failed to read value!\n");
        return (-1);
    }

    close(fd);

    return (atoi(value_str));
}

static int GPIOWrite(int pin, int value) {
    static const char s_values_str[] = "01";

    char path[VALUE_MAX];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio value for writing!\n");
        return (-1);
    }

    if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
        fprintf(stderr, "Failed to write value!\n");
        return (-1);
    }

    close(fd);
    return (0);
}

// 자이로 센서의 라인 0,3 입력부분
void *accelerater() {
    initializeSPI();

    double prev_x = 0;
    double init_x = 0;
    int valid = 0;

    while (start_check == 0)
        ;

    uint16_t x_adc_value = readADC(0);
    init_x = convertToAcceleration(x_adc_value);

    long long pre_sec = 0;
    while (!end_check) {
        uint16_t x_adc_value = readADC(0);
        uint16_t y_adc_value = readADC(1);
        uint16_t z_adc_value = readADC(2);

        float x_acceleration = convertToAcceleration(x_adc_value);
        float y_acceleration = convertToAcceleration(y_adc_value);
        float z_acceleration = convertToAcceleration(z_adc_value);

        long long now;
        struct timeb milli_now;
        time_t sec_now;

        time(&sec_now);
        ftime(&milli_now);
        now = sec_now * 1000 + milli_now.millitm;
        now -= start_time;

        if (x_acceleration < 0.1 && x_acceleration > -0.1) {
            valid = 1;
        }

        if (pre_sec + INPUT_BOUND <= now && x_acceleration >= 0.8 && valid == 1) {
            char msg[BUFFER_MSG] = "";
            snprintf(msg, BUFFER_MSG, "3%lld", now);
            printf("%s\n", msg);
            write(sock, msg, sizeof(msg));
            valid = 0;
            pre_sec = now;
        }
        else if (pre_sec + INPUT_BOUND <= now && x_acceleration <= -0.9 && valid == 1) {
            char msg[BUFFER_MSG] = "";
            snprintf(msg, BUFFER_MSG, "0%lld", now);
            printf("%s\n", msg);
            write(sock, msg, sizeof(msg));
            valid = 0;
            pre_sec = now;
        }

        //   printf("X: %.2fg, Y: %.2fg, Z: %.2fg\n", x_acceleration, y_acceleration, z_acceleration);
    }

    close(spi_fd);
}

// 왼쪽 버튼의 라인 1 입력부분
void *button() {
    state = GPIORead(BIN);
    prev_state = 0;

    long long pre_sec = 0;

    while (!end_check) {
        state = GPIORead(BIN);
        long long now;
        struct timeb milli_now;
        time_t sec_now;

        time(&sec_now);
        ftime(&milli_now);
        now = sec_now * 1000 + milli_now.millitm;
        now -= start_time;

        if (state == 0 && prev_state != 0) {
            if (start_check == 0) {
                ftime(&sec_now);
                ftime(&milli_now);
                start_time = sec_now * 1000 + milli_now.millitm;
                start_check = 1;

                write(sock, "ss", 5);
                continue;
            }
            else {
                char msg[BUFFER_MSG] = "";
                snprintf(msg, BUFFER_MSG, "1%lld", now);
                printf("%s\n", msg);

                if (pre_sec + INPUT_BOUND <= now) {
                    write(sock, msg, sizeof(msg));
                    pre_sec = now;
                }
            }
        }

        prev_state = state;
    }
}

// 오른쪽 버튼의 라인 2 입력부분
void *button2() {
    state2 = GPIORead(AIN);
    prev_state2 = 0;

    long long pre_sec = 0;

    while (!end_check) {
        state2 = GPIORead(AIN);
        long long now;
        struct timeb milli_now;
        time_t sec_now;

        time(&sec_now);
        ftime(&milli_now);
        now = sec_now * 1000 + milli_now.millitm;
        now -= start_time;

        if (state2 == 0 && prev_state2 != 0) {
            if (start_check == 0) {
                ftime(&sec_now);
                ftime(&milli_now);
                start_time = sec_now * 1000 + milli_now.millitm;
                start_check = 1;

                write(sock, "ss", 5);
                continue;
            }
            else {
                char msg[BUFFER_MSG] = "";
                snprintf(msg, BUFFER_MSG, "2%lld", now);
                printf("%s\n", msg);

                if (pre_sec + INPUT_BOUND <= now) {
                    write(sock, msg, sizeof(msg));
                    pre_sec = now;
                }
            }
        }

        prev_state2 = state2;
    }
}

// 진동센서의 스킬 입력부분
void *vibeDetect() {
    pinMode(SENSOR_PIN, INPUT);

    char msg_server[BUFFER_MSG] = "";
    read(sock, msg_server, sizeof(msg_server));

    int cnt = 0;
    while (start_check && end_check) {
        if (digitalRead(SENSOR_PIN) == HIGH) {
            printf("skill set\n");
            write(sock, "R", 5);
            break;
        }
        delay(100);
    }
}

// 공격자의 입력 시간을 12초 간 측정하는 함수
void *countEnd() {
    long long now = 0;
    struct timeb milli_now;
    time_t sec_now;

    while (start_check == 0) {
    };

    printf("count start\n");
    while ((now - start_time) < GAMETIME) {
        time(&sec_now);
        ftime(&milli_now);
        now = sec_now * 1000 + milli_now.millitm;
    }
    end_check = 1;
    printf("turn over\n");
}

// 버튼 소리
void *sound() {
    while (!end_check) {
        if (state == 0 && prev_state != 0) {
            softToneWrite(BuzzerPIN, 784);
            delayMicroseconds(20000);
            softToneWrite(BuzzerPIN, 0);
        }
        if (state2 == 0 && prev_state2 != 0) {
            softToneWrite(BuzzerPIN, 392);
            delayMicroseconds(20000);
            softToneWrite(BuzzerPIN, 0);
        }
    }
}

// gpio 초기 세팅 함수
void initGpio() {
    if (GPIOExport(AOUT) == -1 || GPIOExport(AIN) == -1)
        exit(1);

    if (GPIODirection(AOUT, OUT) == -1 || GPIODirection(AIN, IN) == -1)
        exit(1);

    if (GPIOWrite(AOUT, 1) == -1) {
        exit(1);
    }

    if (GPIOExport(BOUT) == -1 || GPIOExport(BIN) == -1)
        exit(1);

    if (GPIODirection(BOUT, OUT) == -1 || GPIODirection(BIN, IN) == -1)
        exit(1);

    if (GPIOWrite(BOUT, 1) == -1) {
        exit(1);
    }
}

void initVal() {
    start_time = 0;
    start_check = 0;
    end_check = 0;
    state = 0, prev_state = 0, state2 = 0, prev_state2 = 0;
}

int main(int argc, char *argv[]) {
    wiringPiSetup();
    initGpio();

    if (argc != 3) {
        printf("Usage : %s <IP> <port>\n", argv[0]);
        exit(1);
    }

    struct sockaddr_in serv_addr;
    sock = socket(PF_INET, SOCK_STREAM, 0);
    if (sock == -1)
        error_handling("socket() error");

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
    serv_addr.sin_port = htons(atoi(argv[2]));

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
        error_handling("connect() error");

    printf("[*] now connected to %s\n", inet_ntoa(serv_addr.sin_addr));

    pthread_t p_thread[6];
    int thr_id;
    int status;
    char msg_server[BUFFER_MSG] = "";
    int a = read(sock, msg_server, sizeof(msg_server));
    printf("server prepared\n");

    thr_id = pthread_create(&p_thread[3], NULL, vibeDetect, NULL);
    if (thr_id < 0) {
        perror("thread create error : ");
        exit(0);
    }
    pthread_detach(p_thread[3]);

    for (int p = 0; p < TURN; p++) {
        if (p != 0) {
            char msg_server[BUFFER_MSG] = "";
            read(sock, msg_server, sizeof(msg_server));
            printf("server prepared\n");
        }
        initVal();

        softToneCreate(BuzzerPIN);

        thr_id = pthread_create(&p_thread[1], NULL, button, NULL);
        if (thr_id < 0) {
            perror("thread create error : ");
            exit(0);
        }

        thr_id = pthread_create(&p_thread[2], NULL, button2, NULL);
        if (thr_id < 0) {
            perror("thread create error : ");
            exit(0);
        }

        while (start_check == 0) {
        };

        thr_id = pthread_create(&p_thread[4], NULL, countEnd, NULL);
        if (thr_id < 0) {
            perror("thread create error : ");
            exit(0);
        }

        thr_id = pthread_create(&p_thread[0], NULL, accelerater, NULL);
        if (thr_id < 0) {
            perror("thread create error : ");
            exit(0);
        }

        thr_id = pthread_create(&p_thread[5], NULL, sound, NULL);
        if (thr_id < 0) {
            perror("thread create error : ");
            exit(0);
        }

        pthread_join(p_thread[4], (void **)&status);
        pthread_detach(p_thread[0]);
        pthread_detach(p_thread[1]);
        pthread_detach(p_thread[2]);
        pthread_detach(p_thread[5]);

        write(sock, "9", sizeof(BUFFER_MSG));

        // 디펜더 턴에 브금출력
        sleep(8);
        for (int i = 0; i < 192 + 47; i++) {
            softToneWrite(BuzzerPIN, jazz[i % 192]);
            delay(50);
        }
    }

    sleep(10);
    GPIOUnexport(AIN);
    GPIOUnexport(AOUT);
    GPIOUnexport(BIN);
    GPIOUnexport(BOUT);

    close(sock);
    printf("[*] session closed\n");

    return 0;
}
