#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <softTone.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>      
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <time.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

//MAX 버퍼 사이즈
#define BUFFER_MSG 32

//한번 신호를 받고 연속적으로 받기 전에 막기 위한 경계선
#define BOUND 100

//GPIO IN, OUT
#define IN 0
#define OUT 1
#define LOW 0
#define HIGH 1

//GPIO READ, WRITE
#define VALUE_MAX 40
#define DIRECTION_MAX 40

//버튼 2 GPIO 주소
#define AIN 5
#define AOUT 6

//버튼 1 GPIO 주소
#define BIN 20
#define BOUT 21

//부저 GPIO 주소
#define BuzzerPIN 18

//가속도 센서 주소
#define ADXL345_ADDRESS 0x53
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATAX0 0x32

//압력 센서 채널(0~7)
#define CHANNEL 0

//딴딴딴딴 배경음(직접찍음)
int jazz[192] = {
  1046, 0, 0,   1046, 0, 0,   932, 0, 0,    1046, 0, 0, 
  0, 0, 0,      784, 0, 0,    0, 0, 0,      784, 0, 0, 
  1046, 0, 0,   1397, 0, 0,   1318, 0, 0,   1046, 0, 0,
  0, 0, 0,      0, 0, 0,      0, 0, 0,      0, 0, 0,
  
  
  1046, 0, 0,   1046, 0, 0,   932, 0, 0,    1046, 0, 0, 
  0, 0, 0,      784, 0, 0,    0, 0, 0,      784, 0, 0, 
  1046, 0, 0,   1397, 0, 0,   1318, 0, 0,   1046, 0, 0,
  0, 0, 0,      0, 0, 0,      0, 0, 0,      0, 0, 0,

  
  523, 0, 0,    0, 0, 0,      523, 0, 0,    523, 0, 0, 
  659, 0, 0,    0, 0, 0,      659, 0, 0,    0, 0, 0,
  784, 0, 0,    784, 0, 0,    784, 0, 0,    1318, 0, 0,
  0, 0, 0,      1175, 0, 0,   1046, 0, 0,   0, 0, 0,         


  1046, 0, 0,   1046, 0, 0,   932, 0, 0,    1046, 0, 0, 
  0, 0, 0,      784, 0, 0,    0, 0, 0,      784, 0, 0, 
  1046, 0, 0,   1397, 0, 0,   1318, 0, 0,   1046, 0, 0,
  0, 0, 0,      0, 0, 0,      0, 0, 0,      0, 0, 0,
};

//파일디스크립터, 소켓선언
int spi_fd, defender_sock;

//버튼 상태 전역변수
int state1 = 1;
int state2 = 1;
int prev_state1 = 0;
int prev_state2 = 0;

//시작 시간 전역변수
long long start_time;

//압력(궁극기) 전역변수
int onpress = 0;

void error_handling(char *message) {
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
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

//가속도 센서 thread
void *accelerater() {
    int fd;

    // I2C 장치 열기
    if ((fd = wiringPiI2CSetup(ADXL345_ADDRESS)) < 0) {
        perror("Failed to open the i2c bus");
        return 0;
    }

    // ADXL345 초기화, POWER_CTL 레지스터 설정: 측정 활성화(성공 시 0 반환)
    if (wiringPiI2CWriteReg8(fd, ADXL345_POWER_CTL, 0x08) < 0) {
        perror("Failed to initialize ADXL345");
        return 0;
    }

    short x0, y0, z0, x, y, z, X, Y, Z;
    //초기 위치 확인
    x0 = wiringPiI2CReadReg16(fd, ADXL345_DATAX0);
    y0 = wiringPiI2CReadReg16(fd, ADXL345_DATAX0 + 2);
    z0 = wiringPiI2CReadReg16(fd, ADXL345_DATAX0 + 4);

    long long pre_sec = 0;

    while (1) {
        //현재 시간 측정
        long long now_time; 
        struct timeb mili_now;
        time_t sec_now;
        
        time(&sec_now);
        ftime(&mili_now);
        now_time = sec_now * 1000 + mili_now.millitm;
        now_time -= start_time;

        //현재 가속도 센서 위치 측정
        x = wiringPiI2CReadReg16(fd, ADXL345_DATAX0);
        y = wiringPiI2CReadReg16(fd, ADXL345_DATAX0 + 2);
        z = wiringPiI2CReadReg16(fd, ADXL345_DATAX0 + 4);

        //이전 틱과 현재 틱 사이의 가속도 변화량 측정
        X = x - x0; Y = y - y0; Z = z - z0;
        x0 = x, y0 = y, z0 = z;
        
        //왼쪽으로 기울었을 때 감지, 1번 레인에 입력했음을 서버로 전송
        if ((X > 50) && (Y > 10)) {
            char msg[BUFFER_MSG] = "";
            snprintf(msg, BUFFER_MSG, "0%lld", now_time);
            
            //중복입력 방지를 위한 텀
            if (pre_sec + BOUND <= now_time) {
                write(defender_sock, msg, sizeof(msg));
                printf("%s\n", msg);
                pre_sec = now_time;
            }
        }

        //오른쪽으로 기울였을 때 감지, 4번 레인에 입력했음을 서버로 전송
        if ((X < -50) && (Y < -10)) {
            char msg[BUFFER_MSG] = "";
            snprintf(msg, BUFFER_MSG, "3%lld", now_time);

            if (pre_sec + BOUND <= now_time) {
                write(defender_sock, msg, sizeof(msg));
                printf("%s\n", msg);
                pre_sec = now_time;
            }
        }
    }
}

//버튼 1 thread
void *button() {
    //GPIO 에러 확인
    if (GPIOExport(BOUT) == -1 || GPIOExport(BIN) == -1) exit(1);
    if (GPIODirection(BOUT, OUT) == -1 || GPIODirection(BIN, IN) == -1) exit(1);
    if (GPIOWrite(BOUT, 1) == -1) exit(1);

    long long pre_sec = 0;
    
    while (1) {
        //시간 측정
        long long now_time; 
        struct timeb mili_now;
        time_t sec_now;
        
        time(&sec_now);
        ftime(&mili_now);
        now_time = sec_now * 1000 + mili_now.millitm;
        now_time -= start_time;

        state1 = GPIORead(BIN);
        //왼쪽 버튼 눌렀을때 감지, 2번 레인이 들어갔음을 서버로 전송
        if (state1 == 0 && prev_state1 != 0) {
            char msg[BUFFER_MSG] = "";
            snprintf(msg, BUFFER_MSG, "1%lld", now_time);
            
            //중복입력 방지를 위한 텀
            if (pre_sec + BOUND <= now_time) {
                write(defender_sock, msg, sizeof(msg));
                printf("%s\n", msg);
                pre_sec = now_time;
            }
        }
        prev_state1 = state1;
    }
    //GPIO 축출
    GPIOUnexport(BIN);
    GPIOUnexport(BOUT);
}

//버튼 2 thread
void *button2() {
    //GPIO 에러 확인
    if (GPIOExport(AOUT) == -1 || GPIOExport(AIN) == -1) exit(1);
    if (GPIODirection(AOUT, OUT) == -1 || GPIODirection(AIN, IN) == -1) exit(1);
    if (GPIOWrite(AOUT, 1) == -1) exit(1);

    long long pre_sec = 0;

    while (1) {
        //시간 측정
        long long now_time; 
        struct timeb mili_now;
        time_t sec_now;
        
        time(&sec_now);
        ftime(&mili_now);
        now_time = sec_now * 1000 + mili_now.millitm;
        now_time -= start_time;
    
        state2 = GPIORead(AIN);
        //오른쪽 버튼 눌렀을때 감지, 3번 레인이 들어갔음을 서버로 전송
        if (state2 == 0 && prev_state2 != 0) {
            char msg[BUFFER_MSG] = "";
            snprintf(msg, BUFFER_MSG, "2%lld", now_time);

            //중복입력 방지를 위한 텀
            if (pre_sec + BOUND <= now_time) {
                write(defender_sock, msg, sizeof(msg));
                printf("%s\n", msg);
                pre_sec = now_time;
            }
        }
        prev_state2 = state2;
    }
    //GPIO 축출
    GPIOUnexport(AIN);
    GPIOUnexport(AOUT);
}

//압력 센서 신호 변환 함수
int readMCP3008(int channel) {
    unsigned char buffer[3];

    buffer[0] = 0x06 | ((channel & 0x07) >> 2);
    buffer[1] = ((channel & 0x07) << 6);
    buffer[2] = 0x00;

    wiringPiSPIDataRW(CHANNEL, buffer, 3);

    int adcValue;
    adcValue = ((buffer[1] & 0x0F) << 8) | buffer[2];
    return adcValue;
}

//압력 센서 thread
void *pressure() {
    //압력 센서 생성
    if (wiringPiSetup() == -1) { 
        fprintf(stderr, "Unable to initialize wiringPi\n"); 
        exit(1);
    }
    
    if (wiringPiSPISetup(CHANNEL, 1000000) == -1) {
        fprintf(stderr, "Unable to initialize SPI\n");
        exit(1);
    }
    
    while (1) {
        //압력 센서 받아들이기
        int press = readMCP3008(0);
        //일정 값 이상이면 궁극기가 발동되었음을 서버로 전송, 게임 동안 1번만 사용 가능
        if (press > 500 && !onpress) {
            char msg[BUFFER_MSG] = "";
            snprintf(msg, BUFFER_MSG, "6", 1);
            printf("Ultimate Acitivated\n");
            write(defender_sock, msg, sizeof(msg));
            onpress = 1;
        }
    }
}

//부저 thread
void *sound() {
    long long pre_sec = 0;

    while (1) {
        //현재 시간 확인
        long long now_time; 
        struct timeb mili_now;
        time_t sec_now;

        time(&sec_now);
        ftime(&mili_now);
        now_time = sec_now * 1000 + mili_now.millitm;
        now_time -= start_time;
        
        //버튼을 눌렀을 때 소리가 나옴
        //1번 부저
        if (state1 == 0 && prev_state1 != 0) {
            //중복 방지
            if (pre_sec + BOUND <= now_time) {
                softToneWrite(BuzzerPIN, 784);
                delayMicroseconds(20000);
                softToneWrite(BuzzerPIN, 0);
                pre_sec = now_time;
            }
        }
        //2번 부저
        if (state2 == 0 && prev_state2 != 0) {
            //중복 방지
            if (pre_sec + BOUND <= now_time) {
                softToneWrite(BuzzerPIN, 784);
                delayMicroseconds(20000);
                softToneWrite(BuzzerPIN, 0);
                pre_sec = now_time;
            }
        }
    }
}

//한 턴당 입력 시간 확인
void *countEnd() {
    long long now_time; 
    struct timeb mili_now;
    time_t sec_now;

    time(&sec_now);
    ftime(&mili_now);
    now_time = sec_now * 1000 + mili_now.millitm;
    now_time -= start_time;

    //수비자 턴 12초 세기
    while (now_time < 12000) {
        time(&sec_now);
        ftime(&mili_now);
        now_time = sec_now * 1000 + mili_now.millitm;
        now_time -= start_time;
    }
    printf("game set!\n");
    
    //게임이 끝났음을 서버로 전송
    char msg[BUFFER_MSG] = "";
    snprintf(msg, BUFFER_MSG, "9", 1);
    write(defender_sock, msg, sizeof(msg));
}

int main(int argc, char *argv[]) {
    //argument 에러 확인
    if (argc != 3) {
        printf("Usage : %s <IP> <port>\n", argv[0]);
        exit(1);
    }

    //소켓 생성
    struct sockaddr_in serv_addr;
    defender_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (defender_sock == -1)
        error_handling("socket() error");

    //소켓 주소 설정
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
    serv_addr.sin_port = htons(atoi(argv[2]));

    //서버와 연결되었음을 확인
    if (connect(defender_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
        error_handling("connect() error");

    printf("[*] now connected to %s [*]\n", inet_ntoa(serv_addr.sin_addr));

    //부저 생성
    wiringPiSetupGpio();
    softToneCreate(BuzzerPIN);

    //3턴 구현
    for (int i=0; i<3; i++) {
        printf("Game %d\n", i+1);

        //공격 턴 확인
        char msg_server[BUFFER_MSG] = "";
        read(defender_sock, msg_server, sizeof(msg_server));
        printf("Attacker turn. Please Wait...\n");

        //공격턴 때 배경음악 출력
        for (int i=0; i<192+48-1; i++) {
            softToneWrite(BuzzerPIN, jazz[i%192]);
            delay(50);
        }

        //수비턴 확인
        read(defender_sock, msg_server, sizeof(msg_server));
        if (read > 0)
            printf("your turn!\n");
        
        //수비턴 시작 시간 측정
        struct timeb mili_now;
        time_t sec_now;

        ftime(&mili_now);
        ftime(&sec_now);
        start_time = sec_now * 1000 + mili_now.millitm;

        //각 센서 thread 생성
        pthread_t p_thread[6];
        int thr_id, status;
        
        thr_id = pthread_create(&p_thread[0], NULL, countEnd, NULL);
        if (thr_id < 0) { perror("thread create error : "); exit(0); }

        thr_id = pthread_create(&p_thread[1], NULL, accelerater, NULL);
        if (thr_id < 0) { perror("thread create error : "); exit(0); }

        thr_id = pthread_create(&p_thread[2], NULL, button, NULL);
        if (thr_id < 0) { perror("thread create error : "); exit(0); }

        thr_id = pthread_create(&p_thread[3], NULL, button2, NULL);
        if (thr_id < 0) { perror("thread create error : "); exit(0); }

        thr_id = pthread_create(&p_thread[4], NULL, pressure, NULL);
        if (thr_id < 0) { perror("thread create error : "); exit(0); }

        thr_id = pthread_create(&p_thread[5], NULL, sound, NULL);
        if (thr_id < 0) { perror("thread create error : "); exit(0); }
        
        //시간 측정 thread가 끝날때까지 대기
        pthread_join(p_thread[0], (void **)&status);

        //각 thread 자원 회수 및 종료
        for (int i=0; i<6; i++) pthread_detach(p_thread[i]);
        for (int i=0; i<6; i++) pthread_cancel(p_thread[i]);
    }
    
    printf("[*] socket closed [*]\n");
    close(defender_sock);
    return 0;
}
