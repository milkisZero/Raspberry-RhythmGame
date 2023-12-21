#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <wiringPi.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <time.h>
#include <wiringPiSPI.h>

#define BUFFER_SIZE 32
#define GAME_TICK 200

// GPIO 핀 설정
#define DATA_PIN  12  // 데이터 핀 (GPIO 10에 연결)
#define CLOCK_PIN 14 // 클럭 핀 (GPIO 11에 연결)
#define LATCH_PIN 10 // 래치 핀 (GPIO 8에 연결)

// MAX7219 Registers
#define DECODE_MODE   0x09
#define INTENSITY     0x0a
#define SCAN_LIMIT    0x0b
#define SHUTDOWN      0x0c
#define DISPLAY_TEST  0x0f

// dot matrix 상태 배열
unsigned short matrix[4][8], dummyMatrix[4][8];
struct sockaddr_in serv_addr;
int sock;
char buf[BUFFER_SIZE];
int bufMemo[500];
int tmpbuf = 0;
int sock;
int state;
int perfCnt, goodCnt, missCnt;
int atkult;

void error_handling(char *msg){
    fputs(msg, stderr);
    fputc('\n', stderr);
    exit(1);
}

// dot matrix 초기화 함수
void matrixReset(){
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 8; j++){
            matrix[i][j] = 0;
        }
    }
}

void send_SPI_16bits(unsigned short data){
    for(int i = 16; i > 0; i--){
        // 비트마스크
        unsigned short mask = 1 << (i - 1);

        // dot matrix에 data 출력
        digitalWrite(CLOCK_PIN, 0);
        digitalWrite(DATA_PIN, (data & mask) ? 1 : 0);
        digitalWrite(CLOCK_PIN, 1);
    }
}

// dot matrix 출력 함수
void updateMatrix(){
    for(unsigned short j = 0; j < 8; j++){
        digitalWrite(LATCH_PIN, HIGH);
        for(unsigned short i = 0; i < 4; i++){
            send_SPI_16bits(((j + 1) << 8) + matrix[i][j]);
        }
        digitalWrite(LATCH_PIN, LOW);
        digitalWrite(LATCH_PIN, HIGH);
    }
}

// 공격자 스킬을 구현하기 위해 dot matrix에 empty matrix를 출력하는 함수 
void updateDummyMatrix(){
    for(unsigned short j = 0; j < 8; j++){
        digitalWrite(LATCH_PIN, HIGH);
        for(unsigned short i = 0; i < 4; i++){
            send_SPI_16bits(((j + 1) << 8) + dummyMatrix[i][j]);
        }
        digitalWrite(LATCH_PIN, LOW);
        digitalWrite(LATCH_PIN, HIGH);
    }
}

// dot matrix 초기 설정값을 보내기 위한 함수 (8 x 32 chain)
void matrixInit(unsigned short data){
    digitalWrite(LATCH_PIN, LOW);
    for(int i = 0; i < 4; i++){
        send_SPI_16bits(data);
    }
    digitalWrite(LATCH_PIN, HIGH);
}

// 게임의 매 틱마다 dot matrix의 게임판을 한칸씩 움직이기 위한 함수
void slideWindow(){
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 8; j++){
            if(matrix[i][j] & (1 << 6)) matrix[i][j] ^= (1 << 6); // 오버플로우 방지
            if(matrix[i][j] & (1 << 7)) matrix[i][j] ^= (1 << 7);
            matrix[i][j] <<= 2; // 노트가 2x2 size이므로 dot matrix를 2칸씩 슬라이딩 
            if (i != 3 && (matrix[i+1][j] & (1 << 7))) matrix[i][j] |= 3; // dot matrix의 범위를 벗어날때 연결된 다음 dot matrix로 데이터 전송
        }
    }
}

void intHandler(int dummy){
    send_SPI_16bits((SHUTDOWN << 8) + 0);
    exit(0);
}

// 수비자가 노트를 쳤을때 아직 display 하단에 도달하지 않았어도 삭제하기 위한 함수
void deleteDot(int line){
    int cur = line*2;
    for(int i = 7; i >= 3; i -= 2){
        if(matrix[0][cur] & (1 << i)){
            matrix[0][cur] ^= (1 << i); // 2x2 size의 노트의 각 bit를 삭제 
            matrix[0][cur+1] ^= (1 << i);
            matrix[0][cur] ^= (1 << (i - 1));
            matrix[0][cur+1] ^= (1 << (i - 1));
            break;
        }
    }
}

int main(int argc, char *argv[]){
    if (argc != 3){
        printf("Usage : %s <IP> <port>\n", argv[0]);
        exit(1);
    }
    sock = socket(PF_INET, SOCK_STREAM, 0);
    if(sock < 0){
        error_handling("socket() error");
        exit(EXIT_FAILURE);
    }
    memset(&serv_addr, 0, sizeof serv_addr);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
    serv_addr.sin_port = htons(atoi(argv[2]));

    if(connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0){
        error_handling("connect() error");
        exit(EXIT_FAILURE);
    }

    if(wiringPiSetup() == -1){
        exit(1);
    }

    signal(SIGINT, intHandler);

    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);

    matrixReset();
    updateMatrix();

    matrixInit((SCAN_LIMIT << 8) + 7);
    matrixInit((DECODE_MODE << 8) + 0);
    matrixInit((INTENSITY << 8) + 1);
    matrixInit((SHUTDOWN << 8) + 1);
    matrixInit((DISPLAY_TEST << 8) + 0);
    

    // 게임을 총 3번 진행 
    for(int game = 0; game < 3; game++){
        // 게임 시작 대기
        while(1){
            memset(buf, 0, sizeof buf);
            read(sock, buf, sizeof buf);
            if(buf[0] != 0){
                printf("pre : %d\n", buf[0]);
            }
            if(buf[0] == 49){
                buf[0] = 0;
                break;
            }
        }
        memset(bufMemo, 0, sizeof bufMemo);
        // 공격자 turn 
        for(int i=0; i<60; i++){
            memset(buf, 0, sizeof buf);
            read(sock, buf, sizeof buf);
            bufMemo[i] = buf[0]; // 수비자의 turn때 공격자가 입력한 노트를 다시 보여주기 위해 저장 
            buf[0] = 0;

            if(bufMemo[i] & (1 << 0)){
                matrix[3][0] |= 3; // 2x2 size의 노트를 구현, ((1 << 0) + (1 << 1) == 3)
                matrix[3][1] |= 3;
            }
            if(bufMemo[i] & (1 << 1)){
                matrix[3][2] |= 3;
                matrix[3][3] |= 3;
            }
            if(bufMemo[i] & (1 << 2)){
                matrix[3][4] |= 3;
                matrix[3][5] |= 3;
            }
            if(bufMemo[i] & (1 << 3)){
                matrix[3][6] |= 3;
                matrix[3][7] |= 3;
            }
            updateMatrix();
            slideWindow();
            /* dot matrix에 들어가는 값 확인 */
            // for(int j=3; j>=0; j--){
            //     for(int k=0; k<8; k++){
            //         for(int l=0; l<8; l++){
            //             if(matrix[j][l] & (1 << k)) printf("●");
            //             else printf("○");
            //         }
            //         printf("\n");
            //     }
            //     printf("----------------------\n");
            // }
        }
        // 대기시간
        for(int i=0; i<24; i++){
            read(sock, buf, sizeof buf);
            updateMatrix();
            slideWindow();
        }
        // 수비자 turn
        for(int i=0; i<14+60; i++){
            memset(buf, 0, sizeof buf);
            read(sock, buf, sizeof buf);
            // 수비자가 친 노트 지우기
            if(buf[0] & (1 << 0)){
                deleteDot(0);
            }
            if(buf[0] & (1 << 1)){
                deleteDot(1);
            }
            if(buf[0] & (1 << 2)){
                deleteDot(2);
            }
            if(buf[0] & (1 << 3)){
                deleteDot(3);
            }
            if(buf[0] & (1 << 4)){
                // 공격자 궁극기
                atkult = 1;
            }
            if(buf[0] & (1 << 5)){
                // 수비자 궁극기
                matrixReset();
            }
            
            slideWindow();
            
            // 공격자가 입력했던 정보에 맞게 노트 출력 
            if(bufMemo[i] & (1 << 0)){
                matrix[3][0] |= 3;
                matrix[3][1] |= 3;
            }
            if(bufMemo[i] & (1 << 1)){
                matrix[3][2] |= 3;
                matrix[3][3] |= 3;
            }
            if(bufMemo[i] & (1 << 2)){
                matrix[3][4] |= 3;
                matrix[3][5] |= 3;
            }
            if(bufMemo[i] & (1 << 3)){
                matrix[3][6] |= 3;
                matrix[3][7] |= 3;
            }

            // 공격자가 스킬을 사용한 상태
            if(atkult){
                if(atkult == 30){
                    atkult = -1;
                }
                if(atkult & 1) updateDummyMatrix();
                else updateMatrix();
                atkult++;
            }
            else{
                updateMatrix();
            }
            /* dot matrix에 들어가는 값 확인 */
            // for(int j=3; j>=0; j--){
            //     for(int k=0; k<8; k++){
            //         for(int l=0; l<8; l++){
            //             if(matrix[j][l] & (1 << k)) printf("●");
            //             else printf("○");
            //         }
            //         printf("\n");
            //     }
            //     printf("------------------\n");
            // }
        }
        for(int i = 0; i < 3; i++){
            slideWindow();
            updateMatrix();
            delay(200);
        }
    }
    close(sock);
    return 0;
}
