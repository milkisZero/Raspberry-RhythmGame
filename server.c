#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>

// Define some device parameters
#define I2C_ADDR   0x27 // I2C device address

// Define some device constants
#define LCD_CHR  1 // Mode - Sending data
#define LCD_CMD  0 // Mode - Sending command

#define LINE1  0x80 // 1st line
#define LINE2  0xC0 // 2nd line

#define LCD_BACKLIGHT   0x08  // On
// LCD_BACKLIGHT = 0x00  # Off

#define ENABLE  0b00000100 // Enable bit

#define BUFFER_MAX 3
#define DIRECTION_MAX 256
#define VALUE_MAX 256
#define IN 0
#define OUT 1
#define LOW 0
#define HIGH 1
#define R 22
#define G 17
#define Y 27
#define LIFE 036
#define LINE 4
#define TICK 200
#define QSIZE 100

/*

        판정 범위(ms) -  0 ~ 200 : perfect ; 200 ~ 400 : good ; 400 ~ 600 

*/
/*
        Display raspi에 매틱(200ms) 전송하는 data(1byte)의 format

        변수명 : bits

        오른쪽부터 i (0<=i<4) 번째 비트는 i번 line에 입력이 들어왔을 때 1을 저장
        오른쪽부터 4번째 비트는 공격자 필살기가 입력으로 들어왔을 때 1을 저장
        오른쪽부터 5번째 피트는 수비자 필살기가 입력으로 들어왔을 때 1을 저장  

        나머지는 0
*/

int left[LINE], right[LINE], comp[LINE], q[LINE][100], ;
int p1_sock, p2_sock, dis_sock, p1EndSignal, p2EndSignal, Atk_point, Def_point;
int fd, idx, missCnt, goodCnt, perfectCnt, noteCnt, life, dcnt;
long long start_time;
bool runThread[LINE], Def_ult_on, Atk_ult_on, GGon, YYon, RRon, Atk_ult_used, Def_ult_used;
char bits, sig[2], arr1[16], arr2[16], notes[500];


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

  if (-1 ==
      write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
    fprintf(stderr, "Failed to set direction!\n");
    return (-1);
  }

  close(fd);
  return (0);
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


static int PWMExport(int pwmnum) {
#define BUFFER_MAX 3
  char buffer[BUFFER_MAX];
  int fd, byte;

  // TODO: Enter the export path.
  fd = open("/sys/class/pwm/pwmchip0/export", O_WRONLY);
  if (-1 == fd) {
    fprintf(stderr, "Failed to open export for export!\n");
    return (-1);
  }

  byte = snprintf(buffer, BUFFER_MAX, "%d", pwmnum);
  write(fd, buffer, byte);
  close(fd);

  sleep(1);

  return (0);
}

static int PWMEnable(int pwmnum) {
  static const char s_enable_str[] = "1";

  char path[DIRECTION_MAX];
  int fd;

  // TODO: Enter the enable path.
  

  snprintf(path, DIRECTION_MAX, "/sys/class/pwm/pwmchip0/pwm0/enable", pwmnum);
  fd = open(path, O_WRONLY);
  if (-1 == fd) {
    fprintf(stderr, "Failed to open in enable!\n");
    return -1;
  }

  write(fd, s_enable_str, strlen(s_enable_str));
  close(fd);

  return (0);
}

static int PWMWritePeriod(int pwmnum, int value) {
  char s_value_str[VALUE_MAX];
  char path[VALUE_MAX];
  int fd, byte;

  // TODO: Enter the period path.
  snprintf(path, VALUE_MAX, "/sys/class/pwm/pwmchip0/pwm0/period", pwmnum);
  fd = open(path, O_WRONLY);
  if (-1 == fd) {
    fprintf(stderr, "Failed to open in period!\n");
    return (-1);
  }
  byte = snprintf(s_value_str, VALUE_MAX, "%d", value);

  if (-1 == write(fd, s_value_str, byte)) {
    fprintf(stderr, "Failed to write value in period!\n");
    close(fd);
    return -1;
  }
  close(fd);

  return (0);
}

static int PWMWriteDutyCycle(int pwmnum, int value) {
  char s_value_str[VALUE_MAX];
  char path[VALUE_MAX];
  int fd, byte;

  // TODO: Enter the duty_cycle path.
  snprintf(path, VALUE_MAX, "/sys/class/pwm/pwmchip0/pwm0/duty_cycle", pwmnum);
  fd = open(path, O_WRONLY);
  if (-1 == fd) {
    fprintf(stderr, "Failed to open in duty cycle!\n");
    return (-1);
  }
  byte = snprintf(s_value_str, VALUE_MAX, "%d", value);

  if (-1 == write(fd, s_value_str, byte)) {
    fprintf(stderr, "Failed to write value in duty cycle!\n");
    close(fd);
    return -1;
  }
  close(fd);

  return (0);
}

// clr lcd go home loc 0x80
void ClrLcd(void)   {
  lcd_byte(0x01, LCD_CMD);
  lcd_byte(0x02, LCD_CMD);
}

// go to location on LCD
void lcdLoc(int line)   {
  lcd_byte(line, LCD_CMD);
}

// this allows use of any size string
void typeln(const char *s)   {

  while ( *s ) lcd_byte(*(s++), LCD_CHR);

}

void lcd_byte(int bits, int mode)   {

  //Send byte to data pins
  // bits = the data
  // mode = 1 for data, 0 for command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT ;
  bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT ;

  // High bits
  wiringPiI2CReadReg8(fd, bits_high);
  lcd_toggle_enable(bits_high);

  // Low bits
  wiringPiI2CReadReg8(fd, bits_low);
  lcd_toggle_enable(bits_low);
}

void lcd_toggle_enable(int bits)   {
  // Toggle enable pin on LCD display
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits | ENABLE));
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits & ~ENABLE));
  delayMicroseconds(500);
}


void lcd_init()   {
  // Initialise display
  lcd_byte(0x33, LCD_CMD); // Initialise
  lcd_byte(0x32, LCD_CMD); // Initialise
  lcd_byte(0x06, LCD_CMD); // Cursor move direction
  lcd_byte(0x0C, LCD_CMD); // 0x0F On, Blink Off
  lcd_byte(0x28, LCD_CMD); // Data length, number of lines, font size
  lcd_byte(0x01, LCD_CMD); // Clear display
   delayMicroseconds(500);
}

// Line Queue에 관한 함수
bool empty(int lineNum)
{
  if(left[lineNum] == right[lineNum])
  {
    printf("Queue is empty!");
    return true;
  }
  return false;
}

void push(int lineNum, int e) 
{ 
  q[lineNum][right[lineNum]++] = e; 
  return; 
} 

void pop(int lineNum)
{
  if(empty(lineNum))
    return;

  left[lineNum]++; 
  return; 
}

int front(int lineNum)
{
  if(left[lineNum] == right[lineNum])
  {
    printf("Queue is empty!");
    return -1;
  }

  return q[lineNum][left[lineNum]]; 
}

void error_handling(char *message) {
  fputs(message, stderr);
  fputc('\n', stderr);
  exit(1);
}


// 매 라운드 마다 사용하는 변수 초기화
void GameInit()
{
  for(int i=0; i<LINE; i++)
  {
      left[i] = 0;
      right[i] = 0;
  }
  p1EndSignal = 1;
  p2EndSignal = 1;
  Def_ult_on = false, GGon = false, YYon = false, RRon = false;
  idx = 0, missCnt = 0, goodCnt = 0, perfectCnt = 0, bits=0, noteCnt=0, dcnt=0;
}

// 현재 게임 진행시간을 기준으로 라인별 Perfect, good, miss 판정
void* lineNoteProcessor(void* data)
{
    // 이 쓰레드가 관리할 line 번호를 저장
    int lineNum = atoi((char *)data);
    long long now;
    struct timeb milli_now;
    time_t sec_now;
    char m[3];

    while(p2EndSignal)
    {
      time(&sec_now);
      ftime(&milli_now);

      // 현재 시간을 ms 단위로 표현
      now = sec_now * 1000 + milli_now.millitm;
      
      // 게임 시작 시간 이후로 흐른 시간을 저장
      now -= start_time;
      
      // Line Queue에 front-> 현재 시간을 기준으로 지난 note를 queue에서 pop하고 miss 처리한다.
      if(front(lineNum) < now)
      {
        printf("miss1!!\n");
        pop(lineNum);
        if(dcnt < life)
          dcnt++;
        missCnt++;
        RRon = true;
      }

      // 수비자로부터 입력이 들어왔으면 현재 시각과 Line Queue의 front와 비교하여 perfect, good, miss 판정을 한다.    
      if(runThread[lineNum])
      {
        runThread[lineNum] = false;
        // 시간 차이를 diff에 저장
        int diff =  front(lineNum) - comp[lineNum];

        // 입력이 판정 범위 안에 들어왔다면 
        if(diff < TICK*3)
        {
            // dlsplay에 틱마다 전달할 bits의 오른쪽에서 lineNum번째 비트에 1을 넣는다. 
            bits |= (1 << lineNum);


            // perfect
            if(diff < TICK)
            {
              printf("perfect!!\n");
              perfectCnt++;
              GGon = true;
            }
            // good
            else if(diff < TICK*2) 
            {
              printf("good!!\n");
              goodCnt++;
              YYon = true;
            }
            // miss
            else
            {
              printf("miss2!!\n");
              if(dcnt < life)
                dcnt++;
              missCnt++;
              RRon = true;
            }

            // 판정을 한 노드는 Queue에서 pop한다
            pop(lineNum);
        }
      } 
    }      
}


// Defender의 필살기를 처리하는 함수 : 화면에 보이는 노트에 대한 perfect 처리
void* Dultimate()
{  
  long long now;
  struct timeb milli_now;
  time_t sec_now;
  int cnt = 0;  
  char m[5];

  while(p2EndSignal)
  {
    // Def 의 필살기가 발동했으면 실행
    if(Def_ult_on)
    {
      
      bits |= (1 << 5);
      printf("DEF ULT ON!!!!!!!!!\n");

      time(&sec_now);
      ftime(&milli_now);
      now = sec_now * 1000 + milli_now.millitm;
      now -= start_time;

    for(int i=0; i<LINE; i++)
    {
      while(front(i) < now+TICK*16)
      {
        printf("Def ult pop Line :  %d\n", i);
        pop(i);
        perfectCnt++;
        cnt++;
      }
    }

      printf("ult : %d\n", cnt);
      Def_ult_on = false;
      return;
    }
         
  }
}


// 공격자의 필살기를 처리하는 함수
void* Autimate()
{
    int str_len;
    while(p2EndSignal)
    {
      char m[32];

      // Attacker로 부터 필살기 정보를 입력받음 
      str_len = read(p1_sock, m, sizeof(m));
      if (str_len == -1) error_handling("read() error");
      printf("%s\n", m);
      Atk_ult_on = true;
      printf("Attack Ult On!!!!!!!!!!!!!!!!!!!\n");
      bits |= (1 << 4);
      break;
    }

    return;
    
}

// 노트 판정 결과 LED로 출력 및 남은 목숨 LED를 PWM으로 제어
void* printLED()
{
  long long now, lmR, lmG, lmY;
  struct timeb milli_now;
  time_t sec_now;
  int lifeRatio = 10000000 / life;

  while(p2EndSignal)
  {
    time(&sec_now);
    ftime(&milli_now);
    now = sec_now * 1000 + milli_now.millitm;
    now -= start_time;

    PWMWriteDutyCycle(LIFE, 10000000 - (dcnt*lifeRatio));

    if(RRon)
    {
      GPIOWrite(R, true);

      if(lmR + 300 < now)
      {      
          GPIOWrite(R, false);
          RRon = false;
      }
    }
    else
      lmR = now;

    if(GGon)
    {
      GPIOWrite(G, true);

      if(lmG + 300< now)
      {
          GPIOWrite(G, false);
          GGon = false;
      }
    }
    else
      lmG = now;

    if(YYon)
    {
      GPIOWrite(Y, true);

      if(lmY + 300 < now)
      {
          GPIOWrite(Y, false);
          YYon = false;
      }
    }
    else
      lmY = now;
  }
}

// Defender가 입력할 때 정보를 틱단위로 dispaly ras-pi에 전송
void* DmakeTick(void *data)
{
  struct timeb milli_now;
  time_t sec_now;
  long long lastModify =0;
  int now;

  time(&sec_now);
  ftime(&milli_now);

    while(p2EndSignal)
    {
        time(&sec_now);
        ftime(&milli_now);
        now = sec_now * 1000 + milli_now.millitm;

        if(lastModify + TICK <= now)
        {
            char m[2];
            lastModify = now;
            m[0] = bits;
            write(dis_sock, m, 1);
            bits = 0;
        }
        
    }
}

// Attacker가 입력할 때의 정보를 틱단위로 display ras-pi에 전송
void* AmakeTick(void *data)
{
  struct timeb milli_now;
  time_t sec_now;
  long long lastModify=0, now;

  time(&sec_now);
  ftime(&milli_now);
  while(p1EndSignal)
    {
        time(&sec_now);
        ftime(&milli_now);
        now = sec_now * 1000 + milli_now.millitm;

        if(lastModify + TICK <= now)
        {
            char m[2];
            lastModify = now;
            m[0] = bits;
            //notes[idx++] = bits;
            write(dis_sock, m, 1);
            
            bits = 0;
        }
        
    }
}

void printLCD()
{
    ClrLcd();
    lcdLoc(LINE1);
    typeln(arr1);
    lcdLoc(LINE2);
    typeln(arr2);
}

int main(int argc, char *argv[]) {

  int state = 1, status, GameTurn = 3s;
  int serv_sock, clnt_sock = -1, score, expectedScore;
  struct sockaddr_in serv_addr, clnt_addr;
  socklen_t clnt_addr_size;
  char sline[2], sig[2];
  char p0[] = "0", p1[] = "1", p2[] = "2", p3[] = "3", p4[]="4"; 
  struct timeb milli_now;
  time_t sec_now;
  pthread_t p_thread[10];

    if (GPIOExport(R) == -1 || GPIOExport(G) == -1 || GPIOExport(Y) == -1) {
    return 1;
  }

    if (GPIODirection(R, OUT) == -1 || GPIODirection(G, OUT) == -1 || GPIODirection(Y, OUT) == -1) {
    return 2;
  }

  PWMExport(LIFE);
  PWMWritePeriod(LIFE, 10000000);
  PWMWriteDutyCycle(LIFE, 0);
  PWMEnable(LIFE);

   if (wiringPiSetup () == -1) exit (1);

  fd = wiringPiI2CSetup(I2C_ADDR);

  lcd_init(); // setup LCD

  strcpy(arr1,"    Connect");
  strcpy(arr2,"   Waiting...");
  
  printLCD();

  if (argc != 2) {
    printf("Usage : %s <port>\n", argv[0]);
  }
  serv_sock = socket(PF_INET, SOCK_STREAM, 0);
  if (serv_sock == -1) error_handling("socket() error");

  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_addr.sin_port = htons(atoi(argv[1]));

  if (bind(serv_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
    error_handling("bind() error");

  if (listen(serv_sock, 5) == -1) error_handling("listen() error");

  int str_len, line, time;

  clnt_addr_size = sizeof(clnt_addr);
  

  // Attacker socket accept
  p1_sock = accept(serv_sock, (struct sockaddr *)&clnt_addr, &clnt_addr_size);
  printf("P1 Connection established\n");
  strcpy(arr1,"    Player1");
  strcpy(arr2,"    Conneted");
  printLCD();
  
  // Defender socket accept
  p2_sock = accept(serv_sock, (struct sockaddr *)&clnt_addr, &clnt_addr_size);
  printf("P2 Connection established\n");
  strcpy(arr1,"    Player2");
  strcpy(arr2,"    Connected");
  printLCD();

  // Display socket accept
  dis_sock = accept(serv_sock, (struct sockaddr *)&clnt_addr, &clnt_addr_size);
  printf("Dlsplay Connection established\n");
  strcpy(arr1,"    Display");
  strcpy(arr2,"    Connected");
  printLCD();
  sleep(1);

  // Game Start
  while(GameTurn--)
  {
    char msg[32] = {0, };
    GameInit();
    strcpy(arr1," Get Ready for");
    strcpy(arr2,"The Next Battle");
    printLCD();
    sig[0] = '1';
    msg[0] = '1';

    write(p1_sock, msg, sizeof(msg));
    
    // Attacker Input Start 
    str_len = read(p1_sock, msg, sizeof(msg));
    if (str_len == -1) error_handling("read() error");
    printf("asdfsadfdsf\n");
    write(dis_sock, sig, 1);
    write(p2_sock, sig, 1);
    
    pthread_create(&p_thread[4], NULL, AmakeTick, (void *)p4);
    
    printf("Attacker Input Start!!\n");
    strcpy(arr1,"    Attacker");
    strcpy(arr2,"  Input Start!!");
    printLCD();
    
    // Attacker에서 종료 신호가 올 때 까지 입력받음
    while(1)
    {
      char msg[32] = {0,};

      str_len = read(p1_sock, msg, sizeof(msg));
      if (str_len == -1) error_handling("read() error1");
      printf("%s\n", msg);
      line = msg[0]-'0';

      if(line == 9)
        break;
      
      noteCnt++;
      time = atoi(msg+1);
      // display에 전송할 line input 정보를 저장
      bits |= (1 << line);
      printf("line : %d\t time : %d\n", line, time);

      // Line Queue에 받은 정보를 push
      push(line, time);           
    }

    life = noteCnt * 0.3;
    printf("life : %d\n", life);
    for(int i=0; i<LINE; i++)
    push(i, (int)1e9);
    
    int t = 8;

    while(t--)
    {
      snprintf(arr1, 15, "      %ds", t);
      strcpy(arr2,"    ");
      printLCD();
      sleep(1);
    }

    write(p2_sock, "1", 1);
    write(p1_sock, "1", 1);

    ftime(&sec_now);
    ftime(&milli_now);
    start_time = sec_now * 1000 + milli_now.millitm;

    p1EndSignal = 0;

    pthread_join(p_thread[4], (void **)&status);

    pthread_create(&p_thread[0], NULL, lineNoteProcessor, (void *)p0);
    pthread_create(&p_thread[1], NULL, lineNoteProcessor, (void *)p1);
    pthread_create(&p_thread[2], NULL, lineNoteProcessor, (void *)p2);
    pthread_create(&p_thread[3], NULL, lineNoteProcessor, (void *)p3);
    pthread_create(&p_thread[5], NULL, printLED, NULL);
    pthread_create(&p_thread[6], NULL, Dultimate, NULL);

    if(!Atk_ult_used)
    {
      pthread_create(&p_thread[7], NULL, Autimate, NULL);
      pthread_detach(p_thread[7]);
    }

    pthread_detach(p_thread[0]);
    pthread_detach(p_thread[1]);
    pthread_detach(p_thread[2]);
    pthread_detach(p_thread[3]);
    pthread_detach(p_thread[5]);
    pthread_detach(p_thread[6]);

    strcpy(arr1,"    Defender");
    strcpy(arr2,"  Input Start!!");
    printLCD();

    // Def Game start
    bits = 0;

    pthread_create(&p_thread[8], NULL, DmakeTick, NULL);
    pthread_detach(p_thread[8]);
    PWMWriteDutyCycle(LIFE, 10000000);

    // Defender에서 종료 신호가 올 때 까지 입력받음
    while(1)
    {
        str_len = read(p2_sock, msg, sizeof(msg));
        if (str_len == -1) error_handling("read() error");
        printf("%s\n", msg);
        line = msg[0] - '0';
        
        if(line == 9){
          p2EndSignal = 0;
          break;
        }

        else if(line == 6 && !Def_ult_used)
        {
            Def_ult_on = true;
            Def_ult_used = true;
            continue;
        }

        comp[line]= atoi(msg+1);
        printf("line : %d  time : %d\n", line, comp[line]);
        runThread[line] = true;
    }

    if(!Atk_ult_on)
        pthread_cancel(p_thread[7]);
    else    
        Atk_ult_used = true;

    score = 100 * perfectCnt + goodCnt * 50;
    expectedScore = noteCnt * 50;

    snprintf(arr1, 16,"p:%d g:%d m:%d", perfectCnt, goodCnt, missCnt);
    snprintf(arr2, 16,"  Score : %d", score);
    printLCD();

    GPIOWrite(R, 0);
    GPIOWrite(G, 0);
    GPIOWrite(Y, 0);
    PWMWriteDutyCycle(LIFE, 0);
    printf("%d %d\n", noteCnt, life);
    sleep(3);


    if(life > missCnt || expectedScore < score)
    {
      strcpy(arr1,"Defender");
      strcpy(arr2,"         Win!!!");
      printLCD();
      Def_point++;
    }
    else
    {
      strcpy(arr1,"Attacker");
      strcpy(arr2,"          Win!!!");
      printLCD();
      Atk_point++;
    }
    
    printLCD();

    sleep(5);
  }

    strcpy(arr1,"     Game");
    strcpy(arr2,"      END");
    printLCD();
    
    sleep(3);

    if(Atk_point > Def_point)
    {
      strcpy(arr1,"Attacker");
      strcpy(arr2,"          Win!!!");
    }
    else
    {
      strcpy(arr1,"Defender");
      strcpy(arr2,"         Win!!!");
    }

    printLCD();

  
  if (GPIOUnexport(R) == -1 || GPIOUnexport(G) == -1 || GPIOUnexport(Y) == -1) {
    return 4;
  }

  close(p1_sock);
  close(p2_sock);
  close(dis_sock);
  close(serv_sock);

  return (0);
}
