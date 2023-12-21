# SystemProgramming8

컴파일방법
server.c : gcc -o server server.c -lwiringPi -pthread

attacker.c : gcc -o attacker attacker.c -lwiringPi -pthread

defender.c : gcc -o defender defender.c -lwiringPi -pthread

display.c : gcc -o display display.c -lwiringPi

실행방법 : 다음과 같은 순서대로 파일을 실행한다.
1. server.c에 ip 및 포트번호를 설정한다.
 : ./server server_ip server_portnum -lwiringPi -pthread

2. attacker.c에 server의 ip 및 포트번호를 입력한다.
 : ./attacker server_ip server_portnum -lwiringPi -pthread

3. defender.c에 server의 ip 및 포트번호를 입력한다.
 : ./defender server_ip server_portnum -lwiringPi -pthread

4. dispaly.c에 server의 ip 및 포트번호를 입력한다.
 : ./dispaly server_ip server_portnum -lwiringPi
