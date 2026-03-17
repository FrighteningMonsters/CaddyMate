# P2P WiFi Direct 스크립트 배치 방법

## 파일 구조

```
HTML-CaddyMate-GUI-main/
└── scripts/
    ├── turtlebot_p2p_setup.sh   ← TurtleBot용
    ├── pi_p2p_connect.sh       ← Pi용
    └── README_P2P.md           ← 이 파일
```

---

## TurtleBot에 배치

### 방법 1: USB로 복사
1. PC에서 `turtlebot_p2p_setup.sh`를 USB 드라이브에 복사
2. TurtleBot에 USB 연결
3. TurtleBot에서:
```bash
cp /media/ubuntu/*/turtlebot_p2p_setup.sh ~/
chmod +x ~/turtlebot_p2p_setup.sh
```

### 방법 2: SCP로 전송 (Dice Machine에서)
```bash
scp scripts/turtlebot_p2p_setup.sh ubuntu@192.168.105.74:~/
ssh ubuntu@192.168.105.74 chmod +x ~/turtlebot_p2p_setup.sh
```

### 실행
```bash
bash ~/turtlebot_p2p_setup.sh
```

---

## Pi에 배치

### 방법 1: USB로 복사
1. PC에서 `pi_p2p_connect.sh`를 USB 드라이브에 복사
2. Pi에 USB 연결
3. Pi에서:
```bash
cp /media/pi/*/pi_p2p_connect.sh ~/
chmod +x ~/pi_p2p_connect.sh
```

### 방법 2: Pi가 이미 git clone 되어 있다면
```bash
cd ~/turtlebot3-main
cp scripts/pi_p2p_connect.sh ~/
chmod +x ~/pi_p2p_connect.sh
```

### 실행
```bash
bash ~/pi_p2p_connect.sh
```

---

## 실행 순서

1. **TurtleBot**에서 `bash ~/turtlebot_p2p_setup.sh` 실행
2. **60초 이내에** **Pi**에서 `bash ~/pi_p2p_connect.sh` 실행
3. 성공 시 "P2P connected" 메시지 출력
