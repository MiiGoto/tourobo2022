#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>

FlexCAN CANTransmitter(1000000);
static CAN_message_t m_msg;
static CAN_message_t gm_msg;

int flag = 0;
float vx, vy, vt;
int u[4] = {};
int u2[4] = {};

void setup() {
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, INPUT);//押下で1
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
  MsTimer2::set(4, timerInt);
  MsTimer2::start();
}

static unsigned long testch[6];

void loop() {
  static int data[18];                      //入力データが入る？
  static int dataNumber = 0;                //入力データの数(Serial1.available()の返値)
  static unsigned long lastConnectTime = 0; //直前の通信の時間?
  if (Serial1.available() > 0) {
    for (int dataNum = Serial1.available(); dataNum > 0; dataNum--) {
      if (dataNumber < 0) {
        Serial1.read();
        dataNumber++;
        continue;
      }
      data[dataNumber % 18] = Serial1.read();
      dataNumber++;
      if (dataNumber > 18) {
        dataNumber = 0;
      }
      else if (dataNumber == 18) {
        testch[0] = (((data[1] & 0x07) << 8) | data[0]);          //ch0(364～1024～1684)
        testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));   //ch1(364～1024～1684)
        testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
        testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));   //ch3(364～1024～1684)
        if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {
          for (int i = 1; i < 18; i++) {
            testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);  //ch0(364～1024～1684)
            testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3)); //ch1(364～1024～1684)
            testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
            testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1)); //ch3(364～1024～1684)
            if (364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684) {
              dataNumber = -i;
              break;
            }
          }
          if (dataNumber > 18) {
            dataNumber = -1;
          }
        }
        else {
          dataNumber = 0;
        }
      }
    }
    digitalWrite(13, !digitalRead(13)); //プロポ受信したらLEDチカチカ
    flag = 1;

    Serial.println("jushin");
  }
  else {
    flag = 0;
  }

  if (((data[5] & 0xC0) >> 6) == 1) {
    m_msg.id = 0x200;
    m_msg.len = 8;
    for (int i = 0; i < 4; i++) {
      u[i] = 0;
    }
    if (testch[3] >= 364 && testch[3] <= 1684) {
      vy = map(testch[3], 364, 1684, -5000, 5000);//目標最大rpmは5000
    }
    if (testch[2] >= 364 && testch[2] <= 1684) {
      vx = map(testch[2], 364, 1684, -5000, 5000);
    }
    if (testch[0] >= 364 && testch[0] <= 1684) {
      vt = map(testch[0], 364, 1684, -5000, 5000);
    }

    //    Serial.print(vx);
    //    Serial.print(",");
    //    Serial.print(vy);
    //    Serial.print(",");
    //    Serial.print(vt);
    //    Serial.print(",");

    u[0] = (int)(min(max(-16000, -vx - vy + vt), 16000));
    u[1] = (int)(min(max(-16000, -vx + vy + vt), 16000));
    u[2] = (int)(min(max(-16000, vx + vy + vt), 16000));
    u[3] = (int)(min(max(-16000, vx - vy + vt), 16000));

    //    Serial.println(u[0]);

    for (int i = 0; i < m_msg.len; i++) {
      m_msg.buf[i * 2] = u[i] >> 8;
      m_msg.buf[i * 2 + 1] = u[i] & 0xFF;
    }
    gm_msg.id = 0x1FF;
    gm_msg.len = 8;
    for (int i = 0; i < 4; i++) {
      u2[i] = 0;
    }
    if (testch[1] >= 364 && testch[1] <= 1684) {
      u2[0] = map(testch[1], 364, 1684, -15000, 15000);
    }
    for (int i = 0; i < gm_msg.len; i++) {
      gm_msg.buf[i * 2] = u2[i] >> 8;
      gm_msg.buf[i * 2 + 1] = u2[i] & 0xFF;
    }
    analogWrite(23, 0);
    analogWrite(21, 0);
  }
  else if (((data[5] & 0xC0) >> 6) == 2) {
    for (int i = 0; i < 4; i++) {
      u[i] = 0;
    }
    for (int i = 0; i < m_msg.len; i++) {
      m_msg.buf[i * 2] = u[i] >> 8;
      m_msg.buf[i * 2 + 1] = u[i] & 0xFF;
    }
    for (int i = 0; i < gm_msg.len; i++) {
      gm_msg.buf[i * 2] = u[i] >> 8;
      gm_msg.buf[i * 2 + 1] = u[i] & 0xFF;
    }
    digitalWrite(7, (map(testch[3], 364, 1684, -250, 250) > 0));
    analogWrite(23, abs(map(testch[3], 364, 1684, -250, 250)));
    digitalWrite(9, (map(testch[1], 364, 1684, -250, 250) > 0));
    analogWrite(21, abs(map(testch[1], 364, 1684, -250, 250)));
  }
  else if (((data[5] & 0xC0) >> 6) == 3) {
    for (int i = 0; i < 4; i++) {
      u[i] = 0;
    }
    for (int i = 0; i < m_msg.len; i++) {
      m_msg.buf[i * 2] = u[i] >> 8;
      m_msg.buf[i * 2 + 1] = u[i] & 0xFF;
    }
    for (int i = 0; i < gm_msg.len; i++) {
      gm_msg.buf[i * 2] = u[i] >> 8;
      gm_msg.buf[i * 2 + 1] = u[i] & 0xFF;
    }
    analogWrite(23, 0);
    analogWrite(21, 0);
  }
  if(digitalRead(2)==1){
    analogWrite(23, 0);
    analogWrite(21, 0);
  }
  delay(50);
}

void timerInt() {
  if(digitalRead(2)==1)flag=0;
  if (flag) {
    CANTransmitter.write(gm_msg);
    CANTransmitter.write(m_msg);
  }
}
