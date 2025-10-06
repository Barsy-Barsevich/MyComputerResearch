#include "MPU9250_Barsotion.h"
#include "TWIEX_Barsotion.h"

MPU9250 mpu;

//Глобальная переменная mode определяет ступень TWI-последовательности
volatile uint8_t mode;

//TWI-функции
void TWI_00() {twi.start();}
void TWI_01() {twi.write(0x68<<1);}
void TWI_02() {twi.write(MPU_ACCEL_XOUT_H);}
void TWI_03() {twi.start();}
void TWI_04() {twi.write((0x68<<1)|1);}
void TWI_05() {twi.request(true);}
void TWI_06() {mpu.rawaccel[0] = twi.read(); twi.request(true);}
void TWI_07() {mpu.rawaccel[1] = twi.read(); twi.request(true);}
void TWI_08() {mpu.rawaccel[2] = twi.read(); twi.request(true);}
void TWI_09() {mpu.rawaccel[3] = twi.read(); twi.request(true);}
void TWI_10() {mpu.rawaccel[4] = twi.read(); twi.request(true);}
void TWI_11() {mpu.rawaccel[5] = twi.read(); /*twi.end();}*/twi.request(true);}
void TWI_12() {mpu.rawtemp[0] = twi.read(); twi.request(true);}
void TWI_13() {mpu.rawtemp[1] = twi.read(); twi.request(true);}
void TWI_14() {mpu.rawgyro[0] = twi.read(); twi.request(true);}
void TWI_15() {mpu.rawgyro[1] = twi.read(); twi.request(true);}
void TWI_16() {mpu.rawgyro[2] = twi.read(); twi.request(true);}
void TWI_17() {mpu.rawgyro[3] = twi.read(); twi.request(true);}
void TWI_18() {mpu.rawgyro[4] = twi.read(); twi.request(false);}
void TWI_19() {mpu.rawgyro[5] = twi.read(); twi.end();}//*/

//Массив указателей на TWI-функции
void (*twi_queue[20])();

void setup() {
  //Инициализация
  twi_queue[0] = TWI_00;
  twi_queue[1] = TWI_01;
  twi_queue[2] = TWI_02;
  twi_queue[3] = TWI_03;
  twi_queue[4] = TWI_04;
  twi_queue[5] = TWI_05;
  twi_queue[6] = TWI_06;
  twi_queue[7] = TWI_07;
  twi_queue[8] = TWI_08;
  twi_queue[9] = TWI_09;
  twi_queue[10] = TWI_10;
  twi_queue[11] = TWI_11;
  twi_queue[12] = TWI_12;
  twi_queue[13] = TWI_13;
  twi_queue[14] = TWI_14;
  twi_queue[15] = TWI_15;
  twi_queue[16] = TWI_16;
  twi_queue[17] = TWI_17;
  twi_queue[18] = TWI_18;
  twi_queue[19] = TWI_19;//*/
  Serial.begin(1000000);
  Serial.print(mpu.begin(0x68));
  Serial.println(mpu.setGyroScale(GYRO_SCALE_250));
  Serial.println(mpu.setAccelScale(ACCEL_SCALE_2G));
  //Установка времени ошибки
  twi.setErrorTime(50);
}

void loop() {
  //Старт TWI-последовательности
  mode = 1;
  twi.startQueue();
  /*Serial.print(twi.readRegister(0x68, MPU_GYRO_XOUT_L, mpu.rawgyro+0));
  Serial.print(twi.readRegister(0x68, MPU_GYRO_XOUT_H, mpu.rawgyro+1));
  Serial.print(twi.readRegister(0x68, MPU_GYRO_YOUT_L, mpu.rawgyro+2));
  Serial.print(twi.readRegister(0x68, MPU_GYRO_YOUT_H, mpu.rawgyro+3));
  Serial.print(twi.readRegister(0x68, MPU_GYRO_ZOUT_L, mpu.rawgyro+4));
  Serial.print(twi.readRegister(0x68, MPU_GYRO_ZOUT_H, mpu.rawgyro+5));
  Serial.print(" ");*/
  float buf[3];
  mpu.readGyro(buf);
  Serial.print(buf[0]);
  Serial.print(" ");
  Serial.print(buf[1]);
  Serial.print(" ");
  Serial.print(buf[2]);
  Serial.print(" ");
  mpu.readAccel(buf);
  Serial.print(buf[0]);
  Serial.print(" ");
  Serial.print(buf[1]);
  Serial.print(" ");
  Serial.print(buf[2]);
  Serial.print("\n");
  
}

ISR(TIMER2_COMPA_vect) {
  //Принудительное завершение транзакции передачи данных по шине TWI
  twi.end();
  Serial.println(F("Error!"));
}
ISR(TWI_vect) {
  //Вызов TWI-функции под номером mode из массива
  twi_queue[mode]();
  //Инкремент mode
  mode += 1;
}
