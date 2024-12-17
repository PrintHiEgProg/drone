// Подключение необходимых библиотек
#include <Wire.h>  // Для работы с I2C
#include <Adafruit_Sensor.h>  // Общая библиотека для работы с датчиками Adafruit
#include <Adafruit_L3GD20_U.h>  // Для работы с гироскопом L3GD20
#include <Adafruit_MPU6050.h>  // Для работы с акселерометром и гироскопом MPU6050
#include <SoftwareSerial.h>  // Для работы с GPS через SoftwareSerial
#include <TinyGPS++.h>  // Для обработки данных GPS
#include <Servo.h>  // Для управления сервомоторами
#include "config.h"  // Пользовательский файл конфигурации (например, пины и константы)

// Объявление объектов для управления сервомоторами
Servo motor1, motor2, motor3, motor4;

// Объявление объектов для работы с датчиками
Adafruit_MPU6050 mpu;  // Объект для работы с MPU6050
Adafruit_L3GD20_Unified gyro;  // Объект для работы с гироскопом L3GD20

// Объявление объекта для работы с GPS через SoftwareSerial
SoftwareSerial ss(4, 3);  // RX на пине 4, TX на пине 3
TinyGPSPlus gps;  // Объект для обработки данных GPS

// Инициализация переменных для координат точек A и B
float latA = LAT_A;  // Широта точки A (определяется в config.h)
float lonA = LON_A;  // Долгота точки A (определяется в config.h)
float latB = LAT_B;  // Широта точки B (определяется в config.h)
float lonB = LON_B;  // Долгота точки B (определяется в config.h)

void setup() {
  // Инициализация последовательного порта для отладки
  Serial.begin(9600);

  // Инициализация SoftwareSerial для работы с GPS
  ss.begin(GPS_BAUD_RATE);  // Скорость обмена данными с GPS (определяется в config.h)

  // Подключение сервомоторов к соответствующим пинам
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);

  // Настройка пина для светодиода как выхода
  pinMode(LED_PIN, OUTPUT);

  // Инициализация MPU6050 (акселерометр и гироскоп)
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);  // Если не удалось инициализировать, зависнуть в бесконечном цикле
  }

  // Инициализация гироскопа L3GD20
  if (!gyro.begin()) {
    Serial.println("Failed to initialize Gyroscope!");
    while (1);  // Если не удалось инициализировать, зависнуть в бесконечном цикле
  }

  // Инициализация GPS модуля
  Serial.println("Initializing GPS...");
  if (!gps.begin(ss)) {
    Serial.println("GPS module not detected. Please check wiring.");
    while (1);  // Если GPS не обнаружен, зависнуть в бесконечном цикле
  }

  // Калибровка датчиков
  Serial.println("Calibrating sensors...");
  delay(5000);  // Пауза на 5 секунд для калибровки
  Serial.println("Calibration completed.");

  // Включение светодиода для индикации завершения инициализации
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Чтение данных GPS, если они доступны
  while (ss.available() > 0) {
    gps.encode(ss.read());  // Обработка данных GPS
  }

  // Проверка, обновлена ли информация о местоположении
  if (gps.location.isUpdated()) {
    float currentLat = gps.location.lat();  // Текущая широта
    float currentLon = gps.location.lng();  // Текущая долгота

    // Вывод текущего местоположения в последовательный порт
    Serial.print("Current Location: ");
    Serial.print("Lat: "); Serial.print(currentLat, 6);
    Serial.print(" Lon: "); Serial.println(currentLon, 6);

    // Если текущее местоположение не совпадает с точкой B
    if (currentLat != latB && currentLon != lonB) {
      float distance = getDistance(currentLat, currentLon, latB, lonB);  // Расчет расстояния до точки B
      float bearing = getBearing(currentLat, currentLon, latB, lonB);  // Расчет направления до точки B

      // Вывод расстояния и направления в последовательный порт
      Serial.print("Distance to Target: ");
      Serial.print(distance); Serial.println(" km");

      Serial.print("Bearing to Target: ");
      Serial.println(bearing);

      // Управление моторами в зависимости от направления
      controlMotors(bearing);
    }
  }
}

// Функция для расчета расстояния между двумя точками на Земле
float getDistance(float lat1, float lon1, float lat2, float lon2) {
  float lat1Rad = radians(lat1);  // Преобразование широты в радианы
  float lon1Rad = radians(lon1);  // Преобразование долготы в радианы
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);

  float dlat = lat2Rad - lat1Rad;  // Разница в широте
  float dlon = lon2Rad - lon1Rad;  // Разница в долготе

  // Формула гаверсинусов для расчета расстояния
  float a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1Rad) * cos(lat2Rad) * sin(dlon / 2) * sin(dlon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = 6371 * c;  // Расстояние в километрах

  return distance;
}

// Функция для расчета направления (азимута) между двумя точками
float getBearing(float lat1, float lon1, float lat2, float lon2) {
  float lat1Rad = radians(lat1);  // Преобразование широты в радианы
  float lon1Rad = radians(lon1);  // Преобразование долготы в радианы
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);

  float dLon = lon2Rad - lon1Rad;  // Разница в долготе

  // Формула для расчета азимута
  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);

  float bearing = atan2(y, x);  // Азимут в радианах
  bearing = degrees(bearing);  // Преобразование в градусы

  if (bearing < 0) bearing += 360;  // Приведение к диапазону 0-360 градусов

  return bearing;
}

// Функция для управления моторами в зависимости от направления
void controlMotors(float bearing) {
  int baseSpeed = BASE_SPEED;  // Базовая скорость моторов (определяется в config.h)

  // Управление моторами в зависимости от направления (bearing)
  if (bearing < 90) {
    motor1.write(baseSpeed + 20);  // Увеличение скорости для поворота вправо
    motor2.write(baseSpeed);
    motor3.write(baseSpeed);
    motor4.write(baseSpeed + 20);
  } else if (bearing >= 90 && bearing < 180) {
    motor1.write(baseSpeed);
    motor2.write(baseSpeed + 20);  // Увеличение скорости для поворота влево
    motor3.write(baseSpeed + 20);
    motor4.write(baseSpeed);
  } else if (bearing >= 180 && bearing < 270) {
    motor1.write(baseSpeed + 20);  // Увеличение скорости для поворота вправо
    motor2.write(baseSpeed);
    motor3.write(baseSpeed);
    motor4.write(baseSpeed + 20);
  } else {
    motor1.write(baseSpeed);
    motor2.write(baseSpeed + 20);  // Увеличение скорости для поворота влево
    motor3.write(baseSpeed + 20);
    motor4.write(baseSpeed);
  }
}
