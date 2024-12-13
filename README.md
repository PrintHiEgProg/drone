# Техническая документация

## 1. Введение

Этот проект представляет собой систему на базе Arduino, которая управляет четырьмя сервомоторами для навигации между двумя географическими точками. Система использует данные GPS, гироскопа и акселерометра для определения текущего местоположения и направления движения.

## 2. Функциональность

- **GPS-навигация:** Система получает данные о текущем местоположении с помощью модуля GPS и вычисляет расстояние и направление до целевой точки.
- **Управление моторами:** В зависимости от направления движения (bearing), система регулирует скорость и направление вращения четырех сервомоторов.
- **Интеграция с датчиками:** Используются гироскоп и акселерометр для дополнительной стабилизации и коррекции движения.

## 3. Используемые библиотеки

- `Wire.h`: Для работы с I2C-устройствами.
- `Adafruit_Sensor.h`: Базовая библиотека для работы с датчиками Adafruit.
- `Adafruit_L3GD20_U.h`: Для работы с гироскопом L3GD20.
- `Adafruit_MPU6050.h`: Для работы с акселерометром и гироскопом MPU6050.
- `SoftwareSerial.h`: Для работы с GPS-модулем через последовательный порт.
- `TinyGPS++.h`: Для обработки данных GPS.
- `Servo.h`: Для управления сервомоторами.

## 4. Подключение оборудования

- **GPS-модуль:** Подключен к пинам 4 (RX) и 3 (TX) через SoftwareSerial.
- **MPU6050:** Подключен по I2C (SDA и SCL).
- **L3GD20:** Подключен по I2C (SDA и SCL).
- **Сервомоторы:** Подключены к пинам 3, 5, 6 и 9.

## 5. Описание функций

### 5.1 `setup()`

Инициализация всех компонентов системы:

- Настройка последовательного порта для отладки.
- Инициализация GPS-модуля.
- Инициализация сервомоторов.
- Инициализация гироскопа и акселерометра.

### 5.2 `loop()`

Основной цикл программы:

- Чтение данных GPS.
- Обновление текущего местоположения.
- Вычисление расстояния и направления до целевой точки.
- Управление сервомоторами в зависимости от направления движения.

### 5.3 `getDistance(float lat1, float lon1, float lat2, float lon2)`

Вычисляет расстояние между двумя географическими точками с использованием формулы гаверсинусов.

### 5.4 `getBearing(float lat1, float lon1, float lat2, float lon2)`

Вычисляет направление (bearing) от текущей точки до целевой точки.

### 5.5 `controlMotors(float bearing)`

Регулирует скорость и направление вращения сервомоторов в зависимости от направления движения.

## 6. Пример использования

1. Подключите все компоненты к Arduino в соответствии с описанием.
2. Загрузите код на Arduino.
3. Откройте монитор последовательного порта для отладки.
4. Система начнет движение к целевой точке (latB, lonB).

## 7. Зависимости

- Arduino IDE (рекомендуемая версия 1.8.x или выше).
- Установленные библиотеки Adafruit MPU6050, Adafruit L3GD20, TinyGPS++.

## 8. Возможные улучшения

- Добавить обработку ошибок для GPS-модуля.
- Реализовать адаптивную скорость моторов в зависимости от расстояния до цели.
- Интегрировать дополнительные датчики для повышения точности навигации.

## 9. Код нашей программы

```cpp
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Servo.h>

#define MOTOR1_PIN 3
#define MOTOR2_PIN 5
#define MOTOR3_PIN 6
#define MOTOR4_PIN 9
Adafruit_MPU6050 mpu;
Adafruit_L3GD20_Unified gyro;

SoftwareSerial ss(4, 3); //rx, rt
TinyGPSPlus gps;

Servo motor1, motor2, motor3, motor4;

float latA = 40.7128;
float lonA = -74.0060;
float latB = 40.7306;
float lonB = -73.9352;

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }

  if (!gyro.begin()) {
    Serial.println("Failed to initialize Gyroscope!");
    while (1);
  }
  
  Serial.println("Initializing GPS...");
  delay(1000);
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  if (gps.location.isUpdated()) {
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();
    
    Serial.print("Current Location: ");
    Serial.print("Lat: "); Serial.print(currentLat, 6);
    Serial.print(" Lon: "); Serial.println(currentLon, 6);
    
    if (currentLat != latB && currentLon != lonB) {
      float distance = getDistance(currentLat, currentLon, latB, lonB);
      float bearing = getBearing(currentLat, currentLon, latB, lonB);
      
      controlMotors(bearing);
    }
  }
}

float getDistance(float lat1, float lon1, float lat2, float lon2) {
  float lat1Rad = radians(lat1);
  float lon1Rad = radians(lon1);
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);
  
  float dlat = lat2Rad - lat1Rad;
  float dlon = lon2Rad - lon1Rad;
  
  float a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1Rad) * cos(lat2Rad) * sin(dlon / 2) * sin(dlon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = 6371 * c; 
  
  return distance;
}

float getBearing(float lat1, float lon1, float lat2, float lon2) {
  float lat1Rad = radians(lat1);
  float lon1Rad = radians(lon1);
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);
  
  float dLon = lon2Rad - lon1Rad;
  
  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
  
  float bearing = atan2(y, x);
  bearing = degrees(bearing);
  if (bearing < 0) bearing += 360;
  
  return bearing;
}

void controlMotors(float bearing) {
  int baseSpeed = 1500;
  
  if (bearing < 90) {
    motor1.write(baseSpeed + 20);
    motor2.write(baseSpeed);
    motor3.write(baseSpeed); 
    motor4.write(baseSpeed + 20); 

    } else if (bearing >= 90 && bearing < 180) {
    motor1.write(baseSpeed);
    motor2.write(baseSpeed + 20); 
    motor3.write(baseSpeed + 20); 
    motor4.write(baseSpeed); 
  } else if (bearing >= 180 && bearing < 270) {
    motor1.write(baseSpeed + 20); 
    motor2.write(baseSpeed); 
    motor3.write(baseSpeed); 
    motor4.write(baseSpeed + 20); 
  } else {
    motor1.write(baseSpeed); 
    motor2.write(baseSpeed + 20); 
    motor3.write(baseSpeed + 20); 
    motor4.write(baseSpeed); 
  }
}
```

## 10. Спасибо РКСИ!

Проект сделан при поддержке Ростовского Колледжа Связи и Информатики. Хочется отметить Трищук Софию Артёмовну за предоставление этого задания и Сулавко Сергея Николаевича за предоставление среды для тестирования - Wokwi.