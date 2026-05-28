#include <QTRSensors.h>

// Піни моторів
#define MotorA1 4
#define MotorA2 5
#define MotorA_PWM 3 

#define MotorB1 7
#define MotorB2 8
#define MotorB_PWM 9

// Пін standby драйвера моторів
#define STBY 6

// Пін кнопки
#define BUTTON_PIN 2


QTRSensors qtr;

// Кількість датчиків
const uint8_t SensorCount = 6;

// Масив для значень датчиків
uint16_t sensorValues[SensorCount];


// Попередня помилка для D-складової PID
int lastError = 0;


// PID коефіцієнти
const float Kp = 0.11; 
const float Kd = 0.66; 

// Базова швидкість
const int baseSpeed = 240; 

// Максимальна швидкість
const int maxSpeed = 255;

// Швидкість для плавного прискорення
int Accel_Speed = baseSpeed;


// Стани роботи боліда
enum State {

  // Очікування запуску калібрування
  WAITING_FOR_CALIBRATION,

  // Процес калібрування датчиків
  CALIBRATING,

  // Очікування старту
  WAITING_FOR_START,

  // Рух по трасі
  RUNNING,

  // Повна зупинка
  STOPPED
};

State currentState = WAITING_FOR_CALIBRATION;


// Час початку калібрування
unsigned long calibrationStartTime = 0;

// Тривалість калібрування
const unsigned long CALIBRATION_DURATION = 8000;

// Тривалість заїзду
const unsigned long RUN_DURATION = 35000;

// Час старту руху
unsigned long runStartTime = 0;


// Режими завершення
// true  -> автостоп через 20 секунд
// false -> зупинка кнопкою
const bool VARIANT_1 = true;


void setup() {

  // Запуск Serial Monitor
  Serial.begin(9600);

  delay(500);

  Serial.println("\n=== QTR-6 Dynamic Line Follower ===\n");
  
  // Налаштування пінів моторів
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorA_PWM, OUTPUT);

  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  pinMode(MotorB_PWM, OUTPUT);

  // Налаштування кнопки
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Повна зупинка моторів при запуску
  moveMotors(0, 0);

  // Виведення драйвера зі standby режиму
  digitalWrite(STBY, HIGH);
  
  // Налаштування типу QTR датчиків
  qtr.setTypeRC();

  // Підключення датчиків
  qtr.setSensorPins((const uint8_t[]){A5, A4, A3, A2, A1, 12}, SensorCount);

  // Пін керування підсвіткою
  qtr.setEmitterPin(10);
 
  Serial.println("Готово! Натисніть кнопку для калібрування...\n");

  Serial.print("Режим: ");

  Serial.println(VARIANT_1 ?
                 "Автостоп через 20 секунд" :
                 "Зупинка кнопкою");
}


void loop() {

  // Обробка натискання кнопки

  static bool lastButtonState = HIGH;

  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  // Виявлення натискання кнопки
  // (перехід HIGH -> LOW)
  if (currentButtonState == LOW && lastButtonState == HIGH) {

    // Захист від дребезгу кнопки
    delay(50);
    
    // Запуск калібрування
    if (currentState == WAITING_FOR_CALIBRATION) {

      currentState = CALIBRATING;

      calibrationStartTime = millis();

      Serial.println("Калібрування розпочато");
    } 
    
    // Початок руху
    else if (currentState == WAITING_FOR_START) {

      // Затримка перед стартом
      delay(2000);

      currentState = RUNNING;

      runStartTime = millis();

      Serial.println("Старт!");
    }

    // Зупинка кнопкою під час руху
    else if (currentState == RUNNING && !VARIANT_1) {

      currentState = STOPPED;

      Serial.println("Ручна зупинка");
    }

    // Повернення у режим готовності
    else if (currentState == STOPPED) {

      currentState = WAITING_FOR_START;

      Serial.println("Готово до нового старту!");
    }
  }

  lastButtonState = currentButtonState;
  
  
  // Основна логіка автомата станів
  switch (currentState) {

    // Очікування калібрування
    case WAITING_FOR_CALIBRATION:

      moveMotors(0, 0);

      break;
      

    // Калібрування датчиків
    case CALIBRATING:

      performAutoCalibration();

      break;
      

    // Очікування старту
    case WAITING_FOR_START:

      moveMotors(0, 0);

      break;
      

    // Рух по лінії
    case RUNNING:
     
      // Основний PID алгоритм
      followLine();
      
      // Автоматична зупинка по таймеру
      if (VARIANT_1 &&
          (millis() - runStartTime >= RUN_DURATION)) {

        currentState = STOPPED;

        Serial.println("Час завершився -> Стоп");
      }

      break;
      

    // Повна зупинка
    case STOPPED:

      moveMotors(0, 0);

      break;
  }
}


void performAutoCalibration() {

  // Час від початку калібрування
  unsigned long elapsedTime =
      millis() - calibrationStartTime;
  
  // Завершення калібрування
  if (elapsedTime >= CALIBRATION_DURATION) {

    moveMotors(0, 0);

    currentState = WAITING_FOR_START;

    Serial.println("Калібрування завершено. Готово до старту!");

    return;
  }
  
  // Рух вліво-вправо на місці
  // для калібрування датчиків

  unsigned long cyclePosition =
      elapsedTime % 1000;

  int speed = baseSpeed / 4;
  
  // Поворот вправо
  if (cyclePosition < 250)
    moveMotors(speed, -speed);

  // Повернення до центру
  else if (cyclePosition < 500)
    moveMotors(-speed, speed);

  // Поворот вліво
  else if (cyclePosition < 750)
    moveMotors(-speed, speed);

  // Повернення до центру
  else
    moveMotors(speed, -speed);
  
  // Калібрування датчиків
  qtr.calibrate();
}


void followLine() {

  // Зчитування позиції лінії
  int position = qtr.readLineBlack(sensorValues);

  // Відхилення від центру
  int sensorError = position - 2500;

  // Основна PID помилка
  int error = sensorError;
  

  // Тут була спроба реалізації
  // динамічного зміщення центру
  // (залишено закоментованим)

  // if (abs(sensorError) >= 400 &&
  //     abs(sensorError) < 1600) {

  //   if (sensorError > 0)
  //     error = (sensorError + 400) / 2;
  // }

  // else {
  //   error = (sensorError - 400) / 2;
  // }


  // D-складова PID
  int derivative = error - lastError;

  // Зберігаємо помилку
  lastError = error;

  // PID регулювання швидкості
  int motorSpeedAdjustment =
      (error * Kp) + (derivative * Kd);


  // Плавний розгін на прямих ділянках
  if (abs(sensorError) < 400
  ) {

    // Поступове збільшення швидкості
    if (Accel_Speed < maxSpeed)
      Accel_Speed += 5;

  } else {

    // На поворотах повертаємось
    // до базової швидкості
    Accel_Speed = baseSpeed;
  }


  // Розрахунок швидкостей моторів
  int leftMotorSpeed =
      constrain(Accel_Speed + motorSpeedAdjustment,
                -maxSpeed,
                maxSpeed);

  int rightMotorSpeed =
      constrain(Accel_Speed - motorSpeedAdjustment,
                -maxSpeed,
                maxSpeed);
  
  // Передача швидкостей моторам
  moveMotors(leftMotorSpeed, rightMotorSpeed);
}


void moveMotors(int leftSpeed, int rightSpeed) {

  // Керування лівим мотором

  // Рух вперед
  if (leftSpeed >= 0) {

    digitalWrite(MotorA1, HIGH);
    digitalWrite(MotorA2, LOW);

    analogWrite(MotorA_PWM, leftSpeed);

  } else {

    // Рух назад
    digitalWrite(MotorA1, LOW);
    digitalWrite(MotorA2, HIGH);

    analogWrite(MotorA_PWM, abs(leftSpeed));
  }


  // Керування правим мотором

  // Рух вперед
  if (rightSpeed >= 0) {

    digitalWrite(MotorB1, HIGH);
    digitalWrite(MotorB2, LOW);

    analogWrite(MotorB_PWM, rightSpeed);

  } else {

    // Рух назад
    digitalWrite(MotorB1, LOW);
    digitalWrite(MotorB2, HIGH);

    analogWrite(MotorB_PWM, abs(rightSpeed));
  }
}