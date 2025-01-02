// PID Kontrol Değişkenleri
double setPoint = 0;    // İstenilen hız (potansiyometreden gelen)
double input = 0;       // Ölçülen hız (encoder'dan gelen)
double output = 0;      // PWM çıkışı (PID hesaplaması)

// PID Katsayıları
double Kp = 5, Ki = 0.05, Kd = 0.1; // PID ayarları (daha hassas kontrol için optimize edildi)

// PID Hesaplama Değişkenleri
double previousError = 0;
double integral = 0;

// Set Point Filtreleme
double smoothedSetPoint = 0; // Filtrelenmiş set point
const double smoothingFactor = 0.1; // 0-1 arasında bir değer (daha düşük = daha fazla yumuşatma)

// Pin Tanımlamaları
const int analogPin = A1;   // Potansiyometre
const int motorPin = 10;     // Motor PWM çıkışı
const int encoderPin = 2;   // Encoder girişi

// Encoder Değişkenleri
volatile long encoderCount = 0; // Encoder darbeleri
unsigned long prevTime = 0;     // Zaman ölçümü için
const int encoderPulsePerRevolution = 1000; // Encoder için doğru değer

void setup() {
  // Pin ayarları
  pinMode(analogPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(encoderPin, INPUT_PULLUP);

  // Encoder kesmesi
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulses, RISING);

  // Seri Haberleşme
  Serial.begin(9600);
}

void loop() {
  // Potansiyometreden set point değerini oku
  int potValue = analogRead(analogPin);
  double targetSetPoint = (double)potValue * 1000.0 / 1023.0; // Potansiyometreyi 0-1000 RPM arasında ölçekle

  // Set point değerini yumuşat (filtre uygula)
  smoothedSetPoint += smoothingFactor * (targetSetPoint - smoothedSetPoint);
  setPoint = smoothedSetPoint;

  // PWM çıkışını sınırla
  output = constrain(output, 0, 255);

  // Motor hızını ayarla
  analogWrite(motorPin, output);

  // Hız ayarını hassas yapmak için alt eşik değerini kontrol et
  if (setPoint < 10) {  // 10 RPM altındaki değerlerde motor durduruluyor
    setPoint = 0;
    output = 0;
    integral = 0;
    previousError = 0;
    analogWrite(motorPin, 0);
    Serial.println("Motor Durdu");
    delay(100);
    return;
  }

  // Ölçülen hız (input) değerini hesapla
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 100) { // 100 ms'de bir hız hesapla
    noInterrupts();
    double rpm = (encoderCount * 60000.0) / (encoderPulsePerRevolution * (currentTime - prevTime));
    interrupts();
    input = rpm;
    encoderCount = 0;
    prevTime = currentTime;

    // PID kontrol hesaplaması
    double error = setPoint - input;       // Hata
    integral += error * 0.1;              // Hatanın integralini hesapla
    integral = constrain(integral, -255, 255); // Integral birikimi sınırla
    double derivative = (error - previousError) / 0.1; // Hatanın türevi
    output = (Kp * error) + (Ki * integral) + (Kd * derivative); // PID hesaplama
    previousError = error;

    // PWM çıkışını sınırla
    output = constrain(output, 0, 255);

    // Seri monitöre bilgi yazdır
    Serial.print("Set Point: ");
    Serial.print(setPoint);
    Serial.print(" RPM, Input: ");
    Serial.print(input);
    Serial.print(" RPM, Output: ");
    Serial.println(output);
  }
}

// Encoder kesme fonksiyonu
void countPulses() {
  encoderCount++;
}
