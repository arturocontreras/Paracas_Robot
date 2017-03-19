
#include "DHT.h"
#define DHTPIN 53
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);



float read_temp_amb(){
  float t = dht.readTemperature();
  return t;
}

float read_hum_amb(){
  float h = dht.readHumidity();
  return h;
}
  

//  if (isnan(t) && isnan(h)) {
//    Serial.println("Failed to read from DHT");
//  } else {
//    Serial.print("Humidity: ");
//    Serial.print(h);
//    Serial.print(" %\t");
//    Serial.print("Temperature: ");
//    Serial.print(t);
//    Serial.println(" *C");
//  }
//  delay(1000);
//}
