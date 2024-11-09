// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries

// SensESP v3 Engine monitor - Nov 2024 (UNTESTED ON V3)

#include <memory>
#include <Arduino.h>

#include <Adafruit_BMP280.h>
#include <Wire.h>

#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/ui/config_item.h"

#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/transforms/frequency.h"

using namespace sensesp;
using namespace sensesp::onewire;
using namespace reactesp;


class TemperatureInterpreter : public CurveInterpolator {
 public:
  TemperatureInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our temperature sender to degrees Kelvin
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin));
    add_sample(CurveInterpolator::Sample(20, 393.15));
    add_sample(CurveInterpolator::Sample(30, 383.15));
    add_sample(CurveInterpolator::Sample(40, 373.15));
    add_sample(CurveInterpolator::Sample(55, 363.15));
    add_sample(CurveInterpolator::Sample(70, 353.15));
    add_sample(CurveInterpolator::Sample(100, 343.15));
    add_sample(CurveInterpolator::Sample(140, 333.15));
    add_sample(CurveInterpolator::Sample(200, 323.15));
    add_sample(CurveInterpolator::Sample(300, 317.15));
    add_sample(CurveInterpolator::Sample(400, 313.15)); 
  }
};

class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to m3/s
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, m3/s));
    add_sample(CurveInterpolator::Sample(500, 0.00000011));
    add_sample(CurveInterpolator::Sample(1000, 0.00000019));
    add_sample(CurveInterpolator::Sample(1500, 0.0000003));
    add_sample(CurveInterpolator::Sample(1800, 0.00000041));
    add_sample(CurveInterpolator::Sample(2000, 0.00000052));
    add_sample(CurveInterpolator::Sample(2200, 0.00000066));
    add_sample(CurveInterpolator::Sample(2400, 0.00000079));
    add_sample(CurveInterpolator::Sample(2600, 0.00000097));
    add_sample(CurveInterpolator::Sample(2800, 0.00000124));
    add_sample(CurveInterpolator::Sample(3000, 0.00000153));
    add_sample(CurveInterpolator::Sample(3200, 0.00000183));
    add_sample(CurveInterpolator::Sample(3400, 0.000002));
    add_sample(CurveInterpolator::Sample(3800, 0.00000205));  
  }
};

  Adafruit_BMP280 bmp280;

  float read_temp_callback() { return (bmp280.readTemperature() + 273.15);}
  float read_pressure_callback() { return (bmp280.readPressure());}

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("my-sensesp-project")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

///  1-Wire Temp Sensors ///
/// Exhaust Temp Sensors ///

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(17);

  auto* exhaust_temp =
      new OneWireTemperature(dts, 1000, "/Exhaust Temperature/oneWire");

    ConfigItem(exhaust_temp)
      ->set_title("Exhaust Temperature Sender")
      ->set_description("Device ID of the engine exhaust sender")
      ->set_sort_order(100);

    auto exhaust_temp_calibration =
      new Linear(1.0, 0.0, "/Exhaust_Temperature/linear");

    ConfigItem(exhaust_temp_calibration)
      ->set_title("Exhaust Temperature Calibration")
      ->set_description("Calibration for the exhaust temperature sensor")
      ->set_sort_order(200);

    auto exhaust_temp_sk_output = new SKOutputFloat(
      "propulsion.engine.exhaustTemperature", "/Exhaust_Temperature/skPath");
     
     ConfigItem(exhaust_temp_sk_output)
      ->set_title("Exhaust Temperature Signal K Path")
      ->set_description("Signal K path for the exhaust temperature")
      ->set_sort_order(300);

    exhaust_temp->connect_to(exhaust_temp_calibration)
      ->connect_to(exhaust_temp_sk_output);

/// Oil Temp Sensors ///

  auto oil_temp =
      new OneWireTemperature(dts, 1000, "/Oil Temperature/oneWire");

    ConfigItem(oil_temp)
      ->set_title("Oil Temperature Sender")
      ->set_description("Device ID of the engine oil sender")
      ->set_sort_order(100);

    auto oil_temp_calibration =
      new Linear(1.0, 0.0, "/oil_Temperature/linear");

    ConfigItem(oil_temp_calibration)
      ->set_title("Oil Temperature Calibration")
      ->set_description("Calibration for the oil temperature sensor")
      ->set_sort_order(200);

    auto oil_temp_sk_output = new SKOutputFloat(
      "propulsion.engine.oilTemperature", "/oil_Temperature/skPath");
     
     ConfigItem(oil_temp_sk_output)
      ->set_title("Oil Temperature Signal K Path")
      ->set_description("Signal K path for the oil temperature")
      ->set_sort_order(300);

    oil_temp->connect_to(oil_temp_calibration)
      ->connect_to(oil_temp_sk_output);


 //RPM Application/////

  const char* config_path_calibrate = "/Engine RPM/calibrate";
  const char* config_path_skpath = "/Engine RPM/sk_path";
  const float multiplier = 1.0;

  auto* sensor = new DigitalInputCounter(16, INPUT_PULLUP, RISING, 500);

  sensor->connect_to(new Frequency(multiplier, config_path_calibrate))  
  // connect the output of sensor to the input of Frequency()
         ->connect_to(new MovingAverage(2, 1.0,"/Engine RPM/movingAVG"))
         ->connect_to(new SKOutputFloat("propulsion.engine.revolutions", config_path_skpath));  
          // connect the output of Frequency() to a Signal K Output as a number

  sensor->connect_to(new Frequency(6))
  // times by 6 to go from Hz to RPM
          ->connect_to(new MovingAverage(4, 1.0,"/Engine Fuel/movingAVG"))
          ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
          ->connect_to(new SKOutputFloat("propulsion.engine.fuel.rate", "/Engine Fuel/sk_path"));                                       

/// BMP280 SENSOR CODE - Engine Room Temp Sensor ////  

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  bmp280.begin(0x76);

  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* engine_room_temp =
      new RepeatSensor<float>(5000, read_temp_callback);

  auto* engine_room_pressure = 
      new RepeatSensor<float>(60000, read_pressure_callback);

  // Send the temperature to the Signal K server as a Float
  engine_room_temp->connect_to(new SKOutputFloat("propulsion.engineRoom.temperature"));

  engine_room_pressure->connect_to(new SKOutputFloat("propulsion.engineRoom.pressure"));

//// Engine Temp Config ////

const float Vin = 3.5;
const float R1 = 120.0;
auto* analog_input = new AnalogInput(36, 2000);

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new TemperatureInterpreter("/Engine Temp/curve"))
      ->connect_to(new Linear(1.0, 0.9, "/Engine Temp/calibrate"))
      ->connect_to(new MovingAverage(4, 1.0,"/Engine Temp/movingAVG"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature", "/Engine Temp/sk_path"));

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature.raw"));

//// Bilge Monitor /////

auto* bilge = new DigitalInputState(25, INPUT_PULLUP, 5000);

auto int_to_string_function = [](int input) ->String {
     if (input == 1) {
       return "Water present!";
     } 
     else { // input == 0
       return "bilge clear";
     }
};

auto int_to_string_transform = new LambdaTransform<int, String>(int_to_string_function);

bilge->connect_to(int_to_string_transform)
      ->connect_to(new SKOutputString("propulsion.engine.bilge"));



  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
