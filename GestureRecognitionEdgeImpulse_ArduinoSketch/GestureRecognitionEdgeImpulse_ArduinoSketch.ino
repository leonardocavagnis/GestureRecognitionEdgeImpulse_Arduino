/*
  GestureRecognitionEdgeImpulse_ArduinoSketch

  Board: Arduino Giga R1 WiFi + Arduino Giga Display Shield

  by Leonardo Cavagnis
*/

/* Includes ---------------------------------------------------------------- */
#include <Egg-breaker_inferencing.h>
#include "Arduino_BMI270_BMM150.h"

#include "Arduino_H7_Video.h"
#include "Arduino_GigaDisplayTouch.h"

#include "lvgl.h"

Arduino_H7_Video          Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch  TouchDetector;

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  4.0f
#define ACC_THRESHOLD_G     2.5f

/* Private variables ------------------------------------------------------- */
BoschSensorClass imu(Wire1);

lv_obj_t * img1;

void setup() {
  Serial.begin(115200);
  
  Serial.println("Edge Impulse, LVGL & Arduino Giga Display - Happy Easter");
  
  // Init IMU of the GIGA Display Shield
  if (imu.begin() == 1) {
      ei_printf("Failed to initialize IMU!\r\n");
      while(1);
  }

  // Init display and touch
  Display.begin();
  TouchDetector.begin();

  // Add white background
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0xFFFFFF), LV_PART_MAIN);

  img1 = lv_img_create(lv_screen_active());

  LV_IMG_DECLARE(img_egg);
  lv_img_set_src(img1, &img_egg);
  lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_size(img1, 200, 200);
}

void loop() {
    // Check if a movement is detected
    bool moveDetect = false;
    if (imu.accelerationAvailable()) {
      float acc_x, acc_y, acc_z;
      imu.readAcceleration(acc_x, acc_y, acc_z);
      float aSum = fabs(acc_x) + fabs(acc_y) + fabs(acc_z);
      
      if (aSum >= ACC_THRESHOLD_G) {
        moveDetect = true;
      } else {
        moveDetect = false;
      }
    }

    // if a movement is detected start sampling and inferencing
    if (moveDetect) {
      ei_printf("Sampling...\n");
      
      // Allocate a buffer here for the values we'll read from the IMU
      float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
      
      for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 6) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
      
        imu.readAcceleration(buffer[ix + 0], buffer[ix + 1], buffer[ix + 2]);
        imu.readGyroscope(buffer[ix + 3], buffer[ix + 4], buffer[ix + 5]);
      
        for (int i = 0; i < 3; i++) {
            if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
                buffer[ix + i] = (buffer[ix + i] >= 0.0) ? MAX_ACCEPTED_RANGE : -MAX_ACCEPTED_RANGE;      
            }
        }
      
        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;
      
        delayMicroseconds(next_tick - micros());
      }
      
      // Turn the raw buffer in a signal which we can the classify
      signal_t signal;
      int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
      }
      
      // Run the classifier
      ei_impulse_result_t result = { 0 };
      
      err = run_classifier(&signal, &result, false);
      if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
      }
      
      // print the predictions
      ei_printf("Predictions ");
      ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
      ei_printf(": \n");
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        if (result.classification[ix].value > 0.9) {
          if(strcmp(result.classification[ix].label, "updown") == 0) {
              LV_IMG_DECLARE(img_egg_openup);
              lv_img_set_src(img1, &img_egg_openup);
              lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);
              lv_obj_set_size(img1, 200, 200);
            } else if (strcmp(result.classification[ix].label, "rightleft") == 0) {
              LV_IMG_DECLARE(img_egg_openright);
              lv_img_set_src(img1, &img_egg_openright);
              lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);
              lv_obj_set_size(img1, 200, 200);
            }
          }
      }
      moveDetect = false;
    } // end moveDetect

    // Reset image if the display is touched
    GDTpoint_t points[5];
    if (TouchDetector.getTouchPoints(points) > 0) {
      LV_IMG_DECLARE(img_egg);
      lv_img_set_src(img1, &img_egg);
      lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);
      lv_obj_set_size(img1, 200, 200);
    }

    /* Feed LVGL engine */
    lv_timer_handler();
}