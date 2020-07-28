/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
#define PC_9                            0

/**
 * Dummy sensor class object
 */
DS1820  ds1820(PC_9);

// Machine Learning libraries from EdgeImpulse
#include "stm32l475e_iot01_accelero.h"
#include "ei_run_classifier.h"
using namespace ei;

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * Entry point for application
 */
int main(void)
{
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/**
 * Sends a message to the Network Server
 */

/**static void send_message()
 *{
 *    uint16_t packet_len;
 *    int16_t retcode;
 *    int32_t sensor_value;
 *
 *    if (ds1820.begin()) {
 *        ds1820.startConversion();
 *        sensor_value = ds1820.read();
 *        printf("\r\n Dummy Sensor Value = %d \r\n", sensor_value);
 *        ds1820.startConversion();
 *    } else {
 *        printf("\r\n No sensor found \r\n");
 *        return;
 *    }
 *
 *    packet_len = sprintf((char *) tx_buffer, "Dummy Sensor Value is %d",
 *                         sensor_value);
 *
 *    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
 *                           MSG_UNCONFIRMED_FLAG);
 *
 *    if (retcode < 0) {
 *        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
 *        : printf("\r\n send() - Error code %d \r\n", retcode);
 *
 *        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
 *            //retry in 3 seconds
 *            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
 *                ev_queue.call_in(3000, send_message);
 *            }
 *        }
 *        return;
 *    }
 *
 *    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
 *    memset(tx_buffer, 0, sizeof(tx_buffer));
 *}
 */
#define TX_INTERVAL         3000
#define MINIMUM_CONFIDENCE  0.7

static void send_message() {
   // our window size is 2 seconds, but we don't want to rely on a single window (lots of false positives / negatives)
   // so we capture 4 seconds of data, then slide over this 4 seconds in 2 second windows
   // and if 70% of the windows are classified the same we send a message

   // 0. allocate a buffer that can hold the 4s data
   static float accel_data[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * 2] = { 0 };
   static uint8_t last_top_result = 99;
   static bool first_send_message = true;

   // instantiate the sensor if it's the first message
   if (first_send_message) {
       int accel_init = BSP_ACCELERO_Init();
       if (accel_init != ACCELERO_OK) {
           printf("Failed to initialize accelerometer\n");
           return;
       }
       first_send_message = false;
   }

   // 1. capture raw data into the buffer
   Timer timer;
   timer.start();

   printf("Sampling data for 4 seconds...\n");

   size_t accel_data_ix = 0;

   while (1) {
       int64_t next_tick = timer.read_us() + static_cast<int64_t>(EI_CLASSIFIER_INTERVAL_MS * 1000);

       int16_t accel_data_i16[3] = { 0 };
       BSP_ACCELERO_AccGetXYZ(accel_data_i16);

       accel_data[accel_data_ix++] = static_cast<float>(accel_data_i16[0]) / 100.0f;
       accel_data[accel_data_ix++] = static_cast<float>(accel_data_i16[1]) / 100.0f;
       accel_data[accel_data_ix++] = static_cast<float>(accel_data_i16[2]) / 100.0f;

       // end of the buffer
       if (accel_data_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * 2) {
           break;
       }

       // let's sleep for (wait_time / 1000) - 1 ms. then busy loop from there
       uint64_t wait_time = next_tick - timer.read_us();

       // sleep OK (/1000 already floors it)
       ThisThread::sleep_for((wait_time / 1000) - 1);

       // busy loop til next tick
       while (next_tick > timer.read_us());
   }

   printf("Sampling data OK\n");

   // 2. slide over the 4 seconds of data, and store the output per window in the `count` variable
   uint8_t count[EI_CLASSIFIER_LABEL_COUNT + 2] = { 0 };
   for (size_t ax = 0; ax < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ax += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME * 6) {
       signal_t features_signal;
       int r = numpy::signal_from_buffer(accel_data + ax, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &features_signal);
       if (r != 0) {
           printf("Failed to convert buffer into signal (%d)\n", r);
           ev_queue.call_in(TX_INTERVAL, send_message);
           return;
       }

       ei_impulse_result_t result;

       // invoke the impulse
       EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

       if (res != 0) {
           printf("Failed to run neural network (%d)\n", res);
           ev_queue.call_in(TX_INTERVAL, send_message);
           return;
       }

       bool conclusion = false;

       // print the predictions per window (useful for debugging)
       printf("[");
   #if EI_CLASSIFIER_HAS_ANOMALY == 1
       printf("%.3f, ", result.anomaly);
       if (result.anomaly >= 0.5) {
           count[EI_CLASSIFIER_LABEL_COUNT]++;
           conclusion = true;
       }
   #endif

       for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
           if (result.classification[ix].value >= MINIMUM_CONFIDENCE && !conclusion) {
               count[ix]++;
               conclusion = true;
           }

           printf("%.5f", result.classification[ix].value);
           if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
               printf(", ");
           }
       }

       printf("]\n");

       if (!conclusion) {
           count[EI_CLASSIFIER_LABEL_COUNT + 1]++;
       }
   }

   // 3. make sure we meet the minimum probability threshold (70%)
   uint8_t result_count = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME * 6) + 1;
   uint8_t confidence_threshold = MINIMUM_CONFIDENCE * result_count;

   uint8_t top_result = 0;
   uint8_t top_count = 0;
   bool met_confidence_threshold = false;
   for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT + 2; ix++) {
       if (count[ix] > top_count) {
           top_result = ix;
           top_count = count[ix];
       }
       if (count[ix] > confidence_threshold) {
           met_confidence_threshold = true;
       }
       if (ix < EI_CLASSIFIER_LABEL_COUNT) {
           printf("%s: ", ei_classifier_inferencing_categories[ix]);
       }
       else if (ix == EI_CLASSIFIER_LABEL_COUNT) {
           printf("anomaly: ");
       }
       else if (ix == EI_CLASSIFIER_LABEL_COUNT + 1) {
           printf("uncertain: ");
       }
       printf("%d", count[ix]);
       if (ix != EI_CLASSIFIER_LABEL_COUNT + 1) {
           printf(", ");
       }
   }
   printf("\n");

   if (!met_confidence_threshold) {
       printf("Result didn't meet confidence threshold, doing nothing\r\n");
       ev_queue.call_in(1, send_message);
       return;
   }

   if (top_result == last_top_result) {
       printf("Result was unchanged, doing nothing\r\n");
       ev_queue.call_in(1, send_message);
       return;
   }
   last_top_result = top_result;

   // 4. changed! Send out a message over LoRaWAN back to TTN
   int retcode = lorawan.send(15, count, EI_CLASSIFIER_LABEL_COUNT + 2, MSG_UNCONFIRMED_FLAG);

   if (retcode < 0) {
       retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - Duty cycle violation\r\n")
               : printf("send - Error code %d \r\n", retcode);

       if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
           //retry in 3 seconds
           ev_queue.call_in(3000, send_message);
       }
       return;
   }

   printf("%d bytes scheduled for transmission \r\n", retcode);
}


/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
