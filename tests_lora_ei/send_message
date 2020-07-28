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
