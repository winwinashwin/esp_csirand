// trng_csi.h
#ifndef TRNG_CSI_H
#define TRNG_CSI_H

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include "mbedtls/md.h"

static const char *TAG = "csi_rng";

/* ---------- Configurable constants ---------- */
#define NUM_TOTAL_SUBCARRIERS 64 // 64 subcarriers for 802.11 OFDM (0..31 and -32..-1)
#define SELECT_STEP 8            // choose every 8th usable subcarrier (tune)
#define HMAC_INPUT_BITS 512      // chunk size of bits to HMAC
#define HMAC_KEY_LEN 16          // can be any length for HMAC; using 16 bytes here

// Hard-coded HMAC key (example, replace with your desired key)
// Use the AES-derived key you provided earlier:
static const uint8_t HMAC_KEY[HMAC_KEY_LEN] = {
    0x47, 0x71, 0x18, 0x16, 0xE9, 0x1D, 0x6F, 0xF0,
    0x59, 0xBB, 0xBF, 0x2B, 0xF5, 0x8E, 0x0F, 0xD3};

/* ---------- FreeRTOS queue / worker ---------- */
typedef struct
{
    uint8_t lsb_mask_bytes[(NUM_TOTAL_SUBCARRIERS >> 3) + 1]; // optional: not used, keep for extension
    uint8_t lsb_bits[NUM_TOTAL_SUBCARRIERS];                  // stores the selected subcarrier LSBs for one frame (0/1)
    uint8_t num_selected;                                     // count of selected subcarriers present in lsb_bits[]
} csi_lsb_msg_t;

static QueueHandle_t s_lsb_queue = NULL;
static TaskHandle_t s_worker_task = NULL;

/* Precompute usable subcarrier indices, in the re-ordered -32..31 array indexing
   In Python you had: subcarrier_indices = [-32..-1, 0..31] and usable indices in -26..26 excluding pilots and 0.
   We create an array of indexes (positions) into CSI buf's complex samples for selected subcarriers. */
static int8_t s_selected_positions[NUM_TOTAL_SUBCARRIERS];
static uint8_t s_num_selected_positions = 0;

/* Helper: compute index mapping from subcarrier value (-32..-1,0..31) to sample array index [0..63]
   We'll produce the same ordering as your Python: subcarrier_indices = [-32..-1,0..31] */
static void compute_selected_positions(int step)
{
    int subcarrier_values[NUM_TOTAL_SUBCARRIERS];
    int idx = 0;
    for (int i = -32; i < 0; ++i)
        subcarrier_values[idx++] = i;
    for (int i = 0; i < 32; ++i)
        subcarrier_values[idx++] = i;
    // usable indices from -26..26 excluding pilots and 0
    bool usable_mask[NUM_TOTAL_SUBCARRIERS];
    memset(usable_mask, 0, sizeof(usable_mask));
    const int pilot_set[] = {-21, -7, 7, 21};
    for (int j = 0; j < NUM_TOTAL_SUBCARRIERS; ++j)
    {
        int val = subcarrier_values[j];
        if (val >= -26 && val <= 26 && val != 0)
        {
            bool is_pilot = false;
            for (size_t p = 0; p < sizeof(pilot_set) / sizeof(pilot_set[0]); ++p)
            {
                if (val == pilot_set[p])
                {
                    is_pilot = true;
                    break;
                }
            }
            if (!is_pilot)
                usable_mask[j] = true;
        }
    }
    // collect usable positions
    int usable_positions[NUM_TOTAL_SUBCARRIERS];
    int usable_count = 0;
    for (int j = 0; j < NUM_TOTAL_SUBCARRIERS; ++j)
    {
        if (usable_mask[j])
            usable_positions[usable_count++] = j;
    }
    // pick every `step`th usable subcarrier
    s_num_selected_positions = 0;
    for (int k = 0; k < usable_count; k += step)
    {
        s_selected_positions[s_num_selected_positions++] = (int8_t)usable_positions[k];
    }
    ESP_LOGI(TAG, "Selected %d subcarriers (step=%d)", s_num_selected_positions, step);
}

/* Cheap integer amplitude approximation (fast): (|re| + |im|) >> 1 */
static inline uint8_t approx_amplitude_int8(int8_t re, int8_t im)
{
    int a = re >= 0 ? re : -re;
    int b = im >= 0 ? im : -im;
    return (uint8_t)(((a + b) >> 1) & 0xFF);
}

/* ------------------ CSI callback ------------------
   Extracts LSBs of selected subcarriers and queues them.
   Important: keep it short. No heavy crypto here.
*/
void wifi_csi_rx_cb_trng(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf)
        return;

    // prepare message
    csi_lsb_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.num_selected = 0;

    // info->buf contains interleaved I and Q: in your python you had im = data[0::2], re = data[1::2]
    // On ESP buffer the same layout is usually present: treat buf as int8 array
    int bytes = info->len; // number of bytes in buf
    int pairs = bytes >> 1;
    // Ensure we don't overflow
    int max_pairs = NUM_TOTAL_SUBCARRIERS;
    if (pairs < max_pairs)
        max_pairs = pairs;

    int8_t *data = (int8_t *)info->buf;

    // For each selected position (which indexes into the reordered array 0..63),
    // find corresponding re/im pair. The ordering here assumes the sample
    // ordering matches the subcarrier_values mapping we used earlier.
    for (int s = 0; s < s_num_selected_positions; ++s)
    {
        int pos = s_selected_positions[s]; // position in 0..63 in our mapping
        if (pos >= max_pairs)
            continue;
        // In our mapping above, pos is index into the ordered [-32..-1,0..31] array.
        // The CSI hardware places subcarriers in that order as data pairs.
        msg.lsb_bits[msg.num_selected++] = approx_amplitude_int8(data[pos * 2 + 1], data[pos * 2 + 0]);
    }

    if (s_lsb_queue)
    {
        BaseType_t ret = xQueueSendFromISR(s_lsb_queue, &msg, NULL);
        if (ret != pdTRUE)
        {
            // Queue is full → drop oldest item
            void *dummy;
            xQueueReceiveFromISR(s_lsb_queue, &dummy, NULL); // remove oldest
            // Try sending again (should succeed now)
            ret = xQueueSendFromISR(s_lsb_queue, &msg, NULL);
            if (ret != pdTRUE)
            {
                ESP_LOGE(TAG, "error when pushing new item to queue");
            }
        }
    }
}

/* ------------------ Worker task ------------------
   Concatenate LSB bits from successive messages until we have HMAC_INPUT_BITS, then HMAC-SHA256 it.
   Output digest bytes to UART (or to any other sink).
*/
static void trng_worker_task(void *arg)
{
    csi_lsb_msg_t msg;
    // buffer to accumulate raw bits (0/1)
    uint8_t bit_buffer[HMAC_INPUT_BITS];
    size_t bit_pos = 0;

    // Diagnostics: prepare mbedtls
    const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!md_info)
    {
        ESP_LOGE(TAG, "mbedtls md_info SHA256 not available");
        vTaskDelete(NULL);
        return;
    }

    uint8_t digest[32];
    const int bytes_needed = ((HMAC_INPUT_BITS + 7) >> 3);
    uint8_t input_bytes[bytes_needed];
    while (1)
    {
        if (xQueueReceive(s_lsb_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            // append bits from this frame's selected subcarriers
            for (int i = 0; i < msg.num_selected && bit_pos < HMAC_INPUT_BITS; ++i)
            {
                uint8_t amp = msg.lsb_bits[i] & 0x0F;
                const int N_LSB = 4;
                for (int j = 0; j < N_LSB && bit_pos < HMAC_INPUT_BITS; ++j)
                {
                    bit_buffer[bit_pos++] = (amp >> j) & 1;
                }
            }

            // We don't have enough bits for HMAC, continue to next cycle
            if (bit_pos < HMAC_INPUT_BITS)
                continue;

            // pack bits to bytes
            memset(input_bytes, 0, bytes_needed);
            for (int b = 0; b < HMAC_INPUT_BITS; ++b)
            {
                int byte_idx = b >> 3;
                int bit_idx = 7 - (b % 8); // pack msb-first in each byte
                if (bit_buffer[b])
                    input_bytes[byte_idx] |= (1u << bit_idx);
            }

            // perform HMAC-SHA256
            mbedtls_md_context_t ctx;
            mbedtls_md_init(&ctx);
            if (mbedtls_md_setup(&ctx, md_info, 1) != 0)
            {
                ESP_LOGE(TAG, "mbedtls_md_setup failed");
                mbedtls_md_free(&ctx);
                bit_pos = 0;
                continue;
            }
            mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
            mbedtls_md_hmac_update(&ctx, input_bytes, bytes_needed);
            mbedtls_md_hmac_finish(&ctx, digest);
            mbedtls_md_free(&ctx);

            for (int i = 0; i < sizeof(digest); i++)
                ets_printf("%c", digest[i]);

            // Reset buffer
            bit_pos = 0;
        }
    }
}

/* ------------------ Initialization ------------------ */
void trng_init(void)
{
    // precompute positions with given step
    compute_selected_positions(SELECT_STEP);

    // create queue for lsb messages (depth 64 frames)
    s_lsb_queue = xQueueCreate(64, sizeof(csi_lsb_msg_t));
    if (!s_lsb_queue)
    {
        ESP_LOGE(TAG, "Failed to create LSB queue");
        return;
    }

    // start worker task
    xTaskCreate(trng_worker_task, "trng_worker", 4096, NULL, tskIDLE_PRIORITY + 2, &s_worker_task);

    ESP_LOGI(TAG, "TRNG initialized");
}

#endif // TRNG_CSI_H
