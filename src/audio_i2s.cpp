/*
 * audio_i2s.cpp — Audio directo por I2S + DAC interno ESP32 (pin 25)
 *
 * Reemplaza completamente:
 *   - fabgl::SoundGenerator
 *   - EmuAudioGenerator (getSample() por muestra)
 *   - audioRing en PSRAM
 *
 * Ventajas:
 *   - DMA transfiere audio sin intervención de CPU
 *   - Ring buffer en DRAM (rápido, no PSRAM)
 *   - Sin overhead de callback por muestra
 *   - Latencia menor, sin glitches por PSRAM lenta
 *
 * Uso:
 *   1. Elimina la clase EmuAudioGenerator y SoundGen del .ino
 *   2. Llama audioInit() donde tenías initAudio()
 *   3. audioFeedSamples() y audioFeedStereoMixed() tienen la misma firma
 */
 
#include "audio_i2s.h"
#include "debug.h"
 
#include <string.h>
#include <driver/i2s.h>
#include <driver/dac.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
 
// ============================================================================
// Configuración
// ============================================================================
 
#define AUDIO_SAMPLE_RATE   22050
#define AUDIO_I2S_PORT      I2S_NUM_0
 
// DMA: 4 buffers de 256 muestras = 16ms de audio total en DMA
// Ajusta dma_buf_len si hay glitches (sube) o latencia alta (baja)
#define DMA_BUF_COUNT       4
#define DMA_BUF_LEN         128
 
// Ring buffer en DRAM (no PSRAM) para acceso rápido desde la tarea de audio
// 4096 muestras = ~185ms a 22050Hz — suficiente para cualquier emulador
#define RING_SIZE           2048
#define RING_MASK           (RING_SIZE - 1)
 
// ============================================================================
// Estado interno
// ============================================================================
 
// ring en PSRAM via heap_caps — audioFeedSamples (Core 1) y audioTask (Core 0)
// son tareas normales (no ISR) y pueden acceder a PSRAM.
// Libera 1KB de DRAM BSS.
static int16_t  *ring = nullptr;
static volatile int ringWrite = 0;
static volatile int ringRead  = 0;
 
static uint16_t i2s_out[DMA_BUF_LEN * 2];
 
static bool         audioEnabled  = false;
static TaskHandle_t audioTaskHdl  = NULL;
static bool         audioRunning  = false;
 
// ============================================================================
// Tarea de audio — corre en Core 0, alimenta el DMA continuamente
// ============================================================================
 
static void audioTask(void *arg)
{
    size_t written;
 
    while (audioRunning) {
        int wr = ringWrite;
        int rd = ringRead;
 
        // Calcular muestras disponibles
        int available = (wr - rd) & RING_MASK;
 
        if (!audioEnabled || available == 0) {
            // Sin datos: enviar silencio para no bloquear el DMA
            memset(i2s_out, 0x80, sizeof(i2s_out)); // 0x80 = punto medio DAC
            i2s_write(AUDIO_I2S_PORT, i2s_out, sizeof(i2s_out), &written, portMAX_DELAY);
            continue;
        }
 
        // Llenar buffer I2S con datos del ring
        // DAC interno: muestras unsigned [0..65535], 0x8000 = silencio
        int samples = available < DMA_BUF_LEN ? available : DMA_BUF_LEN;
 
        for (int i = 0; i < samples; i++) {
            // Convertir int16_t [-32768..32767] → uint16_t [0..65535]
            // El DAC interno del ESP32 usa los 8 bits altos del canal izquierdo
            uint16_t dac_val = (uint16_t)((int32_t)ring[rd] + 32768);
            rd = (rd + 1) & RING_MASK;
 
            // I2S stereo: muestra en ambos canales (L en byte alto, R en byte bajo)
            // El formato del DAC interno en I2S es: [R_high][R_low][L_high][L_low]
            // Pin 25 = DAC1 = canal derecho en I2S_NUM_0
            i2s_out[i * 2]     = dac_val; // Canal derecho (pin 25)
            i2s_out[i * 2 + 1] = dac_val; // Canal izquierdo (pin 26, no conectado)
        }
 
        // Rellenar resto del buffer con silencio si hay menos muestras
        for (int i = samples; i < DMA_BUF_LEN; i++) {
            i2s_out[i * 2]     = 0x8000;
            i2s_out[i * 2 + 1] = 0x8000;
        }
 
        ringRead = rd;
 
        i2s_write(AUDIO_I2S_PORT, i2s_out, DMA_BUF_LEN * 2 * sizeof(uint16_t),
                  &written, portMAX_DELAY);
    }
 
    vTaskDelete(NULL);
}
 
// ============================================================================
// API pública
// ============================================================================
 
void audioInit(void)
{
    // Allocar ring buffer en PSRAM — libera 1KB de DRAM BSS
    if (!ring)
        ring = (int16_t*)heap_caps_calloc(RING_SIZE, sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ring) return; // sin PSRAM — no hay audio
    ringWrite = 0;
    ringRead  = 0;
    audioEnabled = true;
    audioRunning = true;
 
    // Configurar I2S en modo DAC interno
    i2s_config_t i2s_cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate          = AUDIO_SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ALL_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,  // silencio automático si DMA vacío
        .fixed_mclk           = 0,
    };
 
    esp_err_t err = i2s_driver_install(AUDIO_I2S_PORT, &i2s_cfg, 0, NULL);
    if (err != ESP_OK) {
        DBG_ERROR("Audio", "I2S install failed: %d", err);
        return;
    }
 
    // Activar DAC interno — pin 25 = DAC_CHANNEL_1 = I2S right channel
    err = i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
    if (err != ESP_OK) {
        DBG_ERROR("Audio", "DAC mode failed: %d", err);
        return;
    }
 
    // Tarea de audio en Core 0 (la emulación corre en Core 1)
    // Prioridad 5 — suficiente para no perder muestras sin bloquear la emulación
    xTaskCreatePinnedToCore(
        audioTask,
        "audioTask",
        2048,        // stack — suficiente para la tarea
        NULL,
        5,           // prioridad
        &audioTaskHdl,
        0          // Core 0
    );
 
    DBG_INFO("Audio", "I2S DAC iniciado a %d Hz, pin 25", AUDIO_SAMPLE_RATE);
}
 
// Alimentar muestras mono (NES, PCE, SNES, Lynx, Genesis)
void audioFeedSamples(const int16_t *samples, int count)
{
    if (!samples || count <= 0 || !audioEnabled || !ring) return;
 
    int wr = ringWrite;
    int rd = ringRead;
 
    for (int i = 0; i < count; i++) {
        int next = (wr + 1) & RING_MASK;
        if (next == rd) break; // ring lleno — descartar
        ring[wr] = samples[i];
        wr = next;
    }
 
    ringWrite = wr;
}
 
// Alimentar muestras estéreo mezcladas (SMS/GG)
void audioFeedStereoMixed(const int16_t *left, const int16_t *right, int count)
{
    if (!left || !right || count <= 0 || !audioEnabled || !ring) return;
 
    int wr = ringWrite;
    int rd = ringRead;
 
    for (int i = 0; i < count; i++) {
        int next = (wr + 1) & RING_MASK;
        if (next == rd) break;
        int32_t mixed = ((int32_t)left[i] + (int32_t)right[i]) >> 1;
        if (mixed >  32767) mixed =  32767;
        if (mixed < -32768) mixed = -32768;
        ring[wr] = (int16_t)mixed;
        wr = next;
    }
 
    ringWrite = wr;
}
 
void audioSetEnabled(bool enabled)
{
    audioEnabled = enabled;
    if (!enabled) {
        // Vaciar ring para silencio inmediato
        ringRead = ringWrite;
    }
}
 
void audioShutdown(void)
{
    audioRunning = false;
    audioEnabled = false;
 
    if (audioTaskHdl) {
        vTaskDelay(pdMS_TO_TICKS(50));
        audioTaskHdl = NULL;
    }
 
    i2s_driver_uninstall(AUDIO_I2S_PORT);
    DBG_VERBOSE("Audio", "I2S shutdown");
}
 