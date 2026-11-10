#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/clocks.h"

// =====================================================================
// 1. РАСПИНОВКА СОГЛАСНО ВАШЕЙ СХЕМЕ
// =====================================================================

// Кнопки управления (активный LOW, внутренний Pull-Up)
#define PIN_BTN_UP     0
#define PIN_BTN_DOWN   1
#define PIN_BTN_LEFT   2
#define PIN_BTN_RIGHT  3
#define PIN_BTN_OK     22

// I2C0: модуль ADS1115 (реостат громкости 10 кОм на входе A0)
#define PIN_I2C_SDA    4
#define PIN_I2C_SCL    5
#define ADS1115_ADDR   0x48

// I2S: модуль ЦАП PCM5102A (через PIO0)
#define PIN_I2S_BCK    6
#define PIN_I2S_LRCK   7
#define PIN_I2S_DIN    8

// Галетный переключатель эффектов (4 положения, активный LOW)
#define PIN_SW_POS1    10
#define PIN_SW_POS2    11
#define PIN_SW_POS3    12
#define PIN_SW_POS4    13

// Дисплей ILI9341 (SPI0)
#define PIN_LCD_DC     14
#define PIN_LCD_RST    15
#define PIN_LCD_MISO   16
#define PIN_LCD_CS     17
#define PIN_LCD_SCK    18
#define PIN_LCD_MOSI   19

// Пьезопредусилитель MAX4466 (согласно вашей таблице: GP26 = ADC0, GP27 = ADC1)
#define PIN_PIEZO_ADC0 26
#define PIN_PIEZO_ADC1 27

// =====================================================================
// 2. ПАРАМЕТРЫ АУДИО (32 кГц ДЛЯ ГАРАНТИРОВАННОГО ЗАХВАТА ФАПЧ PCM5102A)
// =====================================================================

#define SAMPLE_RATE        32000   // 32 кГц -> BCK = 1.024 МГц (требование PLL PCM5102A)
#define WINDOW_SAMPLES     1024    // Окно анализа 32 мс при 32 кГц
#define NUM_NOTES          8
#define DELAY_BUF_SIZE     6000    // Буфер эха

// Настройки чувствительности пьезотракта
#define PIEZO_NOISE_GATE   3       // Порог тишины (в отсчетах АЦП)
#define PIEZO_GAIN         90      // Усиление живого сигнала с MAX4466
#define PLUCK_PP_THRESH    30      // Порог скачка размаха (Peak-to-Peak) для фиксации удара

typedef enum {
    FX_CLEAN = 0,
    FX_OVERDRIVE = 1,
    FX_DISTORTION = 2,
    FX_DELAY = 3
} effect_mode_t;

static const float NOTE_FREQS[NUM_NOTES] = {
    261.63f, 293.66f, 329.63f, 349.23f,
    392.00f, 440.00f, 493.88f, 523.25f
};

static const char *NOTE_NAMES[NUM_NOTES] = {
    "C4", "D4", "E4", "F4", "G4", "A4", "B4", "C5"
};

static const char *FX_NAMES[4] = {
    "CLEAN", "OVERDRIVE", "DISTORTION", "DELAY/ECHO"
};

// Глобальные переменные обмена между ядрами
static volatile float g_master_volume = 0.90f;
static volatile effect_mode_t g_effect_mode = FX_CLEAN;
static volatile bool g_speaker_test_active = false;
static volatile float g_synth_freq = 0.0f;
static volatile int32_t g_synth_rem_samples = 0;
static volatile int32_t g_synth_total_samples = 1;

// Диагностическая телеметрия АЦП и частотомера
static volatile int32_t g_dc_gp26 = 2048;
static volatile int32_t g_pp_gp26 = 0;
static volatile int32_t g_dc_gp27 = 2048;
static volatile int32_t g_pp_gp27 = 0;
static volatile float   g_measured_freq_hz = 0.0f;
static volatile int     g_closest_note_idx = -1;
static volatile bool    g_new_pluck_flag = false;

static int16_t delay_buffer[DELAY_BUF_SIZE];
static uint32_t delay_idx = 0;

static void play_test_tone(float freq, int duration_ms) {
    g_synth_total_samples = (duration_ms * SAMPLE_RATE) / 1000;
    g_synth_freq = freq;
    g_synth_rem_samples = g_synth_total_samples;
}

// =====================================================================
// 3. ЭТАЛОННЫЙ I2S ДРАЙВЕР НА PIO0 С SIDE-SET (64 ТАКТА НА КАДР)
// =====================================================================
// Side-set управляет пинами GP6 (BCK - младший бит) и GP7 (LRCK - старший бит)
// одновременно с выводом данных на GP8 (DIN), давая идеальный меандр I2S.
static void init_pio_i2s(PIO pio, uint sm) {
    uint16_t i2s_prog_instr[8] = {
        // Левый канал (LRCK = 0, биты 15..1)
        pio_encode_out(pio_pins, 1)  | pio_encode_sideset(2, 0b00), // 0: DIN=bit, BCK=0, LRCK=0
        pio_encode_jmp_x_dec(0)      | pio_encode_sideset(2, 0b01), // 1: jmp 0,   BCK=1, LRCK=0
        // Младший бит левого канала (переход LRCK=1 за 1 такт до правого канала по стандарту I2S)
        pio_encode_out(pio_pins, 1)  | pio_encode_sideset(2, 0b10), // 2: DIN=LSB, BCK=0, LRCK=1
        pio_encode_set(pio_x, 14)    | pio_encode_sideset(2, 0b11), // 3: X=14,    BCK=1, LRCK=1
        // Правый канал (LRCK = 1, биты 15..1)
        pio_encode_out(pio_pins, 1)  | pio_encode_sideset(2, 0b10), // 4: DIN=bit, BCK=0, LRCK=1
        pio_encode_jmp_x_dec(4)      | pio_encode_sideset(2, 0b11), // 5: jmp 4,   BCK=1, LRCK=1
        // Младший бит правого канала + автопулл 32-битного слова из FIFO
        pio_encode_out(pio_pins, 1)  | pio_encode_sideset(2, 0b00), // 6: DIN=LSB, BCK=0, LRCK=0
        pio_encode_set(pio_x, 14)    | pio_encode_sideset(2, 0b01)  // 7: X=14,    BCK=1, LRCK=0
    };

    struct pio_program prog = {
        .instructions = i2s_prog_instr,
        .length = 8,
        .origin = 0
    };

    uint offset = pio_add_program(pio, &prog);
    pio_sm_config c = pio_get_default_sm_config();

    sm_config_set_wrap(&c, offset + 0, offset + 7);
    sm_config_set_out_pins(&c, PIN_I2S_DIN, 1);
    sm_config_set_sideset_pins(&c, PIN_I2S_BCK); // GP6 = BCK, GP7 = LRCK
    sm_config_set_sideset(&c, 2, false, false);
    sm_config_set_out_shift(&c, false, true, 32); // MSB first, autopull 32 bits
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // 32 бита на стереокадр * 2 такта PIO на бит = 64 такта PIO на сэмпл
    float div = (float)clock_get_hz(clk_sys) / (float)(SAMPLE_RATE * 64);
    sm_config_set_clkdiv(&c, div);

    pio_gpio_init(pio, PIN_I2S_BCK);
    pio_gpio_init(pio, PIN_I2S_LRCK);
    pio_gpio_init(pio, PIN_I2S_DIN);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_I2S_BCK, 3, true);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_exec(pio, sm, pio_encode_set(pio_x, 14) | pio_encode_sideset(2, 0b00));
    pio_sm_set_enabled(pio, sm, true);
}

// =====================================================================
// 4. ЯДРО 1: ЧТЕНИЕ ПЬЕЗЫ НА GP26, ЧАСТОТОМЕР И ВЫВОД ЗВУКА В ДИНАМИКИ
// =====================================================================
static void core1_audio_entry(void) {
    int32_t dc_est0 = 2048 << 8;
    int32_t dc_est1 = 2048 << 8;
    int32_t lp_audio = 0;
    int32_t bp_tuner = 0; // Сглаженный сигнал для точного подсчета частоты язычка

    int32_t min0 = 4095, max0 = 0;
    int32_t min1 = 4095, max1 = 0;
    int sample_cnt = 0;

    // Переменные частотомера по переходам через ноль
    int32_t prev_bp = 0;
    int zc_count = 0;
    int first_zc_sample = -1;
    int last_zc_sample = -1;
    int total_zc_timer = 0;

    float synth_phase = 0.0f;
    uint32_t dual_check_div = 0;
    int32_t cached_raw1 = 2048;

    while (true) {
        // 1. Чтение основного предусилителя MAX4466 на GP26 (ADC0)
        adc_select_input(0);
        int32_t raw0 = (int32_t)adc_read();

        // Раз в 8 отсчетов опрашиваем GP27 (ADC1) для диагностики второго канала
        dual_check_div++;
        if ((dual_check_div & 7) == 0) {
            adc_select_input(1);
            cached_raw1 = (int32_t)adc_read();
        }
        int32_t raw1 = cached_raw1;

        if (raw0 < min0) min0 = raw0;
        if (raw0 > max0) max0 = raw0;
        if (raw1 < min1) min1 = raw1;
        if (raw1 > max1) max1 = raw1;

        // Автоматическая подстройка под любой постоянный уровень DC на GP26
        dc_est0 += ((raw0 << 8) - dc_est0) >> 9;
        dc_est1 += ((raw1 << 8) - dc_est1) >> 9;

        int32_t sig0 = raw0 - (dc_est0 >> 8);
        int32_t sig1 = raw1 - (dc_est1 >> 8);

        // Основной сигнал берем с GP26 (ADC0)
        int32_t sig = sig0;

        // 2. Фильтр частотомера (отсекает ВЧ-звон, оставляя основной тон язычка 200..600 Гц)
        bp_tuner += (sig - bp_tuner) >> 3;
        if (prev_bp < 0 && bp_tuner >= 0 && abs(sig) > 8) {
            if (first_zc_sample < 0) {
                first_zc_sample = total_zc_timer;
            }
            last_zc_sample = total_zc_timer;
            zc_count++;
        }
        prev_bp = bp_tuner;
        total_zc_timer++;

        // 3. Шумоподавитель и усиление живого звука с пьезоэлемента
        int32_t clean_sig = (abs(sig) < PIEZO_NOISE_GATE) ? 0 : sig;
        int32_t x = clean_sig * PIEZO_GAIN;

        // 4. Подмешивание синтезатора проверки динамиков (если активен)
        int32_t rem = g_synth_rem_samples;
        if (rem > 0) {
            float freq = g_synth_freq;
            float env = (float)rem / (float)g_synth_total_samples;
            float wave = sinf(synth_phase) + 0.35f * sinf(synth_phase * 2.0f);
            synth_phase += (2.0f * 3.14159265f * freq) / (float)SAMPLE_RATE;
            if (synth_phase > 6.2831853f) synth_phase -= 6.2831853f;
            x += (int32_t)(wave * env * 18000.0f);
            g_synth_rem_samples = rem - 1;
        }

        // 5. Подведение итогов окна анализа каждые 32 мс (1024 отсчета)
        sample_cnt++;
        if (sample_cnt >= WINDOW_SAMPLES) {
            int32_t pp0 = max0 - min0;
            int32_t pp1 = max1 - min1;
            g_dc_gp26 = dc_est0 >> 8;
            g_pp_gp26 = pp0;
            g_dc_gp27 = dc_est1 >> 8;
            g_pp_gp27 = pp1;

            if (pp0 >= PLUCK_PP_THRESH) {
                // Вычисляем реальную частоту колебаний язычка в Гц
                if (zc_count >= 3 && last_zc_sample > first_zc_sample) {
                    int elapsed = last_zc_sample - first_zc_sample;
                    float freq = ((float)(zc_count - 1) * (float)SAMPLE_RATE) / (float)elapsed;
                    if (freq >= 150.0f && freq <= 900.0f) {
                        g_measured_freq_hz = freq;
                        // Находим ближайшую ноту строя До-мажор
                        int best_n = 0;
                        float min_diff = 9999.0f;
                        for (int i = 0; i < NUM_NOTES; i++) {
                            float d = fabsf(freq - NOTE_FREQS[i]);
                            if (d < min_diff) {
                                min_diff = d;
                                best_n = i;
                            }
                        }
                        g_closest_note_idx = best_n;
                    }
                }
                g_new_pluck_flag = true;
            }

            min0 = 4095; max0 = 0;
            min1 = 4095; max1 = 0;
            sample_cnt = 0;
            zc_count = 0;
            first_zc_sample = -1;
            last_zc_sample = -1;
            total_zc_timer = 0;
        }

        // 6. Наложение гитарных эффектов
        effect_mode_t mode = g_effect_mode;
        int32_t y = x;

        if (mode == FX_CLEAN) {
            lp_audio += (x - lp_audio) >> 1;
            y = lp_audio;
        } else if (mode == FX_OVERDRIVE) {
            int32_t drive = x * 4;
            if (drive > 24000) drive = 24000 + ((drive - 24000) >> 3);
            else if (drive < -18000) drive = -18000 + ((drive + 18000) >> 3);
            lp_audio += (drive - lp_audio) >> 2;
            y = lp_audio;
        } else if (mode == FX_DISTORTION) {
            int32_t fuzz = x * 10;
            if (fuzz > 22000) fuzz = 22000;
            else if (fuzz < -22000) fuzz = -22000;
            lp_audio += (fuzz - lp_audio) >> 2;
            y = lp_audio;
        } else if (mode == FX_DELAY) {
            int16_t delayed = delay_buffer[delay_idx];
            y = x + (delayed >> 1);
            int32_t fb = x + ((delayed * 7) >> 4);
            if (fb > 32767) fb = 32767;
            if (fb < -32768) fb = -32768;
            delay_buffer[delay_idx] = (int16_t)fb;
            delay_idx++;
            if (delay_idx >= DELAY_BUF_SIZE) delay_idx = 0;
        }

        // 7. Мастер-громкость и вывод в ЦАП PCM5102A
        y = (int32_t)((float)y * g_master_volume);
        if (y > 32767) y = 32767;
        if (y < -32768) y = -32768;

        uint16_t out16 = (uint16_t)(int16_t)y;
        uint32_t stereo_word = ((uint32_t)out16 << 16) | out16;
        pio_sm_put_blocking(pio0, 0, stereo_word);
    }
}

// =====================================================================
// 5. ДРАЙВЕР ADS1115 (ГРОМКОСТЬ) И ЭКРАНА ILI9341
// =====================================================================
static void ads1115_start_single_a0(void) {
    uint8_t buf[3] = {0x01, 0xC3, 0x83};
    i2c_write_timeout_us(i2c0, ADS1115_ADDR, buf, 3, false, 2000);
}

static float ads1115_read_volume(void) {
    uint8_t reg = 0x00;
    uint8_t data[2] = {0, 0};
    if (i2c_write_timeout_us(i2c0, ADS1115_ADDR, &reg, 1, true, 2000) == 1) {
        if (i2c_read_timeout_us(i2c0, ADS1115_ADDR, data, 2, false, 2000) == 2) {
            int16_t raw = (int16_t)((data[0] << 8) | data[1]);
            if (raw < 0) raw = 0;
            float v = (float)raw / 26400.0f;
            if (v > 1.0f) v = 1.0f;
            if (v < 0.35f) v = 0.35f; // Защита от полного выключения звука
            ads1115_start_single_a0();
            return v;
        }
    }
    ads1115_start_single_a0();
    return 0.90f;
}

static void lcd_cmd(uint8_t cmd) {
    gpio_put(PIN_LCD_DC, 0);
    gpio_put(PIN_LCD_CS, 0);
    spi_write_blocking(spi0, &cmd, 1);
    gpio_put(PIN_LCD_CS, 1);
}

static void lcd_data(uint8_t data) {
    gpio_put(PIN_LCD_DC, 1);
    gpio_put(PIN_LCD_CS, 0);
    spi_write_blocking(spi0, &data, 1);
    gpio_put(PIN_LCD_CS, 1);
}

static void lcd_set_window(int x0, int y0, int x1, int y1) {
    lcd_cmd(0x2A);
    lcd_data(x0 >> 8); lcd_data(x0 & 0xFF);
    lcd_data(x1 >> 8); lcd_data(x1 & 0xFF);
    lcd_cmd(0x2B);
    lcd_data(y0 >> 8); lcd_data(y0 & 0xFF);
    lcd_data(y1 >> 8); lcd_data(y1 & 0xFF);
    lcd_cmd(0x2C);
}

static void lcd_init(void) {
    gpio_put(PIN_LCD_RST, 0); sleep_ms(20);
    gpio_put(PIN_LCD_RST, 1); sleep_ms(150);
    lcd_cmd(0x01); sleep_ms(10);
    lcd_cmd(0x11); sleep_ms(120);
    lcd_cmd(0x3A); lcd_data(0x55);
    lcd_cmd(0x36); lcd_data(0x28);
    lcd_cmd(0x29); sleep_ms(20);
}

static void lcd_fill_rect(int x, int y, int w, int h, uint16_t color) {
    if (x >= 320 || y >= 240 || w <= 0 || h <= 0) return;
    lcd_set_window(x, y, x + w - 1, y + h - 1);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    uint8_t line_buf[640];
    for (int i = 0; i < w; i++) {
        line_buf[i * 2] = hi;
        line_buf[i * 2 + 1] = lo;
    }
    gpio_put(PIN_LCD_DC, 1);
    gpio_put(PIN_LCD_CS, 0);
    for (int r = 0; r < h; r++) {
        spi_write_blocking(spi0, line_buf, w * 2);
    }
    gpio_put(PIN_LCD_CS, 1);
}

static uint8_t get_glyph_row(char c, int row) {
    if (c >= '0' && c <= '9') {
        static const uint8_t digits[10][7] = {
            {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
            {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},{0x1F,0x02,0x04,0x06,0x01,0x11,0x0E},
            {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},{0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
            {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},{0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
            {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},{0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}
        };
        return digits[c - '0'][row];
    }
    if (c >= 'A' && c <= 'Z') {
        static const uint8_t letters[26][7] = {
            {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},{0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
            {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},{0x1C,0x12,0x11,0x11,0x11,0x12,0x1C},
            {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},{0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
            {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},{0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
            {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},{0x07,0x02,0x02,0x02,0x12,0x12,0x0C},
            {0x11,0x12,0x14,0x18,0x14,0x12,0x11},{0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
            {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},{0x11,0x19,0x15,0x13,0x11,0x11,0x11},
            {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},{0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
            {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},{0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
            {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},{0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
            {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},{0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
            {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},{0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
            {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},{0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}
        };
        return letters[c - 'A'][row];
    }
    if (c == ':') return (row == 2 || row == 4) ? 0x04 : 0x00;
    if (c == '-') return (row == 3) ? 0x0E : 0x00;
    if (c == '.') return (row == 6) ? 0x04 : 0x00;
    return 0x00;
}

static void lcd_draw_text(int x, int y, const char *str, uint16_t fg, uint16_t bg, int scale) {
    int cw = 6 * scale, ch = 7 * scale;
    uint8_t char_buf[18 * 21 * 2];
    uint8_t fg_hi = fg >> 8, fg_lo = fg & 0xFF;
    uint8_t bg_hi = bg >> 8, bg_lo = bg & 0xFF;

    while (*str) {
        char c = *str++;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (x + cw <= 320 && y + ch <= 240) {
            for (int r = 0; r < 7; r++) {
                uint8_t mask = get_glyph_row(c, r);
                for (int col = 0; col < 6; col++) {
                    bool on = (col < 5) && (mask & (1 << (4 - col)));
                    uint8_t hi = on ? fg_hi : bg_hi;
                    uint8_t lo = on ? fg_lo : bg_lo;
                    for (int sr = 0; sr < scale; sr++) {
                        for (int sc = 0; sc < scale; sc++) {
                            int idx = ((r * scale + sr) * cw + (col * scale + sc)) * 2;
                            char_buf[idx] = hi;
                            char_buf[idx + 1] = lo;
                        }
                    }
                }
            }
            lcd_set_window(x, y, x + cw - 1, y + ch - 1);
            gpio_put(PIN_LCD_DC, 1);
            gpio_put(PIN_LCD_CS, 0);
            spi_write_blocking(spi0, char_buf, cw * ch * 2);
            gpio_put(PIN_LCD_CS, 1);
        }
        x += cw;
    }
}

static void update_rotary_switch(void) {
    if (!gpio_get(PIN_SW_POS1))      g_effect_mode = FX_CLEAN;
    else if (!gpio_get(PIN_SW_POS2)) g_effect_mode = FX_OVERDRIVE;
    else if (!gpio_get(PIN_SW_POS3)) g_effect_mode = FX_DISTORTION;
    else if (!gpio_get(PIN_SW_POS4)) g_effect_mode = FX_DELAY;
}

// Ноты мелодии Undertale - Memory для теста динамиков
static const uint8_t TEST_SONG_NOTES[] = {
    0, 4, 3, 0, 2, 2, 3,  0, 2, 4, 3, 0, 2, 2, 3,  0, 4, 3, 0, 2, 3, 2, 0, 1
};

// =====================================================================
// 6. ГЛАВНЫЙ ЦИКЛ ТЕСТИРОВАНИЯ И НАСТРОЙКИ
// =====================================================================
int main(void) {
    stdio_init_all();

    const uint btn_pins[] = {
        PIN_BTN_UP, PIN_BTN_DOWN, PIN_BTN_LEFT, PIN_BTN_RIGHT, PIN_BTN_OK,
        PIN_SW_POS1, PIN_SW_POS2, PIN_SW_POS3, PIN_SW_POS4
    };
    for (size_t i = 0; i < sizeof(btn_pins)/sizeof(btn_pins[0]); i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }

    i2c_init(i2c0, 100000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    ads1115_start_single_a0();

    adc_init();
    adc_gpio_init(PIN_PIEZO_ADC0); // GP26 (ADC0)
    adc_gpio_init(PIN_PIEZO_ADC1); // GP27 (ADC1)

    spi_init(spi0, 31250000);
    gpio_set_function(PIN_LCD_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_LCD_SCK, GPIO_FUNC_SPI);
    gpio_init(PIN_LCD_DC);  gpio_set_dir(PIN_LCD_DC, GPIO_OUT);
    gpio_init(PIN_LCD_RST); gpio_set_dir(PIN_LCD_RST, GPIO_OUT);
    gpio_init(PIN_LCD_CS);  gpio_set_dir(PIN_LCD_CS, GPIO_OUT);
    gpio_put(PIN_LCD_CS, 1);

    lcd_init();
    init_pio_i2s(pio0, 0);
    multicore_launch_core1(core1_audio_entry);

    // Отрисовка шапки диагностического экрана
    lcd_fill_rect(0, 0, 320, 240, 0x0863);
    lcd_fill_rect(0, 0, 320, 26, 0x0190);
    lcd_draw_text(8, 6, "KALIMBA 32KHZ AUDIO TEST", 0x07FF, 0x0190, 2);
    lcd_draw_text(8, 34, "PRESS OK (GP22): SPEAKER TEST ON/OFF", 0xFFE0, 0x0863, 1);
    lcd_draw_text(8, 50, "PLUCK ANY TINE TO MEASURE FREQUENCY", 0x07E0, 0x0863, 1);

    // Стартовый звуковой тест (4 ноты аккорда До-мажор)
    sleep_ms(600);
    printf("\n=== KALIMBA 32KHZ I2S & GP26 PIEZO DIAGNOSTICS ===\n");
    play_test_tone(261.63f, 220); sleep_ms(250);
    play_test_tone(329.63f, 220); sleep_ms(250);
    play_test_tone(392.00f, 220); sleep_ms(250);
    play_test_tone(523.25f, 400);

    uint32_t last_i2c_ms = 0;
    uint32_t last_btn_ms = 0;
    uint32_t last_song_ms = 0;
    uint32_t last_log_ms = 0;
    size_t song_idx = 0;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // 1. Кнопка OK (GP22) включает/выключает автопроигрывание мелодии на динамики
        if (now - last_btn_ms > 220) {
            if (!gpio_get(PIN_BTN_OK)) {
                g_speaker_test_active = !g_speaker_test_active;
                last_btn_ms = now;
                printf("[BUTTON OK] Speaker Auto-Test -> %s\n",
                       g_speaker_test_active ? "ENABLED" : "DISABLED");
            }
        }

        // 2. Генерация нот тестовой мелодии при включенном автотесте динамиков
        if (g_speaker_test_active && (now - last_song_ms >= 520)) {
            last_song_ms = now;
            uint8_t n = TEST_SONG_NOTES[song_idx];
            song_idx = (song_idx + 1) % (sizeof(TEST_SONG_NOTES)/sizeof(TEST_SONG_NOTES[0]));
            play_test_tone(NOTE_FREQS[n], 460);
            printf("[SPEAKER TEST] Playing note %s (%.1f Hz)\n", NOTE_NAMES[n], NOTE_FREQS[n]);
        }

        // 3. Опрос галетного переключателя и громкости
        if (now - last_i2c_ms >= 80) {
            last_i2c_ms = now;
            update_rotary_switch();
            g_master_volume = ads1115_read_volume();
        }

        // 4. Мгновенная реакция на щипок пьезодатчика (GP26) + вывод реальной частоты язычка
        if (g_new_pluck_flag) {
            g_new_pluck_flag = false;
            float v_pp = ((float)g_pp_gp26 * 3.3f) / 4095.0f;
            int idx = g_closest_note_idx;
            float f_meas = g_measured_freq_hz;

            if (idx >= 0 && f_meas > 0.0f) {
                printf("[PIEZO GP26 HIT!] Swing=%.3f V (%d cts) | Measured Freq=%.1f Hz | Closest Note=%s (Target=%.1f Hz)\n",
                       v_pp, (int)g_pp_gp26, f_meas, NOTE_NAMES[idx], NOTE_FREQS[idx]);

                char f_str[36];
                snprintf(f_str, sizeof(f_str), "FREQ:%d HZ (%s:%d HZ)",
                         (int)f_meas, NOTE_NAMES[idx], (int)NOTE_FREQS[idx]);
                lcd_fill_rect(8, 125, 304, 24, 0x0863);
                lcd_draw_text(8, 128, f_str, 0x07E0, 0x0863, 2);
            } else {
                printf("[PIEZO GP26 HIT!] Swing=%.3f V (%d cts) (short tap)\n", v_pp, (int)g_pp_gp26);
            }
        }

        // 5. Обновление экрана и вывод статуса в COM-порт раз в 500 мс
        if (now - last_log_ms >= 500) {
            last_log_ms = now;
            float dc26_v = ((float)g_dc_gp26 * 3.3f) / 4095.0f;
            float pp26_v = ((float)g_pp_gp26 * 3.3f) / 4095.0f;
            float dc27_v = ((float)g_dc_gp27 * 3.3f) / 4095.0f;
            float pp27_v = ((float)g_pp_gp27 * 3.3f) / 4095.0f;

            printf("[STATUS] GP26(ADC0): DC=%.2fV(%d) PP=%.3fV(%d) | GP27(ADC1): DC=%.2fV(%d) PP=%.3fV(%d) | FX=%s | SpkTest=%s\n",
                   dc26_v, (int)g_dc_gp26, pp26_v, (int)g_pp_gp26,
                   dc27_v, (int)g_dc_gp27, pp27_v, (int)g_pp_gp27,
                   FX_NAMES[g_effect_mode], g_speaker_test_active ? "ON" : "OFF");

            char line1[40], line2[40];
            snprintf(line1, sizeof(line1), "GP26 DC:%d PP:%d  FX:%s",
                     (int)g_dc_gp26, (int)g_pp_gp26, FX_NAMES[g_effect_mode]);
            snprintf(line2, sizeof(line2), "SPK TEST: %s   VOL: %d",
                     g_speaker_test_active ? "RUNNING" : "OFF (PRESS OK)", (int)(g_master_volume * 100.0f));

            lcd_fill_rect(8, 75, 304, 40, 0x0863);
            lcd_draw_text(8, 78, line1, 0xFFFF, 0x0863, 1);
            lcd_draw_text(8, 96, line2, g_speaker_test_active ? 0x07E0 : 0x07FF, 0x0863, 1);

            // Графическая полоска силы удара по пьезоэлементу
            int bar_w = (g_pp_gp26 * 300) / 600;
            if (bar_w > 300) bar_w = 300;
            lcd_fill_rect(10, 175, 300, 24, 0x0190);
            if (bar_w > 0) {
                lcd_fill_rect(10, 175, bar_w, 24, 0xFD20);
            }
        }

        sleep_ms(20);
    }
    return 0;
}