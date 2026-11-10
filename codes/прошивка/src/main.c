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
// 1. РАСПИНОВКА СХЕМЫ
// =====================================================================

// Кнопки управления интерфейсом (активный LOW, внутренний Pull-Up)
#define PIN_BTN_UP     0
#define PIN_BTN_DOWN   1
#define PIN_BTN_LEFT   2
#define PIN_BTN_RIGHT  3
#define PIN_BTN_OK     22

// Шина I2C0 для ADS1115 (регулятор громкости)
#define PIN_I2C_SDA    4
#define PIN_I2C_SCL    5
#define ADS1115_ADDR   0x48

// Шина I2S для ЦАП PCM5102A (через PIO0)
#define PIN_I2S_BCK    6
#define PIN_I2S_LRCK   7
#define PIN_I2S_DIN    8

// Галетный переключатель эффектов (4 позиции, активный LOW)
#define PIN_SW_POS1    10
#define PIN_SW_POS2    11
#define PIN_SW_POS3    12
#define PIN_SW_POS4    13

// Дисплей ILI9341 (SPI0, 320x240)
#define PIN_LCD_DC     14
#define PIN_LCD_RST    15
#define PIN_LCD_MISO   16
#define PIN_LCD_CS     17
#define PIN_LCD_SCK    18
#define PIN_LCD_MOSI   19

// Пин управления режимом PWM стабилизатора питания RP2040 (снижает шум АЦП)
#define PIN_SMPS_PWM   23

// Вход АЦП от предусилителя MAX4466 с пьезоэлементами
#define PIN_PIEZO_ADC0 26

// =====================================================================
// 2. НАСТРОЙКИ ЗВУКА, ШУМОПОДАВЛЕНИЯ И ПАЛИТРЫ (ЛЕГКО МЕНЯТЬ ЗДЕСЬ)
// =====================================================================

#define SAMPLE_RATE          32000   // 32 кГц для стабильной работы ФАПЧ PCM5102A
#define WINDOW_SAMPLES       1024    // Окно анализа ноты: 32 мс
#define NUM_NOTES            8       // C4, D4, E4, F4, G4, A4, B4, C5
#define DELAY_BUF_SIZE       6000    // Буфер эха: ~187 мс при 32 кГц
#define WAVE_POINTS          240     // Ширина осциллографа танцующей волны
#define MENU_IDLE_TIMEOUT_MS 20000   // Автовозврат к волне через 20 с бездействия

// --- НАСТРОЙКИ ЧИСТОТЫ ЗВУКА И ЧУВСТВИТЕЛЬНОСТИ ---
// Если в паузах прорывается шум — увеличьте GATE_OPEN_THRESH до 22..30.
// Если тихие щипки не слышны — уменьшите GATE_OPEN_THRESH до 10..12.
#define GATE_OPEN_THRESH     16      // Порог открытия шумоподавителя (в отсчетах АЦП)
#define PIEZO_PREAMP_GAIN    28      // Умеренное усиление без перегруза шумом (было 90)
#define PLUCK_DETECT_PP      38      // Минимальный размах (Peak-to-Peak) для засчитывания ноты

// Цветовая палитра (RGB565)
#define COL_BG_NAVY          0x0863
#define COL_BG_GRID          0x1106
#define COL_HEADER           0x0190
#define COL_MIPT_BLUE        0x03DF
#define COL_MIPT_CYAN        0x07FF
#define COL_WHITE            0xFFFF
#define COL_SILVER           0xBDF7
#define COL_GREEN            0x07E0
#define COL_YELLOW           0xFFE0
#define COL_ORANGE           0xFD20
#define COL_RED              0xF800

typedef enum {
    FX_CLEAN = 0,
    FX_OVERDRIVE = 1,
    FX_DISTORTION = 2,
    FX_DELAY = 3
} effect_mode_t;

typedef enum {
    UI_STATE_WAVE = 0, // По умолчанию: Танцующая волна
    UI_STATE_MENU = 1, // Меню выбора мелодии
    UI_STATE_GAME = 2  // Игра «Падающие ноты»
} ui_state_t;

static const float NOTE_FREQS[NUM_NOTES] = {
    261.63f, 293.66f, 329.63f, 349.23f,
    392.00f, 440.00f, 493.88f, 523.25f
};

static const char *NOTE_NAMES[NUM_NOTES] = {
    "C4", "D4", "E4", "F4", "G4", "A4", "B4", "C5"
};

static const uint16_t NOTE_COLORS[NUM_NOTES] = {
    0xF800, 0xFD20, 0xFFE0, 0x07E0, 0x07FF, 0x3B9F, 0xB81F, 0xF81F
};

static const char *FX_NAMES[4] = {
    "CLEAN", "OVERDRIVE", "DISTORTION", "DELAY/ECHO"
};

// Глобальные переменные обмена между Core 0 и Core 1
static volatile float g_master_volume = 0.85f;
static volatile effect_mode_t g_effect_mode = FX_CLEAN;
static volatile int g_detected_note = -1;
static volatile float g_detected_freq_hz = 0.0f;
static volatile uint32_t g_note_detect_time = 0;
static volatile int32_t g_wave_amplitude = 0;
static volatile int8_t g_wave_buffer[WAVE_POINTS];

// Встроенный синтезатор (для стартового аккорда и демо-режима)
static volatile float g_synth_freq = 0.0f;
static volatile int32_t g_synth_rem_samples = 0;
static volatile int32_t g_synth_total_samples = 1;

// Диагностика для COM-порта
static volatile int32_t g_diag_dc = 2048;
static volatile int32_t g_diag_pp = 0;
static volatile bool g_diag_hit_flag = false;

static float goertzel_coeff[NUM_NOTES];
static int16_t delay_buffer[DELAY_BUF_SIZE];
static uint32_t delay_idx = 0;

static void trigger_synth_tone(float freq, int duration_ms) {
    g_synth_total_samples = (duration_ms * SAMPLE_RATE) / 1000;
    g_synth_freq = freq;
    g_synth_rem_samples = g_synth_total_samples;
}

// =====================================================================
// 3. ПРОВЕРЕННЫЙ I2S ДРАЙВЕР НА PIO0 (32 кГц, SIDE-SET)
// =====================================================================
static void init_pio_i2s(PIO pio, uint sm) {
    uint16_t i2s_prog_instr[8] = {
        pio_encode_out(pio_pins, 1)  | pio_encode_sideset(2, 0b00),
        pio_encode_jmp_x_dec(0)      | pio_encode_sideset(2, 0b01),
        pio_encode_out(pio_pins, 1)  | pio_encode_sideset(2, 0b10),
        pio_encode_set(pio_x, 14)    | pio_encode_sideset(2, 0b11),
        pio_encode_out(pio_pins, 1)  | pio_encode_sideset(2, 0b10),
        pio_encode_jmp_x_dec(4)      | pio_encode_sideset(2, 0b11),
        pio_encode_out(pio_pins, 1)  | pio_encode_sideset(2, 0b00),
        pio_encode_set(pio_x, 14)    | pio_encode_sideset(2, 0b01)
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
    sm_config_set_sideset_pins(&c, PIN_I2S_BCK);
    sm_config_set_sideset(&c, 2, false, false);
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

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
// 4. ЯДРО 1: ЧИСТЫЙ DSP-ТРАКТ, ФИЛЬТРЫ, ЭФФЕКТЫ И ДЕТЕКТОР НОТ
// =====================================================================
static void core1_audio_entry(void) {
    for (int i = 0; i < NUM_NOTES; i++) {
        float omega = (2.0f * 3.14159265f * NOTE_FREQS[i]) / (float)SAMPLE_RATE;
        goertzel_coeff[i] = 2.0f * cosf(omega);
    }

    // Фиксируем мультиплексор АЦП строго на канале 0 (GP26), чтобы не было щелчков!
    adc_select_input(0);

    int32_t dc_est = 2048 << 8;
    int32_t in_lpf1 = 0;     // Входной ФНЧ 1-го порядка (~3 кГц) против ВЧ-писка экрана
    int32_t in_lpf2 = 0;     // Входной ФНЧ 2-го порядка
    int32_t env_follower = 0;// Детектор огибающей для плавного шумоподавителя (Noise Gate)
    int32_t cab_lpf1 = 0;    // Фильтр гитарного кабинета 1 (срезает «песок» в Distortion)
    int32_t cab_lpf2 = 0;    // Фильтр гитарного кабинета 2

    float q1[NUM_NOTES] = {0};
    float q2[NUM_NOTES] = {0};
    int sample_cnt = 0;
    int32_t min_raw = 4095, max_raw = 0;

    // Переменные частотомера
    int32_t bp_tuner = 0;
    int32_t prev_bp = 0;
    int zc_count = 0;
    int first_zc = -1, last_zc = -1, zc_timer = 0;

    // Переменные осциллографа танцующей волны
    int wave_idx = 0, wave_decim = 0;
    int32_t prev_vis = 0;
    bool wave_capturing = false;
    float synth_phase = 0.0f;

    while (true) {
        // 1. Усреднение 2 выборок АЦП подряд (снижает белый шум АЦП на 3 дБ)
        int32_t r1 = (int32_t)adc_read();
        int32_t r2 = (int32_t)adc_read();
        int32_t raw = (r1 + r2) >> 1;

        if (raw < min_raw) min_raw = raw;
        if (raw > max_raw) max_raw = raw;

        // Вычитание постоянной составляющей (ФВЧ ~120 Гц против сетевого фона 50 Гц)
        dc_est += ((raw << 8) - dc_est) >> 8;
        int32_t raw_ac = raw - (dc_est >> 8);

        // 2. Входной двухкаскадный ФНЧ (отрезает ВЧ-шум SPI дисплея и квантования)
        in_lpf1 += (raw_ac - in_lpf1) >> 1;
        in_lpf2 += (in_lpf1 - in_lpf2) >> 1;
        int32_t clean_ac = in_lpf2;

        // 3. Плавный шумоподавитель (Envelope Expander / Noise Gate)
        int32_t abs_in = abs(clean_ac) << 4;
        if (abs_in > env_follower) {
            env_follower += (abs_in - env_follower) >> 2; // Быстрая атака (мгновенное открытие)
        } else {
            env_follower += (abs_in - env_follower) >> 8; // Плавное затухание хвоста ноты
        }

        int32_t env_counts = env_follower >> 4;
        int32_t gated_sig = 0;
        if (env_counts > GATE_OPEN_THRESH) {
            // Плавное открытие гейта без щелчков
            int32_t gate_scale = env_counts - GATE_OPEN_THRESH;
            if (gate_scale > 32) gate_scale = 32;
            gated_sig = (clean_ac * gate_scale) >> 5;
        }

        int32_t x = gated_sig * PIEZO_PREAMP_GAIN;

        // 4. Подмешивание встроенного синтезатора (при старте или демо-режиме)
        int32_t rem = g_synth_rem_samples;
        if (rem > 0) {
            float freq = g_synth_freq;
            float env = (float)rem / (float)g_synth_total_samples;
            float wave = sinf(synth_phase) + 0.25f * sinf(synth_phase * 2.0f);
            synth_phase += (2.0f * 3.14159265f * freq) / (float)SAMPLE_RATE;
            if (synth_phase > 6.2831853f) synth_phase -= 6.2831853f;
            x += (int32_t)(wave * env * 14000.0f);
            g_synth_rem_samples = rem - 1;
        }

        // 5. Захват волны для экрана
        int32_t vis = (gated_sig * 2) + (x >> 9);
        if (!wave_capturing) {
            if (prev_vis < 0 && vis >= 0 && abs(vis) > 6) {
                wave_capturing = true;
                wave_idx = 0;
            }
        } else {
            wave_decim++;
            if (wave_decim >= 4) {
                wave_decim = 0;
                int32_t sc = vis / 2;
                if (sc > 58) sc = 58;
                if (sc < -58) sc = -58;
                g_wave_buffer[wave_idx++] = (int8_t)sc;
                if (wave_idx >= WAVE_POINTS) wave_capturing = false;
            }
        }
        prev_vis = vis;

        // 6. Гибридный детектор нот (Герцель + Частотомер переходов через 0)
        bp_tuner += (clean_ac - bp_tuner) >> 3;
        if (prev_bp < 0 && bp_tuner >= 0 && abs(clean_ac) > 8) {
            if (first_zc < 0) first_zc = zc_timer;
            last_zc = zc_timer;
            zc_count++;
        }
        prev_bp = bp_tuner;
        zc_timer++;

        float xf = (float)clean_ac;
        for (int i = 0; i < NUM_NOTES; i++) {
            float q0 = goertzel_coeff[i] * q1[i] - q2[i] + xf;
            q2[i] = q1[i];
            q1[i] = q0;
        }

        sample_cnt++;
        if (sample_cnt >= WINDOW_SAMPLES) {
            int32_t pp = max_raw - min_raw;
            g_diag_dc = dc_est >> 8;
            g_diag_pp = pp;
            g_wave_amplitude = (pp > GATE_OPEN_THRESH * 2) ? (pp / 2) : 0;

            if (pp >= PLUCK_DETECT_PP) {
                int detected_idx = -1;
                float meas_hz = 0.0f;

                // Способ А: точное измерение частоты по периоду колебаний
                if (zc_count >= 3 && last_zc > first_zc) {
                    meas_hz = ((float)(zc_count - 1) * (float)SAMPLE_RATE) / (float)(last_zc - first_zc);
                    if (meas_hz >= 180.0f && meas_hz <= 750.0f) {
                        float min_d = 9999.0f;
                        for (int i = 0; i < NUM_NOTES; i++) {
                            float d = fabsf(meas_hz - NOTE_FREQS[i]);
                            if (d < min_d) { min_d = d; detected_idx = i; }
                        }
                    }
                }

                // Способ Б: резервная проверка по банкам Герцеля
                if (detected_idx < 0) {
                    float best_mag = 40000.0f;
                    for (int i = 0; i < NUM_NOTES; i++) {
                        float mag = q1[i] * q1[i] + q2[i] * q2[i] - goertzel_coeff[i] * q1[i] * q2[i];
                        if (mag > best_mag) {
                            best_mag = mag;
                            detected_idx = i;
                            meas_hz = NOTE_FREQS[i];
                        }
                    }
                }

                if (detected_idx >= 0) {
                    g_detected_note = detected_idx;
                    g_detected_freq_hz = meas_hz;
                    g_note_detect_time = to_ms_since_boot(get_absolute_time());
                    g_diag_hit_flag = true;
                }
            }

            memset(q1, 0, sizeof(q1));
            memset(q2, 0, sizeof(q2));
            sample_cnt = 0;
            min_raw = 4095; max_raw = 0;
            zc_count = 0; first_zc = -1; last_zc = -1; zc_timer = 0;
        }

        // 7. Музыкальные гитарные эффекты (с кабинет-фильтрацией ВЧ-песка)
        effect_mode_t mode = g_effect_mode;
        int32_t y = 0;

        if (mode == FX_CLEAN) {
            // Чистый акустический тембр
            cab_lpf1 += (x - cab_lpf1) >> 1;
            y = cab_lpf1;
        } else if (mode == FX_OVERDRIVE) {
            // Мягкий ламповый Tube Overdrive (гладкая кривая без жестких углов)
            float fn = (float)x / 9000.0f;
            float soft = (2.0f * fn) / (1.0f + fabsf(fn));
            int32_t drv = (int32_t)(soft * 16000.0f);
            cab_lpf1 += (drv - cab_lpf1) >> 2;
            y = cab_lpf1;
        } else if (mode == FX_DISTORTION) {
            // Плавный певучий Distortion + двухполюсный спикер-симулятор (убирает зуд и песок)
            float fn = (float)x / 3500.0f;
            float sat = fn / (1.0f + fabsf(fn));
            // Легкая асимметрия четных гармоник (как у гитарного усилителя)
            float asym = sat + 0.12f * sat * sat;
            int32_t dist = (int32_t)(asym * 19000.0f);
            cab_lpf1 += (dist - cab_lpf1) >> 2;
            cab_lpf2 += (cab_lpf1 - cab_lpf2) >> 2;
            y = cab_lpf2;
        } else if (mode == FX_DELAY) {
            // Пространственное эхо с теплым затуханием повторов
            int16_t delayed = delay_buffer[delay_idx];
            cab_lpf1 += (x - cab_lpf1) >> 1;
            y = cab_lpf1 + (delayed >> 1);
            int32_t fb = cab_lpf1 + ((delayed * 7) >> 4);
            // Фильтрация ВЧ в петле обратной связи эха
            cab_lpf2 += (fb - cab_lpf2) >> 1;
            if (cab_lpf2 > 32767) cab_lpf2 = 32767;
            if (cab_lpf2 < -32768) cab_lpf2 = -32768;
            delay_buffer[delay_idx] = (int16_t)cab_lpf2;
            delay_idx++;
            if (delay_idx >= DELAY_BUF_SIZE) delay_idx = 0;
        }

        // 8. Мастер-громкость и отправка в I2S ЦАП
        y = (int32_t)((float)y * g_master_volume);
        if (y > 32767) y = 32767;
        if (y < -32768) y = -32768;

        uint16_t out16 = (uint16_t)(int16_t)y;
        uint32_t stereo_word = ((uint32_t)out16 << 16) | out16;
        pio_sm_put_blocking(pio0, 0, stereo_word);
    }
}

// =====================================================================
// 5. ДРАЙВЕР ADS1115 (ГРОМКОСТЬ ПО I2C)
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
            if (v < 0.25f) v = 0.25f;
            ads1115_start_single_a0();
            return v;
        }
    }
    ads1115_start_single_a0();
    return g_master_volume;
}

// =====================================================================
// 6. ГРАФИЧЕСКИЙ ДРАЙВЕР ILI9341 И ЛОГОТИП МФТИ
// =====================================================================
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
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > 320) w = 320 - x;
    if (y + h > 240) h = 240 - y;

    lcd_set_window(x, y, x + w - 1, y + h - 1);

    uint8_t hi = color >> 8, lo = color & 0xFF;
    uint8_t line_buf[640];
    for (int i = 0; i < w; i++) {
        line_buf[i * 2] = hi;
        line_buf[i * 2 + 1] = lo;
    }

    gpio_put(PIN_LCD_DC, 1);
    gpio_put(PIN_LCD_CS, 0);
    for (int row = 0; row < h; row++) {
        spi_write_blocking(spi0, line_buf, w * 2);
    }
    gpio_put(PIN_LCD_CS, 1);
}

static void draw_styled_background(void) {
    lcd_fill_rect(0, 0, 320, 240, COL_BG_NAVY);
    for (int x = 0; x < 320; x += 20) {
        lcd_fill_rect(x, 24, 1, 216, COL_BG_GRID);
    }
    for (int y = 24; y < 240; y += 20) {
        lcd_fill_rect(0, y, 320, 1, COL_BG_GRID);
    }
}

static void draw_mipt_logo(int x0, int y0) {
    const int W = 72;
    const int H = 36;
    uint8_t buf[72 * 36 * 2];

    static const char *MIPT_BITMAP[7] = {
        "##...##..#####..######..##...##",
        "###.###.##.|.##...##....##..###",
        "#######.##.|.##...##....##.####",
        "##.#.##.##.|.##...##....####.##",
        "##...##..#####....##....###..##",
        "##...##....|......##....##...##",
        "##...##....|......##....##...##"
    };

    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) {
            uint16_t col = COL_BG_NAVY;

            if ((r == 0 || r == H - 1) && c >= 3 && c < W - 3) col = COL_MIPT_BLUE;
            else if ((c == 0 || c == W - 1) && r >= 3 && r < H - 3) col = COL_MIPT_BLUE;
            else if ((r == 1 || r == H - 2) && (c == 2 || c == W - 3)) col = COL_MIPT_BLUE;
            else if ((c == 1 || c == W - 2) && (r == 2 || r == H - 3)) col = COL_MIPT_BLUE;

            int wave_y = 14;
            if (c >= 6 && c < 22) wave_y = 14;
            else if (c >= 22 && c < 26) wave_y = 14 + (c - 22);
            else if (c >= 26 && c < 33) wave_y = 18 - (c - 26) * 2;
            else if (c >= 33 && c < 39) wave_y = 4 + (c - 33) * 3;
            else if (c >= 39 && c < 44) wave_y = 22 - (c - 39) * 2;
            else if (c >= 44 && c < 47) wave_y = 12 + (c - 44);
            else if (c >= 47 && c < 66) wave_y = 14;

            if (c >= 6 && c < 66) {
                if (abs(r - wave_y) <= 1) {
                    col = (r == wave_y) ? COL_MIPT_CYAN : COL_MIPT_BLUE;
                }
            }

            int br = r - 25;
            int bc = c - 20;
            if (br >= 0 && br < 7 && bc >= 0 && bc < 31) {
                char px = MIPT_BITMAP[br][bc];
                if (px == '#' || px == '|') {
                    col = COL_WHITE;
                }
            }

            int idx = (r * W + c) * 2;
            buf[idx] = col >> 8;
            buf[idx + 1] = col & 0xFF;
        }
    }

    lcd_set_window(x0, y0, x0 + W - 1, y0 + H - 1);
    gpio_put(PIN_LCD_DC, 1);
    gpio_put(PIN_LCD_CS, 0);
    spi_write_blocking(spi0, buf, sizeof(buf));
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
    if (c == '/') return (row < 6 && row > 0) ? (1 << (row - 1)) : 0x00;
    if (c == '-') return (row == 3) ? 0x0E : 0x00;
    if (c == '.') return (row == 6) ? 0x04 : 0x00;
    return 0x00;
}

static void lcd_draw_text(int x, int y, const char *str, uint16_t fg, uint16_t bg, int scale) {
    if (scale < 1) scale = 1;
    if (scale > 3) scale = 3;
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

// =====================================================================
// 7. КОЛЛЕКЦИЯ МЕЛОДИЙ И ИГРА «ПАДАЮЩИЕ НОТЫ»
// =====================================================================
typedef struct {
    uint8_t tine;      // 0=C4, 1=D4, 2=E4, 3=F4, 4=G4, 5=A4, 6=B4, 7=C5
    uint16_t delay_ms;
} song_step_t;

typedef struct {
    const char *title;
    const song_step_t *steps;
    int length;
    bool auto_synth;
} melody_t;

static const song_step_t SONG_UNDERTALE_MEMORY[] = {
    {0,520}, {4,520}, {3,520}, {0,520}, {2,520}, {2,520}, {3,1040},
    {0,520}, {2,520}, {4,520}, {3,520}, {0,520}, {2,520}, {2,520}, {3,1040},
    {0,520}, {4,520}, {3,520}, {0,520}, {2,520}, {3,520}, {2,520}, {0,520}, {1,1200},
    {0,520}, {2,520}, {5,520}, {4,520}, {3,520}, {4,520}, {5,520}, {7,520}, {6,700}, {4,520}, {1,1400}
};

static const song_step_t SONG_SCALE[] = {
    {0,600},{1,600},{2,600},{3,600},{4,600},{5,600},{6,600},{7,900},
    {7,600},{6,600},{5,600},{4,600},{3,600},{2,600},{1,600},{0,1200}
};

static const song_step_t SONG_ODE[] = {
    {2,550},{2,550},{3,550},{4,550},{4,550},{3,550},{2,550},{1,550},
    {0,550},{0,550},{1,550},{2,550},{2,800},{1,400},{1,1000},
    {2,550},{2,550},{3,550},{4,550},{4,550},{3,550},{2,550},{1,550},
    {0,550},{0,550},{1,550},{2,550},{1,800},{0,400},{0,1200}
};

static const song_step_t SONG_STAR[] = {
    {0,550},{0,550},{4,550},{4,550},{5,550},{5,550},{4,1000},
    {3,550},{3,550},{2,550},{2,550},{1,550},{1,550},{0,1000},
    {4,550},{4,550},{3,550},{3,550},{2,550},{2,550},{1,1000},
    {4,550},{4,550},{3,550},{3,550},{2,550},{2,550},{1,1200}
};

#define NUM_MELODIES 5
static const melody_t MELODIES[NUM_MELODIES] = {
    {"1: UNDERTALE - MEMORY", SONG_UNDERTALE_MEMORY, 35, false},
    {"2: C-MAJOR SCALE",      SONG_SCALE,            16, false},
    {"3: ODE TO JOY",         SONG_ODE,              30, false},
    {"4: TWINKLE STAR",       SONG_STAR,             28, false},
    {"5: AUTO SYNTH DEMO",    SONG_UNDERTALE_MEMORY, 35, true}
};

#define MAX_FALLING 12
typedef struct {
    bool active;
    bool hit;
    uint8_t tine;
    int y;
    int prev_y;
} falling_note_t;

static void update_rotary_switch(void) {
    effect_mode_t next_mode = g_effect_mode;
    if (!gpio_get(PIN_SW_POS1))      next_mode = FX_CLEAN;
    else if (!gpio_get(PIN_SW_POS2)) next_mode = FX_OVERDRIVE;
    else if (!gpio_get(PIN_SW_POS3)) next_mode = FX_DISTORTION;
    else if (!gpio_get(PIN_SW_POS4)) next_mode = FX_DELAY;

    if (next_mode != g_effect_mode) {
        g_effect_mode = next_mode;
        printf("[SWITCH] Effect mode -> %s\n", FX_NAMES[next_mode]);
    }
}

static bool any_button_pressed(void) {
    return (!gpio_get(PIN_BTN_UP) || !gpio_get(PIN_BTN_DOWN) ||
            !gpio_get(PIN_BTN_LEFT) || !gpio_get(PIN_BTN_RIGHT) ||
            !gpio_get(PIN_BTN_OK));
}

// =====================================================================
// 8. ГЛАВНЫЙ ЦИКЛ (ЯДРО 0: ТАНЦУЮЩАЯ ВОЛНА, МЕНЮ И ПАДАЮЩИЕ НОТЫ)
// =====================================================================
int main(void) {
    stdio_init_all();

    // Переводим импульсный стабилизатор питания RP2040 в малошумящий режим PWM
    gpio_init(PIN_SMPS_PWM);
    gpio_set_dir(PIN_SMPS_PWM, GPIO_OUT);
    gpio_put(PIN_SMPS_PWM, 1);

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
    adc_gpio_init(PIN_PIEZO_ADC0);

    // Частота SPI0 31.25 МГц (дает меньше ВЧ-наводок на АЦП, чем 62.5 МГц)
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

    // Приветственный мягкий аккорд при включении
    sleep_ms(400);
    trigger_synth_tone(261.63f, 160); sleep_ms(180);
    trigger_synth_tone(329.63f, 160); sleep_ms(180);
    trigger_synth_tone(392.00f, 260);

    ui_state_t ui_state = UI_STATE_WAVE;
    bool need_redraw = true;
    int selected_song = 0;
    int fall_speed = 3;
    int score = 0;

    falling_note_t notes[MAX_FALLING];
    int next_step_idx = 0;
    uint32_t next_spawn_ms = 0;
    uint32_t last_input_ms = 0;
    uint32_t last_i2c_ms = 0;
    effect_mode_t prev_fx = (effect_mode_t)99;
    int prev_detected = -2;

    int16_t prev_wave_y1[WAVE_POINTS];
    int16_t prev_wave_y2[WAVE_POINTS];
    for (int i = 0; i < WAVE_POINTS; i++) {
        prev_wave_y1[i] = 120;
        prev_wave_y2[i] = 120;
    }
    float wave_phase = 0.0f;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (now - last_i2c_ms >= 60) {
            last_i2c_ms = now;
            update_rotary_switch();
            g_master_volume = ads1115_read_volume();
        }

        if (g_diag_hit_flag) {
            g_diag_hit_flag = false;
            int n = g_detected_note;
            if (n >= 0 && n < NUM_NOTES) {
                printf("[PIEZO] Note=%s (Measured=%.1f Hz, Target=%.1f Hz) | GP26_PP=%d cts | DC=%d\n",
                       NOTE_NAMES[n], g_detected_freq_hz, NOTE_FREQS[n], (int)g_diag_pp, (int)g_diag_dc);
            }
        }

        int cur_note = g_detected_note;
        if (cur_note != -1 && (now - g_note_detect_time > 350)) {
            g_detected_note = -1;
            cur_note = -1;
        }

        if (ui_state == UI_STATE_MENU && (now - last_input_ms > MENU_IDLE_TIMEOUT_MS)) {
            ui_state = UI_STATE_WAVE;
            need_redraw = true;
        }

        // Обработка кнопок управления
        if (now - last_input_ms > 180) {
            if (ui_state == UI_STATE_WAVE) {
                if (any_button_pressed()) {
                    ui_state = UI_STATE_MENU;
                    need_redraw = true;
                    last_input_ms = now;
                }
            } else if (ui_state == UI_STATE_MENU) {
                if (!gpio_get(PIN_BTN_UP)) {
                    selected_song = (selected_song + NUM_MELODIES - 1) % NUM_MELODIES;
                    need_redraw = true;
                    last_input_ms = now;
                } else if (!gpio_get(PIN_BTN_DOWN)) {
                    selected_song = (selected_song + 1) % NUM_MELODIES;
                    need_redraw = true;
                    last_input_ms = now;
                } else if (!gpio_get(PIN_BTN_RIGHT)) {
                    fall_speed = (fall_speed >= 5) ? 2 : (fall_speed + 1);
                    need_redraw = true;
                    last_input_ms = now;
                } else if (!gpio_get(PIN_BTN_LEFT)) {
                    ui_state = UI_STATE_WAVE;
                    need_redraw = true;
                    last_input_ms = now;
                } else if (!gpio_get(PIN_BTN_OK)) {
                    ui_state = UI_STATE_GAME;
                    need_redraw = true;
                    score = 0;
                    next_step_idx = 0;
                    next_spawn_ms = now + 400;
                    memset(notes, 0, sizeof(notes));
                    last_input_ms = now;
                }
            } else if (ui_state == UI_STATE_GAME) {
                if (!gpio_get(PIN_BTN_LEFT) || !gpio_get(PIN_BTN_OK)) {
                    ui_state = UI_STATE_MENU;
                    need_redraw = true;
                    last_input_ms = now;
                }
            }
        }

        // =================================================================
        // РЕЖИМ 1: ТАНЦУЮЩАЯ ВОЛНА + ЖИВОЙ ТЮНЕР И ЛОГОТИП МФТИ
        // =================================================================
        if (ui_state == UI_STATE_WAVE) {
            if (need_redraw) {
                draw_styled_background();
                lcd_fill_rect(0, 0, 320, 24, COL_HEADER);
                lcd_draw_text(8, 5, "KALIMBA DSP", COL_MIPT_CYAN, COL_HEADER, 2);
                lcd_draw_text(8, 222, "PRESS ANY BUTTON FOR MENU", COL_SILVER, COL_BG_NAVY, 1);
                draw_mipt_logo(244, 200);

                for (int i = 0; i < WAVE_POINTS; i++) {
                    prev_wave_y1[i] = 120;
                    prev_wave_y2[i] = 120;
                }
                prev_fx = (effect_mode_t)99;
                prev_detected = -2;
                need_redraw = false;
            }

            if (prev_fx != g_effect_mode) {
                char fx_str[20];
                snprintf(fx_str, sizeof(fx_str), "FX:%s", FX_NAMES[g_effect_mode]);
                lcd_fill_rect(164, 4, 152, 16, COL_HEADER);
                lcd_draw_text(168, 8, fx_str, COL_YELLOW, COL_HEADER, 1);
                prev_fx = g_effect_mode;
            }

            if (cur_note != prev_detected) {
                lcd_fill_rect(8, 30, 230, 22, COL_BG_NAVY);
                if (cur_note >= 0 && cur_note < NUM_NOTES) {
                    char nbuf[32];
                    snprintf(nbuf, sizeof(nbuf), "%s (%d HZ)", NOTE_NAMES[cur_note], (int)g_detected_freq_hz);
                    lcd_draw_text(10, 33, nbuf, NOTE_COLORS[cur_note], COL_BG_NAVY, 2);
                } else {
                    lcd_draw_text(10, 36, "PLAY ANY TINE...", COL_SILVER, COL_BG_NAVY, 1);
                }
                prev_detected = cur_note;
            }

            wave_phase += 0.20f;
            uint16_t main_col = (cur_note >= 0) ? NOTE_COLORS[cur_note] : COL_MIPT_CYAN;
            uint16_t harm_col = COL_MIPT_BLUE;
            int32_t amp = g_wave_amplitude;

            for (int i = 0; i < WAVE_POINTS; i++) {
                int screen_x = 8 + i;
                uint16_t bg1 = ((screen_x % 20 == 0) || (prev_wave_y1[i] % 20 == 0)) ? COL_BG_GRID : COL_BG_NAVY;
                uint16_t bg2 = ((screen_x % 20 == 0) || (prev_wave_y2[i] % 20 == 0)) ? COL_BG_GRID : COL_BG_NAVY;
                lcd_fill_rect(screen_x, prev_wave_y2[i], 1, 2, bg2);
                lcd_fill_rect(screen_x, prev_wave_y1[i], 1, 3, bg1);

                float env = sinf((3.14159265f * (float)i) / (float)WAVE_POINTS);
                float live = (float)g_wave_buffer[i] * 1.2f;
                float synth_amp = 3.5f + (float)(amp > 60 ? 60 : amp) * 0.45f;
                float s1 = sinf(wave_phase + (float)i * 0.11f) * synth_amp;
                float s2 = cosf(wave_phase * 1.5f - (float)i * 0.07f) * (synth_amp * 0.55f);

                int y1 = 120 + (int)((live + s1) * env);
                int y2 = 120 + (int)((live * 0.5f + s2) * env);

                if (y1 < 58) y1 = 58;
                if (y1 > 185) y1 = 185;
                if (y2 < 58) y2 = 58;
                if (y2 > 185) y2 = 185;

                lcd_fill_rect(screen_x, y2, 1, 2, harm_col);
                lcd_fill_rect(screen_x, y1, 1, 3, main_col);

                prev_wave_y1[i] = (int16_t)y1;
                prev_wave_y2[i] = (int16_t)y2;
            }
            sleep_ms(16);
        }
        // =================================================================
        // РЕЖИМ 2: МЕНЮ ВЫБОРА МЕЛОДИЙ
        // =================================================================
        else if (ui_state == UI_STATE_MENU) {
            if (need_redraw || prev_fx != g_effect_mode) {
                if (need_redraw) {
                    draw_styled_background();
                    lcd_fill_rect(0, 0, 320, 26, COL_HEADER);
                    lcd_draw_text(10, 6, "ELECTRONIC KALIMBA MIPT", COL_WHITE, COL_HEADER, 2);
                    lcd_draw_text(10, 31, "OK:START  LEFT:WAVE  RIGHT:SPEED", COL_GREEN, COL_BG_NAVY, 1);
                    draw_mipt_logo(244, 200);
                }

                for (int i = 0; i < NUM_MELODIES; i++) {
                    bool sel = (i == selected_song);
                    uint16_t bg = sel ? COL_MIPT_BLUE : COL_HEADER;
                    uint16_t fg = sel ? COL_YELLOW : (MELODIES[i].auto_synth ? COL_MIPT_CYAN : COL_WHITE);
                    lcd_fill_rect(10, 44 + i * 24, 226, 20, bg);
                    if (sel) {
                        lcd_fill_rect(10, 44 + i * 24, 4, 20, COL_MIPT_CYAN);
                    }
                    lcd_draw_text(18, 50 + i * 24, MELODIES[i].title, fg, bg, 1);
                }

                char spd_buf[36];
                snprintf(spd_buf, sizeof(spd_buf), "SPEED:%d  FX:%s", fall_speed, FX_NAMES[g_effect_mode]);
                lcd_fill_rect(10, 170, 226, 16, COL_BG_NAVY);
                lcd_draw_text(10, 174, spd_buf, COL_MIPT_CYAN, COL_BG_NAVY, 1);

                lcd_draw_text(10, 198, "TUNER:", COL_SILVER, COL_BG_NAVY, 1);
                prev_fx = g_effect_mode;
                prev_detected = -2;
                need_redraw = false;
            }

            if (cur_note != prev_detected) {
                lcd_fill_rect(56, 192, 160, 20, COL_BG_NAVY);
                if (cur_note >= 0 && cur_note < NUM_NOTES) {
                    char tbuf[24];
                    snprintf(tbuf, sizeof(tbuf), "%s (%dHZ)", NOTE_NAMES[cur_note], (int)g_detected_freq_hz);
                    lcd_draw_text(58, 194, tbuf, NOTE_COLORS[cur_note], COL_BG_NAVY, 2);
                } else {
                    lcd_draw_text(58, 194, "--", COL_SILVER, COL_BG_NAVY, 2);
                }
                prev_detected = cur_note;
            }
            sleep_ms(20);
        }
        // =================================================================
        // РЕЖИМ 3: ИГРА «ПАДАЮЩИЕ НОТЫ»
        // =================================================================
        else if (ui_state == UI_STATE_GAME) {
            if (need_redraw) {
                draw_styled_background();
                lcd_fill_rect(0, 0, 320, 22, COL_HEADER);
                lcd_draw_text(8, 7, MELODIES[selected_song].title, COL_YELLOW, COL_HEADER, 1);

                for (int t = 0; t < NUM_NOTES; t++) {
                    lcd_fill_rect(t * 40, 22, 1, 176, COL_MIPT_BLUE);
                    lcd_draw_text(t * 40 + 10, 204, NOTE_NAMES[t], COL_WHITE, COL_BG_NAVY, 2);
                }
                lcd_fill_rect(0, 192, 320, 3, COL_GREEN);
                draw_mipt_logo(244, 200);
                need_redraw = false;
            }

            const melody_t *mel = &MELODIES[selected_song];
            if (now >= next_spawn_ms) {
                uint8_t lane = mel->steps[next_step_idx].tine;
                uint16_t d_ms = mel->steps[next_step_idx].delay_ms;

                if (mel->auto_synth) {
                    trigger_synth_tone(NOTE_FREQS[lane], d_ms - 40);
                }

                for (int i = 0; i < MAX_FALLING; i++) {
                    if (!notes[i].active) {
                        notes[i].active = true;
                        notes[i].hit = false;
                        notes[i].tine = lane;
                        notes[i].y = 24;
                        notes[i].prev_y = 24;
                        break;
                    }
                }
                next_spawn_ms = now + d_ms;
                next_step_idx = (next_step_idx + 1) % mel->length;
            }

            for (int i = 0; i < MAX_FALLING; i++) {
                if (!notes[i].active) continue;

                int x = notes[i].tine * 40 + 5;
                lcd_fill_rect(x, notes[i].prev_y, 30, 12, COL_BG_NAVY);

                notes[i].y += fall_speed;
                notes[i].prev_y = notes[i].y;

                if (!notes[i].hit && notes[i].y >= 170 && notes[i].y <= 200) {
                    if (cur_note == notes[i].tine) {
                        notes[i].hit = true;
                        score += 10;
                        char sc_buf[16];
                        snprintf(sc_buf, sizeof(sc_buf), "SCORE:%d", score);
                        lcd_fill_rect(230, 4, 86, 14, COL_HEADER);
                        lcd_draw_text(234, 7, sc_buf, COL_GREEN, COL_HEADER, 1);
                    }
                }

                if (notes[i].y > 195) {
                    notes[i].active = false;
                    lcd_fill_rect(x, 192, 30, 3, COL_GREEN);
                } else {
                    uint16_t color = notes[i].hit ? COL_GREEN : NOTE_COLORS[notes[i].tine];
                    lcd_fill_rect(x, notes[i].y, 30, 12, color);
                }
            }

            if (cur_note != prev_detected) {
                for (int t = 0; t < 6; t++) {
                    uint16_t col = (t == cur_note) ? COL_GREEN : COL_WHITE;
                    lcd_draw_text(t * 40 + 10, 204, NOTE_NAMES[t], col, COL_BG_NAVY, 2);
                }
                prev_detected = cur_note;
            }

            sleep_ms(22);
        }
    }
    return 0;
}