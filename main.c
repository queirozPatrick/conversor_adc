#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"

// Definições de pinos
#define PINO_LED_VERMELHO 13
#define PINO_LED_VERDE 11
#define PINO_LED_AZUL 12

#define PINO_EIXO_X 26
#define PINO_EIXO_Y 27
#define PINO_BOTAO_JOYSTICK 22
#define PINO_BOTAO_A 5

#define PINO_I2C_SDA 14
#define PINO_I2C_SCL 15
#define ENDERECO_I2C 0x3C

#define LARGURA_DISPLAY 128
#define ALTURA_DISPLAY 64

#define ATRASO_DEBOUNCE_MS 200
#define ZONA_MORTA 200
#define FATOR_SUAVIZACAO 0.5f
#define AJUSTE_Y 52
#define VALOR_MAXIMO_PWM 255

// Variáveis globais
volatile bool pwm_habilitado = true;
volatile bool led_verde_ligado = false;
volatile uint8_t estilo_borda = 0; // 0 = sem borda, 1 = borda simples, 2 = borda dupla, 3 = borda tripla

volatile absolute_time_t ultimo_tempo_botao_joystick;
volatile absolute_time_t ultimo_tempo_botao_a;

ssd1306_t display;
uint fatia_vermelho, fatia_azul;
uint canal_vermelho, canal_azul;

// Protótipos de funções
void configurar_gpio();
void configurar_pwm();
void configurar_adc();
void configurar_i2c();
void inicializar_display();
void callback_botao(uint gpio, uint32_t eventos);
void atualizar_leds(uint16_t brilho_vermelho, uint16_t brilho_azul);
void desenhar_borda();
void desenhar_quadrado(uint8_t posicao_x, uint8_t posicao_y);
uint16_t filtrar_leitura(uint16_t leitura_atual, uint16_t leitura_anterior);
uint16_t calibrar_eixo_y(uint16_t leitura);
uint16_t calcular_brilho(uint16_t leitura, uint16_t centro);

// Função principal
int main()
{
    stdio_init_all();

    // Declaração da variável contador_printf
    int contador_printf = 0;
    
    configurar_gpio();
    configurar_pwm();
    configurar_adc();
    configurar_i2c();
    inicializar_display();

    uint16_t leitura_filtrada_x = 2048;
    uint16_t leitura_filtrada_y = 2048;

    while (true)
    {
        adc_select_input(0);
        uint16_t leitura_cru_y = adc_read();
        adc_select_input(1);
        uint16_t leitura_cru_x = adc_read();

        leitura_filtrada_x = filtrar_leitura(leitura_cru_x, leitura_filtrada_x);
        leitura_filtrada_y = filtrar_leitura(leitura_cru_y, leitura_filtrada_y);

        // Incrementa o contador
        contador_printf++;

        // Imprime as leituras do joystick no monitor serial a cada 10 ms
        if (contador_printf >= 5) // 50 ms / 5 = 10 ms
        {
            printf("Eixo X: %d, Eixo Y: %d\n", leitura_filtrada_x, leitura_filtrada_y);
            contador_printf = 0; // Reseta o contador
        }

        uint16_t nova_leitura_y = calibrar_eixo_y(leitura_filtrada_y);

        if ((leitura_filtrada_x > (2048 - ZONA_MORTA)) && (leitura_filtrada_x < (2048 + ZONA_MORTA)))
            leitura_filtrada_x = 2048;
        if ((nova_leitura_y > (2048 - ZONA_MORTA)) && (nova_leitura_y < (2048 + ZONA_MORTA)))
            nova_leitura_y = 2048;

        uint16_t brilho_vermelho = calcular_brilho(leitura_filtrada_x, 2048);
        uint16_t brilho_azul = calcular_brilho(nova_leitura_y, 2048);

        atualizar_leds(brilho_vermelho, brilho_azul);
        gpio_put(PINO_LED_VERDE, led_verde_ligado);

        uint8_t posicao_x = (uint32_t)leitura_filtrada_x * (LARGURA_DISPLAY - 8) / 4095;
        uint8_t posicao_y = (uint32_t)(4095 - nova_leitura_y) * (ALTURA_DISPLAY - 8) / 4095;

        ssd1306_fill(&display, false);
        desenhar_borda();
        desenhar_quadrado(posicao_x, posicao_y);
        ssd1306_send_data(&display);

        sleep_ms(50);
    }

    return 0;
}

// Configuração dos GPIOs
void configurar_gpio()
{
    gpio_init(PINO_BOTAO_JOYSTICK);
    gpio_set_dir(PINO_BOTAO_JOYSTICK, GPIO_IN);
    gpio_pull_up(PINO_BOTAO_JOYSTICK);
    gpio_set_irq_enabled_with_callback(PINO_BOTAO_JOYSTICK, GPIO_IRQ_EDGE_FALL, true, &callback_botao);

    gpio_init(PINO_BOTAO_A);
    gpio_set_dir(PINO_BOTAO_A, GPIO_IN);
    gpio_pull_up(PINO_BOTAO_A);
    gpio_set_irq_enabled(PINO_BOTAO_A, GPIO_IRQ_EDGE_FALL, true);

    gpio_init(PINO_LED_VERMELHO);
    gpio_set_dir(PINO_LED_VERMELHO, GPIO_OUT);
    gpio_init(PINO_LED_VERDE);
    gpio_set_dir(PINO_LED_VERDE, GPIO_OUT);
    gpio_init(PINO_LED_AZUL);
    gpio_set_dir(PINO_LED_AZUL, GPIO_OUT);
}

// Configuração do PWM
void configurar_pwm()
{
    gpio_set_function(PINO_LED_VERMELHO, GPIO_FUNC_PWM);
    fatia_vermelho = pwm_gpio_to_slice_num(PINO_LED_VERMELHO);
    canal_vermelho = pwm_gpio_to_channel(PINO_LED_VERMELHO);
    pwm_set_wrap(fatia_vermelho, VALOR_MAXIMO_PWM);
    pwm_set_clkdiv(fatia_vermelho, 125.f);
    pwm_set_enabled(fatia_vermelho, true);

    gpio_set_function(PINO_LED_AZUL, GPIO_FUNC_PWM);
    fatia_azul = pwm_gpio_to_slice_num(PINO_LED_AZUL);
    canal_azul = pwm_gpio_to_channel(PINO_LED_AZUL);
    pwm_set_wrap(fatia_azul, VALOR_MAXIMO_PWM);
    pwm_set_clkdiv(fatia_azul, 125.f);
    pwm_set_enabled(fatia_azul, true);
}

// Configuração do ADC
void configurar_adc()
{
    adc_init();
    adc_gpio_init(PINO_EIXO_X);
    adc_gpio_init(PINO_EIXO_Y);
}

// Configuração do I2C
void configurar_i2c()
{
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(PINO_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PINO_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PINO_I2C_SDA);
    gpio_pull_up(PINO_I2C_SCL);
}

// Inicialização do display
void inicializar_display()
{
    ssd1306_init(&display, LARGURA_DISPLAY, ALTURA_DISPLAY, false, ENDERECO_I2C, i2c1);
    ssd1306_config(&display);
}

// Callback para os botões
void callback_botao(uint gpio, uint32_t eventos)
{
    absolute_time_t agora = get_absolute_time();

    if (gpio == PINO_BOTAO_JOYSTICK)
    {
        if (absolute_time_diff_us(ultimo_tempo_botao_joystick, agora) < ATRASO_DEBOUNCE_MS * 1000)
            return;
        ultimo_tempo_botao_joystick = agora;

        // Alterna o estilo da borda entre 0 (sem borda), 1, 2 e 3
        estilo_borda = (estilo_borda + 1) % 4;

        // Liga o LED verde apenas se a borda estiver ativa (estilo_borda != 0)
        led_verde_ligado = (estilo_borda != 0);
    }
    else if (gpio == PINO_BOTAO_A)
    {
        if (absolute_time_diff_us(ultimo_tempo_botao_a, agora) < ATRASO_DEBOUNCE_MS * 1000)
            return;
        ultimo_tempo_botao_a = agora;

        pwm_habilitado = !pwm_habilitado;
        if (!pwm_habilitado)
        {
            pwm_set_chan_level(fatia_vermelho, canal_vermelho, 0);
            pwm_set_chan_level(fatia_azul, canal_azul, 0);
        }
    }
}

// Atualiza os LEDs PWM
void atualizar_leds(uint16_t brilho_vermelho, uint16_t brilho_azul)
{
    if (pwm_habilitado)
    {
        pwm_set_chan_level(fatia_vermelho, canal_vermelho, brilho_vermelho);
        pwm_set_chan_level(fatia_azul, canal_azul, brilho_azul);
    }
}

// Desenha a borda no display
void desenhar_borda()
{
    if (estilo_borda == 1)
    {
        // Estilo 1: borda simples
        ssd1306_rect(&display, 0, 0, LARGURA_DISPLAY, ALTURA_DISPLAY, true, false);
    }
    else if (estilo_borda == 2)
    {
        // Estilo 2: borda dupla
        ssd1306_rect(&display, 0, 0, LARGURA_DISPLAY, ALTURA_DISPLAY, true, false);
        ssd1306_rect(&display, 2, 2, LARGURA_DISPLAY - 4, ALTURA_DISPLAY - 4, true, false);
    }
    else if (estilo_borda == 3)
    {
        // Estilo 3: borda tripla
        ssd1306_rect(&display, 0, 0, LARGURA_DISPLAY, ALTURA_DISPLAY, true, false);
        ssd1306_rect(&display, 2, 2, LARGURA_DISPLAY - 4, ALTURA_DISPLAY - 4, true, false);
        ssd1306_rect(&display, 4, 4, LARGURA_DISPLAY - 8, ALTURA_DISPLAY - 8, true, false);
    }
    // Se estilo_borda == 0, não desenha nada
}

// Desenha o quadrado móvel no display
void desenhar_quadrado(uint8_t posicao_x, uint8_t posicao_y)
{
    ssd1306_rect(&display, posicao_y, posicao_x, 8, 8, true, true);
}

// Filtra a leitura do ADC usando EMA
uint16_t filtrar_leitura(uint16_t leitura_atual, uint16_t leitura_anterior)
{
    return (uint16_t)(FATOR_SUAVIZACAO * leitura_atual + (1.0f - FATOR_SUAVIZACAO) * leitura_anterior);
}

// Calibra o eixo Y
uint16_t calibrar_eixo_y(uint16_t leitura)
{
    uint16_t leitura_calibrada = leitura >= AJUSTE_Y ? leitura - AJUSTE_Y : 0;
    if (leitura_calibrada > (4095 - AJUSTE_Y))
        leitura_calibrada = 4095 - AJUSTE_Y;
    return (uint16_t)(((uint32_t)leitura_calibrada * 4095) / (4095 - AJUSTE_Y));
}

// Calcula o brilho do LED com base na leitura do ADC
uint16_t calcular_brilho(uint16_t leitura, uint16_t centro)
{
    if (leitura < centro - ZONA_MORTA)
        return ((uint32_t)(centro - ZONA_MORTA - leitura) * VALOR_MAXIMO_PWM) / (centro - ZONA_MORTA);
    else if (leitura > centro + ZONA_MORTA)
        return ((uint32_t)(leitura - (centro + ZONA_MORTA)) * VALOR_MAXIMO_PWM) / (4095 - centro - ZONA_MORTA);
    else
        return 0;
}