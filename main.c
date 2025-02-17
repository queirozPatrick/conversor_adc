#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"      
#include "hardware/adc.h"      // Biblioteca para controlar o conversor analógico-digital (ADC)
#include "hardware/i2c.h"      // Biblioteca para comunicação I2C
#include "lib/ssd1306.h"       // Biblioteca para controlar o display OLED SSD1306
#include "hardware/pwm.h"      // Biblioteca para controlar PWM

// Definições de pinos para LEDs e botões
#define LED_AZUL 12            // Pino do LED azul
#define LED_VERMELHO 13        // Pino do LED vermelho
#define LED_VERMELHO 13        // Pino do LED vermelho
#define LED_VERDE 11           // Pino do LED verde
#define BOTAO_JOYSTICK 22      // Pino do botão do joystick
#define BOTAO_A 5              // Pino do botão A
#define CANAL_ADC_Y 0          // Canal ADC para o eixo Y do joystick
#define CANAL_ADC_X 1          // Canal ADC para o eixo X do joystick

// Definições para o joystick
#define CENTRO_JOYSTICK 2048   // Valor central do joystick (12 bits, 0-4095)
#define ZONA_MORTA 200         // Zona morta para evitar leituras sensíveis ao redor do centro

// Definições para o display OLED
#define PORTA_I2C i2c1         // Porta I2C utilizada
#define PIN_I2C_SDA 14         // Pino dados do I2C
#define PIN_I2C_SCL 15         // Pino clock do I2C
#define ENDERECO_SSD1306 0x3C  // Endereço I2C do display LED
#define LARGURA_DISPLAY 128    // Largura do display em pixels
#define ALTURA_DISPLAY 64      // Altura do display em pixels

// Variáveis globais voláteis (para uso em interrupções)
volatile bool estado_pwm = true;               // Estado do PWM (ligado/desligado)
volatile bool estado_led_verde = false;        // Estado do LED verde
volatile uint8_t tipo_borda = 0;               // Tipo de borda a ser desenhada no display
volatile absolute_time_t tempo_ultimo_joystick; // Último tempo que o botão do joystick foi pressionado
volatile bool status_botao_joystick = false;   // Estado do botão do joystick
volatile absolute_time_t tempo_ultimo_botao_a; // Último tempo que o botão A foi pressionado
volatile bool status_botao_a = false;          // Estado do botão A

// Função para inicializar o PWM em um pino específico
void inicializar_pwm(uint pino) {
    gpio_set_function(pino, GPIO_FUNC_PWM);    // Configura o pino para função PWM
    uint slice = pwm_gpio_to_slice_num(pino);  // Obtém o slice (fatia) do PWM associado ao pino
    pwm_config configuracao = pwm_get_default_config(); // Obtém a configuração padrão do PWM
    pwm_config_set_wrap(&configuracao, 4095);  // Define o valor máximo do PWM (12 bits)
    pwm_init(slice, &configuracao, true);      // Inicializa o PWM com a configuração definida
}

// Função para converter a leitura do joystick em um valor PWM
uint16_t converter_joystick_para_pwm(uint16_t valor) {
    if (abs((int16_t)valor - CENTRO_JOYSTICK) < ZONA_MORTA) // Verifica se está na zona morta
        return 0;                                           // Retorna 0 se estiver na zona morta
    return (valor < CENTRO_JOYSTICK) ? (CENTRO_JOYSTICK - valor) : (valor - CENTRO_JOYSTICK); // Retorna o valor proporcional
}

// Função para desenhar bordas no display OLED
void desenhar_borda(ssd1306_t *display, uint8_t estilo) {
   // ssd1306_fill(display, false); // Limpa o display

    switch (estilo) {
        case 0:
            // Borda simples
            ssd1306_rect(display, 3, 3, 122, 58, true, false);
            break;
        case 1:
            // Borda dupla
            ssd1306_rect(display, 3, 3, 122, 58, true, false);
            ssd1306_rect(display, 6, 6, 116, 52, true, false);
            break;
        case 2:
            // Borda tripla
            ssd1306_rect(display, 3, 3, 122, 58, true, false);
            ssd1306_rect(display, 6, 6, 116, 52, true, false);
            ssd1306_rect(display, 9, 9, 110, 46, true, false);
            break;
    }
}

// Função de tratamento de interrupção dos botões
void tratar_interrupcao_gpio(uint gpio, uint32_t eventos) {
    absolute_time_t agora = get_absolute_time(); // Obtém o tempo atual
    if (gpio == BOTAO_JOYSTICK && absolute_time_diff_us(tempo_ultimo_joystick, agora) > 50000) {
        tempo_ultimo_joystick = agora; // Atualiza o tempo do último evento do joystick
        if (!gpio_get(BOTAO_JOYSTICK)) { // Verifica se o botão do joystick foi pressionado
            if (!status_botao_joystick) { // Verifica se o botão não estava pressionado antes
                status_botao_joystick = true; // Atualiza o estado do botão
                estado_led_verde = !estado_led_verde; // Alterna o estado do LED verde
                gpio_put(LED_VERDE, estado_led_verde); // Atualiza o LED verde
                tipo_borda = (tipo_borda + 1) % 3; // Alterna o tipo de borda
            }
        } else {
            status_botao_joystick = false; // Atualiza o estado do botão quando solto
        }
    } else if (gpio == BOTAO_A && absolute_time_diff_us(tempo_ultimo_botao_a, agora) > 50000) {
        tempo_ultimo_botao_a = agora; // Atualiza o tempo do último evento do botão A
        if (!gpio_get(BOTAO_A)) { // Verifica se o botão A foi pressionado
            if (!status_botao_a) { // Verifica se o botão não estava pressionado antes
                status_botao_a = true; // Atualiza o estado do botão
                estado_pwm = !estado_pwm; // Alterna o estado do PWM
                if (!estado_pwm) { // Se o PWM foi desligado
                    pwm_set_gpio_level(LED_AZUL, 0); // Desliga o LED azul
                    pwm_set_gpio_level(LED_VERMELHO, 0); // Desliga o LED vermelho
                }
            }
        } else {
            status_botao_a = false; // Atualiza o estado do botão quando solto
        }
    }
}

// Função principal
int main() {
    stdio_init_all(); // Inicializa a comunicação serial (para debug)

    // Inicialização do I2C para o display OLED
    i2c_init(PORTA_I2C, 400 * 1000); // Inicializa o I2C com frequência de 400 kHz
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C); // Configura o pino SDA para I2C
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C); // Configura o pino SCL para I2C
    gpio_pull_up(PIN_I2C_SDA); // Habilita o pull-up no pino SDA
    gpio_pull_up(PIN_I2C_SCL); // Habilita o pull-up no pino SCL

    // Inicialização do display OLED
    ssd1306_t display;
    ssd1306_init(&display, LARGURA_DISPLAY, ALTURA_DISPLAY, false, ENDERECO_SSD1306, PORTA_I2C); // Inicializa o display
    ssd1306_config(&display); // Configura o display
    ssd1306_send_data(&display); // Envia dados iniciais
    ssd1306_fill(&display, false); // Limpa o display
    ssd1306_send_data(&display); // Envia os dados atualizados

    // Inicialização dos botões e LEDs
    gpio_init(BOTAO_JOYSTICK); // Inicializa o pino do botão do joystick
    gpio_set_dir(BOTAO_JOYSTICK, GPIO_IN); // Configura o pino como entrada
    gpio_pull_up(BOTAO_JOYSTICK); // Habilita o pull-up no pino
    gpio_init(LED_VERDE); // Inicializa o pino do LED verde
    gpio_set_dir(LED_VERDE, GPIO_OUT); // Configura o pino como saída
    gpio_put(LED_VERDE, 0); // Desliga o LED verde inicialmente
    gpio_init(BOTAO_A); // Inicializa o pino do botão A
    gpio_set_dir(BOTAO_A, GPIO_IN); // Configura o pino como entrada
    gpio_pull_up(BOTAO_A); // Habilita o pull-up no pino

    // Configuração das interrupções dos botões
    gpio_set_irq_enabled_with_callback(BOTAO_JOYSTICK, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, tratar_interrupcao_gpio); // Habilita interrupção no botão do joystick
    gpio_set_irq_enabled(BOTAO_A, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true); // Habilita interrupção no botão A

    // Inicialização do ADC para leitura do joystick
    adc_init(); // Inicializa o ADC
    adc_gpio_init(26); // Configura o pino 26 para leitura analógica (eixo Y)
    adc_gpio_init(27); // Configura o pino 27 para leitura analógica (eixo X)

    // Inicialização do PWM para os LEDs azul e vermelho
    inicializar_pwm(LED_AZUL); // Configura o PWM para o LED azul
    inicializar_pwm(LED_VERMELHO); // Configura o PWM para o LED vermelho

    // Variáveis para controle da posição no display
    int16_t pos_x = LARGURA_DISPLAY / 2; // Define a posição inicial no meio do display
    int16_t pos_y = ALTURA_DISPLAY / 2;  // Define a posição inicial no meio do display
    uint16_t ultimo_x = CENTRO_JOYSTICK, ultimo_y = CENTRO_JOYSTICK; // Últimas leituras do joystick

    // Loop principal
    while (1) {
        // Leitura dos eixos X e Y do joystick
        adc_select_input(CANAL_ADC_Y); // Seleciona o canal do eixo Y
        uint16_t y = adc_read(); // Lê o valor do eixo Y
        adc_select_input(CANAL_ADC_X); // Seleciona o canal do eixo X
        uint16_t x = adc_read(); // Lê o valor do eixo X

        // Verifica se houve mudança significativa na leitura do joystick
        if (abs((int16_t)x - (int16_t)ultimo_x) > 10 || abs((int16_t)y - (int16_t)ultimo_y) > 10) {
            // Converte as leituras do joystick em valores PWM
            uint16_t pwm_azul = converter_joystick_para_pwm(y); // Valor PWM para o LED azul
            uint16_t pwm_vermelho = converter_joystick_para_pwm(x); // Valor PWM para o LED vermelho

            // Calcula o deslocamento no display com base nas leituras do joystick
            int16_t desloc_x = (-((int16_t)y - CENTRO_JOYSTICK) * 40) / 2048; // Deslocamento no eixo X
            int16_t desloc_y = (((int16_t)x - CENTRO_JOYSTICK) * 20) / 2048; // Deslocamento no eixo Y

            // Atualiza a posição no display, limitando aos limites da tela
            pos_x = (pos_x + desloc_x < 0) ? 0 : (pos_x + desloc_x > 56) ? 56 : pos_x + desloc_x;
            pos_y = (pos_y + desloc_y < 0) ? 0 : (pos_y + desloc_y > 120) ? 120 : pos_y + desloc_y;

            // Atualiza os LEDs com os valores PWM (se o PWM estiver ativado)
            if (estado_pwm) {
                pwm_set_gpio_level(LED_AZUL, pwm_azul); // Atualiza o LED azul
                pwm_set_gpio_level(LED_VERMELHO, pwm_vermelho); // Atualiza o LED vermelho
            }

            // Atualiza as últimas leituras do joystick
            ultimo_x = x;
            ultimo_y = y;
        }

        // Atualização do display OLED
        ssd1306_fill(&display, false); // Limpa o display
        ssd1306_rect(&display, pos_x, pos_y, 8, 8, true, true); // Desenha um quadrado na posição atual
        if (estado_led_verde) desenhar_borda(&display, tipo_borda); // Desenha a borda (se o LED verde estiver ligado)
        ssd1306_rect(&display, pos_x, pos_y, 8, 8, true, true); // Desenha um quadrado na posição atual
        ssd1306_send_data(&display); // Envia os dados atualizados para o display
        sleep_ms(10); // Pequena pausa para evitar atualizações muito rápidas
        
    }
    return 0;
}