# Controle de LEDs RGB, Display OLED e Joystick com PWM

Este projeto foi desenvolvido como parte do curso **EmbarcaTech**, promovido pelo **CEPEDI**, **SOFTEX** e **HwIT - Hardware Innovation Technologies**, para consolidar conhecimentos em controle de LEDs, leitura de potenciômetros, uso de PWM, ADC, I2C e manipulação de displays OLED.

---

## 📝 Enunciado do Projeto

O objetivo do projeto é criar um sistema que integre hardware e software para:

- Controlar LEDs RGB (Vermelho, Verde e Azul) com PWM.
- Usar um display OLED SSD1306 via comunicação I2C para exibir informações.
- Manipular um joystick com leitura analógica para controlar a posição de um quadrado no display.
- Configurar interrupções para botões de controle (Botão Joystick e Botão A).
- Implementar um filtro para suavizar a leitura do joystick e garantir uma resposta mais precisa.

---

## 🎥 Demonstração do Projeto
- Vídeo do projeto: [Video no Youtube](https://youtu.be/pKIGuwDHmYg)

## 🔧 Componentes Utilizados

- **LED Vermelho, Verde e Azul (RGB)**: Controlados via PWM, conectados nas GPIOs 13, 11 e 12, respectivamente.
- **Joystick**: Conectado aos pinos GPIO 26 (Eixo X) e GPIO 27 (Eixo Y).
- **Botão Joystick**: Conectado à GPIO 22.
- **Botão A**: Conectado à GPIO 5.
- **Display OLED SSD1306**: Conectado via I2C (GPIO 14 - SDA, GPIO 15 - SCL).
- **Placa de Desenvolvimento BitDogLab com RP2040**.

---

## 🎯 Funcionalidades do Projeto

1. **Controle de LEDs RGB via PWM**:
   - LEDs RGB são controlados com base nas leituras do eixo X e Y do joystick, com brilho ajustado através de PWM.

2. **Exibição no Display OLED**:
   - O display OLED exibe a posição de um quadrado com base no movimento do joystick, com bordas configuráveis.
   - As leituras do joystick são suavizadas para garantir uma movimentação precisa.

3. **Interrupções e Debouncing**:
   - Botões de controle (Joystick e Botão A) são configurados com interrupções para evitar múltiplos acionamentos.
   - O sistema implementa debouncing para garantir um funcionamento estável.

4. **Função de Calibração e Filtro de Joystick**:
   - O joystick é calibrado e suas leituras são suavizadas para garantir maior precisão na movimentação do quadrado no display.

---

## 📂 Organização do Código

### Arquivos Incluídos

- **main.c**: Código principal com a implementação de todas as funcionalidades.
- **lib/ssd1306.h**: Biblioteca para controle do display OLED SSD1306.

### Estrutura do Código

1. **Inicialização dos Componentes**:
   - Configuração dos pinos GPIO, PWM, ADC e I2C para os LEDs, botões e display OLED.
   
2. **Leitura do Joystick**:
   - Leitura analógica dos eixos X e Y do joystick, aplicando um filtro para suavizar as leituras.

3. **Controle dos LEDs e Display**:
   - Baseado nas leituras do joystick, o brilho dos LEDs RGB é ajustado e a posição do quadrado é exibida no display OLED.

4. **Rotinas de Interrupção**:
   - Os botões Joystick e A são monitorados com interrupções, com a implementação de debouncing.

5. **Função Principal**:
   - Realiza a leitura contínua do joystick e ajusta os LEDs e a exibição no display OLED.

---

## 🚀 Como Executar o Projeto

### Pré-requisitos

- Placa **BitDogLab** configurada e conectada.
- Ambiente de desenvolvimento configurado com SDK do RP2040.
- **VS Code** com a extensão **Pico SDK** instalada.

### Passos

1. Clone este repositório:
   ```bash
   git clone <https://github.com/queirozPatrick/conversor_adc>
   ```
2. Abra o projeto no **VS Code**.
3. Compile e carregue o código na **Placa BitDogLab**.
4. Conecte-se ao **Serial Monitor** para monitorar as saídas, se necessário.

---

## 📋 Requisitos do Projeto

1. **Controle de PWM**:
   - LEDs RGB controlados via PWM com ajuste de brilho.
   
2. **Leitura de Joystick**:
   - Leitura analógica com suavização para movimentar um quadrado no display OLED.

3. **Display OLED**:
   - Exibição de um quadrado que se move com base na posição do joystick.
   - Exibição de informações configuráveis.

4. **Interrupções e Debouncing**:
   - Tratamento de interrupções de botões para garantir resposta eficiente e sem erros.

---

## 👨‍💻 Autor

**Patrick Queiroz**  
- GitHub: [queirozPatrick](https://github.com/queirozPatrick)  
- LinkedIn: [Patrick Queiroz](https://www.linkedin.com/in/patricksq/)
