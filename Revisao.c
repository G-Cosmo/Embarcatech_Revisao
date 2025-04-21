#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "inc/ledMatrix.h"
#include "inc/font.h"
#include "inc/ssd1306.h"

//configurações do I2C
#define I2C_PORT i2c1   //definição do canal
#define I2C_SDA 14      //pino de dados
#define I2C_SCL 15      //pino de clock
#define address 0x3C    //endereço do canal

//configurações do ADC
#define VRX 27          //pino do eixo X
#define VRY 26          //pino do eixo Y
#define buttonJ 22      //pino do botão do joystick
#define sqrCenterX (WIDTH/2)-4      //centro do eixo X
#define sqrCenterY (HEIGHT/2)-4     //centro do eixo Y
#define DEADZONE 100                //tamanho da zona morta

//configuração dos botões
#define buttonA 5           //pino do botão A
#define buttonB 6           //pino do botão B

#define BUZZER_PIN 10   //pino do buzzer


bool color = true;  //variavel que indica que se o pixel está ligado ou desligado
ssd1306_t ssd; //inicializa a estrutura do display

bool buzzer_flag = false;    //flag que indica se o buzzer está ativado (false por padrão)
uint buzzer_freq = 1000;    //frequencia do buzzer
uint wrap_buzzer = 10000;   //wrap do buzzer
uint x_center = 2047;   //centro padrão do joystick
uint y_center = 2047;   //centro padrão do joystick
float dcX = 0;   //percentual do ciclo de trabalho de x (led vermelho)
float dcY = 0;   //percentual do ciclo de trabalho de y (led azul)
float sqrX = 0;  //percentual de movimentação no eixo X do quadrado
float sqrY = 0;  //percentual de movimentação no eixo y do quadrado
float moveFactor = 1.5; //fator divisivo de movimentação do quadrado, quanto maior for o valor, menor será a movimentação do quadrado
uint wrapX = 0;  //wrap do pwm que controla o led vermelho
uint wrapY = 0;  //wrap do pwm que controla o led azul
uint64_t volatile last_time = 0;    //variavel que indica o tempo da ultima demição
uint64_t volatile current_time = 0; //variavel que indica o tempo da atual medição
bool led_flag = true;   //flag que habilita o controle dos leds via pwm (true por padrão)

const uint rgb_led[3] = {13,11,12}; //pinos do led rgb
uint r_intensity = 100;  //intensidade do vermelho
uint g_intensity = 0;     //intensidade do verde
uint b_intensity = 100;  //intensidade do azul


void init_rgb(const uint *rgb)  //função responsável por inicializar o led rgb
{
    for(int i =0; i<3; i++)
    {
        gpio_init(rgb[i]);
        gpio_set_dir(rgb[i], GPIO_OUT);
        gpio_put(rgb[i], false); //garante que os leds comecem apagados
    }
}

void init_buttons() //função responsável por inicializar os botões
{
    gpio_init(buttonA);
    gpio_set_dir(buttonA, GPIO_IN);
    gpio_pull_up(buttonA);

    gpio_init(buttonB);
    gpio_set_dir(buttonB, GPIO_IN);
    gpio_pull_up(buttonB);

    gpio_init(buttonJ);
    gpio_set_dir(buttonJ, GPIO_IN);
    gpio_pull_up(buttonJ);
}

uint init_pwm(uint gpio, uint wrap) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);
    
    pwm_set_enabled(slice_num, true);  
    return slice_num;  
}

void init_display()
{
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  //seta o pino gpio como i2c
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);  //seta o pino gpio como i2c
    gpio_pull_up(I2C_SDA);  //ativa o resistor de pull up para gantir o nível lógico alto
    gpio_pull_up(I2C_SCL);  //ativa o resistor de pull up para gantir o nível lógico alto


    ssd1306_init(&ssd, WIDTH, HEIGHT, false, address, I2C_PORT); //inicializa o display
    ssd1306_config(&ssd); //configura o display
    ssd1306_send_data(&ssd); //envia os dados para o display
  
    //limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

void play_buzzer(uint freq, float duty_cycle) {
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint clock_divider = 4; // Define o divisor do clock (ajuste se necessário)
    uint wrap = clock_get_hz(clk_sys) / (clock_divider * freq);

    pwm_set_clkdiv(slice, clock_divider);
    pwm_set_wrap(slice, wrap);
    pwm_set_gpio_level(BUZZER_PIN, wrap * duty_cycle);
}

void clear_screen() {
    printf("\033[2J\033[H");
}

void animacao() {
    int cores[5][3] = {
        {100, 0, 0},     // Vermelho escuro
        {100, 50, 0},    // Laranja escuro
        {100, 100, 0},   // Amarelo escuro
        {0, 100, 0},     // Verde escuro
        {0, 0, 100}      // Azul escuro
    };

    // --- Fase 1: Arco-íris linha por linha
    for (int linha = 0; linha < 5; linha++) {
        for (int coluna = 0; coluna < 5; coluna++) {
            int pos = getIndex(linha, coluna);
            npSetLED(pos, cores[linha][0], cores[linha][1], cores[linha][2]);
        }
        npWrite();
        sleep_ms(300);
    }

    // --- Fase 1: Desfazendo de cima para baixo
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int pos = getIndex(linha, coluna);
            npSetLED(pos, 0, 0, 0);
        }
        npWrite();
        sleep_ms(200);
    }

    sleep_ms(300);

    // --- Fase 2: Quadrado crescendo com cores variadas
    int camadas[3][3] = {
        {100, 0, 100},    // Roxo para camada externa
        {0, 100, 100},    // Ciano para intermediária
        {255, 255, 255}   // Branco no centro
    };

    for (int raio = 0; raio <= 2; raio++) {
        for (int linha = 0; linha < 5; linha++) {
            for (int coluna = 0; coluna < 5; coluna++) {
                if (abs(linha - 2) <= raio && abs(coluna - 2) <= raio) {
                    int pos = getIndex(linha, coluna);
                    npSetLED(pos, camadas[2 - raio][0], camadas[2 - raio][1], camadas[2 - raio][2]);
                }
            }
        }
        npWrite();
        sleep_ms(300);
    }

    // --- Fase 2: Sumir gradualmente das bordas para o centro
    for (int raio = 2; raio >= 0; raio--) {
        for (int fade = 100; fade >= 0; fade -= 25) {  // Gradualmente apaga cada camada
            for (int linha = 0; linha < 5; linha++) {
                for (int coluna = 0; coluna < 5; coluna++) {
                    if (abs(linha - 2) <= raio && abs(coluna - 2) <= raio) {
                        int pos = getIndex(linha, coluna);
                        int r = (camadas[2 - raio][0] * fade) / 100;
                        int g = (camadas[2 - raio][1] * fade) / 100;
                        int b = (camadas[2 - raio][2] * fade) / 100;
                        npSetLED(pos, r, g, b);
                    }
                }
            }
            npWrite();
            sleep_ms(100);
        }
    }

    npClear();
    npWrite();
}


void gpio_irq_handler(uint gpio, uint32_t events)
{
    //obtém o tempo atual em microssegundos
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    //verifica se passou tempo suficiente desde o último evento
    if (current_time - last_time > 200000) // 200 ms de debouncing
    {

        last_time = current_time; 

        if(gpio == buttonA)
        {
            buzzer_flag = true;
            led_flag = false;
        }
        
        if (gpio == buttonB)    
        {
            buzzer_flag = false;
            led_flag = true;
        }  

    }
}
int main()
{
    stdio_init_all();   //inicializa a biblioteca stdio
    init_rgb(rgb_led);  //inicializa o led rgb
    init_buttons();     //inicializa os botões
    init_display();     //inicializa o display


    npInit(LED_PIN);        //inicializa matriz de led
    npClear();              //limpa a matriz

    adc_init();//inicializa o adc
    adc_gpio_init(VRX);//inicializa o pino X do adc
    adc_gpio_init(VRY);//inicializa o pino Y do adc

    //bloco com leitura inicial do adc para definir onde é o centro do joystick
    sleep_ms(100); 
    adc_select_input(1);
    uint16_t vrx_value = adc_read();
    x_center = vrx_value;
    wrapX = vrx_value;
    printf("\nFirst Values vrx, x_center, wrap: %d, %d, %d", vrx_value, x_center, wrapX);

    adc_select_input(0);
    uint16_t vry_value = adc_read();
    y_center = vry_value;
    wrapY = vry_value;
    printf("\nFirst Values vry, y_center, wrap: %d, %d, %d", vry_value, y_center, wrapY);
    sleep_ms(100);
    //fim da leitura inicial

    uint pwm_buzzer = init_pwm(BUZZER_PIN, wrap_buzzer);// inicializa o pwm do buzzer

    //configura o pwm com base na leitura do centro
    uint pwm_red = init_pwm(rgb_led[0],wrapX);
    uint pwm_blue = init_pwm(rgb_led[2],wrapY);

    //ativa a interrupção dos botões
    gpio_set_irq_enabled_with_callback(buttonA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);   
    gpio_set_irq_enabled_with_callback(buttonB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);   

    while (true) {
        adc_select_input(1);//seleciona a entrada do adc para x
        uint16_t vrx_value = adc_read();//realiza a leitura do valor de x
        printf("\n X value: %d", vrx_value);

        adc_select_input(0);//seleciona a entrada do adc para y
        uint16_t vry_value = adc_read();//realiza a leitura do valor de y
        printf("\n Y value: %d", vry_value);

        if(abs((int)vrx_value - (int)x_center) < DEADZONE)  //verifica se o eixo x está dentro da zona morta
        {
            dcX = 0;    //mantém o led desligado caso esteja
            sqrX = sqrCenterX;  //mantém o quadrado no centro caso esteja
        }else if(vrx_value < x_center)//verifica se o valor lido no adc foi menor do que o centro do joystick
        {
            vrx_value = x_center + (x_center - vrx_value);//ajusta o valor lido do adc para trabalhar com valores maiores do que o centro
            dcX = ((float)(vrx_value - x_center) / (float)wrapX);//calcula o percentual do ciclo de trabalho do eixo x
            printf("\ndcX: %f", dcX);

            sqrX = (1-(dcX/moveFactor))*sqrCenterX; //calcula o percentual de movimentação do quadrado no eixo x
        }else
        {
            dcX = ((float)(vrx_value - x_center) / (float)wrapX);//calcula o percentual do ciclo de trabalho do eixo x
            printf("\ndcX: %f", dcX);

            sqrX = (1+(dcX/moveFactor))*sqrCenterX; //calcula o percentual de movimentação do quadrado no eixo x
        }

        if(abs((int)vry_value - (int)y_center) < DEADZONE)  //verifica se o eixo y está dentro da zona morta
        {
            dcY = 0;    //mantém o led desligado caso esteja
            sqrY = sqrCenterY;  //mantém o quadrado no centro caso esteja
        }else if(vry_value < y_center)//verifica se o valor lido no adc foi menor do que o centro do joystick
        {
            vry_value = y_center + (y_center - vry_value);//ajusta o valor lido do adc para trabalhar com valores maiores do que o centro
            dcY = ((float)(vry_value - y_center) / (float)wrapY);//calcula o percentual do ciclo de trabalho do eixo y
            printf("\ndcy: %f", dcY);

            sqrY = (1+(dcY/moveFactor))*sqrCenterY; //calcula o percentual de movimentação do quadrado no eixo y
        }else
        {
            dcY = ((float)(vry_value - y_center) / (float)wrapY);    //calcula o percentual do ciclo de trabalho do eixo y
            printf("\ndcy: %f", dcY);

            sqrY = (1-(dcY/moveFactor))*sqrCenterY; //calcula o percentual de movimentação do quadrado no eixo y
        }
        
        if(buzzer_flag)
        {
            if(dcY+dcX >= 1.0)
            {
                play_buzzer(buzzer_freq, 0.8); //toca o buzzer com o ciclo de trabalho máximo
                clear_screen();
                printf("\nBuzzer duty cicle: %f", dcY+dcX);
                printf("\nX duty cicle: %f", dcX);
                printf("\nY duty cicle: %f", dcY);


            }else{
                play_buzzer(buzzer_freq, dcY+dcX ); //toca o buzzer variando o ciclo de trabalho
                clear_screen();
                printf("\nBuzzer duty cicle: %f", dcY+dcX);
                printf("\nX duty cicle: %f", dcX);
                printf("\nY duty cicle: %f", dcY);
            }
        }

        ssd1306_fill(&ssd, !color); //limpa o display
        ssd1306_rect(&ssd, sqrY, sqrX, 8, 8, color, color); //desenha o quadrado 8x8 
        ssd1306_rect(&ssd, 3, 3, 122, 58, color, !color); //desenha um retângulo nas bordas do display
        ssd1306_send_data(&ssd);    //atualiza o display

        if(led_flag) 
        {
            pwm_set_gpio_level(rgb_led[0], (uint)(dcX*wrapX));//atualiza o ciclo de trabalho do eixo x
            pwm_set_gpio_level(rgb_led[2], (uint)(dcY*wrapY));//atualiza o ciclo de trabalho do eixo y
        }

        current_time = to_ms_since_boot(get_absolute_time());   //pega o tempo atual

        if(!gpio_get(buttonJ) && (current_time - last_time > 200))//implementa o debouncing
        {
            animacao();
        }

    }
}
