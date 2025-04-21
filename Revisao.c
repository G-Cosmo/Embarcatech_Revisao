#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "inc/ledMatrix.h"
#include "inc/font.h"
#include "inc/ssd1306.h"

//configurações do I2C
#define I2C_PORT i2c1   //definição do canal
#define I2C_SDA 14      //pino de dados
#define I2C_SCL 15      //pino de clock
#define address 0x3C    //endereço do canal


#define buttonA 5           //pino do botão A
#define buttonB 6           //pino do botão B

bool color = true;  //variavel que indica que se o pixel está ligado ou desligado
ssd1306_t ssd; //inicializa a estrutura do display

uint last_time = 0;

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
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    //obtém o tempo atual em microssegundos
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    //verifica se passou tempo suficiente desde o último evento
    if (current_time - last_time > 200000) // 200 ms de debouncing
    {

        last_time = current_time; 

        ssd1306_fill(&ssd, !color); //limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 58, color, !color); //desenha um retângulo

        if(gpio == buttonA)
        {
            printf("\n\n\rInterrupção em A.");  
            gpio_put(rgb_led[1], !gpio_get(rgb_led[1]));    //alterna o estado do led verde

            ssd1306_draw_string(&ssd, "LED Verde", 30, 20); //desenha uma string    
            if(gpio_get(rgb_led[1]))    //verifica se o led está ligado
            {
                ssd1306_draw_string(&ssd, "Ligado", 42, 30);
                printf( "\n\rLED verde ligado.\n");
                printf("\n\rInsira o caracter que deseja imprimir: ");
            }else   //caso esteja desligado
            {
                ssd1306_draw_string(&ssd, "Desligado", 30, 30);
                printf( "\n\rLED verde desligado.\n");
                printf("\n\rInsira o caracter que deseja imprimir: ");
            }

        }else if (gpio == buttonB)    
        {
            printf("\n\n\rInterrupção em B.");
            gpio_put(rgb_led[2], !gpio_get(rgb_led[2]));    //alterna o estado do led azul
            ssd1306_draw_string(&ssd, "LED Azul", 30, 20); //desenha uma string    
            if(gpio_get(rgb_led[2]))    //verifica se o led está ligado
            {
                ssd1306_draw_string(&ssd, "Ligado", 42, 30);
                printf( "\n\rLED azul ligado.\n");
                printf("\n\rInsira o caracter que deseja imprimir: ");
            }else   //caso esteja desligado
            {
                ssd1306_draw_string(&ssd, "Desligado", 30, 30);
                printf( "\n\rLED azul desligado.\n");
                printf("\n\rInsira o caracter que deseja imprimir: ");

            }   
        }  

        ssd1306_send_data(&ssd); //atualiza o display
    }
}



int main()
{
    stdio_init_all();   //inicializa a biblioteca stdio
    init_rgb(rgb_led);  //inicializa o led rgb
    init_buttons();     //inicializa os botões

    npInit(LED_PIN);        //inicializa matriz de led
    npClear();              //limpa a matriz

    //Inicializa o I2C com frequencia de 400Khz
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

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
