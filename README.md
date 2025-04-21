# Embarcatech_Revisao

Repositório criado para a tarefa relacionada à revisão dos periféricos e conceitos aprendidos na primeira fase do programa.


O programa implementa o controle de dois canais (azul e vermelho) de um LED RGB e de um Buzzer através de um PWM, cujos ciclos de trabalho são alterados utilizando os valores obtidos no ADC da leitura do joystick.
Além disso, o botão do Joystick ativa uma animação na matriz de Leds WS2812.
O programa também realiza o controle do display de LEDs SSD1306 de acordo com o movimento do joystick.

# Instruções de compilação

Para compilar o código, são necessárias as seguintes extensões: 

*Raspberry Pi Pico*

*Cmake*

Após instalá-las basta buildar o projeto pelo CMake.

# Instruções de utilização

O projeto se divide em três principais funcionalidades:
1. Interação com joystick:
- O usuário movimenta o joystick, que é lido por meio do ADC. Essa movimentação controla a
posição de um quadrado em tempo real no display OLED, define o duty cycle de dois canais PWM
para ajustar o brilho dos LEDs vermelho e azul, e calcula a soma dos movimentos em X e Y para
determinar a intensidade do som emitido pelo buzzer.

2. Exibição de animação com a matriz de LEDs:
- Uma animação multicolorida é executada na matriz de LEDs WS2812 (5x5). Um arco-íris se
forma gradualmente da esquerda para a direita, em seguida, desaparece gradualmente de cima para
baixo. Após isso, um quadrado se forma do centro às bordas com cores variadas e desaparece das
bordas para o centro com efeito fade out.

3. Controle por botões com interrupções:
- Os dois botões (A e B) são utilizados para ativar/desativar o buzzer e o controle de LEDs. O
botão A faz com que o joystick controle o buzzer e o display, enquanto o botão B faz com que o
joystick controle o LED RGB e o display. As ações de ambos os botões são feitas utilizando
interrupções. Além disso, um tratamento para o efeito de debounce é implementado, fazendo com
que cada botão só seja ativado novamente após um intervalo de 200 milissegundos.
O sistema ainda utiliza a comunicação UART via USB para exibir mensagens de depuração no
terminal, permitindo o acompanhamento em tempo real de valores do joystick, duty cycle dos LEDs
e intensidade do buzzer.


# Vídeo demonstrativo

https://youtu.be/CY-HKA8JBW0
