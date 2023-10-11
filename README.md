# Servo Motor

O servo motor será o responsável por controlar o leme e portanto, por garantir que o barco se
mova na direção correta.

O servo será acionado a partir da variação do sinal PWM. De acordo com o datasheet do MG90S, 
o sinal de PWM corresponde a 20ms (50Hz).

O servo consegue fazer a rotação aproximada de 180º, sendo 90º para a direita e 90º para a esquerda.

Para facilitar a integração de cada componente do circuito eletrônico do barco, criamos uma função
que recebe um ângulo como um dos parâmetros.

A API do servo motor se encontra nos arquivos SERVOMOTOR.c e SERVOMOTOR.h .
