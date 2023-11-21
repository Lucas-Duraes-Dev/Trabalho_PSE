# Estratégias de deslocamento e controle

A estratégia de deslocamento adotada nesse trabalho se baseia na definição de pontos de apoio ao longo do Lago que irão servir para definir uma rota do Ponto de Partida até o Ponto de Chegada. Desenhando o caminho desejado para que o barco siga durante o percurso espera-se que haja um menor desvio e que as chances dele ficar preso junto às margens do lago diminuam.

![photo_2023-11-21_00-06-40](https://github.com/Lucas-Duraes-Dev/Trabalho_PSE/assets/16578068/fb61a101-292e-4ef0-b611-077705785330)

Assim, foi criada uma Máquina de Estados com quatro estados, sendo eles: 
- Estado 0 --> no qual o barco está parado e a velocidade e o ângulo do servo motor são zero
- Estado 1 --> no qual o barco irá ajustar o ângulo entre sua posição inicial e o ponto de destino de apoio, a velocidade do motor será reduzida para que aumente a prescisão com que ele se alinha ao ponto desejado. 
- Estado 2 --> no qual a velocidade do motor aumenta e o barco tem uma trajetória retílinea até o ponto de apoio
- Estado 3 --> no qual o barco está no último ponto de apoio e o próximo destino é a chegada. Nesse momento, o ângulo deve ser ajustado com mais cautela e a velocidade do motor deve reduzir progressivamente de modo que o barco consiga chegar até o portal.

Quando o ângulo entre o ponto atual do barco e o ponto desejado for próximo a zero, irá alterar do Estado 1 para o Estado 2. O magnetômetro será utilizado para conferir se o barco já passou pelo ponto desejado, sendo que quando isso acontecer o barco irá alterar do Estado 2 para o Estado 1. E quando os pontos de apoio tiverem acabado, e o contador utilizado for igual ao número de ponto, irá alterar do Estado 2 para o Estado 3.  

Como o vento gera uma componente de força no barco, consideramos um offset que corrige o ângulo antes de ser passado ao servo motor. O sentido desse offset leva em conta a direção dos ventos dominantes da região da Pampulha, consultadas a partir dos dados do Aeroporto da Pampulha, que mostram que a direção dos ventos é de Leste para Nordeste.

![photo_2023-11-20_22-26-50](https://github.com/Lucas-Duraes-Dev/Trabalho_PSE/assets/16578068/23160dff-91ed-4a06-acfe-d195fb684229)
