# Magnetômetro

O Magnetômetro será o responsável por obter as coordernadas X, Y e Z que indicarão a direção pela qual o 
barco deverá se mover para percorrer o caminho definido pelos Beacons espalhados pelo lado durante o torneio.

O código trabalha com os endereços dos registradores do magnetômetro, que armazenarão as coordenadas dos eixos
X, Y e Z. Utilizando o arco tangente no pontos X e Y, obtém-se o ângulo que será enviado ao servo motor e que
por fim, será o responsável por controlar e mover o leme do barco.

Para o desenvolvimento do código, as informações foram consultadas no Datasheet do HMC5883 disponível no link. https://www.makerhero.com/img/files/download/QMC5883L-Datasheet.pdf
