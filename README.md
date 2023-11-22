# Trabalho Final - Programação de Sistemas Embarcados

## Introdução

Neste trabaho, a arquitetura do sistema freeRTOS para o barco foi elaborada e implementada no STM32CubeIDE 1.6.1. Todos os pinos estão de acordo com o guia fornecido pelo professor e as bibliotecas dos trabalhos 1 ao 5 foram utilizadas no sistema.

## Explicação

A estrutura de alto nível do sistema freeRTOS está disponível no documento **FreeRTOS Barco.pdf**, em anexo neste branch.

A implementação freeRTOS foi desenvolvida pensando em 3 aspectos: *controladores*, *atuadores* e *sensores*. No caso do barco, os seguintes elementos forma identificados:

- Sensores: Magnetômetro, módulo bluetooth
- Atuadores: Servomotor, motor DC
- Controlador: Estratégia de controle do grupo, que será implementada em uma task separada.

Com base nisso, os seguintes elementos do freeRTOS foram criados:

## Tasks

- **controlador**: Recebe as informações das tasks **magnetometro** e **bluetooth**; processa as informações recebidas de acordo com a estratégia de controle adotada; envia as novas configurações para as tarefas **motorDC** e **servoMotor**.
- **bluetooth**: Inicializa o módulo BLE, obtém e processa as informações dos beacons; envia essas informações para a task **controlador** quando solicitado.
- **magnetometro**: Inicializa o magnetômetro; quando solicitado, obtém, processa e envia as informações lidas pelo magnetômetro para a task **controladora**.
- **motorDC**: Configuração do motor DC; recebe as informações de velocidade da task **controlador** e ajusta a velocidade do motor de acordo com o que foi recebido.
- **servoMotor**: Configuração do servoMotor; recebe as informações de ângulo da task **controlador** e ajusta a velocidade do motor de acordo com o que foi recebido. 

## Bits de Evento

A tarefa **controlador** possui um bit de evento para ativar cada uma das outras tarefas. Estes bits são:

- **bitMagnetometro**
- **bitBluetooth**
- **bitMotorDC**
- **bitServoMotor**

**Todas as tasks exceto a *controlador* realizam um(e apenas um) ciclo de suas operações quando o seu respectivo bit de evento for igual a 1. Após isso, o bit de eventos será setado para 0 pela task correspondente**. A task controlador realiza suas atividades de acordo com um timer periódico, cujo tempo em milisegundos é definido pela variável global TEMPO_CICLO_CONTROLE. Para fins práticos, a task controlador governa se as outras 4 tasks realizarão seu ciclo de operação ou não em um dado período de tempo.

Apenas a task **controlador** pode mudar os bits de evento de 0 para 1. Apenas as tasks de sensores e atuadores podem mudar seus respectivos bits de evento de 1 para 0.

Os bits **bitMagnetometro** e **bitBluetooth** serão ativados pela task **controlador** de maneira conjunta. A ativação destes bits em períodos de tempo distintos resultaria em um erro no controle, visto que uma das informações dos sensores estaria potencialmente defasada temporalmente em relação a outra. Os bits **bitMotorDC** e **bitServoMotor** também serão ativados juntos, para garantir o sincronismo máximo na atuação do barco.

## Queues

O sistema possui 4 queues:

- **queueMagnetometro**: Queue utilizada pela task **magnetometro** para enviar informações lidas para a task **controlador**.
- **queueBluetooth**: Queue utilizada pela task **bluetooth** para enviar informações lidas para a task **controlador**.
-**queueMotorDC**: Queue utilizada pela task **controlador** para enviar a velocidade do motor desejada para a task **motorDC**.
-**queueServoMotor**: Queue utilizada pela task **controlador** para enviar o ângulo do motor desejado para o servomotor para a task **servoMotor**.
###### This is a Heading h6

## Etapas de funcionamento do sistema

Tendo em vista as estruturas freeRTOS apresentadas até então, o sistema funcionará segundo os seguintes passos:

1. Inicialização de todas as tasks, queues, bits de evento e periféricos do sistema. Todas as queues são inicializadas sem elementos e todos os bits de evento são inicializados como 0.
2. Task **controlador** espera por TEMPLO_CICLO_CONTROLE milisegundos. 
3. Task **controlador** seta os bits de evento **bitMagnetometro** e **bitBluetooth** para 1 sem intervalos de tempo significativo entre cada bit.
4. Tasks **bluetooth** e **magnetometro** são ativadas uma vez que o seu bit de evento está igual a 1 e realizam suas respectivas leituras, enviando as informações coletadas para as queues **queueBluetooth** e **queueMagnetometro**; caso a adição das leituras às queues tenha sido feita com sucesso, seta os bits de evento para 0 sem intervalo de tempo significativo entre cada bit.
5. Task **controlador** espera até que os bits de evento para as tasks **magnetometro** e **bluetooth** estão iguais a 0 e existam elementos nas 2 filas; realiza o cálculo da velocidade do motor DC e do ângulo do servomotor de acordo com a estratégia de controle do grupo
6. Task **controlador** adiciona o valor da velocidade e o valor do ângulo nas queues **queueServoMotor** e **queueMotorDC**.
7. Task **controlador** seta os bits de evento do motor DC e do servomotor para 1.
8. As tasks **queueMotorDC** e **queueServoMotor** são ativadas uma vez que o seu bit de evento está igual a 1; recebem as informações em suas respectivas queues e realizam seus ajustes correspondentes. Caso os ajustes à velocidade e ao angulo tenham sido executados com sucesso, setam os seus bits de evento para 0
9. Durante o passo 8, a task **controlador** está em espera até que todos os bits de evento estão iguais a 0 e que nenhum elemento esteja presente nas queues do sistema. Quando este for o caso, retorne ao passo 2 e inicie mais um ciclo de operação.

O grupo buscou o sincronismo máximo possível entre a leitura dos 2 sensores e a mudança na velocidade e ângulo dos 2 atuadores


## Observações Principais

- Os sensores não realizarão uma única leitura antes de enviar suas informações à task **controlador**. Serão realizadas várias leituras e apenas o resultado filtrado será adicionado à queue
- Existe uma possiblidade de que o tempo de leitura de cada sensor ou o tempo de acionamento de cada motor possua uma grande diferença de tempo entre si, defasando o efeito do controle. O grupo optou por verificar o comportamento do sistema empiricamente nos momentos de teste e realizar ajustes caso isso se torne um aspecto relevante para a operação do barco.



https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads -> Baixar para o projeto CMOS e mudar no arquivo toolchains.cmake