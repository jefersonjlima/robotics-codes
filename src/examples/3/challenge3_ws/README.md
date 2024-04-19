
## Desafio - Corrida Maluca - Disciplina Robótica.

A descrição da atividade está disponível no link do [moodle](https://moodle.utfpr.edu.br/mod/assign/view.php?id=1551202).

Este rosject contém um exemplo da corredora Penelope na pasta `/desafio_ws/src/corrida_maluca/src/penelope_node.cpp`.

Para rodar toda a dinâmica da atividade de avaliação, deve ser executados os seguintes passos.

* Entre na pasta `desafio_ws` (para os comandos, copie apenas o que estiver destacado em cinza):
    * `cd desafio_ws/`
* Compile os projetos, e crie o link dos projetos. Este comando deve ser rodado toda vez que for alterado o código da corredor.
    * `catkin_make&& source devel/setup.bash`
* Agora neste primeiro terminal vamos rodar o simulador do turtlebot3. Primeiro selecione o modelo de robo.
    * `export TURTLEBOT3_MODEL=burger`
* Depois vamos rodar a launch. Uma copia da launch da simulação do mapa home foi colocado na pasta corrida_maluca/launch/turtlebot3_house.launch.
    * `roslaunch corrida_maluca turtlebot3_house.launch`

> Importante destacar, que será o arquivo da pasta `corrida_maluca/launch/turtlebot3_house.launch` que deverá ser mudado para alterar a região de nacimento do robo.
    
* Agora, mudamos para um segundo terminal. Neste vamos rodar o codigo do nó da penelope.
    * `cd desafio_ws/`
* Depois, vamos compilar o projeto.
    * `catkin_make&& source devel/setup.bash`
* Agora vamos rodar o codigo da penelope com o seguinte comando.
    * `rosrun corrida_maluca penelope_node`
 
> Então aparecerá o log da corredora, mostrando que está sendo aguardado o sinal de largada.

```bash
user:~/desafio_ws$ rosrun corrida_maluca penelope_node
[ INFO] : [Penelope] x:-3.00, y: 1.00, theta: -0.00
[ INFO] : [Penelope] x:-3.00, y: 1.00, theta: -0.00
[ INFO] : [Penelope] x:-3.00, y: 1.00, theta: -0.00
```

* Um terceiro terminal deverá ser aberto para que seja dado o comando de largada.
    * `rostopic pub /corrida/largada std_msgs/Bool "data: false"`
    
> Somente quando data for igual a `true` o robô deverá se mover ao primeiro ponto.

```bash
[ INFO] : [Penelope] x:-3.00, y: 1.00, theta: -0.00
[ INFO] : [Penelope] x:-3.00, y: 1.00, theta: -0.00
[ INFO] : [Penelope] x:-3.00, y: 1.00, theta: -0.00
[ INFO] : [Penelope] VAI VAI VAI!!!!!!!!!
[ INFO] : [Penelope] Erro L: 3.16, control_linear: 0.22
[ INFO] : [Penelope] x:-2.82, y: 1.00, theta: -0.05
[ INFO] : [Penelope] Erro L: 2.99, control_linear: 0.22
[ INFO] : [Penelope] x:-2.60, y: 0.98, theta: -0.14
[ INFO] : [Penelope] Erro L: 2.78, control_linear: 0.22
...
[ INFO] : [Penelope] x:-0.12, y: 0.05, theta: -0.47
[ INFO] : Erro L: 0.13, control_linear: 0.04
[ INFO] : [Penelope] Cheguei proximo ao p1(-0.12,0.05)!!!
[ INFO] : [Penelope] Aguardando novas coordenadas!!!

```


Este ponto está definido no código em:


```c
  ...
  double p_x = 0;
  double p_y = 0;
  ...
  erro_linear = sqrt(pow(p_x - penelope_Pose.x, 2) + pow(p_y - penelope_Pose.y, 2));
  ...
  erro_angular = atan2(p_y - penelope_Pose.y, p_x - penelope_Pose.x);
  ...

```

O desafio agora é criar um vetor de pontos e toda vez que o valor for melhor de um limiar, o robo deverá ir para o próximo ponto...

# Boa sorte a todos!!!

![](https://azure.wgp-cdn.co.uk/app-table-top-gaming/posts/wacky-races-61047.png)



