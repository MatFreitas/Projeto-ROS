# Projeto-ROS


**Edgard Ortiz** email: edgardaon@al.insper.edu.br


**Matheus Freitas** email: matheusfs2@al.insper.edu.br


**Henrique Mualem**

___
**Link vídeo demonstração do robô pegando creepers manualmente:** https://www.youtube.com/watch?v=QkfvS4Prgsw&feature=youtu.be

**Link vídeo demonstração rubrica A goal 1:** https://web.microsoftstream.com/video/69565aec-e9b3-4ac8-9ed1-45eba71547d2

**Link vídeo demonstração rubrica A goal 2:** https://web.microsoftstream.com/video/79b7c58c-82d2-4f69-ad2b-8bc9f4e356a4

## Descrição
O Projeto ROS é um repositório com tarefas da disciplina de Robótica Computacional, orientada pelos professores Fábio Miranda e Antônio Selvatici, 3º Semestre de Engenharia de Computação do Insper 2020. \
Este repositório é dependendente de vários outros repositórios, principalmente desses:\
\
robot20: https://github.com/Insper/robot20 --> Enunciado das atividades e Projeto 1 \
my_simulation: https://github.com/arnaldojr/my_simulation --> Mapas para simulação no Gazebo. 

## Projeto 1
O Projeto 1 consiste em uma simulação de um turtlebot (Waffle) que captura obejtos, no caso "Creepers", automaticamente.
### Como executá-lo:
Para rodar o Projeto 1, é preciso estar com o repositório "robot20" atualizado e, para rodar com versão automatizada do \
robô, é necessário estar com o "my_simulation" nesse commit: 30986a4ea7ecaa0d7f90f2548c53b97faded4eca (para voltar para este 
commit, digite o comando no bash/terminal:\
$ git reset --hard 30986a4ea7ecaa0d7f90f2548c53b97faded4eca \
Após estar com esses repositórios ajustados é necessário, apenas caso o modelo do seu turtlebot no Gazebo não estiver com a \
garra funcional (a garra pode ter um "gripper" não funcional), rodar esse comando: \
\
$ sh $(rospack find my_simulation)/garra/instala_garra.sh \
\
Se o modelo já possuir a garra funcional, rode os seguintes comandos em diferentes terminais: \
\
$ roslaunch my_simulation proj1_mult.launch \
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch \
$ rosrun meu_projeto base_proj_rubricaA.py
### Detalhando o processo:
O código possui um objetivo (lista) chamado "mission_goal", nela, o primeiro elemento é a cor do creeper que o robô vai ter\
que capturar, o segundo é o "id" do creeper alvo e o terceiro é a base que o turtlebot vai devolver/soltar esse creeper. \
Por exemplo, o primeiro "goal" é: ["blue", 11, "cat"]
### Bugs, glitches ou outros problemas
Contatar algum dos emails acima. 
___
