Comandos importantes:

#############################################################
Se quisermos trabalhar na rubrica B, é necessário utilizar uma outra versão do repositório "my_simulation". Para isso precisamos executar os seguintes comandos:
$ cd ~/catkin_ws/src/my_simulation
$ git reset --hard b2ba48085658867be96fbf209141400671b31728
#############################################################

Ctrl + Alt + t

roslaunch my_simulation proj1_base.launch

Ctrl + Shift + t

roslaunch turtlebot3_manipulation_moveit_config move_group.launch

Ctrl + Shift + t

roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch

Ctrl + Shift + t

rqt_image_view

Ctrl + Shift + t

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Ctrl + Alt + t

rostopic echo /ar_pose_marker

Ctrl + Shift + t

rosrun meu_projeto base_proj.py
              ou
rosrun meu_projeto base_proj_test.py
