# FieldWalker
a genetic algorithm for a robot to choose the best way for walking a set of waypoints using carrot chaser 

Este repositório se destina ao trabalho da disciplina ssc0713 do icmc-usp (sistemas evolutivos aplicados à robótica)
não conseguimos completar o trabalho a tempo. Mesmo assim caso queira para utilizá-lo você deve:

->instalar o simulador gazebo (utlilizamos o gazebo 7, ele pode ser baixado em http://gazebosim.org/ )

->instalar o compilador do google protobuf (muito importante que seja a versão 2.6.1, salvo alguma alteração no sistema) que pode ser encontrado em "https://github.com/google/protobuf/releases?after=v3.0.0-alpha-3"

-> acessar a pasta AG/msgs/build no terminal

->executar os comandos "cmake ../"

-> "make"

->acessar a pasta AG/build no terminal

->executar o comando "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:path-para-a-sua-pasta"

->"cmake ../"

->"make"

->então podemos rodar o programa com "./Gazebo_evolve"

por outro lado o teste que não usa o simulador e está presente em permutação pode ser compilado por qualquer compilador de c++
e fornece como resultado dois arquivos txt um com a melhor rota da geração inicial "Dados1.txt" e um com a melhor rota da 
geração final "Dados.txt".
