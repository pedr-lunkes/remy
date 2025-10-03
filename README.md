# Projeto HOME - Remy

Para rodar o docker, e testar os códigos no ROS2 primeiro instale as dependências:
```
cd remy_ws
docker compose build
```
Em seguida, para rodar os códigos, basta usar o comando
```
docker compose up
```
E pronto! Você está dentro de um terminal ubuntu 22.04 com ros2-humble instalado, e com sincronização da pasta src

Primeiro utilize o comando
```
colcon build
```
e rode o pacote ou nó que desejar :D