# Módulo 6 - Ponderada 1 - Desenhando com turtlesim
## Breve descrição:
Neste projeto temo um exemplo de um nó ROS (Robot Operating System) que controla uma tartaruga virtual usando o pacote `turtlesim` do ROS.<br/>
Considere que o código presente em `src/ola_mundo/ola_mundo` chamado `teste.py` tem o código responsável por controlar a tartaruga, esse script é capaz de: 
1. Inicializa um nó ROS chamado "desenhador".
2. Define duas funções de callback:
   - `pose_callback`: Esta função é chamada quando a posição da tartaruga é atualizada, mas no código fornecido está vazia, ou seja, não faz nada.
   - `timer_callback`: Esta função é chamada periodicamente (a cada 0.1 segundo) e controla o movimento da tartaruga. Primeiro, publica um comando para mover a tartaruga para frente, em seguida, para de movê-la após 0.5 segundos. Em seguida, publica um comando para girar a tartaruga em torno de seu eixo z para desenhar um círculo, também por 0.5 segundos.
3. Define duas funções para interagir com o serviço de spawn e kill de tartarugas:
   - `spawn_turtle`: Esta função cria uma nova tartaruga em uma posição específica e com uma orientação específica.
   - `kill_turtle`: Esta função remove uma tartaruga pelo nome.
4. Na função `main`:
   - Inicializa o ROS.
   - Cria uma instância do nó `Desenhador`.
   - Chama `spawn_turtle` para criar uma nova tartaruga em (5.0, 5.0) com orientação 0.0.
   - Aguarda 1 segundo (tempo suficiente para a tartaruga se mover).
   - Chama `kill_turtle` para remover a tartaruga recém-criada.
   - Inicia o loop de eventos ROS com `rclpy.spin`.
   - Quando o loop termina, limpa recursos e encerra o ROS.

## Como instalar e rodar o sistema?
### 1) Clone o repositório: 
Primeiramente clone ou baixe o zip do seguinte repositório https://github.com/lidiamariano/Mod6Pond1
### 2) Instale as dependências: 
Na pasta raiz do repositório existe o arquivo requirements.txt, que será necessário para instalar os pacotes necessários para rodar o projeto. Faça isso pelo seguinte comando:<br/>
`pip install -r requirements.txt`
### 3) Rodando o turtlesim
Abra um terminal dentro da pasta do seu repositório e execute: `ros2 run turtlesim turtlesim_node`
### 5) Construindo ambiente ros:
Ainda na pasta raiz do diretório execute:
`colcon build` e `source install/local_setup.bash` 
Em seguida execute:
`ros2 pkg executables ola_mundo`
Então, podemos finalmente rodar o arquivo com ROS:
`ros2 run ola_mundo ola`
### 5)Observe
Agora o scripts enviará comandos para a tartaruga que está rodando no turtlesim que foi aberto no outro terminal. A primeira tartaruga a aparecer irá construir um desenho e uma outra irá aparecer por um breve momento e depois é morta. Veja no vídeo como sua tartaruga deve estar se movendo: https://youtu.be/sqywAvYiSk8?si=0h0D9P26_bkptgxp


