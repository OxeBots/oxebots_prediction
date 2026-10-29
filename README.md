# oxebots_prediction

Pacote de estimação e predição de estado em tempo real da equipe **OxeBots** (UFBA) para a categoria RoboCup Small Size League – Entry Level (SSL-EL).

Este pacote implementa o nó `kalman_filter_node`, responsável por filtrar o ruído de observação das câmeras SSL-Vision, estimar velocidades lineares e angulares contínuas e prever trajetórias futuras da bola e dos robôs.

---

## 1. Nó Principal: `kalman_filter_node`

### 1.1 Filtro de Kalman da Bola (4D)
- **Vetor de Estado:** $\mathbf{x}_{bola} = [x, y, v_x, v_y]^T$ em coordenadas milimétricas globais.
- **Modelo Físico:** Inclui modelo de atrito por desaceleração constante contra o carpete oficial ($800\text{ mm/s}^2$).
- **Detecção de Chute e Teletransporte:**
  - Caso a distância entre a predição e a medição exceda `ball_kick_threshold_mm` ($80\text{ mm}$), o filtro detecta aceleração impulsiva (chute) e inflaciona a matriz de covariância de processo $Q$.
  - Descontinuidades extremas (como resets no simulador grSim) disparam o reinício do filtro com covariâncias iniciais.

### 1.2 Filtro de Kalman dos Robôs (6D)
- **Vetor de Estado:** $\mathbf{x}_{robô} = [x, y, \theta, v_x, v_y, \omega]^T$.
- Mantém instâncias de filtros independentes para cada ID de robô em campo.
- Proteção contra deduplicação de frames: rejeita pacotes com timestamps idênticos ou não crescentes.

---

## 2. Tópicos de Comunicação

### 2.1 Assinaturas (Subscribers)
- `ball_data` (`oxebots_interfaces/msg/BallPosition`): Posição 2D medida da bola vinda do `game_observer_node`.
- `robot_data` (`oxebots_interfaces/msg/RobotGameData`): Dados de pose medidos dos robôs vindos do `game_observer_node`.

### 2.2 Publicações (Publishers)
- `ball_predicted` (`oxebots_interfaces/msg/BallPredicted`): Posição e vetor de velocidade filtrados da bola.
- `robot_predicted` (`oxebots_interfaces/msg/RobotPredicted`): Posição, orientação e velocidades filtradas dos robôs.

---

## 3. Parâmetros Configuráveis

| Parâmetro | Tipo | Padrão | Descrição |
| :--- | :--- | :--- | :--- |
| `ball_subscriber` | string | `"ball_data"` | Tópico de entrada para a bola. |
| `ball_publisher` | string | `"ball_predicted"` | Tópico de saída para a bola filtrada. |
| `robot_subscriber` | string | `"robot_data"` | Tópico de entrada para os robôs. |
| `robot_publisher` | string | `"robot_predicted"` | Tópico de saída para os robôs filtrados. |
| `team_size` | int | `3` | Número de robôs por time em campo. |
| `maximum_publisher_size` | int | `10` | Profundidade da fila de mensagens (QoS). |
| `ball_pos_measurement_noise` | double | `3.0` | Ruído de medição da posição da bola ($R$). |
| `ball_process_noise` | double | `2000.0` | Ruído de processo do modelo de movimento da bola ($Q$). |
| `ball_friction_accel` | double | `800.0` | Aceleração de desaceleração por atrito com o carpete (mm/s²). |
| `ball_kick_threshold_mm` | double | `80.0` | Limite de erro para detecção de chute e adaptação de ganho. |
| `robot_pos_measurement_noise` | double | `3.0` | Ruído de medição de posição dos robôs ($R_{pos}$). |
| `robot_angle_measurement_noise`| double | `0.02` | Ruído de medição angular em radianos ($R_{\theta}$). |
| `robot_pos_process_noise` | double | `4000.0` | Ruído de processo translacional dos robôs ($Q_{pos}$). |
| `robot_angle_process_noise` | double | `30.0` | Ruído de processo rotacional dos robôs ($Q_{\theta}$). |

---

## 4. Pré-requisitos e Dependências

- **Sistema:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **Bibliotecas:** `rclcpp`, `eigen3_cmake_module`, `Eigen3`, `oxebots_interfaces`.

---

## 5. Como Executar

```bash
ros2 run oxebots_prediction kalman_filter_node
```

Ou através do launch central:
```bash
ros2 launch oxebots_bringup ssl-new.launch.py
```

---

## 6. Licença

Este pacote é disponibilizado sob a licença **GPLv3**.
