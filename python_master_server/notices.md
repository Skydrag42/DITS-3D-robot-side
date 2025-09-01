## **Pré-requis**
1. **Installer le package dynamixel_sdk**
```bash
pip3 install dynamixel-sdk
```
2. **Ajouter au répertoire courrant les fichiers suivant**:
- `motor_controller.py`
- `motor.py`
- `config.py`

3. **Ouvrir le port de communication et initialiser les moteurs**:
```python
from motor_controller import MotorController

ctrl = MotorController()
```
**Remarques**:
- Vous pouvez désormais utiliser les fonctions décrites ci-dessous avec `ctrl.fonction(...)` et `motor_dic[ID].fonction(...)`
- L'unité `u` utilisée ci-dessous n'est pas une unité SI. Elle est utilisée ici pour indiquer que la valeur associée est converti dans le système de mesure du moteur en question.

# **Class MotorController**
## **Variables**:
- `motors_dic`: dictionnaire des moteurs (`ID:motor`)

## **Fonctions**: (`ctrl.fonction()`)
- `close_port()`: ferme le port de communication

- `ping_all()`: test la connexion de tous les moteurs
- `set_all_compliance(<True/False>)`: active/désactive le couple pour tous les moteurs

- `move_all(<{ID1:angle1, ID2:angle2, ...}>, duration)`: prend en argument un dictionnaire associant des ID de moteur à une valeur d'angle qui est ajoutée aux `goal_position` du moteur correspondant et ajuste sa vitesse pour l'atteindre en `duration (s)`
- `move_all_to(<angles>, duration)`: prend en argument un dictionnaire associant des ID de moteur à une valeur d'angle qui devient la nouvelle `goal_position` du moteur correspondant et ajuste sa vitesse pour l'atteindre en `duration (s)`

# **Class Motor**
## **Variables**:
- `ID`, `name` (ex: `41`, `l_shoulder_y`)
- `min_angle (°)`, `max_angle (°)`: position min et max du moteur (ex: `40`, `330`)
- `offset (°)`: correctif entre la position des moteurs et les mesures sur image
- `goal_position (u)`: la position cible actuel du moteur en unité `u`, peux être modifié avec `move` et `move_to`.

## **Fonctions**: (`motor.fonction()`)
- `ping(<motor>)`: test la connexion de `<motor>`
- `set_compliance(<motor>, <True/False>)`: active/désactive le couple de `<motor>`

- `move(<angle> (°), duration (s))`: ajoute `<angle> (°)` à `goal_position` et ajuste la vitesse pour l'atteindre en `duration (s)`
- `move_to(<angle> (°), duration (s))`: remplace `goal_position` par `<angle> (°)` et ajuste la vitesse pour l'atteindre en `duration (s)`

- `get_position()`: renvoie la `position (u)` actuelle du moteur 
- `set_speed(<speed> (°/s))`: la vitesse du moteur devient `<speed> (°/s)` (0 => vitesse max)
- `speed_for_fixed_duration(<duration> (s))`: renvoie la `vitesse (°/s)` permettant au moteur d'atteindre goal_position en `<duration> (s)`

- `set_accel(<accel> (°/s²))`: l'accélération du moteur devient `<accel> (°/s²)`