# Detection System - Object Recognition & Intercept Mode

Система розпізнавання об'єктів та перехоплення цілей для дрона.

## Огляд

Система складається з трьох компонентів:

1. **Detection Client** (Docker) - клієнт розпізнавання об'єктів в Docker контейнері
2. **Detection Server** - TCP сервер для моніторингу даних розпізнавання
3. **Intercept Mode** - автоматичне переслідування та перехоплення цілей

## Архітектура

```
┌─────────────────────────────────────────────────────────────┐
│ Docker Container (jetson-inference)                         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ Detection Client (tmp_gazebo.py)                     │  │
│  │ - Розпізнавання об'єктів                             │  │
│  │ - Обчислення direction_vector                        │  │
│  │ - Відправка даних на сервер                          │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            │ TCP (port 5000)
                            ▼
┌─────────────────────────────────────────────────────────────┐
│ DroneController                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ Detection Server (localhost:5000)                    │  │
│  │ - Приймає дані розпізнавання                         │  │
│  │ - Зберігає останнє detection                         │  │
│  │ - Відстежує timeout                                  │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ Intercept Mode Logic (__updateThrottle)              │  │
│  │ - Перевіряє confidence threshold (75%)               │  │
│  │ - Центрування: yaw (горизонталь)                     │  │
│  │ - Центрування: altitude (вертикаль)                  │  │
│  │ - Рух вперед: pitch + 20 PWM                         │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## Команди Telnet

### 1. Запуск Detection Server (моніторинг)

```bash
# Через telnet на порт 2323
{"msg": "monitor,start"}
# або
{"msg": "monitor,1"}
```

Запускає TCP сервер на `localhost:5000` для приймання даних розпізнавання.

### 2. Зупинка Detection Server

```bash
{"msg": "monitor,stop"}
# або
{"msg": "monitor,0"}
```

### 3. Запуск Detection Client (Docker)

```bash
{"msg": "recognizeClient,start"}
# або
{"msg": "recognizeClient,1"}
```

Запускає Docker контейнер з моделлю розпізнавання об'єктів.

**Що відбувається:**
- Запускається Docker контейнер (`jetson-rtx4090` для Gazebo)
- Виконується скрипт розпізнавання `tmp_gazebo.py`
- Модель аналізує відео з камери
- Результати відправляються на detection server

### 4. Зупинка Detection Client

```bash
{"msg": "recognizeClient,stop"}
# або
{"msg": "recognizeClient,0"}
```

Зупиняє Docker контейнер.

### 5. Disarm (автоматично зупиняє все)

```bash
{"msg": "arm,1"}
```

Зупиняє обидва компоненти (server + client) та вимикає intercept mode.

## Формат даних розпізнавання

Detection client відправляє JSON:

```json
{
  "class_id": 1,
  "confidence": 0.8119,
  "direction_vector": {
    "direction": [0.4024, 0.2337, 0.8851],
    "magnitude": 321.39,
    "magnitude_normalized": 0.6428
  },
  "coordinates": {
    "x_min": 164.14,
    "y_min": -235.22,
    "x_max": 371.76,
    "y_max": -119.72,
    "center": [267.95, -177.47]
  },
  "image_info": {
    "width": 800,
    "height": 600,
    "timestamp": 10
  }
}
```

### Інтерпретація direction_vector

- **direction[0]** (x): Горизонтальне зміщення (-0.5 до 0.5)
  - `> 0`: Ціль справа → обертатися вправо (yaw+)
  - `< 0`: Ціль зліва → обертатися вліво (yaw-)

- **direction[1]** (y): Вертикальне зміщення (-0.5 до 0.5)
  - `> 0`: Ціль вище → підняти висоту
  - `< 0`: Ціль нижче → знизити висоту

- **direction[2]** (z): Не використовується (резерв)

- **magnitude_normalized**: Відстань від центру (0-1)

## Intercept Mode (режим перехоплення)

### Умови активації

1. Detection server запущений (`monitor start`)
2. Confidence ≥ 75% (налаштовується: `INTERCEPT_CONFIDENCE_THRESHOLD`)
3. Дані отримані менш ніж 3 секунди тому (налаштовується: `INTERCEPT_TIMEOUT_SECONDS`)

### Поведінка в Intercept Mode

#### Центрування по горизонталі (Yaw)

```python
if abs(dir_x) > 0.1:  # Deadband
    yaw_correction = dir_x * 100  # Gain
    yaw_pwm = 1500 + yaw_correction
else:
    yaw_pwm = 1500  # Центровано
```

**Приклад:**
- `dir_x = 0.4` → `yaw_pwm = 1540` (обертання вправо)
- `dir_x = -0.3` → `yaw_pwm = 1470` (обертання вліво)

#### Центрування по вертикалі (Altitude)

```python
if abs(dir_y) > 0.1:  # Deadband
    altitude_correction = dir_y * 0.1  # Step per cycle
    target_altitude += altitude_correction
```

**Приклад:**
- `dir_y = 0.3` → Підняти на 0.03м
- `dir_y = -0.2` → Знизити на 0.02м

#### Рух вперед (Pitch)

```python
pitch_pwm = 1500 + 20  # Постійна швидкість вперед
```

#### Roll (бічний рух)

```python
roll_pwm = 1500  # Завжди нейтраль
```

### Вихід з Intercept Mode

Автоматично вимикається якщо:

1. Немає даних розпізнавання протягом 3 секунд
2. Виконується disarm
3. Зупиняється detection server

**Що відбувається:**
- Зупиняється рух вперед: `pitch = 1500`
- Зупиняється обертання: `yaw = 1500`
- Фіксується поточна висота
- Включається стабілізація sky_anchor (якщо була запущена)

## Конфігурація

Всі параметри знаходяться в [src/detection_config.py](src/detection_config.py).

### Основні параметри

```python
# Confidence threshold для активації intercept mode
INTERCEPT_CONFIDENCE_THRESHOLD = 0.75  # 75%

# Timeout без даних (секунди)
INTERCEPT_TIMEOUT_SECONDS = 3.0

# Deadband зони (ігнорувати малі відхилення)
INTERCEPT_DEADBAND_X = 0.1  # Горизонталь
INTERCEPT_DEADBAND_Y = 0.1  # Вертикаль

# Коефіцієнт для yaw (PWM на одиницю direction_vector)
INTERCEPT_YAW_GAIN = 100

# Крок зміни висоти (метри за цикл)
INTERCEPT_ALTITUDE_STEP = 0.1

# Offset для руху вперед (PWM)
INTERCEPT_PITCH_OFFSET = 20
```

### Перевизначення через змінні оточення

```bash
export INTERCEPT_CONFIDENCE_THRESHOLD=0.8
export INTERCEPT_TIMEOUT_SECONDS=5.0
export INTERCEPT_YAW_GAIN=150
```

### Docker конфігурація

```python
# Gazebo
DETECTION_DOCKER_SCRIPT_GAZEBO = "../jetson-inference/run-rtx4090.sh"
DETECTION_MODEL_GAZEBO = "models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx"

# Hardware (Jetson)
DETECTION_DOCKER_SCRIPT_HARDWARE = "../jetson-inference/run-jetson.sh"
```

## Приклад використання

### Сценарій 1: Базове тестування

```bash
# 1. Запустити дрон
./run_gazebo.sh

# 2. Підключитись через telnet
telnet localhost 2323

# 3. Запустити detection server
{"msg": "monitor,start"}

# 4. Arm і зліт (автоматично після arm 0)
{"msg": "arm,0"}

# 5. Запустити detection client
{"msg": "recognizeClient,start"}

# Тепер якщо ціль буде розпізнана з confidence > 75%,
# дрон автоматично увійде в intercept mode і почне переслідування

# 6. Зупинити
{"msg": "arm,1"}  # Disarm (зупинить все)
```

### Сценарій 2: Ручне керування intercept

```bash
# Запустити тільки server спочатку
{"msg": "monitor,start"}

# Arm і підняти дрон
{"msg": "arm,0"}

# Почекати поки дрон стабілізується

# Запустити detection client коли готові
{"msg": "recognizeClient,start"}

# Зупинити client але залишити server активним
{"msg": "recognizeClient,stop"}

# Перезапустити client
{"msg": "recognizeClient,start"}
```

## Логування

Логи записуються в:

- `logs/detection_server.log` - Активність сервера, отримання даних
- `logs/detection_client.log` - Вивід Docker контейнера
- `logs/controller.log` - Події intercept mode

### Моніторинг логів

Використовуйте скрипт для зручності:

```bash
./tail_detection_logs.sh
```

Або вручну:

```bash
tail -f logs/controller.log logs/detection_server.log logs/detection_client.log
```

### Важливі події в логах

**Controller (logs/controller.log):**
```
[WARNING] INTERCEPT MODE ACTIVATED (confidence: 81.19%)
[DEBUG] Intercept: conf=81.19%, dir_x=+0.402, dir_y=+0.234, yaw=1540, pitch=1520
[WARNING] INTERCEPT MODE DEACTIVATED (no detection for 3.0s)
[INFO] Holding altitude at 5.00m
```

**Detection Server (logs/detection_server.log):**
```
[2025-11-21 21:55:10.123] Detection server listening on 0.0.0.0:5000
[2025-11-21 21:55:15.456] Client connected: 172.17.0.2:54321
[21:55:15] 📥 Detection from 172.17.0.2:54321 - class=1, conf=81.19%, dir_x=+0.402, dir_y=+0.234
```

**Detection Client (logs/detection_client.log):**
```
======================================================================
Detection client started at 2025-11-21 21:55:05
======================================================================
[2025-11-21 21:55:06] Shell prompt detected
[2025-11-21 21:55:06] Commands sent:
cd data/dd_smart_agent/
./tmp_gazebo.py --model=... --server_host=172.19.173.99 --server_port=5000
======================================================================
Docker output below:
======================================================================
[Output from detection script]
```

## Troubleshooting

### Detection client не запускається

1. Перевірити чи є Docker:
```bash
docker ps
```

2. Перевірити шлях до скрипту:
```bash
ls ../jetson-inference/run-rtx4090.sh
```

3. Перевірити логи:
```bash
tail -f logs/detection_client.log
```

### Server не отримує дані

1. Перевірити чи server запущений:
```bash
netstat -an | grep 5000
```

2. Перевірити логи server:
```bash
tail -f logs/detection_server.log
```

### Intercept mode не активується

1. Перевірити confidence в логах - має бути > 75%
2. Перевірити чи server запущений (`monitor start`)
3. Перевірити timeout - дані мають надходити регулярно

### Дрон не центрується правильно

Налаштувати gains в `src/detection_config.py`:

```python
# Збільшити для швидшого обертання
INTERCEPT_YAW_GAIN = 150  # Було 100

# Збільшити для більших кроків висоти
INTERCEPT_ALTITUDE_STEP = 0.15  # Було 0.1
```

## Безпека

⚠️ **ВАЖЛИВО:**

1. Завжди тестувати спочатку в симуляції (Gazebo)
2. Intercept mode автоматично рухає дрон вперед - забезпечити вільний простір
3. Використовувати killswitch для аварійної зупинки
4. Disarm автоматично зупиняє всі системи

## Майбутні покращення

- [ ] Використання `magnitude_normalized` для адаптивної швидкості
- [ ] Підтримка `direction_vector[2]` (z-компонента)
- [ ] Траєкторне планування для складніших маневрів
- [ ] Множинні цілі (пріоритизація)
- [ ] Інтеграція з navigation system
