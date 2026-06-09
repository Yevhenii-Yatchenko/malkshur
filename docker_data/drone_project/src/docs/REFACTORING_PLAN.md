# План рефакторингу DroneProject (GRASP-орієнтований)

> Згенеровано 2026-06-09 агентом-архітектором. Пріоритети: Information Expert, Low Coupling, High Cohesion.

Корінь проекту: `/home/wintery/repos/malkshur/docker_data/drone_project/src`

## 1. Карта поточної архітектури

### Процес 1: польотний контролер (`xbee_process_com.py` → `src/`)

| Клас / модуль | Файл | Фактична відповідальність |
|---|---|---|
| `DroneController` | `src/controller.py` (495 р.) | **God-object**: композиція всіх підсистем, головний цикл 100 Гц, командні хендлери, авто-armінг (Timer), state machine режиму перехоплення, інтеграція стабілізації, висотний контроль, відправка RC, cleanup |
| `MAVLinkManager` | `src/mavlink_manager.py` | З'єднання TCP/USB, message rates, arm/disarm, set_mode, RC override — **добре згуртований**, найчистіший модуль |
| `SensorManager` + `AltitudeSensor` (ABC) | `src/sensor_manager.py` | Фабрика LIDAR/барометр — хороша абстракція, але читає глобальний конфіг |
| `AltitudeController` + `PIDController` | `src/pid_controller.py` | Каскадний PID висоти + фільтрація + оцінка швидкості + **сам створює CSV-логер** |
| `PositionController` | `src/position_controller.py` | Каскадний PID XY + перемикання режимів stabilization/navigation по магічному значенню confidence |
| `CommandHandler` | `src/command_handler.py` | Парсинг команд + диспетчеризація + **сам створює TelnetServer** + фоновий потік |
| `TelnetServer` | `src/telnet_server.py` | TCP-сервер + парсинг рядків + довідка + **загортання команд у JSON, який CommandHandler одразу розгортає назад** |
| `StabilizerManager` / `SkyAnchorClient` | `src/stabilizer_manager.py`, `src/sky_anchor_client.py` | Запуск процесу sky_anchor + TCP-клієнт; віддають **сирий dict** нагору |
| `DetectionServer` / `DockerDetectionClient` | `src/detection_server.py`, `src/detection_client.py` | Прийом детекцій по TCP / керування Docker-контейнером; віддають сирий dict |
| `BatteryMonitor`, `LidarSensor`, `SignalHandler`, `logger` | відповідні файли | Залізні підсистеми (hardware-only) + сигнали + кастомний логер з глобальним singleton-реєстром |

### Процес 2: візуальна підсистема (`sky_anchor/`)

Уже непогано декомпозована: `Controller` → `FrameProvider` → `ShiftEvaluator`(`ShiftEstimator` CPU/CUDA) → `NavigationCoordinator`(`CommandModifier`) → `CommandPublisher`(`SkyAnchorServer`, TCP:8888). Камера Gazebo — `gazebo_classic_bridge.py` (pygazebo).

### Граф залежностей (стисло)

```
xbee_process_com → DroneController
  ├→ MAVLinkManager ──→ controller_config (глобально)
  ├→ SensorManager ───→ controller_config, altitude_config
  ├→ StabilizerManager → SkyAnchorClient (TCP:8888 ← sky_anchor)
  ├→ AltitudeController → PIDController, altitude_config, AltitudeCSVLogger (сам створює)
  ├→ PositionController → PIDController, position_config, PositionCSVLogger (сам створює)
  ├→ CommandHandler ──→ TelnetServer (сам створює, TCP:2323)
  ├→ DetectionServer / DockerDetectionClient → detection_config
  └→ BatteryMonitor, SignalHandler
```

Циклічних імпортів **немає** (конфіг-модулі — листя графа). Головна проблема — не цикли, а те, що конфіг-словники і логер-singleton (`logger.py:124` `_logger_instances`) є глобальним станом, який імпортується на всіх рівнях.

### Живий код vs баласт

**Живе:** `src/` (крім зазначеного нижче), `sky_anchor/`, `plot_*.py` (офлайн-аналіз CSV), `gazebo/fix_pygazebo*.py` (build-time фікс для Python 3.10 — інфраструктура).

**Баласт (ніде не імпортується):**
- `src/communicator.py` — XBee DigiMesh; жоден живий модуль його не імпортує. Назва точки входу `xbee_process_com.py` — історичний артефакт (5 рядків, XBee там немає).
- `src/ultrasonic_sensor.py` — замінений LIDAR'ом, не імпортується.
- `src/targetTracker.py` — не імпортується.
- `modules/interfaces.py` — «аспіраційні» ABC, ніким не реалізовані й не імпортовані.
- `convertor/`, `camera_test/`, `help_info/` — разові скрипти/нотатки.
- `dd_shahed/` — окремий C++ підпроект (TensorRT-детекція); викликається лише опосередковано через Docker на залізі, не через Python-імпорти. Не чіпати, але виключити зі скоупу.
- `examples/` — ручні залізні тести; залишити як є.

**Важливо:** `CLAUDE.md` (рядки 35–68) і `pytest.ini` описують каталог `tests/`, **якого не існує**. Юніт-тестів у проекті нуль — документація бреше.

## 2. Топ порушень GRASP (з файлами і рядками)

### Information Expert (логіка не там, де дані)

**IE-1. `DroneController.__updateThrottle` длубається в нутрощах детекції.**
`src/controller.py:296–383`: контролер сам дістає `detection_data.get('direction_vector', {}).get('direction', [0,0,0])`, перевіряє confidence, таймаути, веде state machine перехоплення (`__intercept_mode`). Експерт по даних детекції — `DetectionServer`, але він лише зберігає dict; усі рішення приймає чужий клас. Це і feature envy, і розмазана state machine.

**IE-2. Сирі dict'и як кров системи.**
Ланцюг `SkyAnchorClient._receive_data` → `StabilizerManager.get_stabilizer_data()` → `controller.py:387–404` (`data.get('dx')`, `data.get('matches_percent')`, `data.get('timestamp')`). У sky_anchor існує `ShiftCommand` dataclass (`sky_anchor/app/vision/evaluator.py:13–37`), але на дроті він деградує до JSON-dict і ніколи не відновлюється на стороні клієнта. Дедуплікація за timestamp (`__last_xy_update`, controller.py:389–390) — теж рішення по чужих даних: свіжість даних має визначати їхній власник (StabilizerManager).

**IE-3. Магічний сентинел `confidence == 1.01`.**
`sky_anchor/app/navigation/command_modifier.py:127` ставить `matches_percent=101.0`, а `src/position_controller.py:232–234` робить `if confidence == 1.01: __enable_navigation()`. Канал даних використано як прихований канал керування, з порівнянням float'ів на точну рівність через ділення на 100 у `controller.py:401`. Найкрихкіше місце системи: округлення/зміна формату мовчки зламає перемикання режимів.

**IE-4. Дубльоване знання про ліміт газу.**
`src/controller.py:49–53` (`if value >= 1800: value = 1800`) жорстко дублює `THROTTLE['max']` з `altitude_config.py:40`. Власник інваріанта «безпечний діапазон PWM» не визначений — він розмазаний по контролеру, AltitudeController і конфігу.

**IE-5. CSV-логер знає внутрішню структуру PID.**
`src/altitude_csv_logger.py:141–168` розбирає вкладені dict'и `position_pid.get('gains', {}).get('kp')` — формат `PIDController.get_state()` (pid_controller.py:152–172) жорстко зчеплений із заголовками CSV. Зміна снапшота PID ламає логер мовчки (`data.get(..., 0)` приховує розсинхрон).

### Low Coupling (жорсткі зчеплення)

**LC-1. PID-контролери самі інстанціюють логери.**
`src/pid_controller.py:4, 260`: `self.csv_logger = AltitudeCSVLogger(...)` всередині `AltitudeController.__init__`. Те саме `src/position_controller.py:110`. Наслідок: математику PID неможливо юніт-тестити без створення файлів у `logs/csv/`.

**LC-2. Конфіг-словники вшиті в глибину логіки.**
- `pid_controller.py:186–201`: значення конфігу — **default-аргументи**, зв'язані в момент імпорту.
- `pid_controller.py:337, 345, 356`: `from .altitude_config import THROTTLE/CONTROL/DEBUG` всередині `update()` — імпорт у гарячому циклі 100 Гц.
- `position_controller.py:192–197`: `__enable_stabilization` відновлює коефіцієнти, читаючи глобальний `POSITION_PID_X`.
- `USE_GAZEBO` обчислюється двічі незалежно: `controller_config.py:8` і `detection_config.py:9`.

**LC-3. Створення залежностей замість ін'єкції.**
`command_handler.py:45–47` сам створює `TelnetServer` (дефолтний порт TelnetServer — 23, CommandHandler передає 2323 — мертва пастка). `StabilizerManager` сам створює `SkyAnchorClient`. `DroneController.__init__` — composition root, змішаний з логікою: жодну підсистему не можна підмінити для тесту.

**LC-4. Глобальний стан логера.**
`src/logger.py:124–147`: модульний словник `_logger_instances`; перший виклик фіксує рівень/консоль для всіх наступних. Плюс `os.fsync` на кожен запис (logger.py:69) у циклі 100 Гц.

### High Cohesion (модулі, що роблять кілька речей)

**HC-1. `controller.py` — god-object.**
Сім різних відповідальностей. `__updateThrottle` (рядки 293–428, 135 рядків) насправді оновлює roll/pitch/yaw, цільову висоту, режим перехоплення, і лише потім throttle — назва бреше. Логіка «увімкнути стабілізацію при досягненні висоти» захована в else-гілці (рядки 411–413).

**HC-2. `telnet_server.py` — транспорт + парсинг + протокольний театр.**
`_process_command` (рядки 129–167) парсить команди, потім **загортає їх у JSON `{"msg": ...}`**, кладе в чергу — щоб `CommandHandler.parse_json_message` (command_handler.py:98–116) у тому ж процесі розпакував назад. Безглуздий round-trip, успадкований від XBee-протоколу. Польотні дії TelnetServer не виконує — виконує CommandHandler через колбеки; це місце вже непогане.

**HC-3. `AltitudeController` — чотири роботи.**
Фільтрація сенсора + оцінка швидкості + каскадний PID + шейпінг throttle (rate limit, exp filter) + CSV-логування в одному `update()`.

**HC-4. Тестовий сценарій вшито в продакшн-ініціалізацію (критична знахідка).**
`sky_anchor/app/navigation/coordinator.py:39–50`: конструктор `NavigationCoordinator` створює Timer'и, які **через 5 секунд після старту автоматично командують політ квадратом 1000 px**. Гейт по env `TEST_NAVIGATION` — закоментований. Це активна поведінка, яку треба свідомо зафіксувати або вимкнути.

**HC-5. Прихований side effect у команді arm.**
`controller.py:233–236`: успішний `arm` автоматично викликає `_setHeight([5])` — зліт на 5 м вшито в armінг.

## 3. Цільова архітектура (еволюційна, без переписування)

```
src/
  domain/types.py        # StabilizerReading, DetectionReading, AttitudeSetpoints,
                         # FlightMode(Enum) — frozen dataclass'и замість dict'ів
  config/                # AltitudeConfig, PositionConfig, ... — frozen dataclass'и,
                         # значення беруться 1:1 з існуючих *_config.py
  flight/setpoints.py    # RCSetpoints: володіє roll/pitch/yaw/throttle base + target_altitude,
                         #   клампінг PWM в одному місці (Information Expert для лімітів)
  flight/intercept.py    # InterceptGuidance: state machine перехоплення,
                         #   приймає DetectionReading, повертає AttitudeSetpoints | None
  flight/stabilization.py# StabilizationBehavior: споживає StabilizerReading → PWM через PositionController
  telemetry/recorder.py  # інтерфейс TelemetrySink; CSV-логери стають імплементаціями,
                         #   ІН'ЄКТУЮТЬСЯ в контролери (Null-об'єкт для тестів)
  app.py                 # composition root: будує всі залежності, віддає готовий DroneController
```

**Потоки інформації після рефакторингу:**
1. `SkyAnchorClient` парсить JSON → `StabilizerReading` (typed, з полем `is_navigation: bool` замість сентинела 1.01). `StabilizerManager.poll_new()` повертає лише свіжі читання (дедуплікація — у власника даних).
2. `DetectionServer.get_active_target(timeout, min_confidence)` — сам вирішує, чи детекція актуальна, повертає `DetectionReading` або `None`.
3. `DroneController.loop()` стає тонким оркестратором: читає висоту → питає `InterceptGuidance`, інакше `StabilizationBehavior` → `AltitudeController` → `RCSetpoints.clamp()` → `MAVLinkManager.send_rc_override()`.
4. Командні хендлери виносяться у `FlightCommands` (фасад над mavlink/setpoints/stabilizer), `DroneController.__register_commands` лише зв'язує його з `CommandHandler`.
5. Telnet кладе в чергу сирий рядок команди; JSON-обгортка лишається тільки для реального XBee-шляху (`communicator.py` — в архів).

`MAVLinkManager`, `SensorManager` (ієрархія), `CommandHandler`, увесь `sky_anchor/app/vision/*` — зберігаються майже без змін.

## 4. Поетапний план (кожен крок лишає систему робочою)

**Крок 0. Зафіксувати поведінку (обов'язково перший).**
- Скрипт golden run: `./docker.sh up` → controller → telnet-сценарій `mode,STABILIZE; arm,0` (авто-зліт на 5 м) → hold 90 с → `land`. Зберегти `logs/csv/altitude_control_*.csv`, `position_*.csv`, `logs/controller.log`.
- 3–5 базових прогонів, щоб виміряти природну дисперсію SITL (симуляція недетермінована — порівнювати статистики, не семпли).
- Скрипт порівняння: RMSE висоти відносно target (толеранс ~0.1 м), час виходу на 5 м, частка насиченого throttle, mean/σ PID-термів, ідентичність набору CSV-колонок.
- **Рішення по HC-4**: тестові Timer'и в `coordinator.py:39–50` зараз літають квадратом автоматично. Або зафіксувати це в golden run, або (краще) розкоментувати гейт `TEST_NAVIGATION` окремим першим комітом і перезняти baseline. Це єдина дозволена зміна поведінки, і вона має бути усвідомленою.

**Крок 1. Карантин баласту (нульовий ризик).**
Перемістити `communicator.py`, `ultrasonic_sensor.py`, `targetTracker.py`, `modules/`, `convertor/` у `legacy/` (не імпортуються — нічого не зламається; перевірити grep'ом після переносу). Виправити `CLAUDE.md` (прибрати вигадані tests/) і перейменувати/задокументувати `xbee_process_com.py` → `run_controller.py` (старе ім'я лишити шимом — на нього посилаються entrypoint-скрипти Docker).

**Крок 2. Юніт-тести на чисту математику (до будь-яких змін коду).**
`tests/unit/`: `PIDController` (детермінований через параметр `current_time`: step response, анти-windup, клампінг, dt=0.1 на першій ітерації), `CommandHandler.parse_message`, `ShiftEvaluator._apply_deadband`, `PositionController.pixels_to_meters`. Для `AltitudeController`/`PositionController` — тимчасово monkeypatch CSV-логерів (це й мотивує крок 3).

**Крок 3. Ін'єкція логерів (перший справжній GRASP-фікс, LC-1).**
`AltitudeController(..., csv_logger=None)` і `PositionController(..., csv_logger=None)`: якщо None — створюється як зараз (зворотна сумісність), composition root передає явно. Винести function-level імпорти `pid_controller.py:337,345,356` у `__init__`. Тести з кроку 2 позбавляються monkeypatch.
*Примітка по виконанню:* імпорти винесено на рівень модуля, а не в `__init__` — еквівалентно, бо ніщо не перепризначає атрибути конфіг-модулів (читаються ті самі dict-об'єкти `THROTTLE`/`CONTROL`/`DEBUG`).

**Крок 4. Типізовані дані замість dict'ів (IE-2, IE-3).**
- `StabilizerReading` dataclass; парсинг у `SkyAnchorClient`; `StabilizerManager.poll_new()` з дедуплікацією.
- Замінити сентинел 1.01: sky_anchor додає в payload явне поле `"navigation": true` (адитивна зміна формату — старий клієнт її ігнорує, обидва кінці в одному репо). `PositionController.update(..., navigation: bool)`. Видалити порівняння `confidence == 1.01`.
- `DetectionReading` + перенесення перевірок таймаута/confidence у `DetectionServer.get_active_target()`.

**Крок 5. Розпиляти `__updateThrottle` (HC-1, IE-1).**
Витягти `InterceptGuidance` (state machine + deadband/yaw/altitude-корекції, рядки 296–383) і `StabilizationBehavior` (рядки 386–413). `__updateThrottle` стає ~20-рядковим: вибір behavior → setpoints → altitude PID → send. Golden run після кожного підкроку.

**Крок 6. `RCSetpoints` (IE-4).**
Клас-власник roll/pitch/yaw/throttle/target_altitude з клампінгом з конфігу (єдине джерело ліміту 1800). Прибрати `__set_throttle_base`-хак.

**Крок 7. Composition root + конфіг-об'єкти (LC-2, LC-3).**
`src/app.py` будує: MAVLinkManager → SensorManager → StabilizerManager(client=...) → контролери(config=..., csv_logger=...) → CommandHandler(telnet_server=...). Конфіг-dataclass'и читають значення з існуючих `*_config.py` (числа не переносити руками — імпортувати, щоб тюнінг лишився байт-у-байт). `USE_GAZEBO` — одне місце.

**Крок 8 (опційно, останній).** Прибрати JSON round-trip telnet→CommandHandler (HC-2); розчепити `AltitudeCSVLogger` від форми `get_state()` через плоский снапшот-метод у PID (IE-5).

Залежності: 0 → (1,2 паралельно) → 3 → 4 → 5 → 6 → 7 → 8. Максимальний ефект на одиницю ризику дають кроки 2–4.

## 5. Стратегія верифікації

- **Юніт-тести (нові):** найкращий кандидат — `PIDController` (повністю детермінований, час ін'єктується); далі `AltitudeController.update` з ручним `current_time` (фіксований профіль «зліт 0→5 м» → послідовність throttle має збігатися до і після рефакторингу — справжній regression-тест без SITL), парсери команд, deadband-логіка, `CommandModifier`.
- **Golden run у SITL** (крок 0) після кожного кроку 3–8: статистичне порівняння CSV. Якщо крок не мав міняти поведінку, а метрики випали за дисперсію baseline — відкат.
- **Smoke-критерій:** arm → стабільний hold на 5 м ±0.3 м протягом 60 с → land, без «Control loop overrun» частіше за baseline.

## 6. Ризики і що НЕ чіпати

1. **Не чіпати:** тюнінг PID (`altitude_config.py`, `position_config.py` — значення байт-у-байт), протокольний код `mavlink_manager.py` (18-канальний RC override, message intervals, таймаути arm/disarm), wire-формат JSON на :8888 (тільки адитивні поля), потокову модель (daemon-потоки, локи), `gazebo/fix_pygazebo*.py`.
2. **Поведінкові «баги», які є load-bearing:** `BarometerAltitudeSensor` ніколи не повертає `None` (повертає 0.0/останнє значення, `sensor_manager.py:139,175`) — усі `if current_altitude is None` у контролері в Gazebo-режимі мертві; «виправлення» на None змінить поведінку при втраті телеметрії. `dt=0.1` на першій ітерації PID, анти-windup — не «покращувати» під час рефакторингу.
3. **Сентинел 1.01** — міняти лише синхронно з обох боків (один репо, один коміт).
4. **Auto-setHeight(5) при arm** і **авто-stabilize при досягненні висоти** (controller.py:236, 411–413) — зберегти як є, лише перенести; це частина очікуваного сценарію golden run.
5. **SITL недетермінований** — порівнювати статистики; CSV-логування з `os.fsync` на 100 Гц впливає на таймінги, не міняти логер до кінця (можлива окрема задача після).
6. Hardware-шляхи (LIDAR, BatteryMonitor, XBee, Jetson) не перевіряються симуляцією — у цих файлах тільки механічні зміни (ін'єкція), без переписування.
