# modules/interfaces.py

from abc import ABC, abstractmethod
from typing import Any, Dict, Callable


# 1. Module: Camera Recognition
class AbstractCameraRecognition(ABC):
    """
    Абстрактний клас для модуля розпізнавання з камери.
    Цей модуль сам підключається до камери та генерує події (observer),
    по яких можна робити розрахунки.
    """

    @abstractmethod
    def initialize(self) -> None:
        """
        Ініціалізація підключення до камери, налаштування необхідних параметрів.
        """
        pass

    @abstractmethod
    def start_recognition(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """
        Почати розпізнавання. Реалізувати підписку (Observer), яка через callback
        передаватиме дані для подальших розрахунків.
        
        :param callback: Функція, яка отримує результати розпізнавання у вигляді словника.
        """
        pass

    @abstractmethod
    def stop_recognition(self) -> None:
        """
        Зупинити розпізнавання та відписатися від отримання даних.
        """
        pass

    @abstractmethod
    def close(self) -> None:
        """
        Деініціалізація: закрити підключення до камери, звільнити ресурси.
        """
        pass


# 2. Module: Sound Recognition
class AbstractSoundRecognition(ABC):
    """
    Абстрактний клас для модуля розпізнавання звуку.
    Цей модуль сам підключається до мікрофону і генерує дані,
    які дають змогу зрозуміти напрямок звуку.
    """

    @abstractmethod
    def initialize(self) -> None:
        """
        Ініціалізація підключення до мікрофону, налаштування необхідних параметрів.
        """
        pass

    @abstractmethod
    def start_recognition(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """
        Почати розпізнавання звуку. Реалізувати підписку, яка через callback
        передаватиме дані (наприклад, напрямок звуку) для обробки.
        
        :param callback: Функція, яка отримує результати у вигляді словника.
        """
        pass

    @abstractmethod
    def stop_recognition(self) -> None:
        """
        Зупинити розпізнавання звуку та відписатися від подій.
        """
        pass

    @abstractmethod
    def close(self) -> None:
        """
        Деініціалізація: закрити підключення до мікрофону, звільнити ресурси.
        """
        pass


# 3. Module: Stabilization
class AbstractStabilization(ABC):
    """
    Абстрактний клас для модуля стабілізації.
    Модуль підключається до нижньої камери і генерує події, що допомагають
    коригувати потужність двигунів для запобігання дрейфуванню.
    """

    @abstractmethod
    def initialize(self) -> None:
        """
        Ініціалізація підключення до нижньої камери, налаштування параметрів стабілізації.
        """
        pass

    @abstractmethod
    def start_stabilization(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """
        Почати стабілізацію. Реалізувати підписку, яка через callback
        передаватиме події (наприклад, зсуви), на основі яких виконуватиметься корекція.
        
        :param callback: Функція, яка отримує дані про зсув/дрейф у вигляді словника.
        """
        pass

    @abstractmethod
    def stop_stabilization(self) -> None:
        """
        Зупинити стабілізацію та відписатися від подій.
        """
        pass

    @abstractmethod
    def close(self) -> None:
        """
        Деініціалізація: закрити підключення до камери, звільнити ресурси.
        """
        pass


# 4. Module: Control
class AbstractControl(ABC):
    """
    Абстрактний клас для модуля керування дроном (інтерфейс до польотного контролера).
    Через методи цього класу головний модуль звертатиметься до польотника.
    """

    @abstractmethod
    def connect(self, port_name: str) -> None:
        """
        Підключитись до дрону (або ініціалізація).
        
        :param port_name: Назва або шлях до порту/з'єднання з контролером.
        """
        pass

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """
        Отримати статус дрону. Наприклад, батарея, швидкість, висота, координати.
        
        :return: Словник зі статусними даними.
        """
        pass

    @abstractmethod
    def takeoff(self) -> None:
        """
        Піднятися (take off).
        """
        pass

    @abstractmethod
    def land(self) -> None:
        """
        Опуститись (land).
        """
        pass

    @abstractmethod
    def hover(self) -> None:
        """
        Зависнути (hover) на місці.
        """
        pass

    @abstractmethod
    def move_to_monitoring_position(self, coordinates: Dict[str, float]) -> None:
        """
        Рух на позицію моніторингу.
        
        :param coordinates: Словник із ключами, наприклад, {'lat': ..., 'lon': ..., 'alt': ...}.
        """
        pass

    @abstractmethod
    def return_home(self) -> None:
        """
        Рух додому (return home).
        """
        pass

    @abstractmethod
    def stabilization_correction(self, adjustments: Dict[str, Any]) -> None:
        """
        Корекція по стабілізації (наприклад, на основі даних від стабілізаційного модуля).
        
        :param adjustments: Словник з коригуваннями потужності двигунів.
        """
        pass

    @abstractmethod
    def turn_to_sound_source(self, direction: float) -> None:
        """
        Повернутись у напрямку джерела звуку.
        
        :param direction: Кут (у градусах) відносно поточної орієнтації.
        """
        pass

    @abstractmethod
    def attack_vector(self, vector: Dict[str, float]) -> None:
        """
        Атакувати по заданому вектору.
        
        :param vector: Словник із координатами або швидкістю, напр. {'dx': ..., 'dy': ..., 'dz': ...}.
        """
        pass

    @abstractmethod
    def center_on_target(self, target_info: Dict[str, Any]) -> None:
        """
        Корекція центрування на ціль (наприклад, щоб тримати об'єкт в центрі поля зору).
        
        :param target_info: Інформація про ціль (наприклад, координати або відхилення від центра).
        """
        pass


# 5. Module: Calculations
class AbstractCalculations(ABC):
    """
    Абстрактний клас для модуля розрахунків (математична обробка даних).
    Тут виконуються обчислення векторів руху, позицій і т.д.
    """

    @abstractmethod
    def compute_target_movement_vector(self, target_data: Dict[str, Any]) -> Dict[str, float]:
        """
        Розрахунок вектору руху цілі на основі вхідних даних (наприклад, траєкторія).
        
        :param target_data: Словник з даними про ціль (позиція, швидкість тощо).
        :return: Словник, що описує вектор руху, наприклад {'vx': ..., 'vy': ..., 'vz': ...}.
        """
        pass

    @abstractmethod
    def compute_interception_vector(self, agent_position: Dict[str, float], 
                                    target_position: Dict[str, float]) -> Dict[str, float]:
        """
        Розрахунок вектору руху агента для перехоплення цілі.
        
        :param agent_position: Словник з координатами агента {'x': ..., 'y': ..., 'z': ...}.
        :param target_position: Словник з координатами цілі {'x': ..., 'y': ..., 'z': ...}.
        :return: Вектор перехоплення у форматі {'dx': ..., 'dy': ..., 'dz': ...}.
        """
        pass

    @abstractmethod
    def compute_sound_direction(self, sound_data: Dict[str, Any]) -> float:
        """
        Розрахунок напряму цілі по шуму.
        Формує дані для модуля керування від модуля розпізнавання звуку.
        
        :param sound_data: Словник з даними про звук (наприклад, інтенсивності мікрофонів).
        :return: Кут напрямку (у градусах).
        """
        pass

    @abstractmethod
    def compute_target_position_from_camera(self, camera_data: Dict[str, Any]) -> Dict[str, float]:
        """
        Розрахунок розташування цілі по локальним даним з камери.
        Формує дані для модуля керування від модуля розпізнавання з камери.
        
        :param camera_data: Словник з результатами обробки кадру (наприклад, координати в кадрі).
        :return: Словник з реальними координатами цілі {'x': ..., 'y': ..., 'z': ...}.
        """
        pass

    @abstractmethod
    def compute_fused_target_position(self, own_camera_data: Dict[str, Any], 
                                      remote_data: Dict[str, Any]) -> Dict[str, float]:
        """
        Розрахунок розташування цілі за локальними та додатковими даними
        (камера поточного агента + отримані дані від іншого агента).
        
        :param own_camera_data: Локальні дані з камери поточного агента.
        :param remote_data: Додаткові дані, отримані від іншого агента.
        :return: Словник з координатами цілі в єдиній системі координат.
        """
        pass

    @abstractmethod
    def compute_centering_correction(self, camera_frame_data: Dict[str, Any]) -> Dict[str, float]:
        """
        Центрування напряму по цілі (фінальний елемент перехоплення).
        Формує дані для модуля керування від модуля розпізнавання з камери.
        
        :param camera_frame_data: Інформація про поточний кадр (відхилення цілі від центра кадру).
        :return: Словник з корекцією {'dx_percent': ..., 'dy_percent': ...}.
        """
        pass


# 6. Module: Communication
class AbstractCommunication(ABC):
    """
    Абстрактний клас для модуля комунікації між агентами.
    Відповідає за ініціалізацію мережі, надсилання повідомлень, реконект тощо.
    """

    @abstractmethod
    def initialize(self, port_name: str) -> None:
        """
        Ініціалізація модуля комунікації (передається назва порту/інтерфейсу).
        
        :param port_name: Назва або шлях до порту/з'єднання.
        """
        pass

    @abstractmethod
    def send_to_all(self, message: Any) -> None:
        """
        Відправити повідомлення всім підключеним вузлам.
        
        :param message: Будь-який тип даних, що буде серіалізовано та надіслано.
        """
        pass

    @abstractmethod
    def send_to_one(self, node_id: str, message: Any) -> None:
        """
        Відправити повідомлення одному вузлу.
        
        :param node_id: Ідентифікатор вузла-одержувача.
        :param message: Дані для передачі.
        """
        pass

    @abstractmethod
    def reset_node_map(self) -> None:
        """
        Скинути (очистити) карту доступних вузлів. Наприклад, при оновленні списку.
        """
        pass

    @abstractmethod
    def reconnect(self) -> None:
        """
        Спроба реконекту (повторного з'єднання) з мережею або вузлами.
        """
        pass

    @abstractmethod
    def subscribe_messages(self, callback: Callable[[Any, str], None]) -> None:
        """
        Підписка на вхідні повідомлення. Кожне повідомлення разом із sender_id
        передається у callback.
        
        :param callback: Функція з сигнатурою (message, sender_id).
        """
        pass


# 7. Module: Main (Coordinator)
class AbstractMainModule(ABC):
    """
    Абстрактний клас для головного модуля, що координує всі інші модулі.
    У майбутньому тут буде закладена основна логіка взаємодії між модулями.
    """

    @abstractmethod
    def initialize(self) -> None:
        """
        Ініціалізація головного модуля: створення інстансів кожного модуля і виклик
        їх методів initialize() там, де це потрібно.
        """
        pass

    @abstractmethod
    def run(self) -> None:
        """
        Запуск основного циклу або логіки взаємодії між модулями.
        Наприклад, підписки, обробка повідомлень, потік даних тощо.
        """
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """
        Завершення роботи: виклик методів close() для кожного модуля,
        звільнення ресурсів.
        """
        pass
