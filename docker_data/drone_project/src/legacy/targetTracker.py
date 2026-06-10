class TargetTracker:
    def __init__(self, image_width, image_height, center_tolerance=50):
        self.image_width = image_width
        self.image_height = image_height
        self.center_tolerance = center_tolerance

    def analyze_target_position(self, data_dict, roll, pitch, throtle, yaw):
        # Розшифровка координат
        center_x, center_y = data_dict["coordinates"]["center"]
        width = data_dict["image_info"]["width"]
        height = data_dict["image_info"]["height"]

        # Обчислюємо відхилення від центру
        center_offset_x = center_x - width / 2
        center_offset_y = center_y - height / 2

        # Ініціалізуємо базові значення каналів
        _roll = roll  # Канал 1 (ліво/право)
        _pitch = pitch  # Канал 2 (вперед/назад)
        _throttle = throtle  # Канал 3 (вгору/вниз)
        _yaw = yaw  # Канал 4 (поворот)

        # Налаштування меж чутливості
        adjustment_step = 50

        # Коригуємо roll (рух ліворуч/праворуч)
        if abs(center_offset_x) > self.center_tolerance:
            if center_offset_x > 0:
                _roll += adjustment_step  # Рух праворуч
                _yaw += adjustment_step   # Поворот праворуч
            else:
                _roll -= adjustment_step  # Рух ліворуч
                _yaw -= adjustment_step   # Поворот ліворуч

        # Коригуємо throttle (рух вгору/вниз)
        if abs(center_offset_y) > self.center_tolerance:
            if center_offset_y > 0:
                _throttle += adjustment_step  # Рух вниз
            else:
                _throttle -= adjustment_step  # Рух вгору

        # Проста логіка для руху вперед/назад залежно від розміру цілі
        target_width = data_dict["coordinates"]["width"]
        # Якщо ширина більша за певний поріг — рухаємось назад, інакше — вперед
        if target_width > width * 0.5:
            _pitch -= adjustment_step  # Рух назад
        else:
            _pitch += adjustment_step  # Рух вперед

        # Повертаємо рішення щодо каналів
        return _roll, _pitch, _throttle, _yaw
