import time
import threading
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
import secrets
import string
import json
import queue
from src.logger import get_logger


class Communicator:
    def __init__(self):
        self.logger = get_logger("communicator", "logs/communicator.log", log_level="INFO")
        self.device = None
        self.received_message_ids = set()
        self.message_count = 0
        self.clear_list_after = 15
        self.current_discovered_devices = set()
        self.devices_to_send = {}
        self.timer_flag = False
        self.discovery_thread = threading.Thread(target=self.run_device_discovery, daemon=True)
        self.timer_thread = threading.Thread(target=self.run_timer, daemon=True)
        self.message_queue = queue.Queue()
        self.status_discovery = 0
        self.message_parts = {}

    def generate_message_id(self):
        characters = string.ascii_letters + string.digits
        return ''.join(secrets.choice(characters) for _ in range(5))

    def forward_message(self, full_message, source_device, base_message_id):
        try:
            chunk_size = 10
            num_parts = (len(full_message) + chunk_size - 1) // chunk_size

            remote_devices = self.current_discovered_devices

            for i in range(num_parts):
                start_index = i * chunk_size
                end_index = min((i + 1) * chunk_size, len(full_message))
                message_part = full_message[start_index:end_index]

                message_id = f"{base_message_id}{i + 1}"

                data = {
                    "id": message_id,
                    "first": self.device.get_node_id(),
                    "msg": message_part,
                    "l": 1 if i + 1 == num_parts else 0
                }
                message_send = json.dumps(data)

                for remote_device in remote_devices:
                    if remote_device.get_64bit_addr() != source_device.get_64bit_addr():
                        self.logger.debug(f"Forwarding part {i + 1}/{num_parts}: {message_send}")
                        self.device.send_data_async(remote_device, message_send)

        except Exception as e:
            self.logger.error(f"Forwarding error: {str(e)}")

    def message_callback(self, message):
        self.logger.info(f'message_callback {message}')
        if message.remote_device is None:
            self.logger.warning(f"Received message without remote device information: {message.data.decode()}")
            return

        source_device = message.remote_device
        if not message.data:
            return

        message_data = message.data.decode()
        try:
            # Пытаемся декодировать сообщение как JSON
            data = json.loads(message_data)
            message_id = data.get("id")
            sender_name = data.get("first")
            received_message_part = data.get("msg")
            is_last_part = data.get("l", 0)

            if message_id:
                base_id_length = 5
                base_message_id = message_id[:base_id_length]
                part_number = int(message_id[base_id_length:])

                # Если впервые видим этот ID, создаем запись для него
                if base_message_id not in self.message_parts:
                    self.message_parts[base_message_id] = {"parts": {}, "total_parts": 0, "first_sender": sender_name}
                else:
                    # Обновляем first_sender, если он еще не был установлен
                    if "first_sender" not in self.message_parts[base_message_id]:
                        self.message_parts[base_message_id]["first_sender"] = sender_name


                # Сохраняем текущую часть сообщения
                self.message_parts[base_message_id]["parts"][part_number] = received_message_part

                # Если это последняя часть, сохраняем количество частей
                if is_last_part:
                    self.message_parts[base_message_id]["total_parts"] = part_number

                # Проверяем, получены ли все части сообщения
                total_parts = self.message_parts[base_message_id]["total_parts"]
                if total_parts and len(self.message_parts[base_message_id]["parts"]) == total_parts:
                    # Собираем сообщение из всех частей
                    full_message = ''.join(
                        self.message_parts[base_message_id]["parts"][i] for i in range(1, total_parts + 1)
                    )

                    # Формируем окончательный JSON-объект для полного сообщения
                    full_message_json = {
                        "first": self.message_parts[base_message_id]["first_sender"],
                        "from": source_device.get_node_id(),
                        "msg": full_message
                    }

                    # Удаляем ID из записи для предотвращения дубликатов
                    del self.message_parts[base_message_id]

                    self.message_queue.put(json.dumps(full_message_json))

                    # Пробрасываем сообщение дальше
                    self.forward_message(full_message, source_device, base_message_id)

        except json.JSONDecodeError:
            self.logger.error(f"Error decoding JSON message: {message_data}")

    def callback_discover(self):
        xbee_network = self.device.get_network()
        xbee_network.set_discovery_timeout(5) # було 3.2
        try:
            def callback_device_discovered(remote):
                pass

            def callback_discovery_finished(status):
                if status == NetworkDiscoveryStatus.SUCCESS:
                    self.current_discovered_devices = self.device.get_network().get_devices()
                    self.devices_to_send = {key: value for key, value in self.devices_to_send.items() if value["device"] in self.current_discovered_devices}
                    self.status_discovery = 1
                else:
                    #print("Discovery error:", status.description)
                    pass

            xbee_network.add_device_discovered_callback(callback_device_discovered)
            xbee_network.add_discovery_process_finished_callback(callback_discovery_finished)
        except Exception as e:
            self.logger.error(f"Connection error: {str(e)}")

    def run_device_discovery(self):
        while self.device and self.device.is_open():
            xbee_network = self.device.get_network()
            if self.timer_flag:
                xbee_network.clear()
                self.timer_flag = False
            xbee_network.start_discovery_process()
            while xbee_network.is_discovery_running():
                time.sleep(0.1)

    def run_timer(self):
        while self.device and self.device.is_open():
            time.sleep(32)
            self.timer_flag = True

    def start_device_discovery(self):
        self.discovery_thread.start()

    def start_timer(self):
        self.timer_thread.start()

    def connect(self, device_name):
        if self.device is not None:
            self.logger.warning("Device already connected")
            return

        port = device_name
        self.device = DigiMeshDevice(port, 57600)

        try:
            self.device.open()
            self.device.add_data_received_callback(self.message_callback)
        except Exception as e:
            self.logger.error(f"Connection error: {str(e)}")

        self.callback_discover()
        self.start_device_discovery()
        self.start_timer()

    def send(self, message):
        if self.device is None:
            self.logger.error("No device connected")
            return

        self.message_count += 1
        if self.message_count >= self.clear_list_after:
            self.received_message_ids.clear()
            self.message_count = 0

        try:
            remote_devices = self.current_discovered_devices
            base_message_id = self.generate_message_id()  # Базовый ID сообщения
            self.received_message_ids.add(base_message_id)

            # Разделение сообщения на части по 10 символа
            message_parts = [message[i:i+10] for i in range(0, len(message), 10)]
            total_parts = len(message_parts)

            # Отправка каждой части сообщения
            for part_num, message_part in enumerate(message_parts, start=1):
                # Формируем уникальный ID для каждой части сообщения
                message_id = f"{base_message_id}{part_num}"

                # Подготовка данных для отправки
                data = {
                    "id": message_id,  # Уникальный ID для каждой части
                    "first": self.device.get_node_id(),
                    "msg": message_part,
                    "l": 1 if part_num == total_parts else 0  # Метка последней части
                }

                message_send = json.dumps(data)

                # Отправляем сообщение на все удаленные устройства
                for i, remote_device in enumerate(remote_devices):
                    self.device.send_data_async(remote_device, message_send)

        except Exception as e:
            self.logger.error(f"Send error: {str(e)}")

    def send_single(self, remote_address, message):
        if self.device is None:
            self.logger.error("No device connected")
            return

        self.message_count += 1
        if self.message_count >= self.clear_list_after:
            self.received_message_ids.clear()
            self.message_count = 0

        try:
            remote_devices = self.current_discovered_devices
            message_id = self.generate_message_id()  # Генерируем ID для сообщения
            self.received_message_ids.add(message_id)

            # Разделение сообщения на части по 10 символов
            message_parts = [message[i:i+10] for i in range(0, len(message), 10)]
            total_parts = len(message_parts)

            # Поиск целевого устройства
            remote_device = None
            for dev in remote_devices:
                if dev.get_node_id() == remote_address:
                    remote_device = dev
                    break

            if remote_device is None:
                self.logger.error(f"Device not found with address: {remote_address}")
                return

            # Отправка каждой части сообщения
            for part_num, message_part in enumerate(message_parts, start=1):
                # Формируем уникальный ID для каждой части сообщения
                part_id = f"{message_id}{part_num}"

                # Подготовка данных для отправки
                data = {
                    "id": part_id,  # Уникальный ID для каждой части
                    "first": self.device.get_node_id(),
                    "msg": message_part,
                    "l": 1 if part_num == total_parts else 0  # Метка последней части
                }

                message_send = json.dumps(data)
                self.logger.debug(message_send)

                # Отправляем сообщение на целевое устройство
                self.device.send_data_async(remote_device, message_send)
                self.logger.info(f"Part {part_num} sent to: {remote_device.get_node_id()}")

        except Exception as e:
            self.logger.error(f"Send error: {str(e)}")

    def list_devices(self):
        return [dev.get_node_id() for dev in self.current_discovered_devices]

    def refresh(self):
        self.timer_flag = True
