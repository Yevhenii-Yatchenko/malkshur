from typing import Callable, List, Dict, Union, Any, Deque
from abc import ABC, abstractmethod
import time
import random
from collections import deque, defaultdict
import statistics
import threading
from concurrent.futures import ThreadPoolExecutor

# Updated schematic representation of the current model:
# P1 -> T1 -> P2 -> F1 (parallel branches) -> P3, P5, P7 -> J1 -> P9

class Position:
    def __init__(self, name: str, data: Dict[str, Any] = None):
        self.name = name
        self.token = False
        self.data: Dict[str, Any] = {} if data is None else data.copy()
        self.entry_count = 0  # Number of times the position was entered
        self.total_time_active = 0  # Total active time
        self.start_time = None  # Time when the token entered

    def set_token(self, value: bool):
        print(f"[DEBUG] Position {self.name} Token set to {value}")
        if value:
            if self.token is False:  # Only set start time if it's a new token entry
                self.start_time = time.time()
                self.entry_count += 1
        else:
            if self.token and self.start_time:
                self.total_time_active += time.time() - self.start_time
                self.start_time = None
        self.token = value

    def update_data(self, data: Dict[str, Any]):
        self.data.update(data)

    def get_data_copy(self) -> Dict[str, Any]:
        return self.data.copy()

    def get_average_time_active(self):
        return self.total_time_active / self.entry_count if self.entry_count > 0 else 0
    
class Statistics:
    def __init__(self):
        self.transition_entries = defaultdict(int)
        self.transition_times = defaultdict(list)
        self.position_entries = defaultdict(int)

    def log_transition_entry(self, transition_name: str, start_time: float):
        self.transition_entries[transition_name] += 1
        time_spent = time.time() - start_time
        self.transition_times[transition_name].append(time_spent)

    def log_position_entry(self, position_name: str, position: Position):
        self.position_entries[position_name] += 1

    def get_statistics(self, positions: Dict[str, Position]):
        stats = {
            "transitions": {},
            "positions": {}
        }
        for transition, times in self.transition_times.items():
            stats["transitions"][transition] = {
                "entries": self.transition_entries[transition],
                "avg_time": statistics.mean(times) if times else 0,
                "min_time": min(times) if times else 0,
                "max_time": max(times) if times else 0,
                "variance": statistics.variance(times) if len(times) > 1 else 0
            }
        for position_name, position in positions.items():
            stats["positions"][position_name] = {
                "entries": position.entry_count,
                "avg_time": position.get_average_time_active(),
                "total_time": position.total_time_active
            }
        return stats

class TransitionBase(ABC):
    def __init__(self, name: str, input_positions: Union[str, List[str]], output_positions: Union[str, List[str]], action: Callable = None):
        self.name = name
        self.action = action
        self.input_positions = [input_positions] if isinstance(input_positions, str) else input_positions
        self.output_positions = [output_positions] if isinstance(output_positions, str) else output_positions

    def execute(self, positions: Dict[str, Position], statistics: Statistics) -> Union[str, List[str], None]:
        start_time = time.time()
        result = self._execute_logic(positions)
        if result is not None:
            statistics.log_transition_entry(self.name, start_time)
        return result

    @abstractmethod
    def _execute_logic(self, positions: Dict[str, Position]) -> Union[str, List[str], None]:
        pass

    def transfer_data(self, data: Dict[str, Any], positions: Dict[str, Position]):
        for output_name in self.output_positions:
            positions[output_name].update_data(data.copy())

class TTransition(TransitionBase):
    def __init__(self, name: str, input_position: str, output_position: str, action: Callable = None, delay: int = 0):
        super().__init__(name, [input_position], [output_position], action)
        self.delay = delay

    def _execute_logic(self, positions: Dict[str, Position]):
        if self.delay > 0:
            print(f"Executing {self.name} with delay {self.delay}s")
            time.sleep(self.delay)
        input_data = positions[self.input_positions[0]].get_data_copy()
        if self.action:
            self.action(input_data)
        self.transfer_data(input_data, positions)
        return self.output_positions[0]

class XTransition(TransitionBase):
    def __init__(self, name: str, input_position: str, output_positions: List[str], action: Callable = None, condition: Callable[[], int] = None):
        super().__init__(name, [input_position], output_positions, action)
        self.condition = condition if condition else lambda: 0

    def _execute_logic(self, positions: Dict[str, Position]):
        input_data = positions[self.input_positions[0]].get_data_copy()
        if self.action:
            self.action(input_data)

        chosen_index = self.condition()
        print(f"[DEBUG] {self.name} Condition chosen index: {chosen_index}")

        if 0 <= chosen_index < len(self.output_positions):
            selected_output = self.output_positions[chosen_index]
            print(f"[DEBUG] {self.name} Activated position {selected_output}")
            positions[selected_output].update_data(input_data)
            return selected_output
        print(f"[DEBUG] {self.name} condition failed, no transition")
        return None

class YTransition(TransitionBase):
    def __init__(self, name: str, input_positions: List[str], output_position: str, action: Callable = None, condition: Callable[[], bool] = None):
        super().__init__(name, input_positions, [output_position], action)
        self.condition = condition if condition else lambda: True

    def _execute_logic(self, positions: Dict[str, Position]):
        active_positions = sum(1 for pos in self.input_positions if positions[pos].token)
        print(f"[DEBUG] Checking {self.name}, Active inputs: {active_positions} -> {self.input_positions}")
        
        if active_positions > 0 and self.condition():
            input_data = positions[self.input_positions[0]].get_data_copy()
            if self.action:
                self.action(input_data)
            self.transfer_data(input_data, positions)
            print(f"[DEBUG] {self.name} Activated position {self.output_positions[0]}")
            return self.output_positions[0]
        print(f"[DEBUG] {self.name} conditions NOT met")
        return None

class FTransition(TransitionBase):
    def __init__(self, name: str, input_position: str, output_positions: List[str]):
        super().__init__(name, [input_position], output_positions)

    def _execute_logic(self, positions: Dict[str, Position]):
        input_data = positions[self.input_positions[0]].get_data_copy()
        self.transfer_data(input_data, positions)
        return self.output_positions

class JTransition(TransitionBase):
    def __init__(self, name: str, input_positions: List[str], output_position: str, action: Callable = None, data_merge_function: Callable[[Dict[str, Dict[str, Any]]], Dict[str, Any]] = None):
        super().__init__(name, input_positions, [output_position], action)
        self.triggered = False
        self.data_merge_function = data_merge_function
        self.lock = threading.Lock()  # Додаємо блокування для потокобезпечності

    def _execute_logic(self, positions: Dict[str, Position]):
        print(f"[DEBUG] Checking JTransition {self.name} conditions...")
        
        all_tokens = [positions[pos].token for pos in self.input_positions]
        print(f"[DEBUG] {self.name} input tokens: {dict(zip(self.input_positions, all_tokens))}")

        if all(all_tokens):
            with self.lock:  # 🔹 Лише виконання J-переходу блокується
                print(f"[DEBUG] All input positions have tokens. Executing {self.name}.")
                
                # Збираємо дані з усіх вхідних позицій
                data_dict = {pos: positions[pos].get_data_copy() for pos in self.input_positions}
                merged_data = self.data_merge_function(data_dict) if self.data_merge_function else data_dict[self.input_positions[0]]

                if self.action:
                    self.action(merged_data)

                self.transfer_data(merged_data, positions)

                # Скидаємо токени вхідних позицій
                for pos in self.input_positions:
                    positions[pos].set_token(False)

                print(f"[DEBUG] {self.name} transition completed, moving to {self.output_positions[0]}.")

                self.triggered = True
                return self.output_positions[0]

        print(f"[DEBUG] {self.name} conditions not met yet.")
        return None


class QueueTransitionBase(TransitionBase):
    name_of_transaction: str = "Queue Transition"

    def __init__(self, name: str, input_position: str, output_position: str, action: Callable = None, priority: bool = False):
        super().__init__(name, input_position, output_position, action)
        self.queue: Deque[Dict[str, Any]] = deque()
        self.priority = priority

    def prepare(self, positions: Dict[str, Position]):
        if positions[self.input_positions[0]].token:
            self.queue.append(positions[self.input_positions[0]].get_data_copy())
            positions[self.input_positions[0]].set_token(False)
            print(f"{self.name}: Data added to {self.name_of_transaction}")

    @abstractmethod
    def dequeue(self) -> Dict[str, Any]:
        pass

    def _execute_logic(self, positions: Dict[str, Position]):
        self.prepare(positions)
        if self.queue and positions[self.output_positions[0]].token is False:
            selected_data = self.dequeue()
            if self.action:
                self.action(selected_data)
            positions[self.output_positions[0]].update_data(selected_data)
            positions[self.output_positions[0]].set_token(True)
            print(f"{self.name}: Data moved from {self.name_of_transaction} to {self.output_positions[0]}")

class QFTransition(QueueTransitionBase):  # FIFO Queue
    name_of_transaction = "FIFO Queue"
    
    def dequeue(self) -> Dict[str, Any]:
        if self.queue:
            if self.priority:
                selected_data = max(self.queue, key=lambda d: d.get("priority", 0))
                self.queue.remove(selected_data)
                return selected_data
            return self.queue.popleft()
        return {}

class QLTransition(QueueTransitionBase):  # LIFO Queue
    name_of_transaction = "LIFO Queue"
    
    def dequeue(self) -> Dict[str, Any]:
        if self.queue:
            if self.priority:
                selected_data = max(self.queue, key=lambda d: d.get("priority", 0))
                self.queue.remove(selected_data)
                return selected_data
            return self.queue.pop()
        return {}

class E_Network:
    def __init__(self, model_id):
        self.model_id = model_id  # Зберігаємо ID
        self.positions: Dict[str, Position] = {}
        self.transitions: Dict[str, TransitionBase] = {}
        self.start_position: str = ""
        self.finished = False
        self.end_position: str = ""
        self.statistics = Statistics()
        self.executor = ThreadPoolExecutor(max_workers=10)  # Обмеження кількості потоків

    def log(self, message):
        """Форматуємо лог з ID моделі"""
        print(f"[MODEL-{self.model_id}] {message}")

    def add_position(self, name: str, data: Dict[str, Any] = None):
        self.positions[name] = Position(name, data)

    def set_start_position(self, name: str):
        if name in self.positions:
            self.start_position = name
            self.positions[name].set_token(True)
            
    def set_token_to_position(self, name: str, need_process_now: bool = False):
        if name in self.positions:
            self.positions[name].set_token(True)
            self.log(f"[INFO] Token set externally for position: {name}")
            if need_process_now:
                self.log(f"[INFO] Triggering immediate processing for position: {name}")
                self.process_position(name)

    def set_end_position(self, name: str):
        if name in self.positions:
            self.end_position = name

    def add_transition(self, transition: TransitionBase):
        self.transitions[transition.name] = transition

    def execute_transition(self, transition: TransitionBase):
        """ Виконує перехід та передає дані у наступні позиції """
        self.log(f"[DEBUG] Executing transition {transition.name}, input positions: "
            f"{[pos for pos in transition.input_positions]} -> output positions: {transition.output_positions}")

        next_positions = transition.execute(self.positions, self.statistics)

        if isinstance(next_positions, list):
            if isinstance(transition, FTransition): # 🔹 FTransition обробляється паралельно через executor
                futures = []
                for pos in next_positions:
                    self.positions[pos].set_token(True)
                    self.log(f"[DEBUG] Transition {transition.name} activated position {pos}")

                    future = self.executor.submit(self.process_position, pos)  # Запускаємо потоки
                    futures.append(future)

                # 🔸 Очікуємо завершення всіх гілок
                for future in futures:
                    future.result()
            else:
                # 🔹 Всі інші обробляються послідовно, БЕЗ executor
                for pos in next_positions:
                    self.positions[pos].set_token(True)
                    self.log(f"[DEBUG] Transition {transition.name} activated position {pos}")
                    self.process_position(pos)  # Обробляємо послідовно для всіх інших переходів

        elif isinstance(next_positions, str):
            self.positions[next_positions].set_token(True)
            self.log(f"[DEBUG] Transition {transition.name} activated position {next_positions}")
            self.process_position(next_positions)  # Обробляємо одразу

            # ✅ Завершення моделі, якщо досягнута кінцева позиція
            if next_positions == self.end_position:
                self.log(f"[INFO] End position {self.end_position} reached. Finishing simulation.")
                self.finished = True

    def process_position(self, position_name):
        """ Обробляє позицію одразу після її активації, без очікування executor """
        self.log(f"Processing position: {position_name}")

        for transition_name, transition in self.transitions.items():
            if position_name in transition.input_positions:
                # 🔹 Просто викликаємо напряму (без executor.submit)
                self.execute_transition(transition)


    def run(self):
        """ Основний цикл роботи мережі з реактивним запуском переходів """
        self.log(f"Processing position: {self.start_position}")
        self.positions[self.start_position].set_token(True)

        # 🔹 Замість чекаючого циклу запускаємо процеси одразу
        self.process_position(self.start_position)

        # ⏳ Чекаємо завершення, поки не досягнута кінцева позиція
        while not self.finished:
            time.sleep(0.5)  # невелика пауза, щоб не грузити CPU

        self.log("Simulation completed.")
        self.log("Collected Statistics:")
        stats = self.statistics.get_statistics(self.positions)
        for category, category_data in stats.items():
            self.log(f"{category.capitalize()} Statistics:")
            for item, data in category_data.items():
                self.log(f"{item}: {data}")


def setup_model(model_id):
    model = E_Network(model_id)
    model.add_position("P1")
    model.add_position("P2")
    model.add_position("P3")
    model.add_position("P4")
    model.add_position("P5")
    model.add_position("P6")
    model.add_position("P7")
    model.add_position("P8")
    model.add_position("P9")
    model.add_position("P10")
    model.add_position("P11")
    model.add_position("P12")
    model.add_position("P13")
    model.add_position("P14")
    model.add_position("P15")
    model.add_position("P16")
    model.add_position("P17")

    j1 = JTransition("J1", ["P1", "P2"], "P3")
    j2 = JTransition("J2", ["P16", "P7"], "P17")
    x1 = XTransition("X1", "P3", ["P4", "P12"], condition=lambda: condition_x1())
    y1 = YTransition("Y1", ["P4", "P5"], "P6")
    x2 = XTransition("X2", "P6", ["P7", "P8"], condition=lambda: condition_x2(2))
    t1 = TTransition("T1", "P17", "P9", delay=1)
    t2 = TTransition("T2", "P9", "P10", delay=1)
    t3 = TTransition("T3", "P10", "P11", delay=1)
    t4 = TTransition("T4", "P8", "P5", delay=1, action=action_increment_repeat)
    f1 = FTransition("F1", "P12", ["P13", "P2"])
    t5 = TTransition("T5", "P13", "P14", delay=5)
    y2 = YTransition("Y2", ["P11", "P14"], "P15")

    model.add_transition(j1)
    model.add_transition(j2)
    model.add_transition(x1)
    model.add_transition(y1)
    model.add_transition(x2)
    model.add_transition(t1)
    model.add_transition(t2)
    model.add_transition(t3)
    model.add_transition(t4)
    model.add_transition(f1)
    model.add_transition(t5)
    model.add_transition(y2)

    model.set_start_position("P1")
    model.set_token_to_position("P2")
    model.set_end_position("P15")
    return model

# Actions for individual transitions

repeat_counter = 0  # Глобальний лічильник циклів

def condition_x1() -> int:
    return 0

# def condition_x2() -> int:
#     return 1

def condition_x_random(output_count: int) -> int:
    """
    Condition for XTransition: randomly selects one of the available outputs.
    """
    return random.randint(0, output_count - 1)  # Випадковий вибір гілки

def condition_x2(output_count: int) -> int:
    global repeat_counter
    if repeat_counter >= 3:
        return 0  # Змінюємо напрям після n ітерацій
    return 1  # Спочатку йдемо в сторону циклу

def action_increment_repeat(data: Dict[str, Any]):
    global repeat_counter
    repeat_counter += 1
    print(f"[DEBUG] 🔁 repeat_counter incremented to {repeat_counter}")

def run_model_instance(model_id):
    """Запускає новий екземпляр моделі з унікальним ID"""
    model = setup_model(model_id)  # Передаємо ID моделі

    # Через n секунд активуємо позицію P16
    def trigger_external_event():
        model.log("[INFO] 🔔 Зовнішній тригер активує P16")
        model.set_token_to_position("P16", need_process_now=True)

    timer = threading.Timer(5, trigger_external_event)
    timer.start()

    model.run()

run_model_instance(1)

# Головний цикл на 10 секунд
# start_time = time.time()
# model_counter = 0  # Лічильник моделей

# while time.time() - start_time < 2:
#     model_counter += 1  # Інкрементуємо ID для нової моделі
#     print(f"\n[INFO] Запуск моделі #{model_counter} в {time.strftime('%H:%M:%S')}")

#     # Запускаємо модель у новому потоці
#     thread = threading.Thread(target=run_model_instance, args=(model_counter,))
#     thread.start()

#     time.sleep(1)  # Чекаємо 1 секунду перед запуском наступної моделі

def action_t1(data):
    data["newParam"] = "Some imprtant value"

    data["A"] += 1
    data["B"] += 1
    data["C"] += 1
    print(f"T1 executed with data: {data}")

def action_t2(data):
    data["A"] += 1
    print(f"{time.time()} T2 executed with data: {data}")

def action_t3(data):
    data["B"] += 1
    print(f"{time.time()} T3 executed with data: {data}")

def action_t4(data):
    data["C"] += 1
    print(f"{time.time()} T4 executed with data: {data}")

def action_j1(data):
    data["j_passed"] = True
    print(f"J1 executed, j_passed set to True with data: {data}")

def custom_data_merge(data_dict: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "A": data_dict.get("P4", {}).get("A", 0),
        "B": data_dict.get("P6", {}).get("B", 0),
        "C": data_dict.get("P8", {}).get("C", 0)
    }
