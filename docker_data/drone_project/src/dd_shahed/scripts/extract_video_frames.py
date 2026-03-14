import argparse
import os
import shutil
import sys
from pathlib import Path

import cv2


def get_video_files(directory):
    """
    Получает список всех видеофайлов в указанной директории.

    Args:
        directory: Путь к директории с видео

    Returns:
        Список путей к видеофайлам
    """
    video_extensions = {".mp4", ".avi", ".mov", ".mkv", ".flv", ".wmv", ".webm", ".m4v"}
    video_files = []

    for root, dirs, files in os.walk(directory):
        for file in files:
            if Path(file).suffix.lower() in video_extensions:
                video_files.append(os.path.join(root, file))

    return video_files


def extract_frames(video_path, output_frames_dir, video_name_base):
    """
    Извлекает все кадры из видео и сохраняет их в указанную директорию.

    Args:
        video_path: Путь к видеофайлу
        output_frames_dir: Директория для сохранения кадров
        video_name_base: Базовое имя видео (без расширения) для именования кадров

    Returns:
        Количество извлеченных кадров
    """
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"   -> ОШИБКА: Не удалось открыть видео {video_path}")
        return 0

    frame_count = 0
    frame_number = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_number += 1
        frame_filename = f"{video_name_base}_{frame_number:06d}.jpg"
        frame_path = os.path.join(output_frames_dir, frame_filename)

        cv2.imwrite(frame_path, frame)
        frame_count += 1

    cap.release()
    return frame_count


def process_video(video_path, output_base_path):
    """
    Обрабатывает одно видео: копирует оригинал и извлекает кадры.

    Args:
        video_path: Путь к видеофайлу
        output_base_path: Базовый путь для сохранения результатов

    Returns:
        Кортеж (успешно ли обработано, количество кадров)
    """
    video_path_obj = Path(video_path)
    video_name = video_path_obj.stem  # Имя без расширения
    video_extension = video_path_obj.suffix  # Расширение

    # Создаем папку для этого видео
    video_output_dir = os.path.join(output_base_path, video_name)
    os.makedirs(video_output_dir, exist_ok=True)

    # Копируем оригинал видео
    original_destination = os.path.join(video_output_dir, video_path_obj.name)
    try:
        shutil.copy2(video_path, original_destination)
        print(f"   -> Оригинал скопирован: {video_path_obj.name}")
    except Exception as e:
        print(f"   -> ОШИБКА при копировании оригинала: {e}")
        return False, 0

    # Создаем папку для кадров
    frames_dir = os.path.join(video_output_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    # Извлекаем кадры
    print(f"   -> Извлечение кадров...")
    frames_count = extract_frames(video_path, frames_dir, video_name)

    if frames_count > 0:
        print(f"   -> Извлечено кадров: {frames_count}")
        return True, frames_count
    else:
        print(f"   -> ОШИБКА: Не удалось извлечь кадры")
        return False, 0


def main(input_videos_path, output_path):
    """
    Основная функция обработки всех видео в указанной директории.

    Args:
        input_videos_path: Путь к папке с видео
        output_path: Путь для сохранения результатов
    """
    if not os.path.isdir(input_videos_path):
        print(f"ОШИБКА: Директория с видео не найдена: {input_videos_path}")
        sys.exit(1)

    # Создаем выходную директорию, если её нет
    os.makedirs(output_path, exist_ok=True)

    # Получаем список всех видеофайлов
    video_files = get_video_files(input_videos_path)

    if not video_files:
        print(f"В директории {input_videos_path} не найдено видеофайлов")
        return

    print(f"\n=== ЗАПУСК ОБРАБОТКИ ВИДЕО ===")
    print(f"Входная директория: {input_videos_path}")
    print(f"Выходная директория: {output_path}")
    print(f"Найдено видеофайлов: {len(video_files)}")
    print("=" * 35 + "\n")

    stats = {
        "videos_processed": 0,
        "videos_failed": 0,
        "total_frames": 0,
    }

    for i, video_path in enumerate(video_files, 1):
        video_name = os.path.basename(video_path)
        print(f"[{i}/{len(video_files)}] Обработка: {video_name}")

        success, frames_count = process_video(video_path, output_path)

        if success:
            stats["videos_processed"] += 1
            stats["total_frames"] += frames_count
        else:
            stats["videos_failed"] += 1

        print()  # Пустая строка для читаемости

    print("=" * 35)
    print("       ИТОГОВАЯ СТАТИСТИКА       ")
    print("=" * 35)
    print(f"Видео обработано:     {stats['videos_processed']}")
    print(f"Видео с ошибками:     {stats['videos_failed']}")
    print(f"Всего кадров извлечено: {stats['total_frames']}")
    print("=" * 35)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Извлекает кадры из видеофайлов и сохраняет их в структурированном виде"
    )
    parser.add_argument(
        "--input",
        type=str,
        required=True,
        help="Путь к папке с видеофайлами",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Путь для сохранения результатов (создастся структура: video_name/video.mp4 и video_name/frames/)",
    )

    args = parser.parse_args()

    main(args.input, args.output)
