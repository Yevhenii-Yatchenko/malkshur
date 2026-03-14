import argparse
import asyncio
import json
import os
import re
import sys
from datetime import datetime, timezone

from telethon import TelegramClient, utils


def clean_filename_text(text):
    """
    Очищает текст от эмодзи и форматирования Markdown для использования в имени файла.

    Args:
        text: Исходный текст

    Returns:
        Очищенный текст без эмодзи и форматирования
    """
    # Удаляем форматирование Markdown: **, __, *, _, `, ~, [], ()
    # Удаляем жирный текст **text**, курсив *text*, подчеркивание __text__
    text = re.sub(r"\*\*([^*]+)\*\*", r"\1", text)  # **text** -> text
    text = re.sub(r"__([^_]+)__", r"\1", text)  # __text__ -> text
    text = re.sub(r"\*([^*]+)\*", r"\1", text)  # *text* -> text
    text = re.sub(r"_([^_]+)_", r"\1", text)  # _text_ -> text
    text = re.sub(r"`([^`]+)`", r"\1", text)  # `text` -> text
    text = re.sub(r"~~([^~]+)~~", r"\1", text)  # ~~text~~ -> text
    text = re.sub(r"\[([^\]]+)\]\([^\)]+\)", r"\1", text)  # [text](url) -> text

    # Удаляем оставшиеся одиночные символы форматирования
    text = re.sub(r"[*_`~\[\]()]", "", text)

    # Удаляем эмодзи (Unicode символы в диапазонах эмодзи)
    # Это включает большинство эмодзи из разных блоков Unicode
    emoji_pattern = re.compile(
        "["
        "\U0001f600-\U0001f64f"  # Emoticons
        "\U0001f300-\U0001f5ff"  # Symbols & Pictographs
        "\U0001f680-\U0001f6ff"  # Transport & Map Symbols
        "\U0001f1e0-\U0001f1ff"  # Flags
        "\U00002702-\U000027b0"  # Dingbats
        "\U000024c2-\U0001f251"  # Enclosed characters
        "\U0001f900-\U0001f9ff"  # Supplemental Symbols and Pictographs
        "\U0001fa00-\U0001fa6f"  # Chess Symbols
        "\U0001fa70-\U0001faff"  # Symbols and Pictographs Extended-A
        "]+",
        flags=re.UNICODE,
    )
    text = emoji_pattern.sub("", text)

    # Удаляем множественные пробелы и обрезаем пробелы по краям
    text = re.sub(r"\s+", " ", text).strip()

    return text


# Функция загрузки конфига
def load_config(config_path):
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"ОШИБКА: Файл конфигурации не найден по пути: {config_path}")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"ОШИБКА: Неверный формат JSON в файле {config_path}")
        sys.exit(1)


# Функция парсинга даты
def parse_date(date_str, end_of_day=False):
    try:
        dt = datetime.strptime(date_str, "%Y-%m-%d")
        if end_of_day:
            dt = dt.replace(hour=23, minute=59, second=59)
        return dt.replace(tzinfo=timezone.utc)
    except ValueError:
        print(f"Ошибка: Неверный формат даты '{date_str}'. Используйте YYYY-MM-DD")
        sys.exit(1)


async def main(config, start_date, end_date):
    # Распаковываем конфиг
    api_id = config.get("api_id")
    api_hash = config.get("api_hash")
    base_path = config.get("download_path", "./downloads")
    target_chats = config.get("chats", [])
    keywords = config.get("keywords_regex", r"(?i)(шахед|shahed)")

    if not api_id or not api_hash:
        print("ОШИБКА: В конфиге не указаны api_id или api_hash")
        return

    session_name = "session_monitor"
    client = TelegramClient(session_name, api_id, api_hash)

    await client.start()

    print(f"\n=== ЗАПУСК ПОИСКА ===")
    print(f"Период: {start_date.date()} — {end_date.date()}")
    print(f"Всего чатов: {len(target_chats)}")
    print("=====================\n")

    stats = {
        "chats_checked": 0,
        "messages_found": 0,
        "images_saved": 0,
        "videos_saved": 0,
        "errors": 0,
    }

    pattern = re.compile(keywords)
    images_path = os.path.join(base_path, "images")
    video_path = os.path.join(base_path, "video")

    os.makedirs(images_path, exist_ok=True)
    os.makedirs(video_path, exist_ok=True)

    for chat in target_chats:
        stats["chats_checked"] += 1
        print(
            f"[{datetime.now().strftime('%H:%M:%S')}] Чат {stats['chats_checked']}/{len(target_chats)}: {chat}"
        )

        try:
            async for message in client.iter_messages(chat):
                if message.date > end_date:
                    continue
                if message.date < start_date:
                    print(f"   -> Лимит даты ({start_date.date()}). Чат завершен.")
                    break

                if not message.text:
                    continue

                if pattern.search(message.text):
                    stats["messages_found"] += 1

                    destination = None
                    file_type = None

                    # 1. Получаем правильное расширение файла
                    # utils.get_extension сам определит, это .jpg, .png, .mp4 или .mov
                    ext = utils.get_extension(message.media)

                    # Если расширение не определилось (редко), ставим заглушку
                    if not ext:
                        if message.photo:
                            ext = ".jpg"
                        elif message.video:
                            ext = ".mp4"
                        else:
                            ext = ""

                    # 2. Формируем чистое имя файла
                    # Заменяем спецсимволы в имени чата, чтобы не ломать пути Windows
                    safe_chat_name = (
                        str(chat).replace("/", "_").replace("\\", "_").replace(":", "_")
                    )

                    # Форматируем время сообщения для имени файла
                    # Формат: YYYY-MM-DD_HH-MM-SS
                    message_time = message.date.strftime("%Y-%m-%d_%H-%M-%S")

                    clean_text = clean_filename_text(message.text)
                    clean_text = clean_text[:40].replace("\n", " ")
                    text_to_filename = clean_text.replace(" ", "_")
                    text_to_filename = re.sub(r'[<>:"|?*\\/]', "_", text_to_filename)
                    text_to_filename = re.sub(r"_+", "_", text_to_filename).strip("_")

                    filename = (
                        f"{safe_chat_name}_{message_time}_{text_to_filename}{ext}"
                    )

                    if message.photo:
                        file_type = "IMAGE"
                        destination = os.path.join(images_path, filename)
                    elif message.video or (
                        message.document and "video" in message.document.mime_type
                    ):
                        file_type = "VIDEO"
                        destination = os.path.join(video_path, filename)

                    if destination:
                        # Для вывода используем оригинальный текст (первые 40 символов)
                        display_text = message.text[:40].replace("\n", " ")
                        print(f"   [НАЙДЕНО] {file_type} | '{display_text}...'")

                        try:
                            # force_document=False помогает сохранить фото именно как фото, а не файл
                            path = await client.download_media(
                                message, file=destination
                            )
                            if path:
                                print(f"   -> Сохранено: {os.path.basename(path)}")
                                if file_type == "IMAGE":
                                    stats["images_saved"] += 1
                                else:
                                    stats["videos_saved"] += 1
                        except Exception as e:
                            print(f"   -> Ошибка скачивания: {e}")
                            stats["errors"] += 1

        except ValueError:
            print(f"!!! Ошибка: Чат '{chat}' не найден.")
            stats["errors"] += 1
        except Exception as e:
            print(f"!!! Ошибка в чате {chat}: {e}")
            stats["errors"] += 1

    await client.disconnect()

    print("\n" + "=" * 30)
    print("       ИТОГОВАЯ СТАТИСТИКА       ")
    print("=" * 30)
    print(f"Чатов обработано: {stats['chats_checked']}")
    print(f"Сообщений найдено:{stats['messages_found']}")
    print(f"Скачано фото:     {stats['images_saved']}")
    print(f"Скачано видео:    {stats['videos_saved']}")
    print("=" * 30)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True)
    parser.add_argument("--start", type=str, required=True)
    parser.add_argument("--end", type=str, required=False)

    args = parser.parse_args()
    config_data = load_config(args.config)
    start_dt = parse_date(args.start)

    if args.end:
        end_dt = parse_date(args.end, end_of_day=True)
    else:
        end_dt = datetime.now(timezone.utc)

    asyncio.run(main(config_data, start_dt, end_dt))
