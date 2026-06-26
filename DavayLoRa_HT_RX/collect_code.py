# collect_code.py для сборки кода проекта DavayLoRa_HT_RX в Markdown файл

import os

# Настройки
PROJECT_NAME = "DavayLoRa_HT_RX"
VERSION = "1.72"
OUTPUT_FILE = f"DavayLoRa_HT_RX_Snapshot_v{VERSION}.md"
EXTENSIONS = ('.h', '.cpp', '.hpp', '.c')
EXCLUDE_DIRS = {'.pio', '.git', 'bin', 'lib'} # Исключаем папки сборки

def collect_code():
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as outfile:
        outfile.write(f"# Project: {PROJECT_NAME} Snapshot V{VERSION}\n\n")
        
        for root, dirs, files in os.walk('.'):
            # Пропускаем исключенные папки
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            
            for file in files:
                if file.endswith(EXTENSIONS):
                    file_path = os.path.join(root, file)
                    outfile.write(f"## File: {file_path}\n")
                    outfile.write("```cpp\n")
                    try:
                        with open(file_path, 'r', encoding='utf-8') as infile:
                            outfile.write(infile.read())
                    except Exception as e:
                        outfile.write(f"// Error reading file: {e}\n")
                    outfile.write("\n```\n\n---\n\n")

    print(f"Готово! Весь код собран в файл: {OUTPUT_FILE}")

if __name__ == "__main__":
    collect_code()