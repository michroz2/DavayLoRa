import os

if 'env' in locals() or 'env' in globals():
    Import("env")
else:
    from SCons.Script import DefaultEnvironment
    env = DefaultEnvironment()

def make_factory_bin(source, target, env):
    print("\n>>> Запуск объединения бинарных файлов для Веб-прошивальщика <<<")
    
    project_dir = env.subst("$PROJECT_DIR")
    build_dir = env.subst("$BUILD_DIR")
    dist_dir = os.path.join(project_dir, "web_installer")
    
    if not os.path.exists(dist_dir):
        os.makedirs(dist_dir)
        
    factory_bin = os.path.join(dist_dir, "factory.bin")
    mcu = env.BoardConfig().get("build.mcu", "esp32s3")
    
    python_exe = env.subst("$PYTHONEXE")
    esptool_dir = env.PioPlatform().get_package_dir("tool-esptoolpy")
    esptool_py = os.path.join(esptool_dir, "esptool.py")
    
    # Инициализация команды сборки
    cmd = f'"{python_exe}" "{esptool_py}" --chip {mcu} merge_bin -o "{factory_bin}" '
    
    # Добавляем системные разделы (bootloader, partitions, boot_app0)
    for offset, image in env.get("FLASH_EXTRA_IMAGES", []):
        cmd += f'{offset} "{env.subst(image)}" '
        
    # Явное указание адреса 0x10000 для основной прошивки ESP32-S3 во избежание конфликта с bootloader на 0x0
    firmware_offset = "0x10000"
    cmd += f'{firmware_offset} "{os.path.join(build_dir, "firmware.bin")}"'
    
    # Выполнение команды и перехват кода возврата
    result = env.Execute(cmd)
    
    if result == 0:
        print(f">>> УСПЕХ: Файл factory.bin сохранен в {factory_bin} <<<\n")
    else:
        print(f">>> ОШИБКА: Сборка factory.bin завершилась с кодом {result} <<<\n")

env.AddPostAction("$BUILD_DIR/firmware.bin", make_factory_bin)