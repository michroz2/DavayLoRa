/**
 * @file webpage.h
 * @version 1.74
 * @brief HTML-интерфейс для Captive Portal (TX)
 */

 #ifndef WEBPAGE_H
 #define WEBPAGE_H
 
 #include <Arduino.h>
 
 const char index_html[] PROGMEM = R"rawliteral(
 <!DOCTYPE HTML>
 <html lang="ru">
 <head>
     <meta charset="UTF-8">
     <meta name="viewport" content="width=device-width, initial-scale=1">
     <title>DavayLoRa Config</title>
     <style>
       body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; padding: 15px; max-width: 600px; margin: 0 auto; }
       h2 { text-align: center; color: #4CAF50; margin-bottom: 10px; font-size: 24px;}
       .timer { text-align: center; color: #ff9800; font-size: 18px; margin-bottom: 20px; font-weight: bold; }
       
       fieldset, details { border: 1px solid #4CAF50; border-radius: 8px; margin-bottom: 25px; padding: 20px; background: #1e1e1e; }
       legend, summary { color: #4CAF50; font-weight: bold; font-size: 18px; padding: 0 10px; }
       summary { cursor: pointer; user-select: none; outline: none; }
       details[open] summary { margin-bottom: 15px; }
 
       label { display: block; margin-top: 15px; margin-bottom: 5px; color: #cccccc; font-size: 14px; }
       input[type="number"] { width: 100%; padding: 12px; border-radius: 6px; border: 1px solid #444; background: #2a2a2a; color: #fff; box-sizing: border-box; font-size: 16px; transition: border 0.3s; }
       input[type="number"]:focus { border-color: #4CAF50; outline: none; }
       .checkbox-container { display: flex; align-items: center; margin-top: 15px; background: #2a2a2a; padding: 12px; border-radius: 6px; }
       input[type="checkbox"] { transform: scale(1.5); margin: 0 10px 0 5px; accent-color: #4CAF50; }
       .checkbox-container span { font-size: 15px; color: #fff; }
       .buttons-container { display: flex; flex-direction: column; gap: 15px; margin-bottom: 30px; margin-top: 10px; }
       input[type="submit"] { background-color: #4CAF50; color: white; padding: 16px; border: none; border-radius: 6px; cursor: pointer; width: 100%; font-size: 18px; font-weight: bold; transition: background 0.3s; box-shadow: 0 4px 6px rgba(0,0,0,0.3); box-sizing: border-box; }
       input[type="submit"]:hover { background-color: #45a049; }
       .cancel-btn { background-color: #f44336; color: white; text-decoration: none; padding: 16px; display: flex; align-items: center; justify-content: center; border-radius: 6px; width: 100%; font-size: 18px; font-weight: bold; transition: background 0.3s; box-shadow: 0 4px 6px rgba(0,0,0,0.3); box-sizing: border-box; }
       .cancel-btn:hover { background-color: #d32f2f; }
       .default-btn { background-color: #2196F3; color: white; padding: 16px; border: none; border-radius: 6px; cursor: pointer; width: 100%; font-size: 18px; font-weight: bold; transition: background 0.3s; box-shadow: 0 4px 6px rgba(0,0,0,0.3); box-sizing: border-box; }
       .default-btn:hover { background-color: #1976D2; }
     </style>
     <script>
       let timeLeft = %TIME_LEFT%;
       function updateTimer() {
         if (timeLeft <= 0) {
           document.getElementById('timeDisplay').innerText = "00:00";
           alert("⏳ Время конфигурации истекло!");
           window.location.reload();
           return;
         }
         let m = Math.floor(timeLeft / 60).toString().padStart(2, '0');
         let s = (timeLeft % 60).toString().padStart(2, '0');
         document.getElementById('timeDisplay').innerText = m + ":" + s;
         timeLeft--;
         setTimeout(updateTimer, 1000);
       }
       window.onload = updateTimer;
 
       function setDefaults() {
         if (!confirm("Установить значения по умолчанию?\n(Рабочий канал изменен не будет)")) return;
 
         // Чекбоксы
         document.querySelector('input[name="measurebattery"]').checked = true;
         document.querySelector('input[name="dynamicPower"]').checked = true;
         document.querySelector('input[name="rxEnableBigLed"]').checked = true;
         document.querySelector('input[name="rxEnableBuzzer"]').checked = false;
 
         // Текстовые поля (Пульт и Приемник)
         document.querySelector('input[name="pwmledBrightness"]').value = "35";
         document.querySelector('input[name="fbledBrightness"]').value = "255";
         document.querySelector('input[name="rxPwmledBrightness"]').value = "35";
         document.querySelector('input[name="rxBuzzerVolume"]').value = "255";
 
         // Advanced настройки (Мощности и Таймауты)
         document.querySelector('input[name="maxPower"]').value = "20";
         document.querySelector('input[name="minPower"]').value = "-9";
         document.querySelector('input[name="servicePower"]').value = "18";
         document.querySelector('input[name="batteryPeriod"]').value = "300000";
         document.querySelector('input[name="wakeUpHoldTime"]').value = "2000";
         document.querySelector('input[name="wakeUpReleaseWindow"]').value = "2000";
         document.querySelector('input[name="stuckSleepTime"]').value = "10000";
         document.querySelector('input[name="configTimeout"]').value = "600000";
         document.querySelector('input[name="sleepLedDuration"]').value = "4000";
         document.querySelector('input[name="pingTimeout"]').value = "3000";
         document.querySelector('input[name="bigTimeout"]').value = "3600000";
         document.querySelector('input[name="execTimeout"]').value = "30000";
         document.querySelector('input[name="rxCutoffTime"]').value = "10000";
         document.querySelector('input[name="pingTimeoutRX"]').value = "10000";
       }      
     </script>
 </head>
 <body>
     <h2>⚙️ Настройка DavayLoRa</h2>
     <div class="timer">⏳ До автовыхода: <span id="timeDisplay">--:--</span></div>
     <div style="text-align: center; color: #4CAF50; font-size: 16px; margin-bottom: 20px; font-weight: bold;">📡 Рабочая частота: %FREQ_MHZ% МГц</div>
 
     <form action="/save" method="POST">
       <fieldset>
         <legend>🌍 Связь</legend>
         <label title="434-450 МГц, шаг 0.2">Рабочий канал (0-80):</label>
         <input type="number" name="workAddress" value="%ADDR%" min="0" max="80" required>
         
       </fieldset>
 
       <fieldset>
         <legend>🔘 Пульт</legend>
 
         <label title="По умолчанию: 35">Яркость индикатора (0-255):</label>
         <input type="number" name="pwmledBrightness" value="%TX_BIG_LED%" min="0" max="255" required>
 
         <label title="По умолчанию: 255">Яркость кнопки (0-255):</label>
         <input type="number" name="fbledBrightness" value="%TX_FB_LED%" min="0" max="255" required>
 
         <label title="По умолчанию: 3 сек">Интервал проверки связи (мс):</label>
         <input type="number" name="pingTimeout" value="%TX_PING%" min="1000" step="100" required>
         
 
       </fieldset>
 
       <fieldset>
         <legend>💡🔔 Приёмник</legend>
         <div class="checkbox-container">
           <input type="checkbox" name="rxEnableBigLed" value="1" %RX_BIG_EN% title="По умолчанию: Включено">
           <span>СВЕТ</span>
         </div>
         <label title="По умолчанию: 35">Яркость светодиода (0-255):</label>
         <input type="number" name="rxPwmledBrightness" value="%RX_BIG_LED%" min="0" max="255" required>
         
         <div class="checkbox-container">
           <input type="checkbox" name="rxEnableBuzzer" value="1" %RX_BUZ_EN% title="По умолчанию: Выключено">
           <span>ВИБАТОР</span>
         </div>
         <label title="По умолчанию: 255">Сила вибратора (0-255):</label>
         <input type="number" name="rxBuzzerVolume" value="%RX_BUZ_VOL%" min="0" max="255" required>
       </fieldset>
 
       <details>
         <summary>🛠 Advanced</summary>
         
         <label title="По умолчанию: 20 dBm">Рабочая мощность (0...22):</label>
         <input type="number" name="maxPower" value="%MAX_PWR%" min="0" max="22" required>
 
         <div class="checkbox-container">
           <input type="checkbox" name="dynamicPower" value="1" %DYN_PWR% title="По умолчанию: Включено">
           <span>Адаптировать мощность</span>
         </div>
 
         <label title="По умолчанию: -9 дБм">Мин. мощность (-9...0):</label>
         <input type="number" name="minPower" value="%MIN_PWR%" min="-9" max="0" required>
         
         <label title="По умолчанию: 18 дБм">Мощность при настройке (0-22 дБм):</label>
         <input type="number" name="servicePower" value="%SRV_PWR%" min="0" max="22" required>
 
         <div class="checkbox-container">
           <input type="checkbox" name="measurebattery" value="1" %BAT_CHK% title="По умолчанию: Включено">
           <span>Включить проверку батареи</span>
         </div>
 
         <label title="По умолчанию: 300000 мс (5 минут)">Период проверки батареи (мс):</label>
         <input type="number" name="batteryPeriod" value="%BAT_PER%" min="10000" step="1000" required>
         
         <label title="По умолчанию: 2000 мс">Время удержания для вкл. (мс):</label>
         <input type="number" name="wakeUpHoldTime" value="%WK_HOLD%" min="100" step="100" required>
         
         <label title="По умолчанию: 2000 мс">Время для отпускания при вкл. (мс):</label>
         <input type="number" name="wakeUpReleaseWindow" value="%WK_REL%" min="100" step="100" required>
         
         <label title="По умолчанию: 10000 мс (10 сек)">Таймаут «заклинивания» при вкл. (мс):</label>
         <input type="number" name="stuckSleepTime" value="%STUCK_SL%" min="1000" step="1000" required>
         
         <label title="По умолчанию: 600000 мс (10 минут)">Автовыход из настроек (мс):</label>
         <input type="number" name="configTimeout" value="%CONF_TO%" min="60000" step="10000" required>
         
         <label title="По умолчанию: 4000 мс">Индикация выключения (мс):</label>
         <input type="number" name="sleepLedDuration" value="%SLP_LED%" min="100" step="100" required>
 
         <label title="По умолчанию: 3600000 мс (1 час)">Отключение «Пинга» (мс):</label>
         <input type="number" name="bigTimeout" value="%TX_BIG_TO%" min="10000" step="1000" required>
         
         <label title="По умолчанию: 30000 мс (30 сек)">Время для настройки сигналов (мс):</label>
         <input type="number" name="execTimeout" value="%TX_EXEC_TO%" min="10000" step="1000" required>
 
         <label title="По умолчанию: 10000 мс (10 сек)">Отсечка сигнала приёмника (мс):</label>
         <input type="number" name="rxCutoffTime" value="%RX_CUTOFF%" min="100" step="100" required>
         
         <label title="По умолчанию: 10000 мс (10 сек)">Таймаут при потере Пинга (мс):</label>
         <input type="number" name="pingTimeoutRX" value="%RX_PING%" min="1000" step="100" required>
       </details>
 
       <div class="buttons-container">
         <button type="button" class="default-btn" onclick="setDefaults()">🔄 По умолчанию</button>
         <input type="submit" value="💾 Сохранить и Перезагрузить">
         <a href="/cancel" class="cancel-btn">❌ Отмена</a>
       </div>
 
     </form>
 </body>
 </html>
 )rawliteral";
 
 #endif