/**
 * @file webpage.h
 * @version 1.23 (Изменение: Добавлены 3 группы настроек: Общие, TX, RX)
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
     h2 { text-align: center; color: #4CAF50; margin-bottom: 20px; font-size: 24px;}
     fieldset { border: 1px solid #4CAF50; border-radius: 8px; margin-bottom: 25px; padding: 20px; background: #1e1e1e; }
     legend { color: #4CAF50; font-weight: bold; font-size: 18px; padding: 0 10px; }
     label { display: block; margin-top: 15px; margin-bottom: 5px; color: #cccccc; font-size: 14px; }
     input[type="number"] { width: 100%; padding: 12px; border-radius: 6px; border: 1px solid #444; background: #2a2a2a; color: #fff; box-sizing: border-box; font-size: 16px; transition: border 0.3s; }
     input[type="number"]:focus { border-color: #4CAF50; outline: none; }
     .checkbox-container { display: flex; align-items: center; margin-top: 15px; background: #2a2a2a; padding: 12px; border-radius: 6px; }
     input[type="checkbox"] { transform: scale(1.5); margin: 0 10px 0 5px; accent-color: #4CAF50; }
     .checkbox-container span { font-size: 15px; color: #fff; }
     input[type="submit"] { background-color: #4CAF50; color: white; padding: 16px; border: none; border-radius: 6px; cursor: pointer; width: 100%; font-size: 18px; font-weight: bold; transition: background 0.3s; margin-top: 10px; margin-bottom: 30px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
     input[type="submit"]:hover { background-color: #45a049; }
   </style>
 </head>
 <body>
   <h2>⚙️ Настройка DavayLoRa</h2>
   <form action="/save" method="POST">
     
     <fieldset>
       <legend>🌍 Общие настройки</legend>
       <label>Рабочий канал / Адрес (0-19):</label>
       <input type="number" name="workAddress" value="%ADDR%" min="0" max="19" required>
       
       <div class="checkbox-container">
         <input type="checkbox" name="measurebattery" value="1" %BAT_CHK%>
         <span>Включить защиту и проверку батареи</span>
       </div>
       
       <label>Период проверки батареи (мс):</label>
       <input type="number" name="batteryPeriod" value="%BAT_PER%" min="10000" step="1000" required>
       
       <label>Защита от случайного вкл. (удержание, мс):</label>
       <input type="number" name="wakeUpHoldTime" value="%WK_HOLD%" min="100" step="100" required>
       
       <label>Окно отпускания кнопки (мс):</label>
       <input type="number" name="wakeUpReleaseWindow" value="%WK_REL%" min="100" step="100" required>
       
       <label>Сон при зажатой кнопке/магните (мс):</label>
       <input type="number" name="stuckSleepTime" value="%STUCK_SL%" min="1000" step="1000" required>
 
       <label>Макс. время режима конфигурации (мс):</label>
       <input type="number" name="configTimeout" value="%CONF_TO%" min="60000" step="10000" required>
     </fieldset>
 
     <fieldset>
       <legend>📡 Пульт (TX)</legend>
       <label>Яркость кнопки-индикатора (0-255):</label>
       <input type="number" name="pwmledBrightness" value="%TX_BIG_LED%" min="0" max="255" required>
       
       <label>Яркость статусного диода (0-255):</label>
       <input type="number" name="fbledBrightness" value="%TX_FB_LED%" min="0" max="255" required>
       
       <label>Таймаут потери связи (Ping TX, мс):</label>
       <input type="number" name="pingTimeout" value="%TX_PING%" min="1000" step="100" required>
       
       <label>Авто-сон при бездействии (мс):</label>
       <input type="number" name="bigTimeout" value="%TX_BIG_TO%" min="10000" step="1000" required>
 
       <label>Индикация перед сном (мс):</label>
       <input type="number" name="sleepLedDuration" value="%TX_SLP_LED%" min="100" step="100" required>
     </fieldset>
 
     <fieldset>
       <legend>🔔 Приёмник (RX)</legend>
       <div class="checkbox-container">
         <input type="checkbox" name="rxEnableBigLed" value="1" %RX_BIG_EN%>
         <span>Включить силовой светодиод (Фонарь)</span>
       </div>
       <label>Яркость фонаря RX (0-255):</label>
       <input type="number" name="rxPwmledBrightness" value="%RX_BIG_LED%" min="0" max="255" required>
 
       <div class="checkbox-container">
         <input type="checkbox" name="rxEnableBuzzer" value="1" %RX_BUZ_EN%>
         <span>Включить звуковой сигнал (Пищалка)</span>
       </div>
       <label>Громкость пищалки RX (0-255):</label>
       <input type="number" name="rxBuzzerVolume" value="%RX_BUZ_VOL%" min="0" max="255" required>
       
       <label>Таймаут отсечки сигнала (Cutoff, мс):</label>
       <input type="number" name="rxCutoffTime" value="%RX_CUTOFF%" min="100" step="100" required>
       
       <label>Таймаут потери связи (Ping RX, мс):</label>
       <input type="number" name="pingTimeoutRX" value="%RX_PING%" min="1000" step="100" required>
     </fieldset>
 
     <input type="submit" value="💾 Сохранить настройки">
   </form>
 </body>
 </html>
 )rawliteral";
 
 #endif