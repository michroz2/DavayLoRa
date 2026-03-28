/**
 * @file webpage.h
 * @version 1.22 - начальная версия
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
   <title>DavayLoRa TX Config</title>
   <style>
     body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; padding: 20px; max-width: 500px; margin: 0 auto; }
     h2 { text-align: center; color: #4CAF50; margin-bottom: 25px; }
     label { display: block; margin-top: 15px; margin-bottom: 5px; color: #aaaaaa; font-size: 14px; }
     input[type="number"] { width: 100%; padding: 12px; border-radius: 6px; border: 1px solid #333; background: #222; color: #fff; box-sizing: border-box; font-size: 16px; }
     input[type="checkbox"] { transform: scale(1.5); margin-left: 5px; }
     .checkbox-container { display: flex; align-items: center; margin-top: 20px; padding: 10px 0; border-top: 1px solid #333; border-bottom: 1px solid #333; }
     .checkbox-container span { margin-left: 15px; font-size: 16px; }
     input[type="submit"] { background-color: #4CAF50; color: white; padding: 16px; border: none; border-radius: 6px; cursor: pointer; width: 100%; margin-top: 25px; font-size: 18px; font-weight: bold; transition: background 0.3s; }
     input[type="submit"]:hover { background-color: #45a049; }
     .card { background: #1e1e1e; padding: 25px; border-radius: 12px; box-shadow: 0 8px 16px rgba(0,0,0,0.4); }
   </style>
 </head>
 <body>
   <h2>⚙️ DavayLoRa TX Config</h2>
   <div class="card">
     <form action="/save" method="POST">
       <label>Рабочий канал / Адрес (0-19):</label>
       <input type="number" name="workAddress" value="%ADDR%" min="0" max="19" required>
       
       <label>Яркость кнопки-индикатора (0-255):</label>
       <input type="number" name="pwmledBrightness" value="%BIG_LED%" min="0" max="255" required>
       
       <label>Яркость статусного диода (0-255):</label>
       <input type="number" name="fbledBrightness" value="%FB_LED%" min="0" max="255" required>
       
       <label>Таймаут пинга TX (миллисекунды):</label>
       <input type="number" name="pingTimeout" value="%PING%" min="1000" step="100" required>
       
       <div class="checkbox-container">
         <input type="checkbox" name="measurebattery" value="1" %BAT_CHK%>
         <span>Включить проверку батареи</span>
       </div>
       
       <input type="submit" value="Сохранить настройки">
     </form>
   </div>
 </body>
 </html>
 )rawliteral";
 
 #endif