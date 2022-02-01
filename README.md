# DavayLoRa

Простая «дистанционная кнопка» для вызова на базе ЛоРа с радиусом действия порядка 300 метров.

ПЕРЕДАТЧИК имеет кнопку, а кроме этого: 
- статусный LED (обычно встроенный в кнопку);
- мониторинговый LED, который загорается _практически_ совместно с приёмником;
- дополнительный лед статуса батареи (встроенный лед платы микропроцессора).
- движковый выключатель. 

ПРИЁМНИК имеет:
- исполнительный ЛЕД - собственно вызывающая «лампочка» (состоит из 2 параллельных COB LED-ов);
- Баззер (моторчик с эксцентриком, осуществляющим вибрацию);
- Биппер (пищалка);
- DIP3 - переключатель выбора набора исполнительных элементов. В принципе можно выбрать все, часть или ни одного;
- дополнительный лед статуса, в том числе и батареи (встроенный лед платы микропроцессора).
- движковый выключатель. 

При включении как передатчика, так и приёмника:
- Показывается 1-секундное «приветствие» всеми исполнительными элементами, 
- затем встроенный лед статуса батареи выдаёт с интервалом 2 секунды 2 серии с числом вспышек в зависимости от напряжения батареи:
- - voltage > 3.5 - 2 по 1 разу
- - voltage > 3.6 - 2 по 2 раза
- - voltage > 3.7 - 2 по 3 раза
- - voltage > 3.8 - 2 по 4 раза
- - voltage > 4.0 - 2 по 5 раз

Ниже 3.5 вольт каждый прибор поморгает 7 раз и выключается. (Проверка напряжения в дальнейшем происходит каждые 5 минут непрерывной работы).

- Передатчик переходит в режим ожидания _первого_ нажатия на кнопку.
(При этом передатчик не передаёт никаких комманд до первого нажатия.)
- Приёмник переходит в режим ожидания сигналов от передатчика. 
Приёмник никогда не передаёт никаких комманд первый, только отвечает на команды передатчика.
При отсутствии команд от передатчика приёмник с периодичностью 5 секунд выдаёт по 2 коротких вспышки статусным ледом.
- При первом нажатии на кнопку передатчик «просыпается» и начинает посылать команды включения/выключения и «пинги».
- При нажатии на кнопку передатчика передаётся сигнал на приёмник и в случае успешного приёма на приёмнике
срабатывают исполнительные элементы в соответствии с выбором переключателей DIP3. 
- Приёмник посылает ответную комманду, которая в случае успешного приёма передатчиком включает мониторинговый (и статусный) LED передатчика.
Таким образом, испольнительные элементы TX и RX включаются практически одновременно.
- При отпускании кнопки передатчика по команде гасятся исполнительные элементы приёмника и посылается ответная команда
передатчику, по получении которой выключаются мониторинговый и статусный LEDы передатчика.
Таким образом, при успешной связи испольнительные элементы TX и RX включаются и выключаются вместе.
- При отсутствии изменений состояния кнопки, передатчик начинает самостоятельно посылать «пинг» - сигнал с периодичностью 3 секунды.
Этот сигнал, в случае приёма, вызывает однократное мигание статусного ЛЕДа приёмника и ответ приёмника,
который, в свою очередь, в случае успешного приёма передатчиком вызывает однократное мигание статусного ЛЕДа передатчика.
Таким образом, в покое при успешной связи передатчик и приёмник практически синхронно однократно «моргают» с 
периодичностью 3 секунды.
- Включенные исполнительные элементы приёмника продолжительно работают максимально 2 секунды, после чего выключаются
(НЕ посылая сигнала об этом на передатчик). Поэтому при очень длительном нажатии на кнопку исполнительные 
элементы приёмника включаются каждые 3 секунды (по «пингу») и выключаются через 2 секунды после включения.
- При отсутствии ответа от приёмника передатчик с периодичностью 3 секунды выдаёт по 2 коротких вспышки статусным («кнопочным») ледом
(вместо нормально одной).
Таким образом: однократное «моргание» раз в 3 секунды - связь есть, а 2-х кратное моргание приёмника либо передатчика - связи нет.
- Если в течение часа не происходит нажатия кнопки, передатчик перестаёт посылать «пинг» и переходит в режим ожидания первого нажатия на кнопку.
(При этом приёмник, как и в случае потери сигнала, с периодичностью 5 секунд выдаёт по 2 коротких вспышки статусным ледом.)  

Взаимная настройка приёмника и передатчика (рабочей пары) производится в коде синхронным изменением частоты и/или рабочего адреса команд и последующей прошивкой. 


- 10.01.2022  - 	«причёсан» предыдущий код. Добавлены схемы соединения. Новый репозиторий github.
- 11.02.2022	-	исправлены баги и изменено моргание при отсутствии сигнала на 2 вспышки. 
- 11.02.2022	-	добавлено выключение пинга через 1 час отсутствия активности на кнопке
- 11.02.2022	-	добавлена библиотека GyverPower для засыпания процессора при разряженной батарее
- 17.01.2022	-	после проверки работы с севшими батарейками немного изменены напряжения отсечки
- 17.01.2022	-	Добавлено 1-секундное «приветствие» (а то севшие батарейки не мигали совсем)
- 18.01.2022	-	Исправлена бага со включением баззера и биппера на старте.
- 18.01.2022	-	В приветствие на старте добавлено включение всех исполнительных элементов.
- 18.01.2022	-	Добавлены инструкции для успешной работы с модулями с неисправным измерителем напряжения.
- 29.01.2022 Зафиксированы изменения Никиты:
- Диапазоны батареек
- Неработающий встроенный измеритель напряжения
- Рабочий адрес
Исправлен баг с непроверкой рабочего адреса на приёмнике
- TODO: Исправить схемы для самодельного измерителя напряжения
- 01.02.2022 - Попытка ослабить влияние приборов разных пар, работающих на одной частоте.
- использованы Sync Word
- добавлена подстройка частоты на приёмнике
- добавлена задержка при получении несвоего пакета
- Сделан дебаунсинг на кнопке (на всякий случай)
- Файлы разбиты на «табы» для удобства работы в Ардуино ИДЕ
- 01.02.2022  - Убран ответный сигнал на «потухание»
- уменьшены задержки


				