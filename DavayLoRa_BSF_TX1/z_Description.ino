/*
  Код ПЕРЕДАТЧИКА для проекта DavayLoRa
  TX отправляет сообщение на RX, когда нажата/отпущена кнопка, или пингует его по таймеру
  Ожидает ответ-подтверждение на каждое сообщение.
  Проектировался под LoRa Adafruit Feather32u4 433MHz module
  В дальнейшем адаптирован для более доступной платы BSFrance LoRa32u4 - которая ПОЧТИ копия.
  Отличия модулей:
  - физические размеры
  - отличаются делители напряжения измерения батарейки - следует произвести подстройку в коде
  - У BSFrance нужно перерезать перемычку на плате, помеченную: "Closed DI01 -> 6" )
  Для каждой пары TX-RX надо указать в коде одинаковую рабочую частоту
  (изменять её рекомендуется по 0.1 мегагерц, в пределах рабочего диапазона 433.05E6 - 434.79E6)
  и/или выбрать одинаковый совместный байт WORK_ADDRESS

  СОЕДИНЕНИЯ (см. также схему Fritzing и картинку):
  - Перерезать перемычку на плате BSFrance, помеченную: "Closed DI01 -> 6" )
  - Кнопка нормально разомкнутая (NO), со встроенным светодиодом (+/-), использумым для индикации
    обратной связи, батарейки и ошибок.
      NO (любой) -> GND
      NO (другой) -> 6 микропроцессора
      светодиод (+) -> 5 микропроцессора
      светодиод (-) -> GND (или соединить с NO, который GND)
  - Большой светодиод (используется для индикации вызова при нажатии на кнопку)
      плюс -> BAT микропроцессора (или + батареи)
      минус -> сток (drain) полевого тр-ра (центральный вывод)
  - MOSFET 60NO3
      управляющий (gate) полевого тр-ра (левый вывод) -> 11 микропроцессора
      исток (source) полевого тр-ра (правый вывод) -> GND
  - Переключатель выключения
      центр (или край) -> GND
      край (или центр) -> EN микропроцессора
      (при замкнутом переключателе прибор вЫключен, заряжать батарейку при этом можно;
      при разомкнутом - прибор включен)
  - Батарею LiPo 1S подключить или припаять к своему JST разъёму

    USB порт можно использовать для зарядки батареи - в любое время
      и для заливки прошивки (при разомкнутом переключателе)

  ПРОБЛЕМА: У некоторых модулей BSFrance не работает встроенный измеритель напряжения
  (для определения этого можно воспользоватьс программой BSFTest.ino со включенным монитором).
  Чтобы исользовать такие модули, следует добавить в схему 2 одинаковых резистора
  по 10-100КОм следующим образом:
  - один вывод каждого резистора паять на (+) и (-) разъёма батареи
  - вторые выводы соединить вместе и присоединить к пину (на выбор) A0 - A5
  (какой удобнее, например A1).
  Также надо скорректировать следующие define-ы:
  #define PIN_BATTERY A1
  #define BATTERY_VOLTAGE_MULTIPLIER 2;
*/
