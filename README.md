# wch
wch experiments
Последовательность
1 Программ. ПРИЕМА на Flnb=739,550МГц. Нужен какой-то источник сигнала, например VNA.
2 Панорама-водопад
3 Коррекция +9750,000МГц, чтобы вместо 739 550 показывал 10 489 550.
Freq=Flnb+9750000
4 Оцифровать ось Х на панораме 10489550, только последние 6 цифр.


Частота приема — маркер на оси, ширина задается в окне SSB Demodulator. Пока задать 3кГц.
5 Ввести LO от минус 200кГц до +50кГц, так «гуляют» LNB. Средний LO=0.
На экране движок и возможность ввода числа с точность 0,1кГц.
6 Разбить шкалу 10489,5 -10490,0 на 5 поддиапазонов:
- 10489,490 — 10489,610МГц,  нужен запас на краях примерно 10кГц, можно и другой,
- 10489,590 — 10489,710МГц,
- 10489,690 — 10489,810МГц,
- 10489,790 — 10489,910МГц,
- 10489,890 — 10490,010МГц.
Переключение ВВЕРХ/ВНИЗ парой кнопок на экране.
7 Программ. Передачи. Ftx=Feq-8089,5
На экране частоту передачи выводить числом: Ftx=F маркера приемника.
8 Экраны SSB Demodulator и SSB Modulator — только частоты звука Low cut и Hi cut.