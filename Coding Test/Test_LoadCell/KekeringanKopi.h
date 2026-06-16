#ifndef KEKERINGAN_KOPI_H
#define KEKERINGAN_KOPI_H

const float BERAT_AWAL_GABAH_KOPI_GRAM = 500.0;

const char* tentukanStatusKekeringanKopi(float beratSekarangGram);
float hitungSelisihBeratKopi(float beratSekarangGram);
float hitungPersentaseSusutKopi(float beratSekarangGram);

#endif
