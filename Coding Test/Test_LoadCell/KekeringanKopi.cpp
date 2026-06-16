#include "KekeringanKopi.h"

const char* tentukanStatusKekeringanKopi(float beratSekarangGram) {
  if (beratSekarangGram > 400.0) {
    return "Basah";
  }

  if (beratSekarangGram >= 320.0) {
    return "Cukup Kering";
  }

  if (beratSekarangGram >= 284.0) {
    return "Kering Siap Simpan";
  }

  if (beratSekarangGram < 260.0) {
    return "Over Dry";
  }

  return "Di bawah target";
}

float hitungSelisihBeratKopi(float beratSekarangGram) {
  return BERAT_AWAL_GABAH_KOPI_GRAM - beratSekarangGram;
}

float hitungPersentaseSusutKopi(float beratSekarangGram) {
  float selisihBerat = hitungSelisihBeratKopi(beratSekarangGram);
  return (selisihBerat / BERAT_AWAL_GABAH_KOPI_GRAM) * 100.0;
}
