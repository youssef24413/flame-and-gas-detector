#include <iostream>
#include <cassert>

// --- إعدادات الثوابت الموجودة في الكود الأصلي ---
#define GAS_INVALID_LOW     50
#define GAS_INVALID_HIGH    4090

// --- 1. الدوال النقية (Pure Functions) من كودك ---
bool isGasValid(int adc) {
  return (adc >= GAS_INVALID_LOW && adc <= GAS_INVALID_HIGH);
}

bool isFlameDetected(int pinVal) {
  return (pinVal == 1); // 1 represents HIGH
}

int decideSystem(bool gasAlert, bool flameAlert) {
  if (flameAlert)      return 2;
  if (gasAlert)        return 1;
  return 0;
}

int convertAdcToPercent(int adc) {
  if (adc < 0)    adc = 0;
  if (adc > 4095) adc = 4095;
  return (adc * 100) / 4095;
}

// --- 2. سيناريوهات الاختبار الآلي (Test Cases) ---
int main() {
    std::cout << "--- Running ESP32 Logic Tests ---\n";

    // Test 1: Gas Validation
    assert(isGasValid(2000) == true);   // قراءة طبيعية
    assert(isGasValid(10) == false);    // قراءة أقل من الطبيعي (عطل)
    assert(isGasValid(4500) == false);  // قراءة أعلى من المسموح (عطل)
    std::cout << "[PASS] isGasValid() logic works.\n";

    // Test 2: Flame Detection
    assert(isFlameDetected(1) == true);
    assert(isFlameDetected(0) == false);
    std::cout << "[PASS] isFlameDetected() logic works.\n";

    // Test 3: System Decision (Priority Check)
    assert(decideSystem(false, false) == 0); // هدوء
    assert(decideSystem(true, false) == 1);  // غاز فقط -> 1
    assert(decideSystem(false, true) == 2);  // حريق فقط -> 2
    assert(decideSystem(true, true) == 2);   // حريق وغاز -> الأولوية للحريق (2)
    std::cout << "[PASS] decideSystem() logic works.\n";

    // Test 4: ADC to Percentage Conversion
    assert(convertAdcToPercent(0) == 0);
    assert(convertAdcToPercent(4095) == 100);
    assert(convertAdcToPercent(2047) == 49 || convertAdcToPercent(2047) == 50);
    assert(convertAdcToPercent(-500) == 0);    // حماية ضد القيم السلبية
    assert(convertAdcToPercent(5000) == 100);  // حماية ضد القيم العالية
    std::cout << "[PASS] convertAdcToPercent() logic works.\n";

    std::cout << "\n✅ ALL UNIT TESTS PASSED SUCCESSFULLY!\n";
    return 0;
}