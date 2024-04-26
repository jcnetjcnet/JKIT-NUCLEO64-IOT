v0.7
arduino 소스 포팅
String class(WString.cpp, WString.h)
dtostrf.c(인터넷에서 구해서 가져옴)
Arduino.h (사용하지 않음)

WString.cpp에서 realloc, free등 동적메모리 관련 함수를 사용하므로 주의가 필요함(heap 관리 필요)


v1.0_f103
DTR_PA15
RTS_PB7

esp32 hreset 추가 (ESP32 hardware reset)

보드 수정 (2.1)