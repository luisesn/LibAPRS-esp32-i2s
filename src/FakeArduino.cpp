#include "FakeArduino.h"
#include <stdio.h>

HardwareSerial Serial;

size_t HardwareSerial::print(const char s[]) {
	if (!s) return 0;
	int n = printf("%s", s);
	return (n > 0) ? (size_t)n : 0;
}

size_t HardwareSerial::print(char c) {
	int n = printf("%c", c);
	return (n > 0) ? (size_t)n : 0;
}

size_t HardwareSerial::print(unsigned char v, int) {
	int n = printf("%u", (unsigned)v);
	return (n > 0) ? (size_t)n : 0;
}

size_t HardwareSerial::print(int v, int) {
	int n = printf("%d", v);
	return (n > 0) ? (size_t)n : 0;
}

size_t HardwareSerial::print(unsigned int v, int) {
	int n = printf("%u", v);
	return (n > 0) ? (size_t)n : 0;
}

size_t HardwareSerial::print(long v, int) {
	int n = printf("%ld", v);
	return (n > 0) ? (size_t)n : 0;
}

size_t HardwareSerial::print(unsigned long v, int) {
	int n = printf("%lu", v);
	return (n > 0) ? (size_t)n : 0;
}

size_t HardwareSerial::print(double v, int p) {
	int n = printf("%.*f", p, v);
	return (n > 0) ? (size_t)n : 0;
}

size_t HardwareSerial::println(const char s[]) {
	size_t n = print(s);
	n += println();
	return n;
}

size_t HardwareSerial::println(char c) {
	size_t n = print(c);
	n += println();
	return n;
}

size_t HardwareSerial::println(unsigned char v, int b) {
	size_t n = print(v, b);
	n += println();
	return n;
}

size_t HardwareSerial::println(int v, int b) {
	size_t n = print(v, b);
	n += println();
	return n;
}

size_t HardwareSerial::println(unsigned int v, int b) {
	size_t n = print(v, b);
	n += println();
	return n;
}

size_t HardwareSerial::println(long v, int b) {
	size_t n = print(v, b);
	n += println();
	return n;
}

size_t HardwareSerial::println(unsigned long v, int b) {
	size_t n = print(v, b);
	n += println();
	return n;
}

size_t HardwareSerial::println(double v, int p) {
	size_t n = print(v, p);
	n += println();
	return n;
}

size_t HardwareSerial::println(void) {
	int n = printf("\n");
	return (n > 0) ? (size_t)n : 0;
}