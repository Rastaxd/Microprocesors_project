/*
 * frame.h
 *
 *  Created on: Jan 6, 2025
 *      Author: Kuba
 */

#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Znaki protokołu
#define FRAME_START     0x7E  // '~' Znak początku ramki
#define FRAME_END       0x7C  // '|' Znak końca ramki
#define ESCAPE_CHAR     0x7D  // '}' Znak escape

// Znaki stuffingowe - co następuje po znaku escape
#define FRAME_START_STUFF  0x5E  // '^' dla ~
#define FRAME_END_STUFF    0x5C  // '\' dla |
#define ESCAPE_CHAR_STUFF  0x5D  // ']' dla }

#define MAX_DATA_LEN  255   // Maksymalna długość danych
#define MAX_FRAME_WITHOUT_STUFFING 135
#define MAX_FRAME_BUFFER_SIZE (MAX_FRAME_WITHOUT_STUFFING * 2)  // Maksymalny rozmiar z byte stuffingiem
#define ERR_GOOD 0
#define ERR_FAIL 1

// Definicje adresów
#define ADDR_LEN 2  // Długość adresu w bajtach

// Struktura ramki
typedef struct {
    char sourceAddress[ADDR_LEN];      // Adres nadawcy ("ST" lub "PC")
    char destinationAddress[ADDR_LEN]; // Adres odbiorcy ("ST" lub "PC")
    uint8_t length;                    // Długość danych (hex)
    uint8_t data[MAX_DATA_LEN];        // Dane (ASCII)
    uint16_t crc;                      // CRC (hex, big endian)
} Frame;

// Nowa struktura dla maszyny stanów
typedef struct {
    bool inFrame;
    bool escapeDetected;
    uint8_t bx[MAX_FRAME_WITHOUT_STUFFING * 2];
    uint16_t bxIndex;
} FrameState;

// Deklaracje funkcji
uint16_t calculateCRC(const uint8_t *data, uint16_t length);
uint16_t createFrame(uint8_t *buffer, const Frame *frame);
int decodeFrame(const uint8_t *buffer, uint16_t length, Frame *frame);

// Nowe deklaracje funkcji
void resetFrameState(void);
void processReceivedChar(uint8_t receivedChar);
void sendStatus(uint8_t status);

/**
 * @brief Sprawdza poprawność adresu
 * @param address Adres do sprawdzenia
 * @return true jeśli adres jest poprawny (ST lub PC)
 */
static inline bool isValidAddress(const char* address) {
    return (memcmp(address, "ST", 2) == 0) || (memcmp(address, "PC", 2) == 0);
}

#endif /* INC_FRAME_H_ */
