/*
 * frame.h
 *
 *  Created on: Jan 6, 2025
 *      Author: Kuba
 */

#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include <stdint.h>

#define SYNCHRO_START 0x7E  // Znak początku ramki
#define SYNCHRO_END   0x7C  // Znak końca ramki
#define ESCAPE_BYTE   0x7D  // Znak escape
#define MAX_DATA_LEN  255   // Maksymalna długość danych
#define CRC_POLYNOMIAL 0x8005 // Wielomian do obliczania CRC

// Struktura ramki
typedef struct {
    uint8_t address;               // Adres ramki (np. 'A', 'B')
    uint8_t length;                // Długość pola danych
    uint8_t data[MAX_DATA_LEN];    // Dane
    uint16_t crc;                  // Suma kontrolna CRC
} Frame;

// Deklaracje funkcji
uint16_t calculateCRC(const uint8_t *data, uint16_t length);
uint16_t createFrame(uint8_t *buffer, const Frame *frame);
int decodeFrame(const uint8_t *buffer, uint16_t length, Frame *frame);


#endif /* INC_FRAME_H_ */
