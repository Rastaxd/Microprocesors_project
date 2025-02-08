/*
 * frame.c
 *
 *  Created on: Jan 6, 2025
 *      Author: Kuba
 */

#include "frame.h"

// Funkcja obliczająca CRC
uint16_t calculateCRC(const uint8_t *data, uint16_t length) {
    static const uint16_t CRC_TABLE[256] = {
    		0x0000, 0x6F63, 0xDEC6, 0xB1A5,   0xD2EF, 0xBD8C, 0x0C29, 0x634A,
    		0xCABD, 0xA5DE, 0x147B, 0x7B18,   0x1852, 0x7731, 0xC694, 0xA9F7,
    		0xFA19, 0x957A, 0x24DF, 0x4BBC,   0x28F6, 0x4795, 0xF630, 0x9953,
    		0x30A4, 0x5FC7, 0xEE62, 0x8101,   0xE24B, 0x8D28, 0x3C8D, 0x53EE,
    		0x9B51, 0xF432, 0x4597, 0x2AF4,   0x49BE, 0x26DD, 0x9778, 0xF81B,
    		0x51EC, 0x3E8F, 0x8F2A, 0xE049,   0x8303, 0xEC60, 0x5DC5, 0x32A6,
    		0x6148, 0x0E2B, 0xBF8E, 0xD0ED,   0xB3A7, 0xDCC4, 0x6D61, 0x0202,
    		0xABF5, 0xC496, 0x7533, 0x1A50,   0x791A, 0x1679, 0xA7DC, 0xC8BF,
    		0x59C1, 0x36A2, 0x8707, 0xE864,   0x8B2E, 0xE44D, 0x55E8, 0x3A8B,
    		0x937C, 0xFC1F, 0x4DBA, 0x22D9,   0x4193, 0x2EF0, 0x9F55, 0xF036,
    		0xA3D8, 0xCCBB, 0x7D1E, 0x127D,   0x7137, 0x1E54, 0xAFF1, 0xC092,
    		0x6965, 0x0606, 0xB7A3, 0xD8C0,   0xBB8A, 0xD4E9, 0x654C, 0x0A2F,
    		0xC290, 0xADF3, 0x1C56, 0x7335,   0x107F, 0x7F1C, 0xCEB9, 0xA1DA,
    		0x082D, 0x674E, 0xD6EB, 0xB988,   0xDAC2, 0xB5A1, 0x0404, 0x6B67,
    		0x3889, 0x57EA, 0xE64F, 0x892C,   0xEA66, 0x8505, 0x34A0, 0x5BC3,
    		0xF234, 0x9D57, 0x2CF2, 0x4391,   0x20DB, 0x4FB8, 0xFE1D, 0x917E,
    		0xB382, 0xDCE1, 0x6D44, 0x0227,   0x616D, 0x0E0E, 0xBFAB, 0xD0C8,
    		0x793F, 0x165C, 0xA7F9, 0xC89A,   0xABD0, 0xC4B3, 0x7516, 0x1A75,
    		0x499B, 0x26F8, 0x975D, 0xF83E,   0x9B74, 0xF417, 0x45B2, 0x2AD1,
    		0x8326, 0xEC45, 0x5DE0, 0x3283,   0x51C9, 0x3EAA, 0x8F0F, 0xE06C,
    		0x28D3, 0x47B0, 0xF615, 0x9976,   0xFA3C, 0x955F, 0x24FA, 0x4B99,
    		0xE26E, 0x8D0D, 0x3CA8, 0x53CB,   0x3081, 0x5FE2, 0xEE47, 0x8124,
    		0xD2CA, 0xBDA9, 0x0C0C, 0x636F,   0x0025, 0x6F46, 0xDEE3, 0xB180,
    		0x1877, 0x7714, 0xC6B1, 0xA9D2,   0xCA98, 0xA5FB, 0x145E, 0x7B3D,
    		0xEA43, 0x8520, 0x3485, 0x5BE6,   0x38AC, 0x57CF, 0xE66A, 0x8909,
    		0x20FE, 0x4F9D, 0xFE38, 0x915B,   0xF211, 0x9D72, 0x2CD7, 0x43B4,
    		0x105A, 0x7F39, 0xCE9C, 0xA1FF,   0xC2B5, 0xADD6, 0x1C73, 0x7310,
    		0xDAE7, 0xB584, 0x0421, 0x6B42,   0x0808, 0x676B, 0xD6CE, 0xB9AD,
    		0x7112, 0x1E71, 0xAFD4, 0xC0B7,   0xA3FD, 0xCC9E, 0x7D3B, 0x1258,
    		0xBBAF, 0xD4CC, 0x6569, 0x0A0A,   0x6940, 0x0623, 0xB786, 0xD8E5,
    		0x8B0B, 0xE468, 0x55CD, 0x3AAE,   0x59E4, 0x3687, 0x8722, 0xE841,
    		0x41B6, 0x2ED5, 0x9F70, 0xF013,   0x9359, 0xFC3A, 0x4D9F, 0x22FC
    };

    uint16_t crc = 0xFFFF; // Dodać tę linię - inicjalizacja CRC
    
    for (uint16_t i = 0; i < length; i++) {
        uint8_t index = (crc >> 8) ^ data[i]; // XOR bieżącego bajtu z górnymi 8 bitami CRC
        crc = (crc << 8) ^ CRC_TABLE[index];  // Aktualizacja CRC
    }

    return crc;
}

// Funkcja tworząca ramkę
uint16_t createFrame(uint8_t *buffer, const Frame *frame) {
    uint16_t index = 0;

    // Znak początku
    buffer[index++] = FRAME_START;

    // Adres nadawcy (2 znaki)
    buffer[index++] = frame->sourceAddress[0];
    buffer[index++] = frame->sourceAddress[1];

    // Adres odbiorcy (2 znaki)
    buffer[index++] = frame->destinationAddress[0];
    buffer[index++] = frame->destinationAddress[1];

    // Długość
    buffer[index++] = frame->length;

    // Dodaj dane (ASCII)
    for (uint16_t i = 0; i < frame->length; i++) {
        if (frame->data[i] == FRAME_START || 
            frame->data[i] == FRAME_END || 
            frame->data[i] == ESCAPE_CHAR) {
            buffer[index++] = ESCAPE_CHAR;
            buffer[index++] = frame->data[i] ^ 0x20;  // Byte stuffing
        } else {
            buffer[index++] = frame->data[i];
        }
    }

    // Oblicz CRC
    uint16_t crc = calculateCRC(frame->data, frame->length);

    // Dodaj CRC jako surowe bajty w formacie big endian
    buffer[index++] = (crc >> 8) & 0xFF;  // Starszy bajt
    buffer[index++] = crc & 0xFF;         // Młodszy bajt

    // Dodaj FRAME_END (hex)
    buffer[index++] = FRAME_END; // 0x7C

    return index;
}

// Funkcja dekodująca ramkę
int decodeFrame(const uint8_t *buffer, uint16_t length, Frame *frame) {
    // Sprawdź minimalną długość przed odczytem
    if (length < 9) return 0;  // Przenieść na początek funkcji
    
    // Sprawdź znak początku
    if (buffer[0] != FRAME_START) return 0;
    
    // Odczyt adresów
    frame->sourceAddress[0] = buffer[1];
    frame->sourceAddress[1] = buffer[2];
    frame->destinationAddress[0] = buffer[3];
    frame->destinationAddress[1] = buffer[4];
    frame->length = buffer[5];
    
    // Sprawdź długość
    if (frame->length > MAX_DATA_LEN || frame->length > (length - 9)) {
        return 0;
    }
    
    uint16_t dataIndex = 0;
    
    // Kopiowanie danych z dekodowaniem byte stuffingu - ZŁY POCZĄTKOWY INDEKS!
    for (uint16_t i = 6; i < length - 3 && dataIndex < frame->length; i++) {
        if (buffer[i] == ESCAPE_CHAR) {
            switch (buffer[++i]) {  // Najpierw inkrementuj i, potem użyj
                case FRAME_START_STUFF:
                    frame->data[dataIndex++] = FRAME_START;
                    break;
                case FRAME_END_STUFF:
                    frame->data[dataIndex++] = FRAME_END;
                    break;
                case ESCAPE_CHAR_STUFF:
                    frame->data[dataIndex++] = ESCAPE_CHAR;
                    break;
                default:
                    return 0; // Błędna sekwencja escape
            }
        } else {
            frame->data[dataIndex++] = buffer[i];
        }
    }
    
    // Sprawdź czy odczytano właściwą ilość danych
    if (dataIndex != frame->length) {
        return 0;
    }
    
    // Odczyt CRC (2 bajty, big endian)
    if (length < 9) return 0;  // Minimalna długość ramki
    frame->crc = (buffer[length - 3] << 8) | buffer[length - 2];
    if (buffer[length - 1] != FRAME_END) return 0;  // Sprawdź końcowy znak
    
    // Weryfikacja CRC
    uint16_t calculatedCRC = calculateCRC(frame->data, frame->length);
    if (calculatedCRC != frame->crc) {
        return 0;
    }
    
    return 1; // Sukces
}

