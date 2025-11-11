#include "morse_translator.h"
#include <string.h>

/**
 * @brief Morse code lookup table
 */
typedef struct {
    const char *morse;  // Morse pattern (dots and dashes)
    char letter;        // Corresponding letter
} MorseCode;

/**
 * @brief International Morse Code alphabet (A-Z)
 */
static const MorseCode morseTable[] = {
    {".-", 'A'},
    {"-...", 'B'},
    {"-.-.", 'C'},
    {"-..", 'D'},
    {".", 'E'},
    {"..-.", 'F'},
    {"--.", 'G'},
    {"....", 'H'},
    {"..", 'I'},
    {".---", 'J'},
    {"-.-", 'K'},
    {".-..", 'L'},
    {"--", 'M'},
    {"-.", 'N'},
    {"---", 'O'},
    {".--.", 'P'},
    {"--.-", 'Q'},
    {".-.", 'R'},
    {"...", 'S'},
    {"-", 'T'},
    {"..-", 'U'},
    {"...-", 'V'},
    {".--", 'W'},
    {"-..-", 'X'},
    {"-.--", 'Y'},
    {"--..", 'Z'}
};

/**
 * @brief Number of entries in the morse table
 */
#define MORSE_TABLE_SIZE (sizeof(morseTable) / sizeof(MorseCode))

char morse_to_letter(const char *morse) {
    // Handle NULL or empty input
    if (morse == NULL || morse[0] == '\0') {
        return '?';
    }
    
    // Search through the lookup table
    for (int i = 0; i < MORSE_TABLE_SIZE; i++) {
        if (strcmp(morse, morseTable[i].morse) == 0) {
            return morseTable[i].letter;
        }
    }
    
    // Pattern not found
    return '?';
}
