#ifndef MORSE_TRANSLATOR_H
#define MORSE_TRANSLATOR_H

/**
 * @file morse_translator.h
 * @brief Basic Morse code to letter translation
 */

/**
 * @brief Translate a morse code pattern to its corresponding letter
 * 
 * @param morse Null-terminated string containing morse pattern (e.g., ".-" for A)
 * @return char The corresponding letter (A-Z), or '?' if pattern is unknown
 * 
 * @example
 *   char letter = morse_to_letter(".-");     // Returns 'A'
 *   char letter = morse_to_letter("xxxxx");  // Returns '?'
 */
char morse_to_letter(const char *morse);

#endif // MORSE_TRANSLATOR_H
