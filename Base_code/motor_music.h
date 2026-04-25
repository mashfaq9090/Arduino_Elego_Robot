/*
 * motor_music.h
 *
 * How it works:
 *   Rapidly alternating motor direction at a frequency
 *   creates a tone — same principle as a speaker coil
 *
 */

#pragma once
#include <Arduino.h>

// ===================== NOTE FREQUENCIES (Hz) =====================

#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_D6  1175
#define NOTE_E6  1319
#define NOTE_G6  1568
#define NOTE_REST 0
#define NOTE_GS4 415
#define NOTE_AS4 466
#define NOTE_DS5 622

// ===================== MOTOR TONE =====================

/*
 * motor_tone()
 * Generates a tone by rapidly toggling motor direction
 * freq      — frequency in Hz
 * duration  — how long to play in ms
 * power     — motor PWM (30-80, higher = louder but harder on motors)
 */
inline void motor_tone(int freq, int duration, int power = 50) {
  if (freq == NOTE_REST) {
    analogWrite(PWR_R, 0);
    analogWrite(PWR_L, 0);
    delay(duration);
    return;
  }

  unsigned long period    = 1000000UL / freq;  // period in microseconds
  unsigned long half      = period / 2;
  unsigned long end_time  = millis() + duration;

  while (millis() < end_time) {
    // Forward half cycle
    digitalWrite(MTR_L, HIGH);
    digitalWrite(MTR_R, HIGH);
    analogWrite(PWR_R, power);
    analogWrite(PWR_L, power);
    delayMicroseconds(half);

    // Reverse half cycle — creates the oscillation
    digitalWrite(MTR_L, LOW);
    digitalWrite(MTR_R, LOW);
    analogWrite(PWR_R, power);
    analogWrite(PWR_L, power);
    delayMicroseconds(half);
  }

  // Stop between notes
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0);
}

// ===================== Some generic tune =====================
inline void sound1() {
  Serial.println("[MUSIC]");

  // Short gap before starting
  delay(300);

  for (int i = 0; i < 3; i++) {
    motor_tone(80, 80, 80);   // low thud
    delay(120);
  }
  delay(200);

  // ---- BRASS HITS ----
  motor_tone(NOTE_G4,  120, 70);  delay(40);
  motor_tone(NOTE_REST, 60);
  motor_tone(NOTE_G4,  120, 70);  delay(40);
  motor_tone(NOTE_REST, 60);
  motor_tone(NOTE_G4,  120, 70);  delay(40);
  motor_tone(NOTE_REST, 80);

  // ---- MAIN MELODY ----
  motor_tone(NOTE_C5,  300, 65);  delay(20);
  motor_tone(NOTE_REST, 40);
  motor_tone(NOTE_E5,  200, 65);  delay(20);
  motor_tone(NOTE_REST, 30);
  motor_tone(NOTE_G5,  400, 65);  delay(20);
  motor_tone(NOTE_REST, 50);

  motor_tone(NOTE_C6,  500, 70);  delay(20);
  motor_tone(NOTE_REST, 60);

  // ---- DESCENDING RUN ----
  motor_tone(NOTE_B5,  150, 65);  delay(10);
  motor_tone(NOTE_A5,  150, 65);  delay(10);
  motor_tone(NOTE_G5,  150, 65);  delay(10);

  // ---- SECOND PHRASE ----
  motor_tone(NOTE_REST, 80);
  motor_tone(NOTE_E5,  200, 65);  delay(20);
  motor_tone(NOTE_G5,  200, 65);  delay(20);
  motor_tone(NOTE_C6,  500, 70);  delay(20);
  motor_tone(NOTE_REST, 60);

  // ---- TRIUMPHANT ENDING ----
  motor_tone(NOTE_E6,  200, 72);  delay(20);
  motor_tone(NOTE_D6,  200, 72);  delay(20);
  motor_tone(NOTE_C6,  600, 75);  delay(20);
  motor_tone(NOTE_REST, 100);

  for (int i = 0; i < 4; i++) {
    motor_tone(80, 60, 80);
    delay(100);
  }
  motor_tone(80, 200, 80);

  // ---- FULL STOP ----
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0);
  Serial.println("[MUSIC] Done");
}


inline void sound2() {
  Serial.println("[MUSIC] Hello I am Rusty");

  // --- HELLO ---
  motor_tone(NOTE_G5, 100, 75);    // "He-" (High energy)
  delay(10);                      // Tiny 'L' gap
  motor_tone(NOTE_E5, 250, 60);    // "-llo" (Vowel shift)
  delay(150);

  // --- I AM ---
  motor_tone(NOTE_C5, 80, 50);     // "I"
  delay(40);
  motor_tone(NOTE_D5, 120, 55);    // "am"
  delay(150);

  // --- RUSTY 
  // "R" - low start
  motor_tone(150, 60, 80);        
  delay(10);
  
  // "U" - Core vowel
  motor_tone(NOTE_E5, 150, 65);   
  
  // "S" - High frequency hiss 
  motor_tone(NOTE_G6, 40, 85);    
  delay(20);                      // 'T' stop 
  
  // "Y" 
  motor_tone(NOTE_C6, 300, 70);   

  // // --- SIGN-OFF CHIRP ---
  // delay(50);
  // motor_tone(NOTE_G6, 50, 80);    
  
  // Full Stop
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0);
}


inline void play_mario_complete() {
  Serial.println("[MUSIC] Mario Theme - Full Optimization");
  
  // ===================== THE FAMOUS INTRO =====================
  // Notes: E5 E5 (rest) E5 (rest) C5 E5 (rest) G5 (rest) G4
  motor_tone(NOTE_E5, 100, 70); delay(50);
  motor_tone(NOTE_E5, 100, 70); delay(120); 
  motor_tone(NOTE_E5, 100, 70); delay(120); 
  motor_tone(NOTE_C5, 100, 70); delay(20);
  motor_tone(NOTE_E5, 120, 75); delay(120);
  motor_tone(NOTE_G5, 150, 85); delay(300); // The high jump
  motor_tone(NOTE_G4, 150, 85); delay(300); // The low drop

  // ===================== THE MAIN VERSE =====================
  // C5 G4 E4 A4 B4 A#4 A4
  motor_tone(NOTE_C5, 150, 70); delay(150);
  motor_tone(NOTE_G4, 150, 70); delay(150);
  motor_tone(NOTE_E4, 150, 70); delay(150);
  
  motor_tone(NOTE_A4, 120, 70); delay(80);
  motor_tone(NOTE_B4, 120, 70); delay(80);
  motor_tone(NOTE_AS4, 120, 70); delay(20); // A# (Sharper 'S' sound)
  motor_tone(NOTE_A4, 120, 70); delay(100);

  // G4 E5 G5 A5 F5 G5
  motor_tone(NOTE_G4, 100, 75); delay(50);
  motor_tone(NOTE_E5, 100, 75); delay(50);
  motor_tone(NOTE_G5, 100, 80); delay(50);
  motor_tone(NOTE_A5, 120, 85); delay(80);
  motor_tone(NOTE_F5, 100, 75); delay(50);
  motor_tone(NOTE_G5, 100, 80); delay(150);

  // E5 C5 D5 B4
  motor_tone(NOTE_E5, 120, 70); delay(80);
  motor_tone(NOTE_C5, 120, 70); delay(80);
  motor_tone(NOTE_D5, 100, 70); delay(50);
  motor_tone(NOTE_B4, 150, 70); delay(200);

  // ===================== THE TRIUMPHANT ENDING =====================
  // Quick descending/ascending run
  motor_tone(NOTE_C5, 150, 75); delay(150);
  motor_tone(NOTE_G4, 150, 70); delay(150);
  motor_tone(NOTE_E4, 150, 70); delay(150);
  
  motor_tone(NOTE_A4, 100, 70); delay(50);
  motor_tone(NOTE_B4, 100, 70); delay(50);
  motor_tone(NOTE_A4, 100, 70); delay(100);
  
  // Chromatic descent (The "wah-wah-wah" sound)
  motor_tone(NOTE_GS4, 80, 65); delay(40);
  motor_tone(NOTE_AS4, 80, 65); delay(40);
  motor_tone(NOTE_GS4, 80, 65); delay(40);
  
  // // The Final Power Hit
  // motor_tone(NOTE_G4, 100, 80); delay(50);
  // motor_tone(NOTE_F5, 100, 85); delay(50);
  // motor_tone(NOTE_G5, 400, 90); // Long hold ending

  // Safety Stop
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0);
  Serial.println("[MUSIC] Mario Finished");
}






















