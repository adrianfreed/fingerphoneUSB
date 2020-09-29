

/*
    Project     nOmni FingerPhone USB
    @author     Adrian Freed
    @link       github.com/dmadison/Segmented-LED-Display-ASCII
    @license    MIT - Copyright (c) 2020 Adrian Freed

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.

*/
#include <stdint.h>

//
// Object-Oriented Design Patterns and Mechanisms and Principles
//    Inversion of Control
//    Singleton
//    Multilevel Inheritance
//    State Machine
//    Visitor 
//    SOLID
//    

constexpr   struct Midi_to_frequency {
  constexpr Midi_to_frequency() : arr() {
    for (auto i = 0; i != 128; ++i)
      arr[i] = 440.0f * powf(2.0f, i - 69);
  }
  float arr[128];
} mtof;

constexpr   struct Midi_to_unitinterval {
  constexpr Midi_to_unitinterval() : arr() {
    for (auto i = 0; i != 128; ++i)
      arr[i] = i / 127.0f;
  }
  float arr[128];
} mtoui;



#include "MIDIUSB.h"

class MIDIoutput {
  protected:
    void noteOn(byte channel, byte pitch, byte velocity) {
      midiEventPacket_t p = {0x09, 0x90 | (channel % 16), pitch, velocity};
      MidiUSB.sendMIDI(p);
    }
    void noteOff(byte channel, byte pitch, byte velocity) {
      midiEventPacket_t p = {0x08, 0x80 | (channel % 16), pitch, velocity};
      MidiUSB.sendMIDI(p);
    }
    void afterTouch(byte channel, byte pitch, byte velocity) {
      midiEventPacket_t p = {0x0A, 0xA0 | (channel % 16), pitch, velocity};
      MidiUSB.sendMIDI(p);
    }
    void controlChange(byte channel, byte control, byte value) {
      midiEventPacket_t p = {0x0B, 0xB0 | (channel % 16), control, value};
      MidiUSB.sendMIDI(p);
    }
    void midiFlush()
    {
      MidiUSB.flush();
    }
};

static const auto serial_print_note_debug = false;
static const auto serial_print_debug = serial_print_note_debug;

static const auto print_everything = true;

typedef decltype(micros()) micros_t;

// simple cooperative scheduler to discouarge processes from creating jitter in gestures
class Cooperative_scheduler  {
    micros_t start_time;                     // in Microseconds
    static const micros_t default_lease_time = 2000;
    micros_t lease_time;
  protected:
    void begin_timer(micros_t l = default_lease_time) {
      lease_time = l;
      start_time = micros();
    };
    bool i_should_yield() {
      return micros() > (start_time + lease_time);
    };
};



static const auto mux_size = 16; //Multiplexer Chip counts to 16
static const auto n_muxes = 2;  //Number of Muxes on the FingerPhone MIDI: 2*16 = 25 musical notes + 7 touchbuttons


typedef int8_t touch_amount_t;

// create MIDI message stream for keyboard notes and buttons
class Keyboardevents : private MIDIoutput {
    static const int MIDDLEC = 60;
    static const auto number_of_keys = n_muxes * mux_size;

    static constexpr bool isanote[number_of_keys] = { true, true, true, true, true, true, true,
                                                      true, true, true, true, false, false, false,
                                                      true, true, true, true, true, true, true,
                                                      true, true, true, false, false, false, false
                                                    };
    static constexpr  size_t keymap[number_of_keys] =  {
      18, 17, 16, 15, 14, 13, 12, 19,
      20, 21, 22, 23, 24, 0, 6, 1,
      6, 5, 4, 3, 2, 1, 0, 7,
      8, 9, 10, 11, 3, 4, 5, 2
    };
    byte chan;
    int transpose;
    size_t keytranspose[number_of_keys];
    unsigned program_number;
  protected:
    Keyboardevents():
      keytranspose {} // 0
    {
      chan = 0;
      transpose = 0;
      program_number = 0;
    }

    void keytouch(touch_amount_t v, size_t index)
    {
      if (isanote[index])
      {
        noteOn( chan, keymap[index] + MIDDLEC + transpose, ((v + 100) > 127) ? 127 : (v + 100));
        midiFlush();

        keytranspose[index] = transpose;
      }
      Serial.println("+");
      if (serial_print_debug) {
        Serial.print("ON ");
        if (isanote[index]) Serial.print("*");
        Serial.print(keymap[index]);
        Serial.print("  ");
        Serial.println(v);
      }
    }

    void keyrelease(touch_amount_t v, size_t index)
    {
      if (isanote[index])
      {
        noteOff( chan, keymap[index] + MIDDLEC +  keytranspose[index], v);
      }
      Serial.println("-");
      if (serial_print_debug) {
        Serial.print("OFF ");
        if (isanote[index]) Serial.print("*");

        Serial.print(keymap[index]);
        Serial.print("  ");
        Serial.println(v);
      }
    }
    void keystilltouching(touch_amount_t v, size_t index)
    {
      if (isanote[index])
      {
        afterTouch( chan, keymap[index] + MIDDLEC +  keytranspose[index], v);
        midiFlush();
      }

      if (serial_print_debug)
      {
        Serial.print("STILL ");
        if (isanote[index]) Serial.print("*");

        Serial.print(keymap[index]);
        Serial.print("  ");
        Serial.println(v);
      }
    }
};

// process touches and dispatch to Keyboard event
class Keyboard : private Keyboardevents {
    typedef enum {OFF, POSSIBLETOUCH, CERTAINTOUCH, POSSIBLERELEASE, CERTAINRELEASE} state_machine_t;
    static const auto number_of_keys = n_muxes * mux_size;
    state_machine_t states[number_of_keys];
    micros_t longago[number_of_keys], before[number_of_keys];
    struct {
      touch_amount_t a, b, c;
    } medians[number_of_keys];
    auto firsttouch(size_t index, touch_amount_t v) {
      // initialize running median filter
      medians[index].a = medians[index].b = medians[index].c = previous[index] = v;
      return v;
    }
    auto median(size_t index, touch_amount_t v) {
      medians[index].a = medians[index].b;
      medians[index].b = medians[index].c;
      medians[index].c = v;
      if ((medians[index].a <= medians[index].b))
      {
        if ((v >= medians[index].a) && (v <= medians[index].b))
          return v;
      }
      else
      {
        if ((v >= medians[index].b) && (v <= medians[index].a))
          return v;

      }
      if (medians[index].b >= medians[index].a && (medians[index].b <= v))
        return medians[index].b;
      if (medians[index].b <= medians[index].a && (medians[index].b >= v))
        return medians[index].b;

      return medians[index].a;
    }
    int previous[number_of_keys];
    int previousfv[number_of_keys];
    bool apply_median_filter;
    bool apply_averaging_filter;
    touch_amount_t  filter(size_t index, touch_amount_t v)
    {
      int32_t value = v;
      if (apply_median_filter)
        value = median(index, value);
      if (apply_averaging_filter)
      {
        const auto scale = 64;
        const auto average_scale = 100;
        const auto filter_weight = 10; // per cent
        previous[index] = (value * scale * filter_weight + (average_scale - filter_weight) * previous[index]) / average_scale;
        {
          int raw = previous[index];
          const auto breakpoint = 60, breakvalue = 96;
          if (raw < (breakpoint * scale))
            value = (breakvalue * raw) / (127 - breakpoint) ;
          else
            value = breakvalue * scale + ((raw - (breakpoint * scale)) * (127 - breakvalue )) /  (127 - breakpoint);

          value /= scale;
        }

      }


      // clamp to MIDI parameter range
      if (value < 0)
        return 0;
      if (value > 127)
        return 127;
      return value;
    }

  protected:
    Keyboard():
      medians {},
      previous {}
    {

      for (auto i = 0; i < number_of_keys; ++i)
      {
        states[i] = OFF;
      }
      apply_median_filter = true;
      apply_averaging_filter = true;

    }

    void touch_into_sm(bool touched, touch_amount_t v, size_t index)
    {
      const auto now = micros();
      if (touched)
        switch (states[index])
        {
          case  OFF:
            states[index] = POSSIBLETOUCH;
            break;
          case POSSIBLETOUCH:
            states[index] = CERTAINTOUCH;
            keytouch(firsttouch(index, v), index);
            previousfv[index] = 128; // this insures that the first aftertouch message will be sent in the unlikely
            // case it has the same value as the previous notes last aftertouch value
            break;
          case CERTAINTOUCH:
            {
              // only sound out a new message if it is different from the last
              touch_amount_t fv = filter(index, v);
              if (fv != previousfv[index])
              {
                previousfv[index] = fv;
                keystilltouching(fv, index);
                if (serial_print_debug)
                {
                  Serial.print(index);

                  Serial.print(" "); Serial.print(v); Serial.print(" ");
                  Serial.println(fv);
                }
              }
            }
            break;
          case POSSIBLERELEASE:
            states[index] = CERTAINTOUCH;     //bounce
            break;
          case CERTAINRELEASE:
            break;
        }
      else
        switch (states[index])
        {
          case  OFF:
            break;
          case POSSIBLETOUCH:
            states[index] = OFF;  //it was a bounce
            break;
          case CERTAINTOUCH:
            states[index] = POSSIBLERELEASE;
            break;
          case POSSIBLERELEASE:
            keyrelease(medians[index].c, index);
            states[index] = CERTAINRELEASE;
            break;
          case CERTAINRELEASE:
            states[index] = OFF;
            break;
        }
      longago[index] = before[index];
      before[index] = now;
    };
}
;


#include "newadc.h"
// read values from muxed adc's and call Keyboard method to handle touch events
 class Touch_board : private Cooperative_scheduler, private Keyboard   {
    typedef decltype(A0) pin_t;
    static const pin_t reset_pin = 9;
    static constexpr pin_t clk_pins[n_muxes] = { 10, 3};
    static constexpr pin_t adc_pins[n_muxes] = { A2, A1 };
    static const auto fast_adc = true;

    void output_low(pin_t pin)
    {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    };
    void faster_adc_settings()
    {
      if (fast_adc)
      {
        ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |    // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz)
                         ADC_CTRLB_RESSEL_12BIT;         // Set ADC resolution to 12 bits
        while (ADC->STATUS.bit.SYNCBUSY);                // Wait for synchronization
        ADC->SAMPCTRL.reg = 0x00;                        // Set max Sampling Time Length to half divided ADC clock pulse (5.33us)
      }
    };

    typedef decltype(analogRead(A0)) adc_t;

    // this data structure is used primarily to store the acquired key touch data for debugging
    // it reflects the low-level hardware: an array of mux chips each switching 16 values into an ADC pin
    struct keyboardarray {
      adc_t mux[n_muxes];
      micros_t time_stamp[n_muxes];
    } touch[mux_size];
    size_t key_pair_index;
    bool last_key_pair() {
      return key_pair_index == (mux_size - 1);
    };
    void store_new_key_value(size_t index, adc_t v, micros_t t)
    {
      touch[key_pair_index].mux[index] = v;
      touch[key_pair_index].time_stamp[index] = t;
    };
    void next_key_pair()
    {
      key_pair_index++;
      if (key_pair_index > (mux_size - 1))
        key_pair_index = 0;
    };


    // calibration constants according to measurements on hardware
    static const adc_t touch_threshold_top = 2100;
    static const adc_t touch_threshold_bottom = 1000;
    uint_least8_t scale(adc_t x)
    {
      int_least32_t r = 127 - (127l * (x - touch_threshold_bottom)) 
              / (touch_threshold_top - touch_threshold_bottom);
      if (r < 0)
        r = 0;
      if (r > 127)
        r = 127;
      return r;
    }

    uint_least8_t acquire_next_keypair()  // returns the number of keys touched
    {
      decltype(acquire_next_keypair()) count = 0; // assert(sizeof(decltype(count))*8 > mux_size);


      // calibration constants according to measurements on hardware
      const unsigned shortsettletime{100}; // microseconds
      const unsigned longsettletime{400}; // microseconds
      for (size_t muxindex = 0; muxindex < n_muxes; ++muxindex)
      {
        digitalWrite(clk_pins[muxindex], HIGH);

        delayMicroseconds(shortsettletime);
        adc_t x = analogRead(adc_pins[muxindex]);
        bool touched = false;
        if ((x < touch_threshold_top))
        {
          ++count;
          delayMicroseconds(longsettletime);
          x = analogRead(adc_pins[muxindex]);
          touched = true;
        }
        store_new_key_value(muxindex, x, micros());
        touch_into_sm(touched, scale(x), muxindex * mux_size + key_pair_index);


        if (last_key_pair())
        {
          digitalWrite(reset_pin, HIGH);
          delayMicroseconds(2);

        }
        digitalWrite(clk_pins[muxindex], LOW);

      }
      if (last_key_pair())
      {
        digitalWrite(reset_pin, LOW);
        delayMicroseconds(2);

      }
      next_key_pair();
      return count;
    };


    Touch_board():
      touch {}
    {
      for (const auto v : clk_pins)
        output_low(v);
      output_low(reset_pin);

      faster_adc_settings();
      delay(3);
      analogReadResolution(12);
      key_pair_index = 0;

    };
    
 public:
    // creates itself once
    static Touch_board& instance()
    {
      static Touch_board Inst;
        return Inst;
    }
    Touch_board(Touch_board &other) = delete;
    void operator=(const class Touch_board &) = delete;

    size_t scan() // returns number of keys touched
    {
      decltype(scan()) count = 0;
      begin_timer();
      for (size_t i = 0; i < mux_size; ++i)
      {
        if (i_should_yield())
            break;
        count += acquire_next_keypair();
      };
      return count;
    }

    void print()
    {
      auto cnt = 0;
      for (const auto  &key : touch)
      {
        for (const auto val : key.mux)
        {
          if (!print_everything)
          {
            if ((val > touch_threshold_top))
            {
              Serial.print("--" );
            }
            else
            {
              ++cnt;
              Serial.print(scale(val));
            }
          }
          else
            Serial.print(val);
          Serial.print("\t");
        }
        Serial.println();
      }
      Serial.print(cnt);
      Serial.println("***");
    }
};

class Touch_board &FingerPhone = Touch_board::instance();


void setup() {
  if (serial_print_debug)
    Serial.begin(9600);
}

void loop() {
  if (FingerPhone.scan() > 0)
  {
    if (serial_print_debug)
    {
      FingerPhone.print();
    }
  }
}
