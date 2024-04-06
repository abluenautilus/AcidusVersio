#include "daisy_versio.h"
#include "daisysp.h"
#include "rosic_Open303.h"
#include "Note.hpp"

using namespace daisy;
using namespace daisysp;
using namespace rosic;

DaisyVersio hw;
Open303 tb303;

enum param_mode {
    BABYFISH,
    NORMAL,
    DEVILFISH
};

uint8_t mode  = BABYFISH;
float sampleRate;
float prevTrigger;
float tuning;
bool slideToNextNote;

bool inCalibration;
const int calibration_max = 65536;
uint16_t calibration_Offset = 2116;
uint16_t calibration_UnitsPerVolt = 12680;
const uint16_t base_octave = 1;

// Persistence
struct Settings {
    float calibration_Offset;
    float calibration_UnitsPerVolt;
    bool operator!=(const Settings& a) {
        return a.calibration_UnitsPerVolt != calibration_UnitsPerVolt;
    }
};
Settings& operator* (const Settings& settings) { return *settings; }
PersistentStorage<Settings> storage(hw.seed.qspi);

void saveData() {
    Settings &localSettings = storage.GetSettings();
    localSettings.calibration_Offset = calibration_Offset;
    localSettings.calibration_UnitsPerVolt = calibration_UnitsPerVolt;
    storage.Save();
}

void loadData() {
    Settings &localSettings = storage.GetSettings();
    calibration_Offset = localSettings.calibration_Offset;
    calibration_UnitsPerVolt = localSettings.calibration_UnitsPerVolt;
}

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    float sample = 0.0;
    for(size_t i = 0; i < size; i += 1)
    {
        sample = (float) tb303.getSample();

        OUT_L[i] = sample;
        OUT_R[i] = sample;

    }
    // Set lights on right side, yellowish
    hw.SetLed(hw.LED_2,sample*2,sample*2,0);
    hw.SetLed(hw.LED_3,sample*2,sample*2,0);
    hw.UpdateLeds();

}

void doTriggerReceived() {

    // Convert CV from tuning jack to midi note number using calibrated scaling
    Note currentNote = Note();
    float raw_cv = hw.knobs[hw.KNOB_0].GetRawValue();
    float volts = DSY_CLAMP((calibration_max - calibration_Offset - raw_cv)/calibration_UnitsPerVolt,0,5.0);
    float midi_pitch = round(((volts) * 12) + (12 * base_octave));
    Note current_note = Note(midi_pitch);

    // Manage note list
    if (!slideToNextNote) {
        tb303.allNotesOff();
    } else {
        tb303.trimNoteList();
        // Retriggering these gives slides a new envelope, uncomment for different slide effect
        // tb303.mainEnv.trigger();
        // tb303.ampEnv.reset();
    }

    int velocity = 150; // What should base note velocity be? 
    tb303.noteOn(current_note.noteNumMIDI, velocity);

}

void doCalibration() {

    inCalibration = true;
    uint8_t numSamples = 10; // Take this many samples per step

    // STEP ONE - RELEASE BUTTON
    hw.tap.Debounce();
    hw.SetLed(0,1,1,1);
    hw.SetLed(1,1,1,1);
    hw.SetLed(2,1,1,1);
    hw.SetLed(3,1,1,1); 
    hw.UpdateLeds();
    while (hw.tap.RawState()) {
        hw.tap.Debounce();
    }
    hw.SetLed(0,0,1,0);
    hw.SetLed(1,0,0,0);
    hw.SetLed(2,0,0,0);
    hw.SetLed(3,0,0,0);
    hw.UpdateLeds();
    while (!hw.tap.RisingEdge()) {
        hw.tap.Debounce();
    }
    while (!hw.tap.FallingEdge()) {
        hw.tap.Debounce();
    }
    float zero_value = hw.knobs[0].GetRawValue();

    // STEP TWO - ONE VOLT
    hw.SetLed(0,0,0,1);
    hw.SetLed(1,0,0,1);
    hw.SetLed(2,0,0,0);
    hw.SetLed(3,0,0,0);
    hw.UpdateLeds();
    while (!hw.tap.RisingEdge()) {
        hw.tap.Debounce();
    }
    while (!hw.tap.FallingEdge()) {
        hw.tap.Debounce();
    }
    float total = 0;
    for (uint8_t x = 0; x < numSamples; ++x) {
        hw.knobs[0].Process();
        total = total + hw.knobs[0].GetRawValue();
    }
    float onevolt_value = total/numSamples;

    // STEP TWO - ONE THREE VOLTS
    hw.SetLed(0,0,1,1);
    hw.SetLed(1,0,1,1);
    hw.SetLed(2,0,1,1);
    hw.SetLed(3,0,0,0);
    hw.UpdateLeds();
    while (!hw.tap.RisingEdge()) {
        hw.tap.Debounce();
    }
    while (!hw.tap.FallingEdge()) {
        hw.tap.Debounce();
    }
    
    total = 0;
    for (uint8_t x = 0; x < numSamples; ++x) {
        hw.knobs[0].Process();
        total = total + hw.knobs[0].GetRawValue();
    }
    float threevolt_value = total/numSamples;

    // Estimate calibration values
    uint16_t offset = calibration_max - zero_value;
    uint16_t first_estimate = calibration_max - offset - onevolt_value;
    float second_estimate = (float)(calibration_max - offset - threevolt_value)/3;
    float avg_estimate = (first_estimate + second_estimate)/2;

    // Save values to QSPI
    calibration_Offset = offset;
    calibration_UnitsPerVolt = (uint16_t)avg_estimate;
    saveData();

    inCalibration = false;

}


int main(void)
{

    // Initialize Versio hardware and start audio, ADC
    hw.Init();
    hw.seed.StartLog(false);
    hw.StartAdc();

    hw.SetLed(0,1,1,1);
    hw.SetLed(1,1,1,1);
    hw.SetLed(2,1,1,1);
    hw.SetLed(3,1,1,1);
    hw.UpdateLeds();

    sampleRate = hw.seed.AudioSampleRate();

    hw.StartAudio(AudioCallback);

    Settings defaults;
    defaults.calibration_Offset = 2116;
    defaults.calibration_UnitsPerVolt = 12680;
    storage.Init(defaults);

    loadData();

    // Set up 303
    tb303.setSampleRate(sampleRate);
    tb303.setVolume(0);
    tb303.setDecay(2000);

    // Check for calibration routine
    hw.ProcessAllControls();
    hw.tap.Debounce();
    if (hw.sw[0].Read() == hw.sw->POS_RIGHT && hw.sw[1].Read() == hw.sw->POS_RIGHT) {
        if (hw.tap.RawState()) { 
            doCalibration();
        }
    }

    while(1) {

        hw.ProcessAllControls();
        hw.tap.Debounce();
        hw.UpdateLeds();

        // Read knob values
        float decay     = hw.GetKnobValue(DaisyVersio::KNOB_1);
        float cutoff    = hw.GetKnobValue(DaisyVersio::KNOB_2);
        float slide     = DSY_CLAMP(hw.GetKnobValue(DaisyVersio::KNOB_3),0,0.5);
        float resonance = hw.GetKnobValue(DaisyVersio::KNOB_4);
        float envmod    = hw.GetKnobValue(DaisyVersio::KNOB_5);
        float accent    = hw.GetKnobValue(DaisyVersio::KNOB_6);

        // Map knob values onto parameters depending on mode
        switch (mode) {
            case BABYFISH:
                tb303.setResonance(resonance * 80 + 10);
                tb303.setCutoff(cutoff * 4000);
                tb303.setAccentDecay(decay * 1000);
                tb303.setAccent(accent * 40);
                tb303.setEnvMod(envmod * 80 + 10);
            break;
            case NORMAL:
                tb303.setResonance(resonance * 90);
                tb303.setCutoff(cutoff * 5000);
                tb303.setAccentDecay(decay * 2000);
                tb303.setAccent(accent * 50);
                tb303.setEnvMod(envmod * 100);
            break;
            case DEVILFISH:
                tb303.setResonance(resonance * 100);
                tb303.setCutoff(cutoff * 10000);
                tb303.setAccentDecay(decay * 30000);
                tb303.setAccent(accent * 100);
                tb303.setEnvMod(envmod * 100);
            break;
        }

        // Activate slide
        if (slide > 0.1) {
            slideToNextNote = true;
        } else {
            slideToNextNote = false;
        }

        // Read in trigger, comes from FSU or button
        bool triggerIn = hw.Gate();
        if (triggerIn &! prevTrigger) {
            doTriggerReceived();
        }
        if(hw.tap.RisingEdge()) {
            doTriggerReceived();
        }

        // Top switch
         if (hw.sw[0].Read() == hw.sw->POS_LEFT) {
                tb303.setWaveform(0);
         } else if (hw.sw[0].Read() == hw.sw->POS_CENTER) {
                tb303.setWaveform(0.5);
         } else {
                tb303.setWaveform(1.0);
         }

        // Bottom switch
         if (hw.sw[1].Read() == hw.sw->POS_LEFT) {
                mode = BABYFISH;
         } else if (hw.sw[1].Read() == hw.sw->POS_CENTER) {
                mode = NORMAL;
         } else {
                mode = DEVILFISH;
         }
        // Set left side lights, orangish
        if (!inCalibration) {
            float env = tb303.mainEnv.getSample();
            hw.SetLed(hw.LED_0,triggerIn,triggerIn/1.7,0);
            hw.SetLed(hw.LED_1,triggerIn,triggerIn/1.7,0);
            hw.UpdateLeds();
        }

    }

}

