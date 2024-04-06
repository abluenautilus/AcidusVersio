#pragma once
#include <string>
#include <map>
#include "Scales.hpp"

#define VOLTS_PER_SEMITONE .08333
#define SEMITONES_PER_OCTAVE 12
#define BASE_OCTAVE 2

class Note {

    public:

        float voltage;
        std::string noteName;
        uint8_t octave;
        uint8_t noteNumMIDI;
        uint8_t toneNum;
        bool muted;

        std::map<std::string,int> noteToNum;
        std::map<int,std::string> numToNote;

        //Create a note with MIDI-style note number
        Note(int createWithNum) {
            makeNoteMap();
            noteNumMIDI = createWithNum;
            toneNum = noteNumMIDI % SEMITONES_PER_OCTAVE + 1;
            noteName = numToNote[toneNum];
            octave = noteNumMIDI / SEMITONES_PER_OCTAVE -1;
            setVoltage();
        }

        //Create a note with a string and octave number
         Note(std::string myNote, int myOctave) {
            makeNoteMap();
            noteName = myNote;
            octave = myOctave;
            toneNum = noteToNum[noteName];
            noteNumMIDI = 12*(octave+1)+(toneNum-1);
            setVoltage();
        }

        //Create a rest note
        Note(std::string myNote) {
            if (myNote=="rest"){
                noteName = "rest";
                octave = 4;
                toneNum = 0;
                noteNumMIDI = 0;
            }
        }

        //Default constructor
        Note() {
            octave = 4;
            toneNum = 1;
            noteName = "C4";
            noteNumMIDI = 60;
            setVoltage();
        }
        
        std::string getNoteNameFromNum(int theNoteNum) {
            
            return numToNote[theNoteNum];

        }

        int getNoteNumFromName(std::string theNoteName) {
            
            return noteToNum[theNoteName];

        }
        //Create maps to map back and forth between note names and nums
        void makeNoteMap() {

            noteToNum.insert(std::pair<std::string,int>("C",1));
            noteToNum.insert(std::pair<std::string,int>("C#",2));
            noteToNum.insert(std::pair<std::string,int>("D",3));
            noteToNum.insert(std::pair<std::string,int>("Eb",4));
            noteToNum.insert(std::pair<std::string,int>("E",5));
            noteToNum.insert(std::pair<std::string,int>("F",6));
            noteToNum.insert(std::pair<std::string,int>("F#",7));
            noteToNum.insert(std::pair<std::string,int>("G",8));
            noteToNum.insert(std::pair<std::string,int>("G#",9));
            noteToNum.insert(std::pair<std::string,int>("A",10));
            noteToNum.insert(std::pair<std::string,int>("Bb",11));
            noteToNum.insert(std::pair<std::string,int>("B",12));

            numToNote.insert(std::pair<int,std::string>(1,"C"));
            numToNote.insert(std::pair<int,std::string>(2,"C#"));
            numToNote.insert(std::pair<int,std::string>(3,"D"));
            numToNote.insert(std::pair<int,std::string>(4,"Eb"));
            numToNote.insert(std::pair<int,std::string>(5,"E"));
            numToNote.insert(std::pair<int,std::string>(6,"F"));
            numToNote.insert(std::pair<int,std::string>(7,"F#"));
            numToNote.insert(std::pair<int,std::string>(8,"G"));
            numToNote.insert(std::pair<int,std::string>(9,"G#"));
            numToNote.insert(std::pair<int,std::string>(10,"A"));
            numToNote.insert(std::pair<int,std::string>(11,"Bb"));
            numToNote.insert(std::pair<int,std::string>(12,"B"));

        }

        //Set MIDI note number
        void setMIDInum() {
            
            int toneNum = noteToNum[noteName];
            noteNumMIDI = ((octave+1)*12)+toneNum-1;

        }

        //Compute the output voltage
        void setVoltage() {
            if (noteName != "rest") {
                voltage = ((toneNum - 1) * VOLTS_PER_SEMITONE) + octave - BASE_OCTAVE;
            } else {
                voltage = 0;
            }
        }


};