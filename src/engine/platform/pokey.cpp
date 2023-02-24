/**
 * Furnace Tracker - multi-system chiptune tracker
 * Copyright (C) 2021-2023 tildearrow and contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "pokey.h"
#include "../engine.h"
#include "../../ta-log.h"

#define rWrite(a,v) if (!skipRegisterWrites) {writes.emplace(a,v); if (dumpWrites) {addWrite(a,v);} }

#define CHIP_DIVIDER 1

const char* regCheatSheetPOKEY[]={
  "AUDF1", "0",
  "AUDC1", "1",
  "AUDF2", "2",
  "AUDC2", "3",
  "AUDF3", "4",
  "AUDC3", "5",
  "AUDF4", "6",
  "AUDC4", "7",
  "AUDCTL", "8",
  NULL
};

const unsigned char waveMap[8]={
  0, 1, 2, 3, 4, 5, 6, 6
};

const char** DivPlatformPOKEY::getRegisterSheet() {
  return regCheatSheetPOKEY;
}

void DivPlatformPOKEY::acquire(short** buf, size_t len) {
  if (useAltASAP) {
    acquireASAP(buf[0],len);
  } else {
    acquireMZ(buf[0],len);
  }
}

void DivPlatformPOKEY::acquireMZ(short* buf, size_t len) {
  for (size_t h=0; h<len; h++) {
    while (!writes.empty()) {
      QueuedWrite w=writes.front();
      Update_pokey_sound_mz(&pokey,w.addr,w.val,0);
      regPool[w.addr&0x0f]=w.val;
      writes.pop();
    }

    mzpokeysnd_process_16(&pokey,&buf[h],1);

    if (++oscBufDelay>=14) {
      oscBufDelay=0;
      oscBuf[0]->data[oscBuf[0]->needle++]=pokey.outvol_0<<11;
      oscBuf[1]->data[oscBuf[1]->needle++]=pokey.outvol_1<<11;
      oscBuf[2]->data[oscBuf[2]->needle++]=pokey.outvol_2<<11;
      oscBuf[3]->data[oscBuf[3]->needle++]=pokey.outvol_3<<11;
    }
  }
}

void DivPlatformPOKEY::acquireASAP(short* buf, size_t len) {
  while (!writes.empty()) {
    QueuedWrite w=writes.front();
    altASAP.write(w.addr, w.val);
    writes.pop();
  }

  for (size_t h=0; h<len; h++) {
    if (++oscBufDelay>=2) {
      oscBufDelay=0;
      buf[h]=altASAP.sampleAudio(oscBuf);
    } else {
      buf[h]=altASAP.sampleAudio();
    }
  }
}

void DivPlatformPOKEY::tick(bool sysTick) {
  for (int i=0; i<4; i++) {
    chan[i].std.next();
    if (chan[i].std.vol.had) {
      chan[i].outVol=VOL_SCALE_LINEAR(chan[i].vol&15,MIN(15,chan[i].std.vol.val),15);
      chan[i].ctlChanged=true;
    }
    if (NEW_ARP_STRAT) {
      chan[i].handleArp();
    } else if (chan[i].std.arp.had) {
      if (!chan[i].inPorta) {
        chan[i].baseFreq=NOTE_PERIODIC(parent->calcArp(chan[i].note,chan[i].std.arp.val));
      }
      chan[i].freqChanged=true;
    }
    if (chan[i].std.duty.had) {
      audctl=chan[i].std.duty.val;
      audctlChanged=true;
    }
    if (chan[i].std.wave.had) {
      chan[i].wave=chan[i].std.wave.val;
      chan[i].ctlChanged=true;
      chan[i].freqChanged=true;
    }
    if (chan[i].std.pitch.had) {
      if (chan[i].std.pitch.mode) {
        chan[i].pitch2+=chan[i].std.pitch.val;
        CLAMP_VAR(chan[i].pitch2,-32768,32767);
      } else {
        chan[i].pitch2=chan[i].std.pitch.val;
      }
      chan[i].freqChanged=true;
    }
  }

  if (audctlChanged) {
    audctlChanged=false;
    rWrite(8,audctl);
    for (int i=0; i<4; i++) {
      chan[i].freqChanged=true;
      chan[i].ctlChanged=true;
    }
  }

  for (int i=0; i<4; i++) {
    // variables for pitch calculation, divisors must never be 0!
    double divisor = 1;
    int coarse_divisor = 1;
    int cycle = 1;

    // register variables
    bool CLOCK_15 = audctl & 0x01;
    //bool HPF_CH24 = audctl & 0x02;
    //bool HPF_CH13 = audctl & 0x04;
    bool JOIN_34 = audctl & 0x08;
    bool JOIN_12 = audctl & 0x10;
    bool CH3_179 = audctl & 0x20;
    bool CH1_179 = audctl & 0x40;
    //bool POLY9 = audctl & 0x80;

    // combined modes for some special output... 
    bool JOIN_16BIT = (((JOIN_12 && CH1_179) && i == 1) || ((JOIN_34 && CH3_179) && i == 3)) ? 1 : 0;
    if (((JOIN_12 && CH1_179) && i == 0) || ((JOIN_34 && CH3_179) && i == 2)) continue;	// Override in 16-bit mode
    bool CLOCK_179 = ((CH1_179 && i == 0) || (CH3_179 && i == 2)) ? 1 : 0;
    if (JOIN_16BIT || CLOCK_179) CLOCK_15 = 0;	// override, these 2 take priority over 15khz mode if they are enabled at the same time

    if (JOIN_16BIT) cycle = 7;
    else if (CLOCK_179) cycle = 4;
    else coarse_divisor = (CLOCK_15) ? 114 : 28;
  
    if (chan[i].freqChanged || chan[i].keyOn || chan[i].keyOff) {
	// use the modulo flags to make sure the correct timbre will be output
	switch (chan[i].wave) 
	{
	case 1:
	case 3:
		divisor = 31;	// Bell tones, not MOD31
		break;
	case 2:
		divisor = 77.5;	// Smooth tones, MOD3 but not MOD5 or MOD31
		break;
	case 6:
		divisor = 2.5;	// Buzzy tones, MOD3 but not MOD5
		break;
	case 7:
		divisor = 7.5;	// Gritty tones, neither MOD3 or MOD5
		if (CLOCK_15) {
			divisor = 2.5;	// Buzzy tones are always output in 15khz mode with Distortion C
			chan[i].wave = 6;	// I really don't wanna do it that way but Furnace is forcing my hand...
		}
		break;
	default:
		//Distortion A is assumed if no valid parameter is supplied 
		break;
	}
	
      chan[i].freq=parent->calcFreq(chan[i].baseFreq,chan[i].pitch,chan[i].fixedArp?chan[i].baseNoteOverride:chan[i].arpOff,chan[i].fixedArp,true,0,chan[i].pitch2,chipClock,(coarse_divisor*divisor))-cycle;
      
      // insert whatever delta method that could be suitable here... 
      bool MOD3 = ((chan[i].freq + cycle) % 3 == 0);
      bool MOD5 = ((chan[i].freq + cycle) % 5 == 0);
      //bool MOD7 = ((chan[i].freq + cycle) % 7 == 0);
      //bool MOD15 = ((chan[i].freq + cycle) % 15 == 0);
      bool MOD31 = ((chan[i].freq + cycle) % 31 == 0);
      //bool MOD73 = ((chan[i].freq + cycle) % 73 == 0);
 
      // Adjustments by modulo
      // TODO: find a way to get the target pitch (in Hz) to process this part...
      switch (chan[i].wave)
      {
      	case 1:
      	case 3:
      		if (MOD31) chan[i].freq = deltaFreq(chan[i].truePitch, chan[i].freq, chipClock, coarse_divisor, divisor, cycle, chan[i].wave);
      		break;
      	case 2:
      		if (!(MOD3 || CLOCK_15) || MOD5) chan[i].freq = deltaFreq(chan[i].truePitch, chan[i].freq, chipClock, coarse_divisor, divisor, cycle, chan[i].wave);
      		break;
      	case 6:
      		if (!(MOD3 || CLOCK_15) || MOD5) chan[i].freq = deltaFreq(chan[i].truePitch, chan[i].freq, chipClock, coarse_divisor, divisor, cycle, chan[i].wave);
      		if (!JOIN_16BIT && !CLOCK_15 && chan[i].freq > 0xFF) {
      			divisor = 7.5;		// Hack: force Gritty tones for lower notes
      			chan[i].wave = 7;	// I really don't wanna do it that way but Furnace is forcing my hand...
      			chan[i].freq=parent->calcFreq(chan[i].baseFreq,chan[i].pitch,chan[i].fixedArp?chan[i].baseNoteOverride:chan[i].arpOff,chan[i].fixedArp,true,0,chan[i].pitch2,chipClock,(coarse_divisor*divisor))-cycle;
      			MOD3 = ((chan[i].freq + cycle) % 3 == 0);
     			MOD5 = ((chan[i].freq + cycle) % 5 == 0);
      			goto Dist_E;		// Teehee
      		}
      		break;
      	case 7:
      	Dist_E:
      		if (MOD3 || MOD5) chan[i].freq = deltaFreq(chan[i].truePitch, chan[i].freq, chipClock, coarse_divisor, divisor, cycle, chan[i].wave);
      		break;
      }

      if (chan[i].freq < 0) chan[i].freq = 0;
      if (!JOIN_16BIT && chan[i].freq > 0xFF) chan[i].freq = 0xFF;
      if (JOIN_16BIT && chan[i].freq > 0xFFFF) chan[i].freq = 0xFFFF;
      
      // write frequency
      if (JOIN_16BIT) {
      	rWrite(i<<1,chan[i].freq>>8);		// 16-bit MSB
      	rWrite((i-1)<<1,chan[i].freq&0xff);	// 16-bit LSB
      } else {
      	rWrite(i<<1,chan[i].freq&0xff);		// 8-Bit, same to 16-bit LSB
      }

      if (chan[i].keyOff) {
        chan[i].ctlChanged=true;
      }
      if (chan[i].keyOn) chan[i].keyOn=false;
      if (chan[i].keyOff) chan[i].keyOff=false;
      chan[i].freqChanged=false;
    }
    if (chan[i].ctlChanged) {
      unsigned char val=((chan[i].active && !isMuted[i])?(chan[i].outVol&15):0)|(waveMap[chan[i].wave&7]<<5);
      chan[i].ctlChanged=false;

      // write waveform/volume
      if (JOIN_16BIT) {
      	rWrite(1+(i<<1),val);		// 16-bit MSB
      	rWrite(1+((i-1)<<1),0);		// 16-bit LSB
      } else {
      	rWrite(1+(i<<1),val);		// 8-Bit, same to 16-bit LSB
      }
    }
  }
}

        int DivPlatformPOKEY::deltaFreq(double pitch, int freq, double clock, int coarse_divisor, double divisor, int cycle, int wave)
        {
            // Begin from the currently invalid Freq
            int tmp_freq_up = freq;
            int tmp_freq_down = freq;

            // The Distortion and Timbre are made up from different combinations, invalid combinations will produce garbage tones
            switch (wave)
            {
                //case Timbre.SMOOTH_4:   // Distortion 4 Smooth, verify MOD3 integrity
                case 2:
                    for (int o = 0; o < 6; o++)
                    {
                        if ((tmp_freq_up + cycle) % 3 != 0 || (tmp_freq_up + cycle) % 5 == 0 || (tmp_freq_up + cycle) % 31 == 0) tmp_freq_up++;
                        if ((tmp_freq_down + cycle) % 3 != 0 || (tmp_freq_down + cycle) % 5 == 0 || (tmp_freq_down + cycle) % 31 == 0) tmp_freq_down--;
                    }
                    break;

                //case Timbre.BUZZY_C:    // Distortion C Buzzy, verify MOD3 integrity
                case 6:
                    if (coarse_divisor == 114)  // 15kHz mode
                    {
                        for (int o = 0; o < 3; o++) // MOD5 must be avoided!
                        {
                            if ((tmp_freq_up + cycle) % 5 == 0) tmp_freq_up++;
                            if ((tmp_freq_down + cycle) % 5 == 0) tmp_freq_down--;
                        }
                        break;
                    }
                    for (int o = 0; o < 6; o++)
                    {
                        if ((tmp_freq_up + cycle) % 3 != 0 || (tmp_freq_up + cycle) % 5 == 0) tmp_freq_up++;
                        if ((tmp_freq_down + cycle) % 3 != 0 || (tmp_freq_down + cycle) % 5 == 0) tmp_freq_down--;
                    }
                    break;

                //case Timbre.GRITTY_C:   // Distortion C Gritty, verify neither MOD3 or MOD5 is used
                case 7:
                    if (coarse_divisor == 114)  // 15kHz mode
                    {
                        for (int o = 0; o < 3; o++) // MOD5 must be avoided!
                        {
                            if ((tmp_freq_up + cycle) % 5 == 0) tmp_freq_up++;
                            if ((tmp_freq_down + cycle) % 5 == 0) tmp_freq_down--;
                        }
                        break;
                    }
                    for (int o = 0; o < 6; o++)
                    {
                        if ((tmp_freq_up + cycle) % 3 == 0 || (tmp_freq_up + cycle) % 5 == 0) tmp_freq_up++;
                        if ((tmp_freq_down + cycle) % 3 == 0 || (tmp_freq_down + cycle) % 5 == 0) tmp_freq_down--;
                    }
                    break;

                default:    // Simpliest delta method, shift up and down by 1
                    tmp_freq_up++;
                    tmp_freq_down--;
                    break;
            }

            // First delta, up
            double tmp_pitch_up = pitch - getPitch(tmp_freq_up, coarse_divisor, divisor, cycle);

            // Second delta, down
            double tmp_pitch_down = getPitch(tmp_freq_down, coarse_divisor, divisor, cycle) - pitch;

            // Return whichever is nearest to the wanted pitch
            return tmp_pitch_down - tmp_pitch_up > 0 ? tmp_freq_up : tmp_freq_down;
        }

double DivPlatformPOKEY::getTruePitch(int semitone) 
{
	double ratio = pow(2.0, 1.0 / 12.0);
	return (parent->song.tuning / 64) * pow(ratio, semitone + 3);
}

double DivPlatformPOKEY::getPitch(int audf, int coarse_divisor, double divisor, int cycle)
{
	return ((chipClock / (coarse_divisor * divisor)) / (audf + cycle)) / 2;
}

int DivPlatformPOKEY::getFreq(double pitch, int coarse_divisor, double divisor, int cycle)
{
	return (int)round(((chipClock / (coarse_divisor * divisor)) / (2 * pitch)) - cycle);
}

int DivPlatformPOKEY::dispatch(DivCommand c) {
  switch (c.cmd) {
    case DIV_CMD_NOTE_ON: {
      DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_POKEY);
      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].baseFreq=NOTE_PERIODIC(c.value);
        chan[c.chan].freqChanged=true;
        chan[c.chan].note=c.value;
        chan[c.chan].truePitch = getTruePitch(c.value);
      }
      chan[c.chan].active=true;
      chan[c.chan].keyOn=true;
      chan[c.chan].macroInit(ins);
      if (!parent->song.brokenOutVol && !chan[c.chan].std.vol.will) {
        chan[c.chan].outVol=chan[c.chan].vol;
        chan[c.chan].ctlChanged=true;
      }
      chan[c.chan].insChanged=false;
      break;
    }
    case DIV_CMD_NOTE_OFF:
      chan[c.chan].active=false;
      chan[c.chan].keyOff=true;
      chan[c.chan].macroInit(NULL);
      break;
    case DIV_CMD_NOTE_OFF_ENV:
    case DIV_CMD_ENV_RELEASE:
      chan[c.chan].std.release();
      break;
    case DIV_CMD_INSTRUMENT:
      if (chan[c.chan].ins!=c.value || c.value2==1) {
        chan[c.chan].ins=c.value;
        chan[c.chan].insChanged=true;
      }
      break;
    case DIV_CMD_VOLUME:
      if (chan[c.chan].vol!=c.value) {
        chan[c.chan].vol=c.value;
        if (!chan[c.chan].std.vol.has) {
          chan[c.chan].outVol=c.value;
          if (chan[c.chan].active) {
            chan[c.chan].ctlChanged=true;
          }
        }
      }
      break;
    case DIV_CMD_GET_VOLUME:
      if (chan[c.chan].std.vol.has) {
        return chan[c.chan].vol;
      }
      return chan[c.chan].outVol;
      break;
    case DIV_CMD_PITCH:
      chan[c.chan].pitch=c.value;
      chan[c.chan].freqChanged=true;
      break;
    case DIV_CMD_WAVE:
      chan[c.chan].wave=c.value;
      chan[c.chan].ctlChanged=true;
      break;
    case DIV_CMD_STD_NOISE_MODE:
      audctl=c.value&0xff;
      audctlChanged=true;
      break;
    case DIV_CMD_NOTE_PORTA: {
      int destFreq=NOTE_PERIODIC(c.value2);
      bool return2=false;
      if (destFreq>chan[c.chan].baseFreq) {
        chan[c.chan].baseFreq+=c.value;
        if (chan[c.chan].baseFreq>=destFreq) {
          chan[c.chan].baseFreq=destFreq;
          return2=true;
        }
      } else {
        chan[c.chan].baseFreq-=c.value;
        if (chan[c.chan].baseFreq<=destFreq) {
          chan[c.chan].baseFreq=destFreq;
          return2=true;
        }
      }
      chan[c.chan].freqChanged=true;
      if (return2) {
        chan[c.chan].inPorta=false;
        return 2;
      }
      break;
    }
    case DIV_CMD_LEGATO:
      chan[c.chan].baseFreq=NOTE_PERIODIC(c.value);
      chan[c.chan].freqChanged=true;
      chan[c.chan].note=c.value;
      break;
    case DIV_CMD_PRE_PORTA:
      if (chan[c.chan].active && c.value2) {
        if (parent->song.resetMacroOnPorta) chan[c.chan].macroInit(parent->getIns(chan[c.chan].ins,DIV_INS_POKEY));
      }
      if (!chan[c.chan].inPorta && c.value && !parent->song.brokenPortaArp && chan[c.chan].std.arp.will && !NEW_ARP_STRAT) chan[c.chan].baseFreq=NOTE_PERIODIC(chan[c.chan].note);
      chan[c.chan].inPorta=c.value;
      break;
    case DIV_CMD_GET_VOLMAX:
      return 15;
      break;
    case DIV_CMD_MACRO_OFF:
      chan[c.chan].std.mask(c.value,true);
      break;
    case DIV_CMD_MACRO_ON:
      chan[c.chan].std.mask(c.value,false);
      break;
    case DIV_ALWAYS_SET_VOLUME:
      return 1;
      break;
    default:
      break;
  }
  return 1;
}

void DivPlatformPOKEY::muteChannel(int ch, bool mute) {
  isMuted[ch]=mute;
  chan[ch].ctlChanged=true;
}

void DivPlatformPOKEY::forceIns() {
  for (int i=0; i<4; i++) {
    chan[i].insChanged=true;
    chan[i].ctlChanged=true;
    chan[i].freqChanged=true;
  }
  audctlChanged=true;
}

void* DivPlatformPOKEY::getChanState(int ch) {
  return &chan[ch];
}

DivMacroInt* DivPlatformPOKEY::getChanMacroInt(int ch) {
  return &chan[ch].std;
}

DivDispatchOscBuffer* DivPlatformPOKEY::getOscBuffer(int ch) {
  return oscBuf[ch];
}

unsigned char* DivPlatformPOKEY::getRegisterPool() {
  if (useAltASAP) {
    return const_cast<unsigned char*>(altASAP.getRegisterPool());
  } else {
    return regPool;
  }
}

int DivPlatformPOKEY::getRegisterPoolSize() {
  return 9;
}

void DivPlatformPOKEY::reset() {
  while (!writes.empty()) writes.pop();
  memset(regPool,0,16);
  for (int i=0; i<4; i++) {
    chan[i]=DivPlatformPOKEY::Channel();
    chan[i].std.setEngine(parent);
  }
  if (dumpWrites) {
    addWrite(0xffffffff,0);
  }

  if (useAltASAP) {
    altASAP.reset();
  } else {
    ResetPokeyState(&pokey);
  }

  audctl=0;
  audctlChanged=true;
}

bool DivPlatformPOKEY::keyOffAffectsArp(int ch) {
  return true;
}

float DivPlatformPOKEY::getPostAmp() {
  return 2.0f;
}

void DivPlatformPOKEY::notifyInsDeletion(void* ins) {
  for (int i=0; i<4; i++) {
    chan[i].std.notifyInsDeletion((DivInstrument*)ins);
  }
}

void DivPlatformPOKEY::setFlags(const DivConfig& flags) {
  if (flags.getInt("clockSel",0)) {
    chipClock=COLOR_PAL*2.0/5.0;
  } else {
    chipClock=COLOR_NTSC/2.0;
  }
  CHECK_CUSTOM_CLOCK;

  if (useAltASAP) {
    rate=chipClock/7;
    for (int i=0; i<4; i++) {
      oscBuf[i]->rate=rate/2;
    }
    altASAP.init(chipClock,rate);
    altASAP.reset();
  } else {
    rate=chipClock;
    for (int i=0; i<4; i++) {
      oscBuf[i]->rate=rate/14;
    }
  }
}

void DivPlatformPOKEY::poke(unsigned int addr, unsigned short val) {
  rWrite(addr,val);
}

void DivPlatformPOKEY::poke(std::vector<DivRegWrite>& wlist) {
  for (DivRegWrite& i: wlist) rWrite(i.addr,i.val);
}

int DivPlatformPOKEY::init(DivEngine* p, int channels, int sugRate, const DivConfig& flags) {
  parent=p;
  dumpWrites=false;
  skipRegisterWrites=false;
  oscBufDelay=0;
  for (int i=0; i<4; i++) {
    isMuted[i]=false;
    oscBuf[i]=new DivDispatchOscBuffer;
  }

  if (!useAltASAP) {
    MZPOKEYSND_Init(&pokey);
  }

  setFlags(flags);
  reset();
  return 6;
}

void DivPlatformPOKEY::quit() {
  for (int i=0; i<4; i++) {
    delete oscBuf[i];
  }
}

void DivPlatformPOKEY::setAltASAP(bool value) {
  useAltASAP=value;
}

DivPlatformPOKEY::~DivPlatformPOKEY() {
}
