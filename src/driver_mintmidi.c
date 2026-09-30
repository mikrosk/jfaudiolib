/*
 Copyright (C) 2026 Miro Kropacek <miro.kropacek@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 See the GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

 */

/**
 * Atari MIDI port output through uMIDI
 *
 * uMIDI's Timer B interrupt calls the sequencer service routine once per
 * MIDI tick. The service routine reports the events through the midifuncs
 * callbacks below, which queue complete MIDI messages for the MIDI ACIA
 * transmit interrupt. Ticks without any event due, as reported by the
 * sequencer, are counted off without calling it.
 *
 * Messages sent while no song plays (resets, volume changes) go out
 * through the XBIOS.
 */

#ifdef __MINT__

#include <stdlib.h>
#include <string.h>
#include <umidi.h>

#include "midifuncs.h"
#include "driver_mintmidi.h"

enum {
   MintMIDIErr_Warning = -2,
   MintMIDIErr_Error   = -1,
   MintMIDIErr_Ok      = 0,
   MintMIDIErr_Uninitialised,
   MintMIDIErr_NoService,
   MintMIDIErr_NoPort
};

static int ErrorCode = MintMIDIErr_Ok;

static UMidiContext s_ctx;
static int s_closeAtExit;

static void Func_NoteOff( int channel, int key, int velocity )
{
   unsigned char msg[3] = { 0x80 | channel, key, velocity };
   UMidiSend(&s_ctx, msg, 3);
}

static void Func_NoteOn( int channel, int key, int velocity )
{
   unsigned char msg[3] = { 0x90 | channel, key, velocity };
   UMidiSend(&s_ctx, msg, 3);
}

static void Func_PolyAftertouch( int channel, int key, int pressure )
{
   unsigned char msg[3] = { 0xa0 | channel, key, pressure };
   UMidiSend(&s_ctx, msg, 3);
}

static void Func_ControlChange( int channel, int number, int value )
{
   unsigned char msg[3] = { 0xb0 | channel, number, value };
   UMidiSend(&s_ctx, msg, 3);
}

static void Func_ProgramChange( int channel, int program )
{
   unsigned char msg[2] = { 0xc0 | channel, program };
   UMidiSend(&s_ctx, msg, 2);
}

static void Func_ChannelAftertouch( int channel, int pressure )
{
   unsigned char msg[2] = { 0xd0 | channel, pressure };
   UMidiSend(&s_ctx, msg, 2);
}

static void Func_PitchBend( int channel, int lsb, int msb )
{
   unsigned char msg[3] = { 0xe0 | channel, lsb, msb };
   UMidiSend(&s_ctx, msg, 3);
}

static void Func_SysEx( const unsigned char * data, int length )
{
   UMidiSend(&s_ctx, data, length);
}

/* the program may exit without shutting the driver down; the kb_midisys
 * hook and the Timer B vector must not outlive it */
static void close_at_exit(void)
{
   UMidiClose(&s_ctx);
}

int MintMIDIDrv_GetError(void)
{
   return ErrorCode;
}

const char *MintMIDIDrv_ErrorString( int ErrorNumber )
{
   const char *ErrorString;

   switch( ErrorNumber )
   {
      case MintMIDIErr_Warning :
      case MintMIDIErr_Error :
         ErrorString = MintMIDIDrv_ErrorString( ErrorCode );
         break;

      case MintMIDIErr_Ok :
         ErrorString = "Atari MIDI ok.";
         break;

      case MintMIDIErr_Uninitialised :
         ErrorString = "Atari MIDI uninitialised.";
         break;

      case MintMIDIErr_NoService :
         ErrorString = "Atari MIDI error: no service routine.";
         break;

      case MintMIDIErr_NoPort :
         ErrorString = "Atari MIDI error: no MIDI port, or another program uses it.";
         break;

      default:
         ErrorString = "Unknown Atari MIDI error.";
         break;
   }

   return ErrorString;
}

int MintMIDIDrv_MIDI_Init(midifuncs *funcs, const char *params)
{
   (void)params;

   MintMIDIDrv_MIDI_Shutdown();
   memset(funcs, 0, sizeof(midifuncs));

   if (!UMidiOpen(&s_ctx)) {
      ErrorCode = MintMIDIErr_NoPort;
      return MintMIDIErr_Error;
   }

   if (!s_closeAtExit) {
      atexit(close_at_exit);
      s_closeAtExit = 1;
   }

   funcs->NoteOff = Func_NoteOff;
   funcs->NoteOn  = Func_NoteOn;
   funcs->PolyAftertouch = Func_PolyAftertouch;
   funcs->ControlChange = Func_ControlChange;
   funcs->ProgramChange = Func_ProgramChange;
   funcs->ChannelAftertouch = Func_ChannelAftertouch;
   funcs->PitchBend = Func_PitchBend;
   funcs->SysEx = Func_SysEx;

   return MintMIDIErr_Ok;
}

void MintMIDIDrv_MIDI_Shutdown(void)
{
   UMidiClose(&s_ctx);
}

int MintMIDIDrv_MIDI_StartPlayback(void (*service)(void))
{
   MintMIDIDrv_MIDI_HaltPlayback();

   if (!service) {
      ErrorCode = MintMIDIErr_NoService;
      return MintMIDIErr_Error;
   }

   UMidiStartTicks(&s_ctx, service);

   return MintMIDIErr_Ok;
}

void MintMIDIDrv_MIDI_HaltPlayback(void)
{
   UMidiStopTicks(&s_ctx);
}

void MintMIDIDrv_MIDI_SetTempo(int tempo, int division)
{
   UMidiSetTickRate(&s_ctx, (unsigned int)tempo * (unsigned int)division);
}

void MintMIDIDrv_MIDI_Lock(void)
{
   UMidiLock(&s_ctx);
}

void MintMIDIDrv_MIDI_Unlock(void)
{
   UMidiUnlock(&s_ctx);
}

#endif
