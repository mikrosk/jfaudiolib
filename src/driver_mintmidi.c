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
 * Atari MIDI port output
 *
 * Two interrupt sources share one FIFO:
 *
 * Timer B runs at 960 Hz. Every period it adds the number of MIDI ticks
 * per period (16.16 fixed point, from the tempo and the division) to an
 * accumulator and calls the sequencer service routine once per whole
 * tick. The service routine reports the events through the midifuncs
 * callbacks below, which append complete MIDI messages to the FIFO.
 *
 * The MIDI ACIA transmit interrupt drains the FIFO at wire speed, one
 * byte per interrupt. Both ACIAs share MFP channel 6; the TOS handler
 * calls kb_midisys and kb_ikbdsys in a loop until the line is released,
 * so kb_midisys is hooked through Kbdvbase() with an XBRA header and the
 * original is chained for the receive side. The transmit interrupt is
 * enabled only while the FIFO holds data: an empty transmit register
 * keeps the line asserted, and with nothing to clear it the TOS loop
 * would never end.
 *
 * Messages sent while no song plays (resets, volume changes) go out
 * polled in supervisor mode, as the transmit interrupt is not armed
 * then.
 */

#ifdef __MINT__

#include <string.h>
#include <mint/osbind.h>
#include <mint/ostruct.h>

#include "midifuncs.h"
#include "driver_mintmidi.h"
#include "asssys.h"

enum {
   MintMIDIErr_Warning = -2,
   MintMIDIErr_Error   = -1,
   MintMIDIErr_Ok      = 0,
   MintMIDIErr_Uninitialised,
   MintMIDIErr_NoService
};

static int ErrorCode = MintMIDIErr_Ok;

#define MIDI_ACIA_CTRL  (*(volatile unsigned char *)0xFFFFFC04L)
#define MIDI_ACIA_DATA  (*(volatile unsigned char *)0xFFFFFC06L)
#define ACIA_TDRE       (1 << 1)

/* /16 clock, 8N1, RTS low, TX interrupt off, RX interrupt on */
#define ACIA_CTRL_TIE_OFF   0x95
/* /16 clock, 8N1, RTS low, TX interrupt on, RX interrupt on */
#define ACIA_CTRL_TIE_ON    0xB5

#define MFP_IERA        (*(volatile unsigned char *)0xFFFFFA07L)
#define MFP_IPRA        (*(volatile unsigned char *)0xFFFFFA0BL)
#define MFP_IMRA        (*(volatile unsigned char *)0xFFFFFA13L)
#define MFP_TBCR        (*(volatile unsigned char *)0xFFFFFA1BL)

#define TIMER_B_CTRL    3       /* prescaler /16 */
#define TIMER_B_DATA    160     /* 2457600 / 16 / 160 = 960 Hz */
#define TIMER_B_HZ      960
#define TIMER_B_VECTOR  0x120

#define FIFO_SIZE       2048
#define FIFO_MASK       2047
/* whole ticks are only processed with this much room, so a tick worth
 * of messages never has to be dropped */
#define FIFO_RESERVE    128

#define MM_STR_(x)      #x
#define MM_STR(x)       MM_STR_(x)
#define MM_SYM(x)       MM_STR(__USER_LABEL_PREFIX__) #x

/* shared with the interrupt handlers */
void (* volatile mintmidi_service)(void);
volatile int mintmidi_locked;
volatile unsigned int mintmidi_tickAdd;     /* MIDI ticks per timer period, 16.16 */
volatile unsigned int mintmidi_tickFrac;
unsigned char mintmidi_fifo[FIFO_SIZE];
volatile unsigned short mintmidi_head, mintmidi_tail;
volatile int mintmidi_tie;                  /* shadow of the transmit interrupt enable */

static volatile int s_playing;
static _KBDVECS *s_kbdvecs;
static long s_oldTimerbVec;
static unsigned char s_oldTbcr;
static int s_oldTimerbEnabled, s_oldTimerbMasked;

void mintmidi_ticks(void);
void mintmidi_acia_hook(void);
extern void mintmidi_timer(void);
extern void mintmidi_handler(void);
extern long mintmidi_oldvec;

/* Timer B handler: the clock, then arm the transmit interrupt when the
 * FIFO holds data. The sequencer only runs when a whole tick is due, so
 * the registers it may clobber are saved only on that path. */
__asm__(
   "\t.text\n"
   "\t.even\n"
   "\t.globl\t" MM_SYM(mintmidi_timer) "\n"
   MM_SYM(mintmidi_timer) ":\n"
   "\tmove.l\t%d0,-(%sp)\n"
   "\ttst.l\t" MM_SYM(mintmidi_locked) "\n"
   "\tbne.s\t1f\n"
   "\tmove.l\t" MM_SYM(mintmidi_tickAdd) ",%d0\n"
   "\tadd.l\t%d0," MM_SYM(mintmidi_tickFrac) "\n"
   "\tcmpi.l\t#0x10000," MM_SYM(mintmidi_tickFrac) "\n"
   "\tbcs.s\t1f\n"
   "\tmovem.l\t%d1/%a0-%a1,-(%sp)\n"
   "\tfmovem.x\t%fp0-%fp1,-(%sp)\n"
   "\tjsr\t" MM_SYM(mintmidi_ticks) "\n"
   "\tfmovem.x\t(%sp)+,%fp0-%fp1\n"
   "\tmovem.l\t(%sp)+,%d1/%a0-%a1\n"
   "1:\n"
   "\tmove.w\t" MM_SYM(mintmidi_tail) ",%d0\n"
   "\tcmp.w\t" MM_SYM(mintmidi_head) ",%d0\n"
   "\tbeq.s\t2f\n"
   "\ttst.l\t" MM_SYM(mintmidi_tie) "\n"
   "\tbne.s\t2f\n"
   "\tmove.l\t#1," MM_SYM(mintmidi_tie) "\n"
   "\tmove.b\t#" MM_STR(ACIA_CTRL_TIE_ON) ",0xfffffc04.w\n"
   "2:\n"
   "\tmove.l\t(%sp)+,%d0\n"
   "\tmove.b\t#0xfe,0xfffffa0f.w\n"
   "\trte\n"
);

/* kb_midisys entry with an XBRA header: the C hook handles the transmit
 * side, then the previous vector gets the receive side. TOS and EmuTOS
 * save d0-d3/a0-a3 around the kbdvecs calls, and the C call clobbers no
 * more than d0-d1/a0-a1. */
__asm__(
   "\t.text\n"
   "\t.even\n"
   "\t.ascii\t\"XBRA\"\n"
   "\t.ascii\t\"JFAL\"\n"
   "\t.globl\t" MM_SYM(mintmidi_oldvec) "\n"
   MM_SYM(mintmidi_oldvec) ":\n"
   "\t.long\t0\n"
   "\t.globl\t" MM_SYM(mintmidi_handler) "\n"
   MM_SYM(mintmidi_handler) ":\n"
   "\tjsr\t" MM_SYM(mintmidi_acia_hook) "\n"
   "\tmove.l\t" MM_SYM(mintmidi_oldvec) ",%a0\n"
   "\tjmp\t(%a0)\n"
);

static inline unsigned short set_ipl7(void)
{
   unsigned short sr;
   __asm__ __volatile__ ("move.w %%sr,%0\n\tori.w #0x0700,%%sr" : "=d"(sr) : : "memory");
   return sr;
}

static inline void restore_ipl(unsigned short sr)
{
   __asm__ __volatile__ ("move.w %0,%%sr" : : "d"(sr) : "memory");
}

static inline unsigned short fifo_space(void)
{
   return FIFO_SIZE - 1 - ((mintmidi_head - mintmidi_tail) & FIFO_MASK);
}

static inline void fifo_push(unsigned char b)
{
   mintmidi_fifo[mintmidi_head] = b;
   mintmidi_head = (mintmidi_head + 1) & FIFO_MASK;
}

void mintmidi_ticks(void)
{
   while (mintmidi_tickFrac >= 65536) {
      if (fifo_space() < FIFO_RESERVE) {
         break;
      }
      mintmidi_tickFrac -= 65536;
      mintmidi_service();
   }
}

/* Called from the TOS ACIA handler on every ACIA interrupt until the
 * line is released. Sends one byte per empty transmit register and
 * disarms the interrupt as soon as the FIFO is empty. */
void mintmidi_acia_hook(void)
{
   if (mintmidi_tie && (MIDI_ACIA_CTRL & ACIA_TDRE)) {
      if (mintmidi_head != mintmidi_tail) {
         MIDI_ACIA_DATA = mintmidi_fifo[mintmidi_tail];
         mintmidi_tail = (mintmidi_tail + 1) & FIFO_MASK;
      } else {
         MIDI_ACIA_CTRL = ACIA_CTRL_TIE_OFF;
         mintmidi_tie = 0;
      }
   }
}

static long sup_send_polled(void)
{
   while (mintmidi_head != mintmidi_tail) {
      while ((MIDI_ACIA_CTRL & ACIA_TDRE) == 0)
         ;
      MIDI_ACIA_DATA = mintmidi_fifo[mintmidi_tail];
      mintmidi_tail = (mintmidi_tail + 1) & FIFO_MASK;
   }
   return 0;
}

/* Appends one complete message. While a song plays the timer interrupt
 * arms the transmit interrupt; otherwise the bytes are sent at once. */
static void send_message(const unsigned char *msg, int length)
{
   int i;

   if (fifo_space() < length) {
      return;
   }
   for (i = 0; i < length; i++) {
      fifo_push(msg[i]);
   }
   if (!s_playing) {
      Supexec(sup_send_polled);
   }
}

static void Func_NoteOff( int channel, int key, int velocity )
{
   unsigned char msg[3] = { 0x80 | channel, key, velocity };
   send_message(msg, 3);
}

static void Func_NoteOn( int channel, int key, int velocity )
{
   unsigned char msg[3] = { 0x90 | channel, key, velocity };
   send_message(msg, 3);
}

static void Func_PolyAftertouch( int channel, int key, int pressure )
{
   unsigned char msg[3] = { 0xa0 | channel, key, pressure };
   send_message(msg, 3);
}

static void Func_ControlChange( int channel, int number, int value )
{
   unsigned char msg[3] = { 0xb0 | channel, number, value };
   send_message(msg, 3);
}

static void Func_ProgramChange( int channel, int program )
{
   unsigned char msg[2] = { 0xc0 | channel, program };
   send_message(msg, 2);
}

static void Func_ChannelAftertouch( int channel, int pressure )
{
   unsigned char msg[2] = { 0xd0 | channel, pressure };
   send_message(msg, 2);
}

static void Func_PitchBend( int channel, int lsb, int msb )
{
   unsigned char msg[3] = { 0xe0 | channel, lsb, msb };
   send_message(msg, 3);
}

static void Func_SysEx( const unsigned char * data, int length )
{
   send_message(data, length);
}

static long sup_hook_install(void)
{
   unsigned short sr = set_ipl7();

   s_oldTbcr = MFP_TBCR;
   s_oldTimerbEnabled = (MFP_IERA & (1 << 0)) != 0;
   s_oldTimerbMasked  = (MFP_IMRA & (1 << 0)) != 0;

   mintmidi_tie = 0;
   MIDI_ACIA_CTRL = ACIA_CTRL_TIE_OFF;

   if (s_kbdvecs->midisys != (long (*)(void))mintmidi_handler) {
      mintmidi_oldvec = (long)s_kbdvecs->midisys;
      s_kbdvecs->midisys = (long (*)(void))mintmidi_handler;
   }

   restore_ipl(sr);
   return 0;
}

/* Unlinks the handler from the kb_midisys XBRA chain */
static long sup_hook_remove(void)
{
   unsigned short sr = set_ipl7();
   long *link = (long *)&s_kbdvecs->midisys;

   MIDI_ACIA_CTRL = ACIA_CTRL_TIE_OFF;
   mintmidi_tie = 0;

   while (*link) {
      long *xbra = (long *)(*link - 12);
      if (*link == (long)mintmidi_handler) {
         *link = mintmidi_oldvec;
         break;
      }
      if (xbra[0] != 0x58425241L) {  /* "XBRA": not a chained handler */
         break;
      }
      link = &xbra[2];
   }

   MFP_TBCR = s_oldTbcr;
   MFP_IPRA = (unsigned char)~(1 << 0);
   if (s_oldTimerbEnabled) {
      MFP_IERA |= 1 << 0;
   }
   if (s_oldTimerbMasked) {
      MFP_IMRA |= 1 << 0;
   }

   restore_ipl(sr);
   return 0;
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

   s_kbdvecs = Kbdvbase();
   mintmidi_head = mintmidi_tail = 0;
   mintmidi_locked = 0;

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
   MintMIDIDrv_MIDI_HaltPlayback();
}

int MintMIDIDrv_MIDI_StartPlayback(void (*service)(void))
{
   MintMIDIDrv_MIDI_HaltPlayback();

   if (!service) {
      ErrorCode = MintMIDIErr_NoService;
      return MintMIDIErr_Error;
   }

   mintmidi_service = service;
   mintmidi_tickFrac = 0;

   Supexec(sup_hook_install);

   Jdisint(MFP_TIMERB);
   s_oldTimerbVec = (long)Setexc(TIMER_B_VECTOR >> 2, (void (*)(void))-1L);
   s_playing = 1;
   Xbtimer(XB_TIMERB, TIMER_B_CTRL, TIMER_B_DATA, mintmidi_timer);

   return MintMIDIErr_Ok;
}

void MintMIDIDrv_MIDI_HaltPlayback(void)
{
   int i;

   if (!s_playing) {
      return;
   }

   Jdisint(MFP_TIMERB);
   Setexc(TIMER_B_VECTOR >> 2, (void (*)(void))s_oldTimerbVec);
   s_playing = 0;

   Supexec(sup_hook_remove);

   /* whatever is left in the FIFO, then silence every channel */
   for (i = 0; i < 16; i++) {
      unsigned char msg[6] = { 0xb0 | i, 0x78, 0, 0xb0 | i, 0x7b, 0 };
      if (fifo_space() >= 6) {
         fifo_push(msg[0]); fifo_push(msg[1]); fifo_push(msg[2]);
         fifo_push(msg[3]); fifo_push(msg[4]); fifo_push(msg[5]);
      }
   }
   Supexec(sup_send_polled);
}

void MintMIDIDrv_MIDI_SetTempo(int tempo, int division)
{
   unsigned long long ticks = (unsigned long long)(unsigned int)tempo * (unsigned int)division;

   mintmidi_tickAdd = (unsigned int)((ticks << 16) / (60 * TIMER_B_HZ));
}

void MintMIDIDrv_MIDI_Lock(void)
{
   mintmidi_locked++;
}

void MintMIDIDrv_MIDI_Unlock(void)
{
   mintmidi_locked--;
}

#endif
