use num_traits::Float;

use num_derive::FromPrimitive;

// - midi messages ------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub enum Message {
    NoteOn { note: u8, velocity: u8 },
    NoteOff { note: u8, velocity: u8 },
    ControlChange { index: u8, value: u8 },
    ProgramChange { value: u8 },
}

// - midi specification -------------------------------------------------------

#[derive(PartialEq, FromPrimitive, Debug)]
#[repr(u8)]
pub enum Byte {
    Unknown = 0x00,

    // Channel Messages
    NoteOff = 0x80,
    NoteOn = 0x90,
    PolyAftertouch = 0xa0,
    ControlChange = 0xb0,
    ProgramChange = 0xc0,
    ChannelAftertouch = 0xd0,
    PitchWheel = 0xe0,

    // System Common Messages
    SysexStart = 0xf0,
    QuarterFrame = 0xf1,
    SongPointer = 0xf2,
    SongSelect = 0xf3,
    TuneRequest = 0xf6,
    SysexEnd = 0xf7,

    // System Realtime Messages
    TimingClock = 0xf8,
    Start = 0xfa,
    Continue = 0xfb,
    Stop = 0xfc,
    ActiveSensing = 0xfe,
    Reset = 0xff,
}

#[derive(PartialEq)]
enum State {
    None,     // ignore data
    OneOfOne, // get 1st byte of 1
    OneOfTwo, // get 1st byte of 2
    TwoOfTwo, // get 2nd byte of 2
    RxSysex,  // get system exclusive bytes
}

// - midi parser --------------------------------------------------------------

pub struct Parser {
    state: State,

    pub channel: u8,
    pub byte: Byte,
    pub data1: u8,
    pub data2: u8,
}

impl Parser {
    pub fn new() -> Parser {
        Parser {
            state: State::None,
            channel: 0,
            byte: Byte::Unknown,
            data1: 0,
            data2: 0,
        }
    }

    pub fn rx<F>(&mut self, byte: u8, f: F)
    where
        F: FnMut(u8, Message),
    {
        if (byte & 0x80) > 0 {
            // status byte
            if self.state == State::RxSysex && byte < 0xf8 {
                self.sysex_end();
                self.state = State::None;
            }
            if byte < 0xf0 {
                // channel message
                self.channel = byte & 0x0f;
                self.byte = num::FromPrimitive::from_u8(byte & 0xf0).unwrap_or(Byte::Unknown);
                if self.byte == Byte::ControlChange {
                    self.state = State::OneOfTwo;
                } else if self.byte == Byte::ProgramChange {
                    self.state = State::OneOfOne;
                } else {
                    // ???
                    self.state = State::OneOfTwo;
                }
            } else if byte < 0xf8 {
                // system common message
                self.byte = Byte::Unknown;
            } else { // system realtime message
            }
        } else {
            // data byte
            match self.state {
                State::None => (),
                State::OneOfOne => {
                    self.data1 = byte;
                    self.emit_message(f);
                    self.state = match self.byte {
                        Byte::Unknown => State::OneOfOne,
                        _ => State::None,
                    };
                }
                State::OneOfTwo => {
                    self.data1 = byte;
                    self.state = State::TwoOfTwo;
                }
                State::TwoOfTwo => {
                    self.data2 = byte;
                    self.emit_message(f);
                    self.state = match self.byte {
                        Byte::Unknown => State::OneOfTwo,
                        _ => State::None,
                    };
                }
                State::RxSysex => (),
            }
        }
    }

    fn sysex_end(&self) {}

    fn emit_message<F>(&self, mut f: F)
    where
        F: FnMut(u8, Message),
    {
        match self.byte {
            Byte::NoteOn => {
                f(
                    self.channel,
                    Message::NoteOn {
                        note: self.data1,
                        velocity: self.data2,
                    },
                );
            }
            Byte::NoteOff => {
                f(
                    self.channel,
                    Message::NoteOff {
                        note: self.data1,
                        velocity: self.data2,
                    },
                );
            }
            Byte::ControlChange => {
                f(
                    self.channel,
                    Message::ControlChange {
                        index: self.data1,
                        value: self.data2,
                    },
                );
            }
            Byte::ProgramChange => {
                f(self.channel, Message::ProgramChange { value: self.data1 });
            }
            _ => (),
        }
    }
}

// - conversions --------------------------------------------------------------

pub fn to_hz(note: u8) -> f32 {
    const SEMITONE: f32 = 1. / 12.;
    f32::powf(2., ((note as f32) - 69.) * SEMITONE) * 440.
}
