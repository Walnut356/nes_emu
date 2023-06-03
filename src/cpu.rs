use crate::instructions::{self, Instruction};
use enumflags2::{bitflags, BitFlags};

#[bitflags]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum State {
    NEGATIVE = 0b1000_0000,
    OVERFLOW = 0b0100_0000,
    RESERVED = 0b0010_0000,
    BREAK = 0b0001_0000,
    DECIMAL = 0b0000_1000,
    INTERRUPT_DISABLE = 0b0000_0100,
    ZERO = 0b0000_0010,
    CARRY = 0b0000_0001,
}

pub struct CPU {
    pub acc: u8,
    pub rxi: u8,
    pub ryi: u8,
    pub flags: BitFlags<State>,
    //from bit 7 to bit 0 these are the negative (N), overflow (V), reserved, break (B), decimal (D), interrupt disable (I), zero (Z) and carry (C) flag
    pub stack_p: u8,
    pub inst_p: u16,
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            acc: 0,
            rxi: 0,
            ryi: 0,
            flags: BitFlags::default(),
            stack_p: 0,
            inst_p: 0,
        }
    }

    pub fn interpret(&mut self, program: Vec<u8>) {
        self.inst_p = 0;

        loop {
            let opcode = program[self.inst_p as usize];
            self.inst_p += 1;

            match Instruction::try_from(opcode).unwrap() {
                Instruction::BRK => return,
                Instruction::LDA => {
                    self.acc = program[self.inst_p as usize];
                    self.inst_p += 1;

                    self.set_zero(self.acc);
                    self.set_negative(self.acc);
                }
                Instruction::TAX => {
                    self.rxi = self.acc;

                    self.set_zero(self.rxi);
                    self.set_negative(self.rxi);
                }
            }
        }
    }

    fn set_zero(&mut self, val: u8) {
        if val == 0 {
            self.flags.insert(State::ZERO);
        } else {
            self.flags.remove(State::ZERO);
        }
    }

    fn set_negative(&mut self, val: u8) {
        if val > 127 {
            self.flags.insert(State::NEGATIVE);
        } else {
            self.flags.remove(State::NEGATIVE);
        }
    }
}
