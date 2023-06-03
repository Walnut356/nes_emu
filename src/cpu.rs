use std::collections::HashMap;

use crate::instructions::*;
use enumflags2::{bitflags, BitFlags};

#[bitflags]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
#[allow(non_camel_case_types)]
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

const INIT_PTR_LOC: usize = 0x8000;

type MemAddr = u16;

pub struct CPU {
    pub acc: u8,
    pub rxi: u8,
    pub ryi: u8,
    pub flags: BitFlags<State>,
    //from bit 7 to bit 0 these are the negative (N), overflow (V), reserved, break (B), decimal (D), interrupt disable (I), zero (Z) and carry (C) flag
    pub stack_ptr: u8,
    pub instr_ptr: MemAddr,
    pub mem: [u8; 0xFFFF],
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            acc: 0,
            rxi: 0,
            ryi: 0,
            flags: BitFlags::default(),
            stack_ptr: 0,
            instr_ptr: 0, // maybe replace with std::io::Cursor?
            mem: [0; 0xFFFF],
        }
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.mem[INIT_PTR_LOC..INIT_PTR_LOC + program.len()].copy_from_slice(&program[..]);
        self.instr_ptr = INIT_PTR_LOC as MemAddr;
    }

    pub fn read_u8(&self, addr: MemAddr) -> u8 {
        self.mem[addr as usize]
    }

    pub fn read_u16(&self, addr: MemAddr) -> u16 {
        let addr = addr as usize;
        let addr_plus_1: usize;

        if addr < u16::MAX as usize {
            addr_plus_1 = 0;
        } else {
            addr_plus_1 = addr + 1;
        }

        u16::from_le_bytes([self.mem[addr], self.mem[addr_plus_1]])
    }

    pub fn write_u8(&mut self, addr: MemAddr, data: u8) {
        self.mem[addr as usize] = data;
    }

    pub fn write_u16(&mut self, addr: MemAddr, data: u16) {
        let addr = addr as usize;
        self.mem[addr] = (data >> 8) as u8;
        self.mem[addr] = (data & 0x00FF) as u8;
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> MemAddr {
        use AddressingMode::*;
        match mode {
            Immediate => self.instr_ptr,
            ZeroPage => self.read_u8(self.instr_ptr) as u16,
            ZeroPage_X => (self.read_u8(self.instr_ptr).wrapping_add(self.rxi)) as u16,
            ZeroPage_Y => (self.read_u8(self.instr_ptr).wrapping_add(self.ryi)) as u16,
            Absolute => self.read_u16(self.instr_ptr),
            Absolute_X => self.read_u16(self.instr_ptr).wrapping_add(self.rxi as u16),
            Absolute_Y => self.read_u16(self.instr_ptr).wrapping_add(self.ryi as u16),
            Indirect => todo!(),
            Indirect_X => {
                let ptr = self.read_u16(self.instr_ptr).wrapping_add(self.rxi as u16);
                self.read_u16(ptr)
            }
            Indirect_Y => {
                let ptr = self.read_u16(self.instr_ptr).wrapping_add(self.ryi as u16);
                self.read_u16(ptr)
            }
            Relative => self.instr_ptr,
            Implied => self.instr_ptr,
            None => panic!("Addressing Mode {:?} is not valid", mode),
        }
    }

    pub fn run(&mut self) {
        let ref opcodes: HashMap<u8, OpCode> = *OPCODES_MAP;
        loop {
            let curr_op = self.read_u8(self.instr_ptr);
            self.instr_ptr += 1;

            let instruction = opcodes
                .get(&curr_op)
                .expect(&format!("OpCode {:x} is not recognized", curr_op));

            match instruction.instr {
                Instruction::ADC => todo!(),
                Instruction::AND => todo!(),
                Instruction::ASL => todo!(),
                Instruction::BCC => todo!(),
                Instruction::BCS => todo!(),
                Instruction::BEQ => todo!(),
                Instruction::BIT => todo!(),
                Instruction::BMI => todo!(),
                Instruction::BNE => todo!(),
                Instruction::BPL => todo!(),
                Instruction::BRK => return,
                Instruction::BVC => todo!(),
                Instruction::BVS => todo!(),
                Instruction::CLC => todo!(),
                Instruction::CLD => todo!(),
                Instruction::CLI => todo!(),
                Instruction::CLV => todo!(),
                Instruction::CMP => todo!(),
                Instruction::CPX => todo!(),
                Instruction::CPY => todo!(),
                Instruction::DEC => todo!(),
                Instruction::DEX => todo!(),
                Instruction::DEY => todo!(),
                Instruction::EOR => todo!(),
                Instruction::INC => todo!(),
                Instruction::INX => {
                    self.inx();
                }
                Instruction::INY => todo!(),
                Instruction::JMP => todo!(),
                Instruction::JSR => todo!(),
                Instruction::LDA => {
                    self.lda(&instruction.addr_mode);
                    self.instr_ptr += instruction.byte_length - 1;
                }
                Instruction::LDX => todo!(),
                Instruction::LDY => todo!(),
                Instruction::LSR => todo!(),
                Instruction::NOP => todo!(),
                Instruction::ORA => todo!(),
                Instruction::PHA => todo!(),
                Instruction::PHP => todo!(),
                Instruction::PLA => todo!(),
                Instruction::PLP => todo!(),
                Instruction::ROL => todo!(),
                Instruction::ROR => todo!(),
                Instruction::RTI => todo!(),
                Instruction::RTS => todo!(),
                Instruction::SBC => todo!(),
                Instruction::SEC => todo!(),
                Instruction::SED => todo!(),
                Instruction::SEI => todo!(),
                Instruction::STA => todo!(),
                Instruction::STX => todo!(),
                Instruction::STY => todo!(),
                Instruction::TAX => {
                    self.tax();
                }
                Instruction::TAY => todo!(),
                Instruction::TSX => todo!(),
                Instruction::TXA => todo!(),
                Instruction::TXS => todo!(),
                Instruction::TYA => todo!(),
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

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.acc = self.read_u8(addr);

        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

    fn tax(&mut self) {
        self.rxi = self.acc;
        self.set_zero(self.rxi);
        self.set_negative(self.rxi);
    }

    fn inx(&mut self) {
        self.rxi = self.rxi.wrapping_add(1);
        self.set_zero(self.rxi);
        self.set_negative(self.rxi);
    }
}
