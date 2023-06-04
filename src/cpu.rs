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
const STACK_OFFSET: usize = 0x01FF;
const STACK_MIN: usize = 0x0100;

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
            stack_ptr: 0xFD,
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
                Instruction::AND => self.and(&instruction.mode),
                Instruction::ASL => self.asl(&instruction.mode),
                Instruction::BCC => {
                    self.bcc();
                    continue;
                }
                Instruction::BCS => {
                    self.bcs();
                    continue;
                }
                Instruction::BEQ => {
                    self.beq();
                    continue;
                }
                Instruction::BIT => todo!(),
                Instruction::BMI => {
                    self.bmi();
                    continue;
                }
                Instruction::BNE => {
                    self.bne();
                    continue;
                }
                Instruction::BPL => {
                    self.bpl();
                    continue;
                }
                Instruction::BRK => return,
                Instruction::BVC => {
                    self.bvc();
                    continue;
                },
                Instruction::BVS => {
                    self.bvs();
                    continue;
                }
                Instruction::CLC => {
                    self.flags.remove(State::CARRY);
                }
                Instruction::CLD => {
                    // this should be a NOP for actual NES games as they don't have decimal mode
                    self.flags.remove(State::RESERVED);
                }
                Instruction::CLI => {
                    self.flags.remove(State::INTERRUPT_DISABLE);
                }
                Instruction::CLV => {
                    self.flags.remove(State::OVERFLOW);
                }
                Instruction::CMP => self.cmp(&instruction.mode),
                Instruction::CPX => self.cpx(&instruction.mode),
                Instruction::CPY => self.cpy(&instruction.mode),
                Instruction::DEC => self.dec(&instruction.mode),
                Instruction::DEX => {
                    self.rxi = self.rxi.wrapping_sub(1);
                    self.set_zero(self.rxi);
                    self.set_negative(self.rxi);
                }
                Instruction::DEY => {
                    self.ryi = self.ryi.wrapping_sub(1);
                    self.set_zero(self.ryi);
                    self.set_negative(self.ryi);
                }
                Instruction::EOR => self.eor(&instruction.mode),
                Instruction::INC => self.inc(&instruction.mode),
                Instruction::INX => {
                    self.rxi = self.rxi.wrapping_add(1);
                    self.set_zero(self.rxi);
                    self.set_negative(self.rxi);
                }
                Instruction::INY => {
                    self.ryi = self.ryi.wrapping_add(1);
                    self.set_zero(self.ryi);
                    self.set_negative(self.ryi);
                },
                Instruction::JMP => {
                    self.jmp(&instruction.mode);
                    continue;
                },
                Instruction::JSR => todo!(),
                Instruction::LDA => self.lda(&instruction.mode),
                Instruction::LDX => self.ldx(&instruction.mode),
                Instruction::LDY => self.ldy(&instruction.mode),
                Instruction::LSR => self.lsr(&instruction.mode),
                Instruction::NOP => continue,
                Instruction::ORA => self.ora(&instruction.mode),
                Instruction::PHA => self.push_stack(self.acc),
                Instruction::PHP => self.push_stack(self.flags.bits() as u8),
                Instruction::PLA => self.acc = self.pop_stack(),
                Instruction::PLP => self.flags.from_bits(self.pop_stack()),
                Instruction::ROL => todo!(),
                Instruction::ROR => todo!(),
                Instruction::RTI => todo!(),
                Instruction::RTS => todo!(),
                Instruction::SBC => todo!(),
                Instruction::SEC => self.flags.insert(State::CARRY),
                Instruction::SED => self.flags.insert(State::DECIMAL),
                Instruction::SEI => self.flags.insert(State::INTERRUPT_DISABLE),
                Instruction::STA => todo!(),
                Instruction::STX => todo!(),
                Instruction::STY => todo!(),
                Instruction::TAX => self.tax(),
                Instruction::TAY => self.tay(),
                Instruction::TSX => todo!(),
                Instruction::TXA => self.txa(),
                Instruction::TXS => todo!(),
                Instruction::TYA => self.tya(),
            }
            self.instr_ptr += instruction.byte_length - 1; // HACK this might be broken? Dunno
        }
    }

    // General Helpers

    /// Sets ZERO bitflag to 1 if argument is 0, otherwise flag is set to 0
    fn set_zero(&mut self, val: u8) {
        if val == 0 {
            self.flags.insert(State::ZERO);
        } else {
            self.flags.remove(State::ZERO);
        }
    }

    /// Sets NEGATIVE bitflag to 1 if argument is 0, otherwise flag is set to 0
    fn set_negative(&mut self, val: u8) {
        if val > 127 {
            self.flags.insert(State::NEGATIVE);
        } else {
            self.flags.remove(State::NEGATIVE);
        }
    }

    /// Sets CARRY bitflag to 1 if argument is 0, otherwise flag is set to 0
    fn set_carry(&mut self, val: u8) {
        if val > 127 {
            self.flags.insert(State::CARRY);
        } else {
            self.flags.remove(State::CARRY);
        }
    }

    /// Puts a given byte onto the stack, decrements stack pointer by 1
    fn push_stack(&mut self, val: u8) {
        self.mem[STACK_OFFSET - (self.stack_ptr as usize)] = val;
        self.stack_ptr -= 1;
    }

    /// Reads and returns the byte at the top of the stack, increments stack pointer
    fn pop_stack(&mut self) -> u8 {
        self.stack_ptr += 1;
        self.mem[STACK_OFFSET - ((self.stack_ptr - 1) as usize)]
    }

    /// Used for branch instructions - reads value at instruction pointer as i8, then adds that value
    /// to the instruction pointer
    fn branch(&mut self) {
        let val = self.read_u8(self.instr_ptr) as i8;
        self.instr_ptr += 1;
        if val > 0 {
            self.instr_ptr += val as u16;
        } else {
            self.instr_ptr -= val as u16;
        }
    }

    // Core Instructions (alphabetical order)

    /// Logical AND
    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.acc = self.acc & self.read_u8(addr);

        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

    /// Shift Left with Carry
    fn asl(&mut self, mode: &AddressingMode) {
        if *mode == AddressingMode::Implied {
            self.set_carry(self.acc);
            self.acc = self.acc << 1;
        } else {
            let addr = self.get_operand_address(&mode) as usize;
            self.mem[addr] = self.mem[addr] << 1;
        }
    }

    /// Branch if Carry Clear
    fn bcc(&mut self) {
        if !self.flags.contains(State::CARRY) {
            self.branch();
        } else {
            self.instr_ptr += 1
        }
    }

    /// Branch if Carry Set
    fn bcs(&mut self) {
        if self.flags.contains(State::CARRY) {
            self.branch();
        } else {
            self.instr_ptr += 1
        }
    }

    /// Branch if Equal
    fn beq(&mut self) {
        if self.flags.contains(State::ZERO) {
            self.branch();
        } else {
            self.instr_ptr += 1
        }
    }

    //fn bit

    /// Branch if Less Than Zero
    fn bmi(&mut self) {
        if self.flags.contains(State::NEGATIVE) {
            self.branch();
        } else {
            self.instr_ptr += 1
        }
    }

    /// Branch if Not Equal
    fn bne(&mut self) {
        if !self.flags.contains(State::ZERO) {
            self.branch();
        } else {
            self.instr_ptr += 1
        }
    }

    /// Branch if Greater Than Zero
    fn bpl(&mut self) {
        if !self.flags.contains(State::NEGATIVE) {
            self.branch();
        } else {
            self.instr_ptr += 1
        }
    }

    /// Branch if Overflow Clear
    fn bvc(&mut self) {
        if !self.flags.contains(State::OVERFLOW) {
            self.branch();
        } else {
            self.instr_ptr += 1
        }
    }

    /// Branch if Overflow Set
    fn bvs(&mut self) {
        if self.flags.contains(State::OVERFLOW) {
            self.branch();
        } else {
            self.instr_ptr += 1
        }
    }

    /// Compare Acc with a value in memory
    fn cmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let result = self.acc - self.read_u8(addr);
        if result >= 0 {
            self.flags.insert(State::CARRY);
        } else {
            self.flags.remove(State::CARRY);
        }
        self.set_zero(result);
        self.set_negative(result);
    }

    /// Compare X register with a value in memory
    fn cpx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let result = self.rxi - self.read_u8(addr);
        if result >= 0 {
            self.flags.insert(State::CARRY);
        } else {
            self.flags.remove(State::CARRY);
        }
        self.set_zero(result);
        self.set_negative(result);
    }

    /// Compare Y register with a value in memory
    fn cpy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let result = self.ryi - self.read_u8(addr);
        if result >= 0 {
            self.flags.insert(State::CARRY);
        } else {
            self.flags.remove(State::CARRY);
        }
        self.set_zero(result);
        self.set_negative(result);
    }

    /// Decrement value (wrapping) at memory location - sets flags as necessary
    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode) as usize;
        self.mem[addr] = self.mem[addr] - 1;
        self.set_zero(self.mem[addr]);
        self.set_negative(self.mem[addr]);
    }

    /// Bitwise exclusive OR between Acc and a value in memory, stored in Acc - sets zero and negative as necessary
    fn eor(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.acc ^= self.read_u8(addr);
        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

    /// Increment value (wrapping) at memory location - sets flags as necessary
    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode) as usize;
        self.mem[addr] = self.mem[addr] + 1;
        self.set_zero(self.mem[addr]);
        self.set_negative(self.mem[addr]);
    }

    /// Unconditional Jump, sets instruction pointer to the value specified by the operand address
    fn jmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.instr_ptr = self.read_u16(addr);
    }

    //jsr

    /// Load Data to Acc
    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.acc = self.read_u8(addr);

        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

    /// Load Data to X Register
    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.rxi = self.read_u8(addr);

        self.set_zero(self.rxi);
        self.set_negative(self.rxi);
    }

    /// Load Data to Y Register
    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.ryi = self.read_u8(addr);

        self.set_zero(self.ryi);
        self.set_negative(self.ryi);
    }

    /// Bitwise Shift Right - sets carry, zero, and negative flag as necessary
    fn lsr(&mut self, mode: &AddressingMode) {
        let result: u8;
        if *mode == AddressingMode::Implied {
            if self.acc & 0b0000_0001 > 0 {
                self.flags.insert(State::CARRY)
            } else {
                self.flags.remove(State::CARRY)
            }
            self.acc = self.acc >> 1;
            let result = self.acc;
        } else {
            let addr = self.get_operand_address(mode) as usize;
            if self.acc & 0b0000_0001 > 0 {
                self.flags.insert(State::CARRY)
            } else {
                self.flags.remove(State::CARRY)
            }
            self.mem[addr] = self.mem[addr] >> 1;
            let result = self.mem[addr];
        }
        self.set_zero(result);
        self.set_negative(result);
    }

    /// Bitwise Inclusive OR between Acc and Memory - sets zero and negative flag as necessary
    fn ora(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.acc |= self.read_u8(addr);
        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

    fn tax(&mut self) {
        self.rxi = self.acc;
        self.set_zero(self.rxi);
        self.set_negative(self.rxi);
    }

    fn tay(&mut self) {
        self.ryi = self.acc;
        self.set_zero(self.ryi);
        self.set_negative(self.ryi);
    }

    fn txa(&mut self) {
        self.acc = self.rxi;
        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

    //txs

    fn tya(&mut self) {
        self.acc = self.ryi;
        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }
}
