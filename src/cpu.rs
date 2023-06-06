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
const STACK_MAX: usize = 0x01FF;
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
        self.mem[0x0600..0x0600 + program.len()].copy_from_slice(&program[..]);
        self.write_u16(0xFFFC, 0x0600);
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
        self.run_with_callback(|_| {});
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut CPU),
    {
        let ref opcodes: HashMap<u8, OpCode> = *OPCODES_MAP;
        loop {
            callback(self);

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
                }
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
                }
                Instruction::JMP => {
                    self.jmp(&instruction.mode);
                    continue;
                }
                Instruction::JSR => {
                    self.jsr(&instruction.mode);
                    continue;
                }
                Instruction::LDA => self.lda(&instruction.mode),
                Instruction::LDX => self.ldx(&instruction.mode),
                Instruction::LDY => self.ldy(&instruction.mode),
                Instruction::LSR => self.lsr(&instruction.mode),
                Instruction::NOP => continue,
                Instruction::ORA => self.ora(&instruction.mode),
                Instruction::PHA => self.push_stack(self.acc),
                Instruction::PHP => self.push_stack(self.flags.bits() as u8),
                Instruction::PLA => self.acc = self.pop_stack(),
                Instruction::PLP => {
                    self.flags = BitFlags::<State, u8>::from_bits(self.pop_stack()).unwrap()
                }
                Instruction::ROL => self.rol(&instruction.mode),
                Instruction::ROR => self.ror(&instruction.mode),
                Instruction::RTI => self.rti(),
                Instruction::RTS => self.rts(),
                Instruction::SBC => self.sbc(&instruction.mode),
                Instruction::SEC => self.flags.insert(State::CARRY),
                Instruction::SED => self.flags.insert(State::DECIMAL),
                Instruction::SEI => self.flags.insert(State::INTERRUPT_DISABLE),
                Instruction::STA => self.sta(&instruction.mode),
                Instruction::STX => self.stx(&instruction.mode),
                Instruction::STY => self.sty(&instruction.mode),
                Instruction::TAX => self.tax(),
                Instruction::TAY => self.tay(),
                Instruction::TSX => self.rxi = self.pop_stack(),
                Instruction::TXA => self.txa(),
                Instruction::TXS => self.push_stack(self.rxi),
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
        self.write_u8((STACK_MIN as MemAddr) + self.stack_ptr as u16, val);
        self.stack_ptr = self.stack_ptr.wrapping_sub(1);
    }

    /// Reads and returns the byte at the top of the stack, increments stack pointer
    fn pop_stack(&mut self) -> u8 {
        self.stack_ptr = self.stack_ptr.wrapping_add(1);
        self.read_u8((STACK_MIN as MemAddr) + self.stack_ptr as u16)
    }

    fn push_pointer(&mut self, val: MemAddr) {
        self.push_stack((val >> 8) as u8);
        self.push_stack((val & 0x00FF) as u8);
    }

    fn pop_pointer(&mut self) -> MemAddr {
        let lo = self.pop_stack() as u16;
        let ho = self.pop_stack() as u16;

        (ho << 8) | lo
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

    /// Add with Carry
    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let val = self.read_u8(addr) as u16;
        let mut carry: u16 = 0;
        if self.flags.contains(State::CARRY) {
            carry = 1
        };

        let sum: u16 = self.acc as u16 + val + carry;

        if sum > 255 {
            self.flags.insert(State::CARRY);
        } else {
            self.flags.remove(State::CARRY);
        }

        let result = sum as u8;

        let val = val as u8;

        if (val ^ result) & (result ^ self.acc) & 0x80 != 0 {
            self.flags.insert(State::OVERFLOW);
        } else {
            self.flags.remove(State::OVERFLOW);
        }

        self.acc = result;
        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

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
    fn inc(&mut self, mode: &AddressingMode) {
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

    fn jsr(&mut self, mode: &AddressingMode) {
        self.push_pointer(self.instr_ptr + 1);
        self.instr_ptr = self.read_u16(self.instr_ptr);
    }

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
        let mut result: u8;
        if *mode == AddressingMode::Implied {
            if self.acc & 0b0000_0001 > 0 {
                self.flags.insert(State::CARRY)
            } else {
                self.flags.remove(State::CARRY)
            }
            self.acc = self.acc >> 1;
            result = self.acc;
        } else {
            let addr = self.get_operand_address(mode) as usize;
            if self.acc & 0b0000_0001 > 0 {
                self.flags.insert(State::CARRY)
            } else {
                self.flags.remove(State::CARRY)
            }
            self.mem[addr] = self.mem[addr] >> 1;
            result = self.mem[addr];
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

    /// Rotate left for Acc or Memory Address, using existing carry - sets zero, negative, and carry flag as necessary
    fn rol(&mut self, mode: &AddressingMode) {
        let mut carry = 0;
        if self.flags.contains(State::CARRY) {
            carry = 1
        }

        if *mode == AddressingMode::Implied {
            if self.acc > 127 {
                self.flags.insert(State::CARRY)
            } else {
                self.flags.remove(State::CARRY)
            }
            self.acc = self.acc << 1;
            self.acc += carry;
            self.set_negative(self.acc);
            self.set_zero(self.acc);
        } else {
            let addr = self.get_operand_address(mode);
            let mut val = self.read_u8(addr);
            if val > 127 {
                self.flags.insert(State::CARRY)
            } else {
                self.flags.remove(State::CARRY)
            }
            val = val << 1;
            val += carry;
            self.write_u8(addr, val);
            self.set_zero(val);
            self.set_negative(val);
        }
    }

    /// Rotate right for Acc or Memory Address, using existing carry - sets zero, negative, and carry flag as necessary
    fn ror(&mut self, mode: &AddressingMode) {
        let mut carry = 0;
        if self.flags.contains(State::CARRY) {
            carry = 1
        }

        if *mode == AddressingMode::Implied {
            if self.acc >> 7 == 1 {
                self.flags.insert(State::CARRY)
            } else {
                self.flags.remove(State::CARRY)
            }
            self.acc = self.acc >> 1;
            self.acc += carry;
            self.set_negative(self.acc);
            self.set_zero(self.acc);
        } else {
            let addr = self.get_operand_address(mode);
            let mut val = self.read_u8(addr);
            if val >> 7 == 1 {
                self.flags.insert(State::CARRY)
            } else {
                self.flags.remove(State::CARRY)
            }
            val = val >> 1;
            val += carry;
            self.write_u8(addr, val);
            self.set_zero(val);
            self.set_negative(val);
        }
    }

    /// Return from interupt - pulls processer flags followed by instruction pointer from stack
    fn rti(&mut self) {
        self.flags = BitFlags::<State, u8>::from_bits(self.pop_stack()).unwrap();
        self.instr_ptr = self.pop_pointer();
    }

    /// Returns to the calling routine - pulls instruction pointer from stack
    fn rts(&mut self) {
        self.instr_ptr = self.pop_pointer();
    }

    /// Subtract with Carry
    fn sbc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut val = self.read_u8(addr);
        val = (val as i8).wrapping_neg().wrapping_sub(1) as u8;

        let mut carry: u16 = 0;
        if self.flags.contains(State::CARRY) {
            carry = 1
        };

        let sum: u16 = (self.acc as u16) + (val as u16) + carry;

        if sum > 255 {
            self.flags.insert(State::CARRY);
        } else {
            self.flags.remove(State::CARRY);
        }

        let result = sum as u8;

        if (val ^ result) & (result ^ self.acc) & 0x80 != 0 {
            self.flags.insert(State::OVERFLOW);
        } else {
            self.flags.remove(State::OVERFLOW);
        }

        self.acc = result;
        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

    /// Store Acc at Memory Address
    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.write_u8(addr, self.acc);
    }

    /// Store X Register at Memory Address
    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.write_u8(addr, self.rxi);
    }

    /// Store Y register at Memory Address
    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.write_u8(addr, self.ryi);
    }

    /// Copy Acc to X Register
    fn tax(&mut self) {
        self.rxi = self.acc;
        self.set_zero(self.rxi);
        self.set_negative(self.rxi);
    }

    /// Copy Acc to Y Register
    fn tay(&mut self) {
        self.ryi = self.acc;
        self.set_zero(self.ryi);
        self.set_negative(self.ryi);
    }

    /// Copy X Register to Acc
    fn txa(&mut self) {
        self.acc = self.rxi;
        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }

    /// Copy Y Register to Acc
    fn tya(&mut self) {
        self.acc = self.ryi;
        self.set_zero(self.acc);
        self.set_negative(self.acc);
    }
}
