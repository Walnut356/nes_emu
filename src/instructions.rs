use std::collections::HashMap;

use lazy_static::lazy_static;
use num_enum::{IntoPrimitive, TryFromPrimitive};

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Implied,
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect,
    Indirect_X,
    Indirect_Y,
    Relative,
    None,
}

#[derive(Debug, Eq, PartialEq, TryFromPrimitive, IntoPrimitive, Copy, Clone)]
#[repr(u8)]
pub enum Instruction {
    /// Add with Carry
    ADC,
    /// Logical AND
    AND,
    /// Arithmetic Shift Left
    ASL,
    /// Branch if Carry Clear
    BCC,
    /// Branch if Carry Set
    BCS,
    /// Branch if Equal
    BEQ,
    /// Bit Test
    BIT,
    /// Branch if Minus
    BMI,
    /// Branch if Not Equal
    BNE,
    /// Branch if Positive
    BPL,
    /// Force Interrupt
    BRK,
    /// Branch if Overflow Clear
    BVC,
    /// Branch if Overflow Set
    BVS,
    /// Clear Carry Flag
    CLC,
    /// Clear Decimal Mode
    CLD,
    /// Clear Interrupt Disable
    CLI,
    /// Clear Overflow Flag
    CLV,
    /// Compare
    CMP,
    /// Compare X Register
    CPX,
    /// Compare Y Register
    CPY,
    /// Decrement Memory
    DEC,
    /// Decrement X Register
    DEX,
    /// Decrement Y Register
    DEY,
    /// Exclusive OR
    EOR,
    /// Increment Memory
    INC,
    /// Increment X Register
    INX,
    /// Increment Y Register
    INY,
    /// Jump
    JMP,
    /// Jump to Subroutine
    JSR,
    /// Load Accumulator
    LDA,
    /// Load X Register
    LDX,
    /// Load Y Register
    LDY,
    /// Logical Shift Right
    LSR,
    /// No Op
    NOP,
    /// Logical Inclusive OR
    ORA,
    /// Push Accumulator to Stack
    PHA,
    /// Push Flags to Stack
    PHP,
    /// Pull u8 from Stack to Accumulator
    PLA,
    /// Pull u8 from Stack to Flags
    PLP,
    /// Rotate Left
    ROL,
    /// Rotate Right
    ROR,
    /// Return from Interrupt
    RTI,
    /// Return from Subroutine
    RTS,
    /// Subtract with Carry
    SBC,
    /// Set Carry Flag
    SEC,
    /// Set Decimal Flag
    SED,
    /// Set Interrupt Disable
    SEI,
    /// Store Accumulator to Memory Address
    STA,
    /// Store X Register to Memory Address
    STX,
    /// Store Y Register to Memory Address
    STY,
    /// Transfer Accumulator to X Register
    TAX,
    /// Transfer Accumulator to Y Register
    TAY,
    /// Transfer Stack Pointer to X Register
    TSX,
    /// Transfer X Register to Accumulator
    TXA,
    /// Transfer X register to Stack Pointer
    TXS,
    /// Transfer Y register to Accumulator
    TYA,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct OpCode {
    pub instr: Instruction,
    pub code: u8,
    pub addr_mode: AddressingMode,
    pub byte_length: u16,
    pub cycles: u8,
}

impl OpCode {
    pub fn new(
        instr: Instruction,
        code: u8,
        addr_mode: AddressingMode,
        bytes: u16,
        cycles: u8,
    ) -> Self {
        OpCode {
            instr,
            code,
            addr_mode,
            byte_length: bytes,
            cycles,
        }
    }
}

lazy_static! {
    pub static ref OPCODES_MAP: HashMap<u8, OpCode> = {
        let mut map: HashMap<u8, OpCode> = HashMap::new();
        use Instruction::*;
        use AddressingMode::*;

        // ADC
        map.insert(0x69, OpCode::new(ADC, 0x69, Immediate, 2, 2));
        map.insert(0x65, OpCode::new(ADC, 0x65, ZeroPage, 2, 3));
        map.insert(0x75, OpCode::new(ADC, 0x75, ZeroPage_X, 2, 4));
        map.insert(0x6D, OpCode::new(ADC, 0x6D, Absolute, 3, 4));
        map.insert(0x7D, OpCode::new(ADC, 0x7D, Absolute_X, 3, 4));
        map.insert(0x79, OpCode::new(ADC, 0x79, Absolute_Y, 3, 4));
        map.insert(0x61, OpCode::new(ADC, 0x61, Indirect_X, 2, 6));
        map.insert(0x71, OpCode::new(ADC, 0x71, Indirect_Y, 2, 5));

        // AND
        map.insert(0x29, OpCode::new(AND, 0x29, Immediate, 2, 2));
        map.insert(0x25, OpCode::new(AND, 0x25, ZeroPage, 2, 3));
        map.insert(0x35, OpCode::new(AND, 0x35, ZeroPage_X, 2, 4));
        map.insert(0x2D, OpCode::new(AND, 0x2D, Absolute, 3, 4));
        map.insert(0x3D, OpCode::new(AND, 0x3D, Absolute_X, 3, 4));
        map.insert(0x39, OpCode::new(AND, 0x39, Absolute_Y, 3, 4));
        map.insert(0x21, OpCode::new(AND, 0x21, Indirect_X, 2, 6));
        map.insert(0x31, OpCode::new(AND, 0x31, Indirect_Y, 2, 5));

        // ASL
        map.insert(0x0A, OpCode::new(ASL, 0x0A, None, 1, 2));
        map.insert(0x06, OpCode::new(ASL, 0x06, ZeroPage, 2, 5));
        map.insert(0x16, OpCode::new(ASL, 0x16, ZeroPage_X, 2, 6));
        map.insert(0x0E, OpCode::new(ASL, 0x0E, Absolute, 3, 6));
        map.insert(0x1E, OpCode::new(ASL, 0x1E, Absolute_X, 3, 7));

        // BCC
        map.insert(0x90, OpCode::new(BCC, 0x90, Relative, 2, 2));

        // BCS
        map.insert(0xB0, OpCode::new(BCS, 0xB0, Relative, 2, 2));

        // BEQ
        map.insert(0xF0, OpCode::new(BCS, 0xF0, Relative, 2, 2));

        // BIT
        map.insert(0x24, OpCode::new(BIT, 0x24, ZeroPage, 2, 3));
        map.insert(0x2C, OpCode::new(BIT, 0x2C, Absolute, 3, 4));

        // BMI
        map.insert(0x30, OpCode::new(BMI, 0xB0, Relative, 2, 2));

        // BNE
        map.insert(0xD0, OpCode::new(BNE, 0xD0, Relative, 2, 2));

        // BPL
        map.insert(0x10, OpCode::new(BPL, 0x10, Relative, 2, 2));

        // BRK
        map.insert(0x00, OpCode::new(BRK, 0x00, Implied, 1, 7));

        // BVC
        map.insert(0x50, OpCode::new(BVC, 0x50, Relative, 2, 2));

        // BVS
        map.insert(0x70, OpCode::new(BVS, 0x70, Relative, 2, 2));

        // CLC
        map.insert(0x18, OpCode::new(CLC, 0x18, Implied, 1, 2));

        // CLD
        map.insert(0xD8, OpCode::new(CLD, 0xD8, Implied, 1, 2));

        // CLI
        map.insert(0x58, OpCode::new(CLI, 0x58, Implied, 1, 2));

        // CLV
        map.insert(0xB8, OpCode::new(CLV, 0xB8, Implied, 1, 2));

        // CMP
        map.insert(0xC9, OpCode::new(CMP, 0xC9, Immediate, 2, 2));
        map.insert(0xC5, OpCode::new(CMP, 0xC5, ZeroPage, 2, 3));
        map.insert(0xD5, OpCode::new(CMP, 0xD5, ZeroPage_X, 2, 4));
        map.insert(0xCD, OpCode::new(CMP, 0xCD, Absolute, 3, 4));
        map.insert(0xDD, OpCode::new(CMP, 0xDD, Absolute_X, 3, 4));
        map.insert(0xD9, OpCode::new(CMP, 0xD9, Absolute_Y, 3, 4));
        map.insert(0xC1, OpCode::new(CMP, 0xC1, Indirect_X, 2, 6));
        map.insert(0xD1, OpCode::new(CMP, 0xD1, Indirect_Y, 2, 5));

        // CPX
        map.insert(0xE0, OpCode::new(CPX, 0xE0, Immediate, 2, 2));
        map.insert(0xE4, OpCode::new(CPX, 0xE4, ZeroPage, 2, 3));
        map.insert(0xEC, OpCode::new(CPX, 0xEC, Absolute, 3, 4));

        // CPY
        map.insert(0xC0, OpCode::new(CPY, 0xC0, Immediate, 2, 2));
        map.insert(0xC4, OpCode::new(CPY, 0xC4, ZeroPage, 2, 3));
        map.insert(0xCC, OpCode::new(CPY, 0xCC, Absolute, 3, 4));

        // DEC
        map.insert(0xC6, OpCode::new(DEC, 0xC6, ZeroPage, 2, 5));
        map.insert(0xD6, OpCode::new(DEC, 0xD6, ZeroPage_X, 2, 6));
        map.insert(0xCE, OpCode::new(DEC, 0xCE, Absolute, 3, 6));
        map.insert(0xDE, OpCode::new(DEC, 0xDE, Absolute_X, 3, 7));

        // DEX
        map.insert(0xCA, OpCode::new(DEX, 0xCA, Implied, 1, 2));

        // DEY
        map.insert(0x88, OpCode::new(DEY, 0x88, Implied, 1, 2));

        // EOR
        map.insert(0x49, OpCode::new(EOR, 0x49, Immediate, 2, 2));
        map.insert(0x45, OpCode::new(EOR, 0x45, ZeroPage, 2, 3));
        map.insert(0x55, OpCode::new(EOR, 0x55, ZeroPage_X, 2, 4));
        map.insert(0x4D, OpCode::new(EOR, 0x4D, Absolute, 3, 4));
        map.insert(0x5D, OpCode::new(EOR, 0x5D, Absolute_X, 3, 4));
        map.insert(0x59, OpCode::new(EOR, 0x59, Absolute_Y, 3, 4));
        map.insert(0x41, OpCode::new(EOR, 0x41, Indirect_X, 2, 6));
        map.insert(0x51, OpCode::new(EOR, 0x51, Indirect_Y, 2, 5));

        // INC
        map.insert(0xE6, OpCode::new(INC, 0xE6, ZeroPage, 2, 5));
        map.insert(0xF6, OpCode::new(INC, 0xF6, ZeroPage_X, 2, 6));
        map.insert(0xEE, OpCode::new(INC, 0xEE, Absolute, 3, 6));
        map.insert(0xFE, OpCode::new(INC, 0xFE, Absolute_X, 3, 7));

        // INX
        map.insert(0xE8, OpCode::new(INX, 0xE8, Implied, 1, 2));

        // INY
        map.insert(0xC8, OpCode::new(INY, 0xC8, Implied, 1, 2));

        // JMP
        map.insert(0x4C, OpCode::new(JMP, 0x4C, Absolute, 3, 3));
        map.insert(0x6C, OpCode::new(JMP, 0x6C, Indirect, 3, 5));

        // JSR
        map.insert(0x20, OpCode::new(JSR, 0x20, Absolute, 3, 6));

        // LDA
        map.insert(0xA9, OpCode::new(LDA, 0xA9, Immediate, 2, 2));
        map.insert(0xA5, OpCode::new(LDA, 0xA5, ZeroPage, 2, 3));
        map.insert(0xB5, OpCode::new(LDA, 0xB5, ZeroPage_X, 2, 4));
        map.insert(0xAD, OpCode::new(LDA, 0xAD, Absolute, 3, 4));
        map.insert(0xBD, OpCode::new(LDA, 0xBD, Absolute_X, 3, 4));
        map.insert(0xB9, OpCode::new(LDA, 0xB9, Absolute_Y, 3, 4));
        map.insert(0xA1, OpCode::new(LDA, 0xA1, Indirect_X, 2, 6));
        map.insert(0xB1, OpCode::new(LDA, 0xB1, Indirect_Y, 2, 5));

        // LDX
        map.insert(0xA2, OpCode::new(LDX, 0xA2, Immediate, 2, 2));
        map.insert(0xA6, OpCode::new(LDX, 0xA6, ZeroPage, 2, 3));
        map.insert(0xB6, OpCode::new(LDX, 0xB6, ZeroPage_Y, 2, 4));
        map.insert(0xAE, OpCode::new(LDX, 0xAE, Absolute, 3, 4));
        map.insert(0xBE, OpCode::new(LDX, 0xBE, Absolute_Y, 3, 4));

        // LDY
        map.insert(0xA0, OpCode::new(LDY, 0xA0, Immediate, 2, 2));
        map.insert(0xA4, OpCode::new(LDY, 0xA4, ZeroPage, 2, 3));
        map.insert(0xB4, OpCode::new(LDY, 0xB4, ZeroPage_X, 2, 4));
        map.insert(0xAC, OpCode::new(LDY, 0xAC, Absolute, 3, 4));
        map.insert(0xBC, OpCode::new(LDY, 0xBC, Absolute_X, 3, 4));

        // LSR
        map.insert(0x4A, OpCode::new(LSR, 0x4A, Implied, 1, 2));
        map.insert(0x46, OpCode::new(LSR, 0x46, ZeroPage, 2, 5));
        map.insert(0x56, OpCode::new(LSR, 0x56, ZeroPage_X, 2, 6));
        map.insert(0x4E, OpCode::new(LSR, 0x4E, Absolute, 3, 6));
        map.insert(0x5E, OpCode::new(LSR, 0x5E, Absolute_X, 3, 7));

        // NOP
        map.insert(0xEA, OpCode::new(NOP, 0xEA, Implied, 1, 2));

        // ORA
        map.insert(0x09, OpCode::new(ORA, 0x09, Immediate, 2, 2));
        map.insert(0x05, OpCode::new(ORA, 0x05, ZeroPage, 2, 3));
        map.insert(0x15, OpCode::new(ORA, 0x15, ZeroPage_X, 2, 4));
        map.insert(0x0D, OpCode::new(ORA, 0x0D, Absolute, 3, 4));
        map.insert(0x1D, OpCode::new(ORA, 0x1D, Absolute_X, 3, 4));
        map.insert(0x19, OpCode::new(ORA, 0x19, Absolute_Y, 3, 4));
        map.insert(0x01, OpCode::new(ORA, 0x01, Indirect_X, 2, 6));
        map.insert(0x11, OpCode::new(ORA, 0x11, Indirect_Y, 2, 5));

        // PHA
        map.insert(0x48, OpCode::new(PHA, 0x48, Implied, 1, 3));

        // PHP
        map.insert(0x08, OpCode::new(PHP, 0x08, Implied, 1, 3));

        // PLA
        map.insert(0x68, OpCode::new(PLA, 0x68, Implied, 1, 4));

        // PLP
        map.insert(0x28, OpCode::new(PLP, 0x28, Implied, 1, 4));

        // ROL
        map.insert(0x2A, OpCode::new(ROL, 0x2A, Implied, 1, 2));
        map.insert(0x26, OpCode::new(ROL, 0x26, ZeroPage, 2, 5));
        map.insert(0x36, OpCode::new(ROL, 0x36, ZeroPage_X, 2, 6));
        map.insert(0x2E, OpCode::new(ROL, 0x2E, Absolute, 3, 6));
        map.insert(0x3E, OpCode::new(ROL, 0x3E, Absolute_X, 3, 7));

        // ROR
        map.insert(0x6A, OpCode::new(ROR, 0x6A, Implied, 1, 2));
        map.insert(0x66, OpCode::new(ROR, 0x66, ZeroPage, 2, 5));
        map.insert(0x76, OpCode::new(ROR, 0x76, ZeroPage_X, 2, 6));
        map.insert(0x6E, OpCode::new(ROR, 0x6E, Absolute, 3, 6));
        map.insert(0x7E, OpCode::new(ROR, 0x7E, Absolute_X, 3, 7));

        // RTI
        map.insert(0x40, OpCode::new(RTI, 0x40, Implied, 1, 6));

        // RTS
        map.insert(0x60, OpCode::new(RTS, 0x40, Implied, 1, 6));

        // SBC
        map.insert(0xE9, OpCode::new(SBC, 0xE9, Immediate, 2, 2));
        map.insert(0xE5, OpCode::new(SBC, 0xE5, ZeroPage, 2, 3));
        map.insert(0xF5, OpCode::new(SBC, 0xF5, ZeroPage_X, 2, 4));
        map.insert(0xED, OpCode::new(SBC, 0xED, Absolute, 3, 4));
        map.insert(0xFD, OpCode::new(SBC, 0xFD, Absolute_X, 3, 4));
        map.insert(0xF9, OpCode::new(SBC, 0xF9, Absolute_Y, 3, 4));
        map.insert(0xE1, OpCode::new(SBC, 0xE1, Indirect_X, 2, 6));
        map.insert(0xF1, OpCode::new(SBC, 0xF1, Indirect_Y, 2, 5));

        // SEC
        map.insert(0x38, OpCode::new(SEC, 0x38, Implied, 1, 2));

        // SED
        map.insert(0xF8, OpCode::new(SED, 0xF8, Implied, 1, 2));

        // SEI
        map.insert(0x78, OpCode::new(SEI, 0x78, Implied, 1, 2));

        // STA
        map.insert(0x85, OpCode::new(STA, 0x85, ZeroPage, 2, 3));
        map.insert(0x95, OpCode::new(STA, 0x95, ZeroPage_X, 2, 4));
        map.insert(0x8D, OpCode::new(STA, 0x8D, Absolute, 3, 4));
        map.insert(0x9D, OpCode::new(STA, 0x9D, Absolute_X, 3, 5));
        map.insert(0x99, OpCode::new(STA, 0x99, Absolute_Y, 3, 5));
        map.insert(0x81, OpCode::new(STA, 0x81, Indirect_X, 2, 6));
        map.insert(0x91, OpCode::new(STA, 0x91, Indirect_Y, 2, 6));

        // STX
        map.insert(0x86, OpCode::new(STX, 0x86, ZeroPage, 2, 3));
        map.insert(0x96, OpCode::new(STX, 0x96, ZeroPage_Y, 2, 4));
        map.insert(0x8E, OpCode::new(STX, 0x8E, Absolute, 3, 4));

        // STY
        map.insert(0x84, OpCode::new(STY, 0x84, ZeroPage, 2, 3));
        map.insert(0x94, OpCode::new(STY, 0x94, ZeroPage_Y, 2, 4));
        map.insert(0x8C, OpCode::new(STY, 0x8C, Absolute, 3, 4));

        // TAX
        map.insert(0xAA, OpCode::new(TAX, 0xAA, Implied, 1, 2));

        // TAY
        map.insert(0xA8, OpCode::new(TAY, 0xA8, Implied, 1, 2));

        // TSX
        map.insert(0xBA, OpCode::new(TSX, 0xBA, Implied, 1, 2));

        // TXA
        map.insert(0x8A, OpCode::new(TXA, 0x8A, Implied, 1, 2));

        // TXS
        map.insert(0x9A, OpCode::new(TXS, 0x9A, Implied, 1, 2));

        // TYA
        map.insert(0x98, OpCode::new(TYA, 0x98, Implied, 1, 2));


        map
    };
}
