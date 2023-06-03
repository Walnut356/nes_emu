use num_enum::{FromPrimitive, IntoPrimitive, TryFromPrimitive};

#[derive(Debug, Eq, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum Instruction {
    BRK = 0x00,
    LDA = 0xA9,
    TAX = 0xAA,
}
