// Instruction reference - https://www.nesdev.org/obelisk-6502-guide/reference.html
// Chip wiki page - https://en.wikipedia.org/wiki/MOS_Technology_6502
// DevDoc pdf - https://www.nesdev.org/NESDoc.pdf
// interactive nes programming - https://skilldrick.github.io/easy6502/

pub mod cpu;
pub mod instructions;

#[cfg(test)]
mod test {
    use crate::cpu::*;
    use crate::instructions::*;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xa9, 0x05, 0x00]);
        cpu.run();
        assert_eq!(cpu.acc, 0x05);
        assert!(cpu.flags.is_empty());
        assert!(cpu.flags.is_empty());
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xa9, 0x00, 0x00]);
        cpu.run();
        assert!(cpu.flags.contains(State::ZERO));
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.acc = 10;
        cpu.load(vec![0xaa, 0x00]);
        cpu.run();
        assert_eq!(cpu.rxi, 10);
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);
        cpu.run();

        assert_eq!(cpu.rxi, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.rxi = 0xff;
        cpu.load(vec![0xe8, 0xe8, 0x00]);
        cpu.run();

        assert_eq!(cpu.rxi, 1)
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.write_u8(0x10, 0x55);

        cpu.load(vec![0xa5, 0x10, 0x00]);
        cpu.run();

        assert_eq!(cpu.acc, 0x55);
    }
}
