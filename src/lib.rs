pub mod cpu;
pub mod instructions;

#[cfg(test)]
mod test {
    use crate::cpu::*;
    use crate::instructions::*;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.acc, 0x05);
        assert!(cpu.flags.is_empty());
        assert!(cpu.flags.is_empty());
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.flags.contains(State::ZERO));
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.acc = 10;
        cpu.interpret(vec![0xaa, 0x00]);
        assert_eq!(cpu.rxi, 10);
    }
}
