use enumflags2::{bitflags, BitFlags};
use nes_emu::cpu::State;

fn main() {
    let mut x: BitFlags<State> = BitFlags::default();
    x |= State::ZERO;
    x.insert(State::NEGATIVE);
    x.remove(State::ZERO);
    println!("{:?}", x);
}
