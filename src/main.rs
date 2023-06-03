use enumflags2::{bitflags, BitFlags};
use nes_emu::cpu::State;

fn main() {
    let x = [1, 2, 3, 4];
    let y = &x[0..3];

    println!("{:?}", y)
}
