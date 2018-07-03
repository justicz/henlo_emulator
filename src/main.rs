use std::time::Instant;
use std::io::prelude::*;
use std::fs::File;

#[derive(Debug)]
enum Register {
    Acc,
    Reg0,
    Reg1,
    Reg2
}

#[derive(Debug)]
struct Immediate {
    value: u8
}

impl Register {
    fn from_code(code: u8) -> Register {
        match code {
            0b00 => Register::Reg0,
            0b01 => Register::Reg1,
            0b10 => Register::Reg2,
            0b11 => Register::Acc,
            _ => panic!("Invalid register")
        }
    }
}

#[derive(Debug)]
enum Instruction {
    Add(Register, Register),
    AddI(Immediate),
    Mul(Register, Register),
    MulI(Immediate),
    LD0(Register, Register),
    LD1(Register, Register),
    ST0(Register, Register),
    ST1(Register, Register),
    JNEZ(Register, Register),
    RJNEZ(Register, Register)
}

impl Instruction {
    fn from_byte(byte: u8) -> Instruction {
        let opcode: u8 = byte >> 4;
        let reg_x = Register::from_code((byte >> 2) & 3);
        let reg_y = Register::from_code(byte & 3);
        let imm = Immediate { value: byte & 15 };

        match opcode {
            0b1000 => Instruction::Add(reg_x, reg_y),
            0b1001 => Instruction::AddI(imm),
            0b1010 => Instruction::Mul(reg_x, reg_y),
            0b1011 => Instruction::MulI(imm),
            0b0000 => Instruction::LD0(reg_x, reg_y),
            0b0001 => Instruction::LD1(reg_x, reg_y),
            0b0010 => Instruction::ST0(reg_x, reg_y),
            0b0011 => Instruction::ST1(reg_x, reg_y),
            0b0100 => Instruction::JNEZ(reg_x, reg_y),
            0b0101 => Instruction::RJNEZ(reg_x, reg_y),
            _ => panic!("Invalid opcode")
        }
    }
}

#[derive(Debug)]
struct Processor {
    pc: u16,
    reg0: u16,
    reg1: u16,
    reg2: u16,
    acc: u16
}

impl Processor {
    fn execute_next(&mut self, rom: &[u8; 65536], ram_0: &mut [u8; 65536], ram_1: &mut [u8; 65536]) {
        let instruction = Instruction::from_byte(rom[self.pc as usize]);

        self.pc = self.pc.wrapping_add(1);

        match instruction {
            Instruction::Add(ref reg_x, ref reg_y) => {
                let new_value = self.read_register(&Register::Acc).wrapping_add(self.read_register(reg_x));
                self.write_register(reg_y, new_value);
            },

            Instruction::AddI(ref imm) => {
                let new_value = self.read_register(&Register::Acc).wrapping_add(imm.value as u16);
                self.write_register(&Register::Acc, new_value);
            },

            Instruction::Mul(ref reg_x, ref reg_y) => {
                let new_value = self.read_register(&Register::Acc).wrapping_mul(self.read_register(reg_x));
                self.write_register(reg_y, new_value);
            },

            Instruction::MulI(ref imm) => {
                let new_value = self.read_register(&Register::Acc).wrapping_mul(imm.value as u16);
                self.write_register(&Register::Acc, new_value);
            },

            Instruction::LD0(ref reg_x, ref reg_y) => {
                let load_address = self.read_register(reg_x) as usize;
                let new_value = ram_0[load_address];
                self.write_register(reg_y, new_value as u16);
            },

            Instruction::LD1(ref reg_x, ref reg_y) => {
                let load_address = self.read_register(reg_x) as usize;
                let new_value = ram_1[load_address];
                self.write_register(reg_y, new_value as u16);
            },

            Instruction::ST0(ref reg_x, ref reg_y) => {
                let store_address = self.read_register(reg_y) as usize;
                ram_0[store_address] = self.read_register(reg_x) as u8;
            },
            
            Instruction::ST1(ref reg_x, ref reg_y) => {
                let store_address = self.read_register(reg_y) as usize;
                ram_1[store_address] = self.read_register(reg_x) as u8;
            },
            
            Instruction::JNEZ(ref reg_x, ref reg_y) => {
                let reg_x_value = self.read_register(reg_x);
                let reg_y_value = self.read_register(reg_y);
                if reg_x_value != 0 {
                    self.pc = reg_y_value;
                }
            },

            Instruction::RJNEZ(ref reg_x, ref reg_y) => {
                let reg_x_value = self.read_register(reg_x);
                let reg_y_value = self.read_register(reg_y);
                if reg_x_value != 0 {
                    self.pc += reg_y_value;
                }
            }
        }
    }

    fn write_register(&mut self, register: &Register, val: u16) {
        match *register {
            Register::Reg0 => self.reg0 = val,
            Register::Reg1 => self.reg1 = val,
            Register::Reg2 => self.reg2 = val,
            Register::Acc => self.acc = val
        }
    }

    fn read_register(&self, register: &Register) -> u16 {
        match *register {
            Register::Reg0 => self.reg0,
            Register::Reg1 => self.reg1,
            Register::Reg2 => self.reg2,
            Register::Acc => self.acc
        }
    }
}

fn main() {
    let mut rom: [u8; 65536] = [0; 65536];
    let mut ram_0: [u8; 65536] = [0; 65536];
    let mut ram_1: [u8; 65536] = [0; 65536];

    let program = File::open("src/tpt.bin").unwrap();

    for (i, byte) in program.bytes().enumerate() {
        if i > rom.len() - 1 {
            panic!("Program too big for ROM")
        }
        let byte = byte.unwrap();
        rom[i] = byte;
        println!("{}: {:?}", i, Instruction::from_byte(byte));
    }

    let mut processor = Processor {
        pc: 0,
        acc: 0,
        reg0: 0,
        reg1: 0,
        reg2: 0
    };

    let start = Instant::now();
    for _ in 1..100 {
        processor.execute_next(&rom, &mut ram_0, &mut ram_1);
        println!("{:?}", processor);
    }
    let elapsed = start.elapsed();
    println!("{:?} ms", (elapsed.as_secs() * 1_000) + (elapsed.subsec_nanos() / 1_000_000) as u64);
}
