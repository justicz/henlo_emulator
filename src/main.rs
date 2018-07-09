use std::time::Instant;
use std::io::prelude::*;
use std::fs::File;

#[derive(Debug)]
enum Register {
    Reg0,
    RegFP,
    RegSP,
    RegAcc
}

#[derive(Debug)]
struct Immediate {
    value: u8
}

impl Register {
    fn from_code(code: u8) -> Register {
        match code {
            0x0 => Register::Reg0,
            0x1 => Register::RegFP,
            0x2 => Register::RegSP,
            0x3 => Register::RegAcc,
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
    BNEZ(Register, Register),
    JMP(Register),
    SH48(Immediate),
    AND(Register, Register),
    OR(Register, Register),
    XOR(Register, Register),
    NEG(Register)
}

impl Instruction {
    fn from_byte(byte: u8) -> Instruction {
        let opcode: u8 = byte >> 4;
        let reg_x = Register::from_code((byte >> 2) & 3);
        let reg_y = Register::from_code(byte & 3);
        let imm = Immediate { value: byte & 15 };

        match opcode {
            0x0 => Instruction::Add(reg_x, reg_y),
            0x1 => Instruction::AddI(imm),
            0x2 => Instruction::Mul(reg_x, reg_y),
            0x3 => Instruction::MulI(imm),
            0x4 => Instruction::LD0(reg_x, reg_y),
            0x5 => Instruction::LD1(reg_x, reg_y),
            0x6 => Instruction::ST0(reg_x, reg_y),
            0x7 => Instruction::ST1(reg_x, reg_y),
            0x8 => Instruction::BNEZ(reg_x, reg_y),
            0x9 => Instruction::JMP(reg_x),
            0xa => Instruction::SH48(imm),
            0xb => Instruction::AND(reg_x, reg_y),
            0xc => Instruction::OR(reg_x, reg_y),
            0xd => Instruction::XOR(reg_x, reg_y),
            0xe => Instruction::NEG(reg_x),
            _ => panic!("Invalid opcode")
        }
    }
}

#[derive(Debug)]
struct Processor {
    pc: u16,
    reg0: u16,
    fp: u16,
    sp: u16,
    acc: u16
}

impl Processor {
    fn execute_next(&mut self, rom: &[u8; 65536], ram_0: &mut [u8; 65536], ram_1: &mut [u8; 65536]) {
        let instruction = Instruction::from_byte(rom[self.pc as usize]);

        self.pc = self.pc.wrapping_add(1);

        match instruction {
            Instruction::Add(ref reg_x, ref reg_y) => {
                let new_value = self.read_register(&Register::RegAcc).wrapping_add(self.read_register(reg_x));
                self.write_register(reg_y, new_value);
            },

            Instruction::AddI(ref imm) => {
                let new_value = self.read_register(&Register::RegAcc).wrapping_add(imm.value as u16);
                self.write_register(&Register::RegAcc, new_value);
            },

            Instruction::Mul(ref reg_x, ref reg_y) => {
                let new_value = self.read_register(&Register::RegAcc).wrapping_mul(self.read_register(reg_x));
                self.write_register(reg_y, new_value);
            },

            Instruction::MulI(ref imm) => {
                let new_value = self.read_register(&Register::RegAcc).wrapping_mul(imm.value as u16);
                self.write_register(&Register::RegAcc, new_value);
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
            
            Instruction::BNEZ(ref reg_x, ref reg_y) => {
                let reg_x_value = self.read_register(reg_x);
                let reg_y_value = self.read_register(reg_y);
                if reg_x_value != 0 {
                    self.pc = reg_y_value;
                }
            },
            Instruction::JMP(ref reg_x) => {
                let reg_x_value = self.read_register(reg_x);
                self.pc = reg_x_value;
            },
            Instruction::SH48(ref imm) => {
                let imm = imm.value as u16;
                let old_value = self.read_register(&Register::RegAcc);
                let mut new_value = 0;
                if imm == 12 {
                    new_value = old_value << 4;
                } else if imm == 14 {
                    new_value = old_value << 8;
                } else if imm == 4 {
                    new_value = old_value >> 4;
                } else if imm == 6 {
                    new_value = old_value >> 8;
                }
                self.write_register(&Register::RegAcc, new_value);
            },
            Instruction::AND(ref reg_x, ref reg_y) => {
                let reg_x_value = self.read_register(reg_x);
                let acc_value = self.read_register(&Register::RegAcc);
                self.write_register(reg_y, reg_x_value & acc_value);
            },
            Instruction::OR(ref reg_x, ref reg_y) => {
                let reg_x_value = self.read_register(reg_x);
                let acc_value = self.read_register(&Register::RegAcc);
                self.write_register(reg_y, reg_x_value | acc_value);
            },
            Instruction::XOR(ref reg_x, ref reg_y) => {
                let reg_x_value = self.read_register(reg_x);
                let acc_value = self.read_register(&Register::RegAcc);
                self.write_register(reg_y, reg_x_value ^ acc_value);
            },
            Instruction::NEG(ref reg_x) => {
                let acc_value = self.read_register(&Register::RegAcc);
                self.write_register(reg_x, (acc_value ^ 0xFFFF).wrapping_add(1));
            }
        }
    }

    fn write_register(&mut self, register: &Register, val: u16) {
        match *register {
            Register::Reg0 => self.reg0 = val,
            Register::RegFP => self.fp = val,
            Register::RegSP => self.sp = val,
            Register::RegAcc => self.acc = val
        }
    }

    fn read_register(&self, register: &Register) -> u16 {
        match *register {
            Register::Reg0 => self.reg0,
            Register::RegFP => self.fp,
            Register::RegSP => self.sp,
            Register::RegAcc => self.acc
        }
    }
}

fn main() {
    let mut rom: [u8; 65536] = [0; 65536];
    let mut ram_0: [u8; 65536] = [0; 65536];
    let mut ram_1: [u8; 65536] = [0; 65536];

    let program = File::open("/tmp/henlo.bin").unwrap();

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
        fp: 0,
        sp: 0
    };

    let start = Instant::now();
    for _ in 1..30 {
        processor.execute_next(&rom, &mut ram_0, &mut ram_1);
        println!("{:?}", processor);
    }
    let elapsed = start.elapsed();
    println!("{:?} ms", (elapsed.as_secs() * 1_000) + (elapsed.subsec_nanos() / 1_000_000) as u64);
}
