use std::time::Instant;
use std::io::prelude::*;
use std::fs::File;

#[derive(Debug)]
enum Register {
    Reg0,
    Reg1,
    Reg2,
    Reg3,
    RegFP,
    RegSP,
    RegAC,
    RegPC
}

#[derive(Debug)]
struct Immediate {
    value: u16
}

#[derive(Debug)]
struct JMPFlag {
    relative: bool
}

impl Register {
    fn from_code(code: u16) -> Register {
        match code {
            0x0 => Register::Reg0,
            0x1 => Register::Reg1,
            0x2 => Register::Reg2,
            0x3 => Register::Reg3,
            0x4 => Register::RegSP,
            0x5 => Register::RegFP,
            0x6 => Register::RegAC,
            0x7 => Register::RegPC,
            _ => panic!("Invalid register")
        }
    }
}

#[derive(Debug)]
enum Instruction {
    ADD(Register, Register, Register),
    ADDI(Register, Immediate),

    MUL(Register, Register, Register),
    MULI(Register, Immediate),

    AND(Register, Register, Register),
    OR(Register, Register, Register),
    XOR(Register, Register, Register),
    MOV(Register, Register),

    NEG(Register, Register),

    LD(Register, Register),
    ST(Register, Register),

    BZ(Register, Register, JMPFlag),
    BNZ(Register, Register, JMPFlag),

    HLT()
}

impl Instruction {
    fn from_word(word: u16) -> Instruction {
        let opcode = word >> 12;

        // opcd sc0 sc1 dst xxx
        // 1111 111 111 111 111
        let reg_x = Register::from_code((word >> 9) & 0x7);
        let reg_y = Register::from_code((word >> 6) & 0x7);
        let reg_z = Register::from_code((word >> 3) & 0x7);

        // opcd sc0 immediate
        // 1111 111 111111111
        let imm = Immediate { value: word & 0x1FF };

        // opcd sc0 jmp r xxxxx
        // 1111 111 111 1 11111
        let jmp_flag = JMPFlag { relative: ((word >> 5) & 1) == 1};

        match opcode {
            0x0 => Instruction::ADD(reg_x, reg_y, reg_z),
            0x1 => Instruction::ADDI(reg_x, imm),

            0x2 => Instruction::MUL(reg_x, reg_y, reg_z),
            0x3 => Instruction::MULI(reg_x, imm),
            
            0x4 => Instruction::AND(reg_x, reg_y, reg_z),
            0x5 => Instruction::OR(reg_x, reg_y, reg_z),
            0x6 => Instruction::XOR(reg_x, reg_y, reg_z),
            0x7 => Instruction::MOV(reg_x, reg_y),
            
            0x8 => Instruction::NEG(reg_x, reg_y),

            0x9 => Instruction::LD(reg_x, reg_y),
            0xa => Instruction::ST(reg_x, reg_y),
            
            0xb => Instruction::BZ(reg_x, reg_y, jmp_flag),
            0xc => Instruction::BNZ(reg_x, reg_y, jmp_flag),

            0xf => Instruction::HLT(),
            _ => panic!("Invalid opcode")
        }
    }
}

#[derive(Debug)]
struct Processor {
    r0: u16,
    r1: u16,
    r2: u16,
    r3: u16,
    sp: u16,
    fp: u16,
    ac: u16,
    pc: u16
}

impl Processor {
    fn execute_next(&mut self, rom: &[u16; 65536], ram: &mut [u16; 65536]) {
        let instruction = Instruction::from_word(rom[self.pc as usize]);

        match instruction {
            Instruction::ADD(ref reg_x, ref reg_y, ref reg_z) => {
                let new_value = self.read_register(reg_x).wrapping_add(self.read_register(reg_y));
                self.write_register(reg_z, new_value);
            },

            Instruction::ADDI(ref reg_x, ref imm) => {
                let reg_x_value = self.read_register(reg_x);
                let new_value = reg_x_value.wrapping_add(imm.value as u16);
                self.write_register(&Register::RegAC, new_value);
            },

            Instruction::MUL(ref reg_x, ref reg_y, ref reg_z) => {
                let new_value = self.read_register(reg_x).wrapping_mul(self.read_register(reg_y));
                self.write_register(reg_z, new_value);
            },

            Instruction::MULI(ref reg_x, ref imm) => {
                let reg_x_value = self.read_register(reg_x);
                let new_value = reg_x_value.wrapping_mul(imm.value as u16);
                self.write_register(&Register::RegAC, new_value);
            },

            Instruction::MOV(ref reg_x, ref reg_y) => {
                let reg_x_value = self.read_register(reg_x);
                self.write_register(reg_y, reg_x_value);
            },

            Instruction::LD(ref reg_x, ref reg_y) => {
                let load_address = self.read_register(reg_x) as usize;
                let new_value = ram[load_address];
                self.write_register(reg_y, new_value as u16);
            },

            Instruction::ST(ref reg_x, ref reg_y) => {
                let store_address = self.read_register(reg_y) as usize;
                ram[store_address] = self.read_register(reg_x) as u16;
            },

            Instruction::BZ(ref reg_x, ref reg_y, ref jmp_flag) => {
                let reg_x_value = self.read_register(reg_x);
                let reg_y_value = self.read_register(reg_y);

                if self.read_register(reg_x) == 0 {
                    if jmp_flag.relative {
                        self.pc = self.pc.wrapping_add(reg_y_value);
                    } else {
                        self.pc = reg_y_value;
                    }
                }
            },
            Instruction::BNZ(ref reg_x, ref reg_y, ref jmp_flag) => {
                let reg_x_value = self.read_register(reg_x);
                let reg_y_value = self.read_register(reg_y);

                if self.read_register(reg_x) != 0 {
                    if jmp_flag.relative {
                        self.pc = self.pc.wrapping_add(reg_y_value);
                    } else {
                        self.pc = reg_y_value;
                    }
                }
            },
            Instruction::AND(ref reg_x, ref reg_y, ref reg_z) => {
                let new_value = self.read_register(reg_x) & self.read_register(reg_y);
                self.write_register(reg_z, new_value);
            },
            Instruction::OR(ref reg_x, ref reg_y, ref reg_z) => {
                let new_value = self.read_register(reg_x) | self.read_register(reg_y);
                self.write_register(reg_z, new_value);
            },
            Instruction::XOR(ref reg_x, ref reg_y, ref reg_z) => {
                let new_value = self.read_register(reg_x) ^ self.read_register(reg_y);
                self.write_register(reg_z, new_value);
            },
            Instruction::NEG(ref reg_x, ref reg_y) => {
                let new_value = self.read_register(reg_x).wrapping_mul(0xFFFF);
                self.write_register(reg_y, new_value);
            },
            Instruction::HLT() => {
                std::process::exit(0);
            }
        };

        self.pc = self.pc.wrapping_add(1);
    }

    fn write_register(&mut self, register: &Register, val: u16) {
        match *register {
            Register::Reg0 => self.r0 = val,
            Register::Reg1 => self.r1 = val,
            Register::Reg2 => self.r2 = val,
            Register::Reg3 => self.r3 = val,
            Register::RegFP => self.fp = val,
            Register::RegSP => self.sp = val,
            Register::RegAC => self.ac = val,
            Register::RegPC => self.pc = val
        }
    }

    fn read_register(&self, register: &Register) -> u16 {
        match *register {
            Register::Reg0 => self.r0,
            Register::Reg1 => self.r1,
            Register::Reg2 => self.r2,
            Register::Reg3 => self.r3,
            Register::RegFP => self.fp,
            Register::RegSP => self.sp,
            Register::RegAC => self.ac,
            Register::RegPC => self.pc
        }
    }
}

fn main() {
    let mut rom: [u16; 65536] = [0xFF; 65536];
    let mut ram: [u16; 65536] = [0; 65536];

    let mut program = File::open("/tmp/henlo.bin").unwrap();

    let mut bbuf = [0; 2];
    let mut i = 0;
    loop {
        match program.read_exact(&mut bbuf) {
            Ok(v) => v,
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::UnexpectedEof => {
                        break;
                    },
                    _ => panic!("Error reading program: {:?}", e)
                }
            }
        }
        rom[i] = (bbuf[0] as u16) * 256 + (bbuf[1] as u16);
        i += 1;
    }

    let mut processor = Processor {
        r0: 0,
        r1: 0,
        r2: 0,
        r3: 0,
        sp: 0,
        fp: 0,
        ac: 0,
        pc: 0
    };

    let start = Instant::now();
    for _ in 1..100 {
        println!("{:?} {:?}", processor, Instruction::from_word(rom[processor.pc as usize]));
        processor.execute_next(&rom, &mut ram);
    }

    let elapsed = start.elapsed();
    println!("{:?} ms", (elapsed.as_secs() * 1_000) + (elapsed.subsec_nanos() / 1_000_000) as u64);
}
