#![allow(unused)]
use itertools::Itertools;
use my_lib::*;
use procon_input::*;
use rand::prelude::*;
use rand_pcg::Mcg128Xsl64;
use std::{
    cell::RefCell,
    clone,
    cmp::{max, min},
    collections::{BTreeMap, BTreeSet, BinaryHeap, VecDeque},
    iter::FromIterator,
    mem::swap,
    ops::*,
    rc::Rc,
    slice::SliceIndex,
};

fn main() {
    let start_time = my_lib::time::update();

    Sim::new().run();

    let end_time = my_lib::time::update();
    let duration = end_time - start_time;
    eprintln!("{:?} ", duration);
}

enum Response {
    NotBroken,
    Broken,
    Finish,
    Invalid,
}

#[derive(Debug, Clone)]
struct Field {
    n: usize,
    c: usize,
    is_broken: Vec<Vec<bool>>,
    total_cost: usize,
}

impl Field {
    fn new(n: usize, c: usize) -> Self {
        let is_broken = vec![vec![false; n]; n];
        Field {
            n,
            c,
            is_broken,
            total_cost: 0,
        }
    }

    fn query(&mut self, pos: &Pos, power: usize) -> Response {
        self.total_cost += power + self.c;
        println!("{} {} {}", pos.y, pos.x, power);

        let r = read_u();
        match r {
            0 => Response::NotBroken,
            1 => {
                self.is_broken[pos.y][pos.x] = true;
                Response::Broken
            }
            2 => {
                self.is_broken[pos.y][pos.x] = true;
                Response::Finish
            }
            _ => Response::Invalid,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Sim {
    input: Input,
}

impl Sim {
    fn new() -> Self {
        // TODO: impl input
        let input = Input::read();
        Sim { input }
    }

    pub fn run(&mut self) {}

    fn move_next(&self, start: Pos, next: Pos) {
        println!(
            "# move from ({}, {}) to ({}, {})",
            start.y, start.x, next.y, next.x
        );
    }

    fn excavate(&self, field: &mut Field, pos: &Pos, power: usize) {
        if field.is_broken[pos.y][pos.x] {
            panic!("this pos has already been broken");
        }
        let result = field.query(pos, power);
        match result {
            Response::Finish => {
                eprintln!("total cost {} ", field.total_cost);
            }
            Response::Invalid => {
                panic!("invalid: {:?}", pos);
            }
            _ => {}
        }
    }
}

#[derive(Debug, Clone)]
pub struct Input {
    n: usize,
    w: usize,
    k: usize,
    c: usize,
    source_vec: Vec<(Pos)>,
    house_vec: Vec<(Pos)>,
}

impl Input {
    fn read() -> Self {
        let (n, w, k, c) = read_uuuu();

        let mut source_vec = vec![];
        for _ in 0..w {
            let (y, x) = read_uu();
            let pos = Pos::new(y, x);
            source_vec.push(pos);
        }

        let mut house_vec = vec![];
        for _ in 0..k {
            let (y, x) = read_uu();
            let pos = Pos::new(y, x);
            house_vec.push(pos);
        }

        Input {
            n,
            w,
            k,
            c,
            source_vec,
            house_vec,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Output {
    //score: usize,
}

impl Output {
    fn new() -> Self {
        Output {}
    }

    fn remove(&self, output: &mut Self, rng: &mut Mcg128Xsl64) {
        // https://atcoder.jp/contests/ahc014/submissions/35567589 L558
    }

    fn submit(&self) {
        //println!("{}", );
    }
}

mod solver {
    use super::*;
}

mod my_lib {
    //! 基本的に問題によらず変えない自作ライブラリ群
    use super::*;
    pub mod time {
        //! 時間管理モジュール
        pub fn update() -> f64 {
            static mut STARTING_TIME_MS: Option<f64> = None;
            let t = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap();
            let time_ms = t.as_secs() as f64 + t.subsec_nanos() as f64 * 1e-9;
            unsafe {
                let now = match STARTING_TIME_MS {
                    Some(starting_time_ms) => time_ms - starting_time_ms,
                    None => {
                        STARTING_TIME_MS = Some(time_ms);
                        0.0 as f64
                    }
                };
                now
            }
        }

        // TODO: set LIMIT
        pub const LIMIT: f64 = 0.3;
    }

    static mut WIDTH: Option<usize> = None;

    #[derive(Debug, Clone, PartialEq, Eq, Copy)]
    pub struct Pos {
        pub y: usize, // ↓
        pub x: usize, // →
    }

    impl Pos {
        pub fn new(y: usize, x: usize) -> Self {
            Pos { y, x }
        }

        pub fn set_width(width: usize) {
            unsafe {
                WIDTH = Some(width);
            }
        }
    }

    pub trait SortFloat {
        fn sort(&mut self);
        fn sort_rev(&mut self);
    }

    impl SortFloat for Vec<f64> {
        fn sort(&mut self) {
            //! 浮動小数点としてNANが含まれないことを約束されている場合のsort処理<br>
            //! 小さい順
            self.sort_by(|a, b| a.partial_cmp(b).unwrap());
        }
        fn sort_rev(&mut self) {
            //! 浮動小数点としてNANが含まれないことを約束されている場合のsort処理<br>  
            //! 大きい順
            self.sort_by(|a, b| b.partial_cmp(a).unwrap());
        }
    }

    pub trait EvenOdd {
        fn is_even(&self) -> bool;
        fn is_odd(&self) -> bool;
    }

    impl EvenOdd for usize {
        fn is_even(&self) -> bool {
            self % 2 == 0
        }

        fn is_odd(&self) -> bool {
            self % 2 != 0
        }
    }
}

mod procon_input {
    use std::{any::type_name, io::*};

    fn read_block<T: std::str::FromStr>() -> T {
        let mut s = String::new();
        let mut buf = [0];
        loop {
            stdin().read(&mut buf).expect("can't read.");
            let c = buf[0] as char;
            if c == ' ' {
                break;
            }
            // for Linux
            if c == '\n' {
                break;
            }
            // for Windows
            if c == '\r' {
                // pop LR(line feed)
                stdin().read(&mut buf).expect("can't read.");
                break;
            }
            s.push(c);
        }
        s.parse::<T>()
            .unwrap_or_else(|_| panic!("can't parse '{}' to {}", s, type_name::<T>()))
    }

    pub fn read_i() -> i64 {
        read_block::<i64>()
    }

    pub fn read_ii() -> (i64, i64) {
        (read_block::<i64>(), read_block::<i64>())
    }

    pub fn read_iii() -> (i64, i64, i64) {
        (
            read_block::<i64>(),
            read_block::<i64>(),
            read_block::<i64>(),
        )
    }

    pub fn read_iiii() -> (i64, i64, i64, i64) {
        (
            read_block::<i64>(),
            read_block::<i64>(),
            read_block::<i64>(),
            read_block::<i64>(),
        )
    }

    pub fn read_u() -> usize {
        read_block::<usize>()
    }

    pub fn read_uu() -> (usize, usize) {
        (read_block::<usize>(), read_block::<usize>())
    }

    pub fn read_uuu() -> (usize, usize, usize) {
        (
            read_block::<usize>(),
            read_block::<usize>(),
            read_block::<usize>(),
        )
    }

    pub fn read_uuuu() -> (usize, usize, usize, usize) {
        (
            read_block::<usize>(),
            read_block::<usize>(),
            read_block::<usize>(),
            read_block::<usize>(),
        )
    }

    pub fn read_f() -> f64 {
        read_block::<f64>()
    }

    pub fn read_ff() -> (f64, f64) {
        (read_block::<f64>(), read_block::<f64>())
    }

    pub fn read_c() -> char {
        read_block::<char>()
    }

    pub fn read_cc() -> (char, char) {
        (read_block::<char>(), read_block::<char>())
    }

    fn read_line() -> String {
        let mut s = String::new();
        stdin().read_line(&mut s).expect("can't read.");
        s.trim()
            .parse()
            .unwrap_or_else(|_| panic!("can't trim in read_line()"))
    }

    pub fn read_vec<T: std::str::FromStr>() -> Vec<T> {
        read_line()
            .split_whitespace()
            .map(|e| {
                e.parse()
                    .unwrap_or_else(|_| panic!("can't parse '{}' to {}", e, type_name::<T>()))
            })
            .collect()
    }

    pub fn read_i_vec() -> Vec<i64> {
        read_line()
            .split_whitespace()
            .map(|e| {
                e.parse()
                    .unwrap_or_else(|_| panic!("can't parse '{}' to {}", e, type_name::<i64>()))
            })
            .collect()
    }

    pub fn read_u_vec() -> Vec<usize> {
        read_line()
            .split_whitespace()
            .map(|e| {
                e.parse()
                    .unwrap_or_else(|_| panic!("can't parse '{}' to {}", e, type_name::<usize>()))
            })
            .collect()
    }

    pub fn read_f_vec() -> Vec<f64> {
        read_line()
            .split_whitespace()
            .map(|e| {
                e.parse()
                    .unwrap_or_else(|_| panic!("can't parse '{}' to {}", e, type_name::<f64>()))
            })
            .collect()
    }

    pub fn read_c_vec() -> Vec<char> {
        read_line()
            .split_whitespace()
            .map(|e| {
                e.parse()
                    .unwrap_or_else(|_| panic!("can't parse '{}' to {}", e, type_name::<char>()))
            })
            .collect()
    }

    pub fn read_line_as_chars() -> Vec<char> {
        read_line().as_bytes().iter().map(|&b| b as char).collect()
    }

    pub fn read_string() -> String {
        read_block()
    }
}
