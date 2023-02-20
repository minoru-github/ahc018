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

const IS_LOCAL: bool = true;

fn main() {
    let start_time = my_lib::time::update();

    Sim::new().run();

    let end_time = my_lib::time::update();
    let duration = end_time - start_time;
    eprintln!("{:?} ", duration);
}

#[derive(Debug, Clone)]
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
    estimated_toughness: Vec<Vec<Option<(usize, f32)>>>,
    est_tough_cands: Vec<Vec<Vec<(usize, f32)>>>,
}

impl Field {
    const WIDTH: usize = 200;
    fn new(n: usize, c: usize) -> Self {
        let is_broken = vec![vec![false; n]; n];
        let estimated_toughness = vec![vec![None; n]; n];
        let est_tough_cands = vec![vec![vec![]; n]; n];

        Field {
            n,
            c,
            is_broken,
            total_cost: 0,
            estimated_toughness,
            est_tough_cands,
        }
    }

    fn excavate(&mut self, pos: &Pos, power: usize) -> Response {
        if self.is_broken[pos.y][pos.x] {
            panic!("this pos has already been broken");
        }
        self.total_cost += power + self.c;
        //TODO
        //println!("{} {} {}", pos.y, pos.x, power);

        if IS_LOCAL {
            unsafe {
                let remained_toughness = TOUGHNESS[pos.y][pos.x];
                let mut responce = Response::Invalid;
                if power >= remained_toughness {
                    TOUGHNESS[pos.y][pos.x] = 0;
                    self.is_broken[pos.y][pos.x] = true;
                    responce = Response::Broken;
                } else {
                    TOUGHNESS[pos.y][pos.x] -= power;
                    responce = Response::NotBroken;
                }
                responce
            }
        } else {
            let r = read_u();
            match r {
                0 => Response::NotBroken,
                1 => {
                    self.is_broken[pos.y][pos.x] = true;
                    Response::Broken
                }
                2 => {
                    self.is_broken[pos.y][pos.x] = true;
                    eprintln!("total cost {} ", self.total_cost);
                    Response::Finish
                }
                _ => {
                    panic!("invalid: {:?}", pos);
                    Response::Invalid
                }
            }
        }
    }

    fn excavate_sources(&mut self, source_vec: &Vec<Pos>) {
        self.excavate_completely(source_vec);
    }

    fn excavate_houses(&mut self, house_vec: &Vec<Pos>) {
        self.excavate_completely(house_vec);
    }

    fn excavate_completely(&mut self, pos_vec: &Vec<Pos>) {
        for pos in pos_vec.iter() {
            if self.is_broken[pos.y][pos.x] {
                continue;
            }

            // 掘削
            const INITIAL: usize = 20;
            let mut power = INITIAL;
            loop {
                let responce = self.excavate(pos, power);
                match responce {
                    Response::Broken => {
                        // 壊れたら周りを推定
                        self.estimate_around(pos, power);
                        break;
                    }
                    Response::NotBroken => {}
                    Response::Finish => {}
                    Response::Invalid => {}
                }
                power *= 2;
                power = power.min(5000);
            }
        }
    }

    const ESTIMATER_BLOCK_UNIT: usize = 1;
    const ESTIMATER_BLOCK_CENTER: usize = Field::ESTIMATER_BLOCK_UNIT * 3;
    const ESTIMATER_BLOCK_WIDTH: usize =
        Field::ESTIMATER_BLOCK_CENTER * 2 - Field::ESTIMATER_BLOCK_UNIT;
    const ESTIMATER_BLOCK_AROUND_WIDTH: usize =
        Field::ESTIMATER_BLOCK_CENTER * 4 - Field::ESTIMATER_BLOCK_UNIT;
    const ESTIMATER_BLOCK_RADIUS: usize =
        Field::ESTIMATER_BLOCK_CENTER * 2 - Field::ESTIMATER_BLOCK_UNIT;
    const MAX_POWER: usize = 800;
    const INITIAL_POWER: usize = 50;
    fn estimate_representative_points_around(&mut self) {
        let mut power = Field::INITIAL_POWER;

        let mut cnt = 0;
        let mut total_power = 0;
        let mut points_not_broken = BTreeSet::new();
        while power <= Field::MAX_POWER {
            for q in 0..(Field::WIDTH / Field::ESTIMATER_BLOCK_WIDTH) {
                let y = Field::ESTIMATER_BLOCK_WIDTH * q + Field::ESTIMATER_BLOCK_CENTER;
                for p in 0..(Field::WIDTH / Field::ESTIMATER_BLOCK_WIDTH) {
                    let x = Field::ESTIMATER_BLOCK_WIDTH * p + Field::ESTIMATER_BLOCK_CENTER;
                    if self.is_broken[y][x] {
                        continue;
                    }
                    points_not_broken.insert((y, x));

                    let pos = &Pos::new(y, x);

                    // 掘削
                    let responce = self.excavate(pos, power);
                    match responce {
                        Response::Broken => {
                            // 壊れたら周りを推定
                            self.estimate_around(pos, power);
                            total_power += power;
                            points_not_broken.remove(&(y, x));
                        }
                        Response::NotBroken => {}
                        Response::Finish => {}
                        Response::Invalid => {}
                    }
                    cnt += 1;
                }
            }
            power *= 2;
        }

        points_not_broken.iter().for_each(|&(y, x)| {
            self.estimate_around(&Pos::new(y, x), 5000);
        });

        self.compute_weighted_average_of_est_tough_cands();

        eprintln!("cnt: {:?} total_power:{}", cnt, total_power);
    }

    fn compute_weighted_average_of_est_tough_cands(&mut self) {
        for y in 0..self.n {
            for x in 0..self.n {
                if self.est_tough_cands[y][x].is_empty() {
                    self.estimated_toughness[y][x] = None;
                } else {
                    let mut total_w = 0.0;
                    let mut total = 0.0;
                    let mut max_prob = 0.0;
                    self.est_tough_cands[y][x].iter().for_each(|(tough, prob)| {
                        total_w += *prob;
                        total += prob * (*tough as f32);
                        if *prob > max_prob {
                            max_prob = *prob;
                        }
                    });
                    let estimated_toughness = (total / total_w) as usize;
                    self.estimated_toughness[y][x] = Some((estimated_toughness, max_prob));
                }
            }
        }
    }

    fn output_estimated_toughness(&self) {
        for y in 0..self.n {
            for x in 0..self.n {
                if let Some(est_tough) = self.estimated_toughness[y][x] {
                    print!("{:>04} ", est_tough.0);
                } else {
                    print!("{} ", 5555);
                }
            }
            println!("");
        }
    }

    fn estimate_around(&mut self, pos: &Pos, power: usize) {
        let check_range = |val: usize, cnt: usize| {
            let val = val as i32 - Field::ESTIMATER_BLOCK_CENTER as i32 + cnt as i32;
            if val < 0 || Field::WIDTH as i32 <= val {
                Err(())
            } else {
                Ok(val as usize)
            }
        };

        for j in 0..Field::ESTIMATER_BLOCK_AROUND_WIDTH {
            let res = check_range(pos.y, j);
            if res.is_err() {
                continue;
            }
            let y = res.ok().unwrap();

            for i in 0..Field::ESTIMATER_BLOCK_AROUND_WIDTH {
                let res = check_range(pos.x, i);
                if res.is_err() {
                    continue;
                }
                let x = res.ok().unwrap();

                let dx = (x as i32 - i as i32).abs();
                let dy = (y as i32 - j as i32).abs();
                let dist = (dx + dy) as f32;
                let max_dist = (Field::ESTIMATER_BLOCK_RADIUS * 2) as f32;
                let prob = (max_dist) / (dist + max_dist).min(1.0);

                let est_tough = (power, prob);
                self.est_tough_cands[y][x].push(est_tough);
            }
        }
    }

    fn sort_houses_by_manhattan_dist(
        &self,
        source_vec: &Vec<Pos>,
        house_vec: &Vec<Pos>,
    ) -> Vec<Pos> {
        let mut sorted_house_vec = vec![];
        for house in house_vec {
            let manhattan_dist = self.compute_manhattan_dist(source_vec, house);
            sorted_house_vec.push((manhattan_dist, *house));
        }
        sorted_house_vec.sort();
        sorted_house_vec.iter().map(|(_, b)| *b).collect_vec()
    }

    fn compute_manhattan_dist(&self, source_vec: &Vec<Pos>, house: &Pos) -> usize {
        let inf = Field::WIDTH * 10;
        let mut min_manhattan_dist = inf;
        for source in source_vec {
            let dx = (source.x as i32 - house.x as i32).abs();
            let dy = (source.y as i32 - house.y as i32).abs();
            let dist = (dx + dy) as usize;
            min_manhattan_dist = min_manhattan_dist.min(dist);
        }
        min_manhattan_dist
    }

    fn connect_water_path(&self, source_vec: &Vec<Pos>, house_vec: &Vec<Pos>) {
        // 水路を含め、水があるところ
        let mut water_set = BTreeSet::new();
        source_vec.iter().for_each(|pos| {
            water_set.insert((pos.y, pos.x));
        });

        let sorted_house_vec = self.sort_houses_by_manhattan_dist(source_vec, house_vec);
        for house in sorted_house_vec {
            let path = self.search_path_to_water(&house, &water_set);
            for pos in path {
                water_set.insert((pos.y, pos.x));
            }
        }
    }

    fn search_path_to_water(&self, house: &Pos, water_set: &BTreeSet<(usize, usize)>) -> Vec<Pos> {
        let (dist, parent) = self.dijkstra(house);

        let inf = Field::WIDTH * 10;
        let mut min_dist = inf;
        let mut water_pos = None;
        for &(y, x) in water_set {
            if min_dist > dist[y][x] {
                min_dist = dist[y][x];
                water_pos = Some(Pos::new(y, x));
            }
        }

        if water_pos.is_none() {
            panic!("can't search path to water");
        }

        let water_pos = water_pos.unwrap();

        let mut path = vec![];
        let mut cur = Pos::new(water_pos.y, water_pos.x);
        while let Some(from) = parent[cur.y][cur.x] {
            path.push(from);
            cur = from;
        }
        path.reverse();

        path
    }

    fn dijkstra(&self, house: &Pos) -> (Vec<Vec<usize>>, Vec<Vec<Option<Pos>>>) {
        let is_out_range = |val: i32| -> bool {
            if val < 0 || Field::WIDTH as i32 <= val {
                true
            } else {
                false
            }
        };

        let dy = vec![1, -1, 0, 0];
        let dx = vec![0, 0, 1, -1];

        let inf = Field::WIDTH * 10;
        let mut dist = vec![vec![inf; self.n]; self.n];
        let mut parent: Vec<Vec<Option<Pos>>> = vec![vec![None; self.n]; self.n];

        let mut heap: BinaryHeap<(i32, (usize, usize))> = BinaryHeap::new();
        heap.push((0, (house.y, house.x)));

        while let Some((d, (y, x))) = heap.pop() {
            let d = (-d) as usize;
            if dist[y][x] < d {
                continue;
            }
            dist[y][x] = d;

            for i in 0..4 {
                let ny = y as i32 + dy[i];
                let nx = x as i32 + dx[i];
                if is_out_range(ny) || is_out_range(nx) {
                    continue;
                }
                let ny = ny as usize;
                let nx = nx as usize;

                let tough = if let Some((tough, _)) = self.estimated_toughness[ny][nx] {
                    tough
                } else {
                    5000
                };

                if dist[ny][nx] > d + tough {
                    dist[ny][nx] = d + tough;
                    heap.push((-1 * dist[ny][nx] as i32, (ny, nx)));
                    parent[ny][nx] = Some(Pos::new(ny, nx));
                }
            }
        }

        (dist, parent)
    }
}

static mut TOUGHNESS: Vec<Vec<usize>> = vec![];

#[derive(Debug, Clone)]
pub struct Sim {
    input: Input,
}

impl Sim {
    fn new() -> Self {
        let input = Input::read();
        Sim { input }
    }

    pub fn run(&mut self) {
        let mut field = Field::new(self.input.n, self.input.c);
        field.excavate_sources(&self.input.source_vec);
        field.excavate_houses(&self.input.house_vec);
        field.estimate_representative_points_around();
        field.output_estimated_toughness();
    }

    fn move_next(&self, start: Pos, next: Pos) {
        println!(
            "# move from ({}, {}) to ({}, {})",
            start.y, start.x, next.y, next.x
        );
    }

    fn get_toughness() {}
}

#[derive(Debug, Clone)]
pub struct Input {
    n: usize,
    w: usize,
    k: usize,
    c: usize,
    source_vec: Vec<(Pos)>,
    house_vec: Vec<(Pos)>,
    toughness: Vec<Vec<usize>>,
}

impl Input {
    fn read() -> Self {
        let (n, w, k, c) = read_uuuu();

        let mut toughness = vec![];
        if IS_LOCAL {
            for y in 0..n {
                let vec: Vec<usize> = read_vec();
                toughness.push(vec);
            }
            unsafe {
                TOUGHNESS = toughness.clone();
            }
        }

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
            toughness,
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

    #[derive(Debug, Clone, PartialEq, Eq, Copy, PartialOrd, Ord)]
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
