#![allow(unused)]
use itertools::Itertools;
use my_lib::*;
use num::{traits::Pow, Integer};
use num_integer::Roots;
use procon_input::*;
use rand::prelude::*;
use rand_pcg::Mcg128Xsl64;
use std::{
    cell::RefCell,
    clone,
    cmp::{max, min},
    collections::{BTreeMap, BTreeSet, BinaryHeap, VecDeque},
    f32::consts::E,
    iter::FromIterator,
    mem::swap,
    ops::*,
    path,
    process::exit,
    rc::Rc,
    slice::SliceIndex,
};

// 方針
// A*で固い岩を避けながら柔らかい岩を探す

const IS_LOCAL: bool = false;

static mut START_TIME: f64 = 0.0;
static mut TOUGHNESS: Vec<Vec<usize>> = Vec::new();

fn main() {
    let start_time = my_lib::time::update();
    if IS_LOCAL {
        unsafe {
            START_TIME = start_time;
        }
    }

    Sim::new().run();
}

#[derive(Debug, Clone)]
enum Response {
    NotBroken,
    Broken,
    Finish,
    Invalid,
    Skip,
}

#[derive(Debug, Clone)]
struct Field {
    n: usize,
    c: usize,
    is_broken: Vec<Vec<bool>>,
    total_cost: usize,
    estimated_toughness: Vec<Vec<Option<(usize, f64)>>>,
    est_tough_cands: Vec<Vec<Vec<(usize, f64)>>>,
    damage: Vec<Vec<usize>>,
    connections: Dsu,
    house_vec: Vec<Pos>,
    source_vec: Vec<Pos>,
}

impl Field {
    const WIDTH: usize = 200;

    fn get_initial_power(cost: usize) -> usize {
        (cost).max(20)
    }

    fn new(n: usize, c: usize, house_vec: Vec<Pos>, source_vec: Vec<Pos>) -> Self {
        let is_broken = vec![vec![false; n]; n];
        let estimated_toughness = vec![vec![None; n]; n];
        let est_tough_cands = vec![vec![vec![]; n]; n];
        let damage = vec![vec![0; n]; n];
        let connections = Dsu::new(n * n);

        Field {
            n,
            c,
            is_broken,
            total_cost: 0,
            estimated_toughness,
            est_tough_cands,
            damage,
            connections,
            house_vec,
            source_vec,
        }
    }

    fn connect_water_path_with_a_star(&mut self, source_vec: &Vec<Pos>, house_vec: &Vec<Pos>) {
        // 水路を含め、水があるところ
        let mut water_set = BTreeSet::new();
        source_vec.iter().for_each(|pos| {
            water_set.insert((pos.y, pos.x));
        });

        let mut sorted_house_vec = Vec::new();

        for house in house_vec.iter() {
            let path_cost = self.compute_dist_for_a_star(&water_set, house);
            sorted_house_vec.push((path_cost, house));
        }
        sorted_house_vec.sort();
        let mut sorted_house_q = VecDeque::new();
        for &(_, house) in sorted_house_vec.iter() {
            sorted_house_q.push_back(*house);
        }

        let mut set = BTreeSet::new();

        while let Some(house) = sorted_house_q.pop_front() {
            if let Some(house) = self.a_star(&house, &water_set) {
                if !set.contains(&house) {
                    set.insert(house);
                    sorted_house_q.push_back(house);
                    continue;
                }
            }
            let path = self.search_path_to_water(&house, &water_set);

            for pos in path {
                water_set.insert((pos.y, pos.x));
                if self.is_broken[pos.y][pos.x] {
                    continue;
                }
                let initial_power = if let Some((tough, _)) = self.estimated_toughness[pos.y][pos.x]
                {
                    tough - self.damage[pos.y][pos.x] / 5
                } else {
                    Field::get_initial_power(self.c)
                };
                println!(
                    "# pos ({}, {}), est {:?}",
                    pos.y, pos.x, self.estimated_toughness[pos.y][pos.x]
                );
                self.excavate_completely(&pos, initial_power);
            }
        }
    }

    fn a_star(&mut self, house: &Pos, water_set: &BTreeSet<(usize, usize)>) -> Option<Pos> {
        let step = 11;
        let goal_allowed = 15;
        let mut min_dist = self.compute_dist_for_a_star(water_set, house);
        let cost = 5000;
        let power = 100;

        let dy = vec![step, -step, 0, 0];
        let dx = vec![0, 0, step, -step];

        let mut f_map = vec![vec![i64::max_value() / 2; self.n]; self.n];
        let mut g_map = vec![vec![i64::max_value() / 2; self.n]; self.n];

        let mut parent: Vec<Vec<Option<Pos>>> = vec![vec![None; self.n]; self.n];
        let mut heap: BinaryHeap<(i64, i64, (usize, usize))> = BinaryHeap::new();
        heap.push((0, 0, (house.y, house.x)));
        let mut is_closed = vec![vec![false; self.n]; self.n];

        let mut total_power = 0;
        is_closed[house.y][house.x] = true;
        loop {
            if self.is_broken[house.y][house.x] {
                total_power = self.damage[house.y][house.x];
                break;
            }
            total_power += power;
            let response = self.excavate(house, power);
            match response {
                Response::Broken => {
                    self.is_broken[house.y][house.x] = true;
                    self.estimated_toughness[house.y][house.x] = Some((0, 1.0));
                    self.estimate_around(&Pos::new(house.y, house.x), power, total_power, true);
                    break;
                }
                _ => {}
            }
        }
        let limit_power = (total_power).max(600) as i64;

        let mut cnt = 0;
        while let Some((f, g, (y, x))) = heap.pop() {
            //println!("# current {} {} ({}, {})", f, g, y, x);
            //println!("# {:?}", heap);

            let f = (-f);
            if f_map[y][x] < f {
                continue;
            }
            f_map[y][x] = f;
            g_map[y][x] = g;

            // cnt += 1;
            // if cnt >= 40 {
            //     return Some(*house);
            // }

            for i in 0..4 {
                let ny = y as i32 + dy[i];
                let nx = x as i32 + dx[i];
                if is_out_range(ny) || is_out_range(nx) {
                    continue;
                }
                let ny = ny as usize;
                let nx = nx as usize;

                let ch_pos = &Pos::new(ny, nx);
                let dist = self.compute_dist_for_a_star(water_set, ch_pos);

                if is_closed[ny][nx] {
                    continue;
                }
                is_closed[ny][nx] = true;

                let dist_limit = min_dist * 2;
                if dist >= dist_limit {
                    println!("# dist_limit {:?}", ch_pos);
                    continue;
                }

                let mut total_cost = 0;
                let mut total_power = 0;
                loop {
                    if self.is_broken[ny][nx] {
                        total_power = self.damage[ny][nx] as i64;
                        break;
                    }
                    total_cost += (power + self.c) as i64;
                    total_power += power as i64;

                    let c_g = g_map[y][x] + total_cost as i64;
                    let c_h = cost * self.compute_dist_for_a_star(water_set, ch_pos);
                    let c_f = c_g + c_h;
                    // println!(
                    //     "# {:?} {:?} {:?} {:?}, {} {}",
                    //     ch_pos, c_f, c_g, c_h, dist, min_dist
                    // );

                    let response = self.excavate(ch_pos, power);
                    match response {
                        Response::Broken => {
                            self.is_broken[ny][nx] = true;
                            self.estimated_toughness[ny][nx] = Some((0, 1.0));
                            self.estimate_around(ch_pos, power, total_power as usize, true);

                            min_dist = dist.min(min_dist);

                            if dist < goal_allowed {
                                return None;
                            }

                            break;
                        }
                        _ => {}
                    }
                    if total_power >= limit_power {
                        total_power = i64::max_value() / 10;
                        total_cost = i64::max_value() / 10;
                        break;
                    }
                }

                let c_g = g_map[y][x] + total_cost as i64;
                let c_h = cost * self.compute_dist_for_a_star(water_set, ch_pos);
                let c_f = c_g + c_h;

                if f_map[ny][nx] > c_f {
                    let child = (-1 * c_f, c_g, (ny, nx));
                    //println!("# current {} {} ({}, {})", -f, g, y, x);
                    //println!("# child {:?} {} ", child, c_h);
                    heap.push(child);
                    parent[ny][nx] = Some(Pos::new(y, x));
                }
            }
        }

        return Some(*house);
    }

    fn compute_dist_for_a_star(&self, water_set: &BTreeSet<(usize, usize)>, house: &Pos) -> i64 {
        let mut min_manhattan_dist = i32::max_value();
        for source in water_set.iter() {
            let dx = (source.1 as i32 - house.x as i32).abs();
            let dy = (source.0 as i32 - house.y as i32).abs();
            let dist = (dx * dx + dy * dy).sqrt();
            min_manhattan_dist = min_manhattan_dist.min(dist);
        }
        min_manhattan_dist as i64
    }

    fn excavate(&mut self, pos: &Pos, power: usize) -> Response {
        if self.is_broken[pos.y][pos.x] {
            return Response::Skip;
        }
        self.damage[pos.y][pos.x] += power;
        self.total_cost += power + self.c;
        println!("{} {} {}", pos.y, pos.x, power);

        if IS_LOCAL {
            unsafe {
                let remained_toughness = TOUGHNESS[pos.y][pos.x];
                let mut responce = Response::Invalid;
                if power >= remained_toughness {
                    TOUGHNESS[pos.y][pos.x] = 0;
                    self.is_broken[pos.y][pos.x] = true;
                    responce = Response::Broken;

                    let dy = vec![1, -1, 0, 0];
                    let dx = vec![0, 0, 1, -1];
                    for i in 0..4 {
                        let ny = pos.y as i32 + dy[i];
                        let nx = pos.x as i32 + dx[i];
                        if is_out_range(ny) || is_out_range(nx) {
                            continue;
                        }
                        let ny = ny as usize;
                        let nx = nx as usize;
                        if self.is_broken[ny][nx] {
                            let a = pos.y * Field::WIDTH + pos.x;
                            let b = ny * Field::WIDTH + nx;
                            self.connections.merge(a, b);
                        }
                    }

                    let mut is_all_house_connected = true;
                    for house in self.house_vec.iter() {
                        let mut is_connected = false;
                        for source in self.source_vec.iter() {
                            let a = house.y * Field::WIDTH + house.x;
                            let b = source.y * Field::WIDTH + source.x;
                            if self.connections.is_same(a, b) {
                                is_connected = true;
                                break;
                            }
                        }
                        if !is_connected {
                            is_all_house_connected = false;
                        }
                    }

                    if is_all_house_connected {
                        eprintln!("{:?} ", 0);
                        eprintln!("{:?} ", self.total_cost);
                        let end_time = my_lib::time::update();
                        let duration = end_time - START_TIME;
                        eprintln!("{:?} ", duration);
                        eprintln!("{:?} ", self.source_vec.len());
                        std::process::exit(0);
                    }
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

                    std::process::exit(0);
                    Response::Finish
                }
                _ => {
                    panic!("invalid: {:?}", pos);
                    Response::Invalid
                }
            }
        }
    }

    fn excavate_completely(&mut self, pos: &Pos, initial_power: usize) {
        // 掘削
        let mut power = initial_power;
        let mut total_power = 0;
        loop {
            if self.damage[pos.y][pos.x] + power >= 5000 {
                power = 5000 - self.damage[pos.y][pos.x];
            }
            total_power += power;
            let responce = self.excavate(pos, power);
            match responce {
                Response::Broken => {
                    // 壊れたら周りを推定
                    self.estimate_around(pos, power, total_power, true);

                    self.estimated_toughness[pos.y][pos.x] = Some((0, 1.0));
                    self.est_tough_cands[pos.y][pos.x].clear();
                    break;
                }
                _ => {}
            }

            power = Field::get_initial_power(self.c);
        }
    }

    fn compute_weighted_average(&mut self, y: usize, x: usize) {
        if let Some((tough, _)) = self.estimated_toughness[y][x] {
            if tough == 0 {
                return;
            }
        }

        if self.est_tough_cands[y][x].is_empty() {
            self.estimated_toughness[y][x] = None;
        } else {
            let mut total_w = 0.0;
            let mut total = 0.0;
            let mut max_prob = 0.0;
            let mut total_prob = 0.0;
            self.est_tough_cands[y][x].iter().for_each(|(tough, prob)| {
                total_w += *prob;
                total += prob * (*tough as f64);
                if *prob > max_prob {
                    max_prob = *prob;
                }
            });
            let estimated_toughness = (total / total_w) as usize;
            self.estimated_toughness[y][x] = Some((estimated_toughness, max_prob));
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

    fn estimate_around(&mut self, pos: &Pos, power: usize, total_power: usize, is_broken: bool) {
        let (tough, est_width) = if is_broken {
            // 強めに壊してる分を補正
            (self.damage[pos.y][pos.x] - power / 2, 11)
        } else {
            (5000, 5)
        };
        let tough = tough.min(5000).max(20);
        let est_center = est_width / 2;
        let est_radius = est_width / 2;
        let check_range = |val: usize, cnt: usize| {
            let val = val as i32 - est_center as i32 + cnt as i32;
            if val < 0 || Field::WIDTH as i32 <= val {
                Err(())
            } else {
                Ok(val as usize)
            }
        };

        for j in 0..est_width {
            let res = check_range(pos.y, j);
            if res.is_err() {
                continue;
            }
            let y = res.ok().unwrap();

            for i in 0..est_width {
                let res = check_range(pos.x, i);
                if res.is_err() {
                    continue;
                }
                let x = res.ok().unwrap();
                if (y, x) == (pos.y, pos.x) {
                    continue;
                }

                let sigmoid = |a: f64| 1.0 / (1.0 + (-100.0 * (a - 0.98)).exp());

                let dx = (i as i32 - est_center as i32).abs();
                let dy = (j as i32 - est_center as i32).abs();
                let dist = (dx + dy) as f64;
                let max_dist = (est_radius * est_radius) as f64;
                let prob = (max_dist * max_dist) / (dist * dist + max_dist * max_dist).max(1.0);
                //let est_tough = (tough, sigmoid(prob as f64) as f32);
                let prob = (prob as f64).powf(60.0) as f64;
                let est_tough = (tough, prob);

                self.est_tough_cands[y][x].push(est_tough);
                self.compute_weighted_average(y, x);

                // println!(
                //     "# pos ({}, {}), est {:?}",
                //     y, x, self.estimated_toughness[y][x]
                // );
            }
        }
    }

    fn search_path_to_water(&self, house: &Pos, water_set: &BTreeSet<(usize, usize)>) -> Vec<Pos> {
        let (dist, parent) = self.dijkstra(house);
        let mut min_dist = usize::max_value();
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
        path.push(cur);
        while let Some(from) = parent[cur.y][cur.x] {
            path.push(from);
            cur = from;
        }
        path.reverse();

        path
    }

    // https://qiita.com/okaponta_/items/018d0cd4f38ead3675c0
    fn dijkstra(&self, house: &Pos) -> (Vec<Vec<usize>>, Vec<Vec<Option<Pos>>>) {
        let dy = vec![1, -1, 0, 0];
        let dx = vec![0, 0, 1, -1];

        let mut dist = vec![vec![usize::max_value(); self.n]; self.n];
        let mut parent: Vec<Vec<Option<Pos>>> = vec![vec![None; self.n]; self.n];

        let mut heap: BinaryHeap<(i32, (usize, usize))> = BinaryHeap::new();
        heap.push((-(self.c as i32), (house.y, house.x)));

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

                if dist[ny][nx] > d + tough + self.c {
                    dist[ny][nx] = d + tough + self.c;
                    heap.push((-1 * dist[ny][nx] as i32, (ny, nx)));
                    parent[ny][nx] = Some(Pos::new(y, x));
                }
            }
        }

        (dist, parent)
    }

    fn research_for_debug(&mut self) {
        for y in 0..self.n {
            for x in 0..self.n {
                let power = 400;
                let pos = &Pos::new(y, x);
                let responce = self.excavate(pos, power);
                // match responce {
                //     Response::Broken => {
                //         // 壊れたら周りを推定
                //         self.estimate_around(pos, power, power, true);

                //         self.estimated_toughness[y][x] = Some((0, 1.0));
                //         self.est_tough_cands[y][x].clear();
                //     }
                //     _ => {}
                // }
            }
        }
    }
}

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
        let mut field = Field::new(
            self.input.n,
            self.input.c,
            self.input.house_vec.clone(),
            self.input.source_vec.clone(),
        );

        field.connect_water_path_with_a_star(&self.input.source_vec, &self.input.house_vec);

        //field.research_for_debug();
        //return;
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

fn is_out_range(val: i32) -> bool {
    if val < 0 || Field::WIDTH as i32 <= val {
        true
    } else {
        false
    }
}

#[derive(Debug, Clone)]
struct Dsu {
    parent_or_size: Vec<i64>, // 親のindex or 親のときはグループのサイズを-1した値(for 経路圧縮)
    num_node: usize,
    num_group: usize,

    // extentions
    min_index: Vec<usize>,
}

impl Dsu {
    pub fn new(n: usize) -> Self {
        let mut min_index = Vec::<usize>::new();
        for index in 0..n as usize {
            min_index.push(index);
        }

        Dsu {
            parent_or_size: vec![-1; n],
            num_node: n,
            num_group: n,
            min_index,
        }
    }

    pub fn leader(&mut self, index: usize) -> usize {
        //! 代表元のindex取得
        assert!(index < self.num_node);

        let parent_index = self.parent_or_size[index];
        if self.parent_or_size[index] < 0 {
            index
        } else {
            let parent_index = self.leader(parent_index as usize);
            self.parent_or_size[index] = parent_index as i64;
            parent_index
        }
    }

    pub fn leader_vec(&self) -> Vec<usize> {
        let mut leaders = Vec::new();
        for (index, size_minus) in self.parent_or_size.iter().enumerate() {
            if *size_minus < 0 {
                leaders.push(index as usize);
            }
        }
        leaders
    }

    pub fn merge(&mut self, a: usize, b: usize) -> usize {
        assert!(a < self.num_node);
        assert!(b < self.num_node);

        let mut leader_a = self.leader(a);
        let mut leader_b = self.leader(b);

        // 既に同じグループ
        if leader_a == leader_b {
            return leader_a;
        }

        // グループのサイズが大きいほうにマージする
        // 代表元のparent_or_sizeにはグループのサイズに-1した値が格納されている
        let group_size_a = -self.parent_or_size[leader_a];
        let group_size_b = -self.parent_or_size[leader_b];
        // aを基準にする
        if group_size_a < group_size_b {
            std::mem::swap(&mut leader_a, &mut leader_b);
        }
        // サイズ加算
        self.parent_or_size[leader_a] += self.parent_or_size[leader_b];
        self.parent_or_size[leader_b] = leader_a as i64;

        // グループ統合により、グループ数が減る
        self.num_group -= 1;

        // グループの最小index更新
        if self.min_index[leader_a] > self.min_index[leader_b] {
            self.min_index[leader_a] = self.min_index[leader_b];
        }

        leader_a
    }

    pub fn is_same(&mut self, a: usize, b: usize) -> bool {
        assert!(a < self.num_node);
        assert!(b < self.num_node);

        self.leader(a) == self.leader(b)
    }

    pub fn group_size(&mut self, leader: usize) -> usize {
        assert!(leader < self.num_node);

        (-self.parent_or_size[leader]) as usize
    }

    pub fn group_num(&mut self) -> usize {
        self.num_group
    }

    pub fn min_index(&mut self, leader: usize) -> usize {
        assert!(leader < self.num_node);

        self.min_index[leader]
    }
}
