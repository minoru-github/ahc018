#![allow(unused)]
use itertools::Itertools;
use my_lib::*;
use num::traits::Pow;
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

const IS_LOCAL_ESTIMATING_FIELD_MODE: bool = false;
const IS_LOCAL: bool = true | IS_LOCAL_ESTIMATING_FIELD_MODE;

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

    // let end_time = my_lib::time::update();
    // let duration = end_time - start_time;
    // eprintln!("{:?} ", duration);
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
    connections: Dsu,
    house_vec: Vec<Pos>,
    source_vec: Vec<Pos>,
}

impl Field {
    const WIDTH: usize = 200;
    fn new(n: usize, c: usize, house_vec: Vec<Pos>, source_vec: Vec<Pos>) -> Self {
        let is_broken = vec![vec![false; n]; n];
        let estimated_toughness = vec![vec![None; n]; n];
        let est_tough_cands = vec![vec![vec![]; n]; n];
        let connections = Dsu::new(n * n);

        Field {
            n,
            c,
            is_broken,
            total_cost: 0,
            estimated_toughness,
            est_tough_cands,
            connections,
            house_vec,
            source_vec,
        }
    }

    fn estimate_between_important_points(&mut self) {
        // 水路を含め、水があるところ
        let mut water_set = BTreeSet::new();
        self.source_vec.iter().for_each(|pos| {
            water_set.insert((pos.y, pos.x));
        });

        let sorted_house_vec =
            self.sort_houses_by_manhattan_dist(&self.source_vec, &self.house_vec);
        for house in sorted_house_vec {
            let path = self.search_path_to_water(&house, &water_set);
            for (cnt, pos) in path.iter().enumerate() {
                if !IS_LOCAL_ESTIMATING_FIELD_MODE {
                    water_set.insert((pos.y, pos.x));
                    if self.is_broken[pos.y][pos.x] {
                        continue;
                    }

                    if cnt % 50 != 0 {
                        continue;
                    }

                    self.excavate_with_estimate(pos, Field::INITIAL_POWER);

                    self.excavate_around(pos, 0, 8, Field::MAX_POWER);
                }
            }
        }
    }

    fn excavate_with_estimate(&mut self, pos: &Pos, power: usize) -> Response {
        let mut total_power = power;
        let mut is_broken = false;
        let mut responce = Response::Invalid;
        while total_power <= Field::MAX_POWER {
            responce = self.excavate(pos, power);
            match responce {
                Response::Broken => {
                    // 壊れたら周りを推定
                    is_broken = true;
                    self.estimate_around(pos, power, total_power, is_broken);

                    self.estimated_toughness[pos.y][pos.x] = Some((0, 1.0));
                    self.est_tough_cands[pos.y][pos.x].clear();
                    break;
                }
                _ => {}
            };
            total_power += power;
        }
        if !is_broken {
            self.estimate_around(pos, power, total_power, is_broken);
        }
        responce
    }

    fn excavate(&mut self, pos: &Pos, power: usize) -> Response {
        if self.is_broken[pos.y][pos.x] {
            // panic!(
            //     "this pos has already been broken. (y,x) : ({}, {})",
            //     pos.y, pos.x
            // );
            return Response::Skip;
        }
        self.total_cost += power + self.c;
        if !IS_LOCAL_ESTIMATING_FIELD_MODE {
            println!("{} {} {}", pos.y, pos.x, power);
        }

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

    fn excavate_around(&mut self, pos: &Pos, begine: usize, end: usize, max_power: usize) {
        let dy = vec![
            10, -10, 0, 0, 15, 15, -15, -15, 25, -25, 0, 0, 30, 30, -30, -30,
        ];
        let dx = vec![
            0, 0, 10, -10, 15, -15, 15, -15, 0, 0, 25, -25, 30, -30, 30, -30,
        ];

        for i in begine..end {
            let ny = pos.y as i32 + dy[i];
            let nx = pos.x as i32 + dx[i];
            if is_out_range(ny) || is_out_range(nx) {
                continue;
            }
            let ny = ny as usize;
            let nx = nx as usize;
            if self.is_broken[ny][nx] {
                continue;
            }

            let range = 8;
            let mut broken_cnt = 0;
            for q in -range..range {
                for p in -range..range {
                    let px = nx as i32 + p;
                    let qy = ny as i32 + q;
                    if is_out_range(qy) || is_out_range(px) {
                        continue;
                    }

                    if self.is_broken[qy as usize][px as usize] {
                        broken_cnt += 1;
                    }
                }
            }

            if broken_cnt >= 2 {
                continue;
            }

            let pos = &Pos::new(ny, nx);
            let mut power = if let Some((tough, _)) = self.estimated_toughness[pos.y][pos.x] {
                tough
            } else {
                Field::INITIAL_POWER
            };
            self.excavate_with_estimate(pos, power);
        }
    }

    fn excavate_sources(&mut self, source_vec: &Vec<Pos>) {
        for pos in source_vec {
            if self.is_broken[pos.y][pos.x] {
                continue;
            }
            self.excavate_completely(pos, Field::INITIAL_POWER / 2);

            self.excavate_around(pos, 0, 16, Field::MAX_POWER);
        }
    }

    fn excavate_houses(&mut self, house_vec: &Vec<Pos>) {
        for pos in house_vec {
            if self.is_broken[pos.y][pos.x] {
                continue;
            }
            self.excavate_completely(pos, Field::INITIAL_POWER / 2);

            self.excavate_around(pos, 0, 16, Field::MAX_POWER);
        }
    }

    fn excavate_completely(&mut self, pos: &Pos, initial_power: usize) {
        // 掘削
        let mut power = initial_power;
        let mut total_power = 0;
        loop {
            if total_power + power >= 5000 {
                power = 5000 - total_power;
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

            power = Field::INITIAL_POWER;
        }
    }

    const EST_BLOCK_UNIT: usize = 5;
    const EST_BLOCK_CENTER: usize = Field::EST_BLOCK_UNIT * 3;
    const EST_BLOCK_WIDTH: usize = Field::EST_BLOCK_CENTER * 2 - Field::EST_BLOCK_UNIT;
    const EST_BLOCK_AROUND_WIDTH: usize = Field::EST_BLOCK_CENTER * 4 - Field::EST_BLOCK_UNIT;
    const EST_BLOCK_RADIUS: usize = Field::EST_BLOCK_CENTER * 2 - Field::EST_BLOCK_UNIT;
    const INITIAL_POWER: usize = 100;
    const MAX_POWER: usize = 300;
    fn estimate_representative_points_around(&mut self) {
        let mut power = Field::INITIAL_POWER;

        let mut total_power = 0;
        let mut points_not_broken = BTreeSet::new();
        while total_power <= Field::MAX_POWER {
            for q in 0..(Field::WIDTH / Field::EST_BLOCK_WIDTH) {
                let y = Field::EST_BLOCK_WIDTH * q + Field::EST_BLOCK_CENTER;
                for p in 0..(Field::WIDTH / Field::EST_BLOCK_WIDTH) {
                    let x = Field::EST_BLOCK_WIDTH * p + Field::EST_BLOCK_CENTER;
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
                            self.estimate_around(pos, power, total_power, true);
                            points_not_broken.remove(&(y, x));

                            self.estimated_toughness[y][x] = Some((0, 1.0));
                            self.est_tough_cands[y][x].clear();
                            break;
                        }
                        _ => {}
                    }
                }
            }
            total_power += power;
        }

        points_not_broken.iter().for_each(|&(y, x)| {
            self.estimate_around(&Pos::new(y, x), power, 5000, false);
        });
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
            (total_power - power / 2, 11)
        } else {
            (5000, 5)
        };
        let tough = tough.min(5000).max(30);
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
                let prob = (prob as f64).powf(45.0) as f64;
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
        let mut min_manhattan_dist = usize::max_value();
        for source in source_vec {
            let dx = (source.x as i32 - house.x as i32).abs();
            let dy = (source.y as i32 - house.y as i32).abs();
            let dist = (dx + dy) as usize;
            min_manhattan_dist = min_manhattan_dist.min(dist);
        }
        min_manhattan_dist
    }

    fn connect_water_path(&mut self, source_vec: &Vec<Pos>, house_vec: &Vec<Pos>) {
        // 水路を含め、水があるところ
        let mut water_set = BTreeSet::new();
        source_vec.iter().for_each(|pos| {
            water_set.insert((pos.y, pos.x));
        });

        //let sorted_house_vec = self.sort_houses_by_manhattan_dist(source_vec, house_vec);
        let mut sorted_house_vec = vec![];

        for house in house_vec.iter() {
            let path = self.search_path_to_water(house, &water_set);
            let path_cost = self.compute_estimated_path_cost(&path);
            //eprintln!("path_cost {:?} ", path_cost);
            sorted_house_vec.push((path_cost, house));
        }
        sorted_house_vec.sort();

        for &(_, house) in sorted_house_vec.iter() {
            let path = self.search_path_to_water(house, &water_set);

            let mut path_corner = vec![];
            for i in 0..(path.len() - 2) {
                let current = path[i];
                let after2 = path[i + 2];
                let dy = (current.y as i32 - after2.y as i32).abs();
                let dx = (current.x as i32 - after2.x as i32).abs();
                if dx != 0 && dy != 0 {
                    path_corner.push(path[i + 1]);
                }
            }

            for pos in path_corner.iter() {
                if self.is_broken[pos.y][pos.x] {
                    continue;
                }
                let initial_power = if let Some((tough, _)) = self.estimated_toughness[pos.y][pos.x]
                {
                    tough
                } else {
                    Field::INITIAL_POWER / 2
                };
                println!(
                    "# corner pos ({}, {}), est {:?}",
                    pos.y, pos.x, self.estimated_toughness[pos.y][pos.x]
                );
                self.excavate_completely(pos, initial_power);
            }
            let path = self.search_path_to_water(house, &water_set);

            for pos in path {
                water_set.insert((pos.y, pos.x));
                if !IS_LOCAL_ESTIMATING_FIELD_MODE {
                    if self.is_broken[pos.y][pos.x] {
                        continue;
                    }
                    let initial_power =
                        if let Some((tough, _)) = self.estimated_toughness[pos.y][pos.x] {
                            tough
                        } else {
                            Field::INITIAL_POWER
                        };
                    println!(
                        "# pos ({}, {}), est {:?}",
                        pos.y, pos.x, self.estimated_toughness[pos.y][pos.x]
                    );
                    self.excavate_completely(&pos, initial_power);
                }
            }
        }
    }

    fn compute_estimated_path_cost(&self, path: &Vec<Pos>) -> usize {
        let mut path_cost = 0;
        for pos in path {
            let tough = if self.is_broken[pos.y][pos.x] {
                0
            } else {
                if let Some((tough, prob)) = self.estimated_toughness[pos.y][pos.x] {
                    //eprintln!("{:?} {}", tough, prob);
                    tough
                } else {
                    5000
                }
            };
            path_cost += tough + self.c;
        }
        //eprintln!("");
        path_cost
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

        //field.research_for_debug();
        //return;
        // 地形の推定フェーズ
        field.excavate_sources(&self.input.source_vec);
        field.excavate_houses(&self.input.house_vec);
        //field.estimate_representative_points_around();
        field.estimate_between_important_points();
        if IS_LOCAL_ESTIMATING_FIELD_MODE {
            field.output_estimated_toughness();
        }

        // 水路を作るフェーズ
        field.connect_water_path(&self.input.source_vec, &self.input.house_vec);
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
