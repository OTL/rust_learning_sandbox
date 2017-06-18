extern crate rand;
extern crate kdtree;
#[macro_use]
extern crate log;
//extern crate alga;
//extern crate nalgebra as na;
//extern crate num_traits;

//use alga::general::Real;
//use alga::linear::EuclideanSpace;
//use num_traits::bounds::Bounded;
use kdtree::distance::squared_euclidean;
//use std::fmt::Debug;
use std::mem;

fn multiple(va: &[f64], b: f64) -> Vec<f64> {
    let mut r = Vec::new();
    for i in 0..va.len() {
        r.push(va[i] * b);
    }
    r
}

fn add(va: &[f64], b: &[f64]) -> Vec<f64> {
    debug_assert!(va.len() == b.len());
    let mut r = Vec::new();
    for i in 0..va.len() {
        r.push(va[i] + b[i]);
    }
    r
}

fn subtract(va: &[f64], b: &[f64]) -> Vec<f64> {
    debug_assert!(va.len() == b.len());
    let mut r = Vec::new();
    for i in 0..va.len() {
        r.push(va[i] - b[i]);
    }
    r
}

pub enum ExtendStatus {
    Reached(usize),
    Advanced(usize),
    Trapped,
}

#[derive(Debug, Clone)]
pub struct Node {
    parent_id: Option<usize>,
    id: usize,
    data: Vec<f64>,
}

impl Node {
    pub fn new(data: Vec<f64>, id: usize) -> Self {
        Node {
            parent_id: None,
            id: id,
            data: data,
        }
    }
}

#[derive(Debug)]
pub struct Tree {
    pub dim: usize,
    pub kdtree: kdtree::KdTree<usize, Vec<f64>>,
    pub vertices: Vec<Node>,
}

impl Tree {
    pub fn new(dim: usize) -> Self {
        Tree {
            dim: dim,
            kdtree: kdtree::KdTree::new(dim),
            vertices: Vec::new(),
        }
    }
    pub fn add_vertex(&mut self, q: Vec<f64>) -> usize {
        let id = self.vertices.len();
        self.kdtree.add(q.clone(), id).unwrap();
        self.vertices.push(Node::new(q, id));
        id
    }
    pub fn add_edge(&mut self, q1_id: usize, q2_id: usize) {
        self.vertices[q2_id].parent_id = Some(q1_id);
    }
    pub fn get_nearest(&self, q: &[f64]) -> usize {
        *self.kdtree.nearest(q, 1, &squared_euclidean).unwrap()[0].1
    }
    pub fn extend<P>(&mut self, q_target: &[f64], extend_length: f64, problem: &P) -> ExtendStatus
        where P: Problem + RandomSample
    {
        let nearest_id = self.get_nearest(q_target);
        let nearest_q = self.vertices[nearest_id].data.clone();
        let diff_target = subtract(&q_target, &nearest_q);
        let diff_dist = squared_euclidean(&q_target, &nearest_q).sqrt();
        let q_new = if diff_dist < extend_length {
            q_target.to_vec()
        } else {
            //nearest_q + (diff_target / diff_dist) * extend_length
            let extend_vec = multiple(&diff_target, extend_length / diff_dist);
            add(&nearest_q, &extend_vec)
        };
        println!("q_new={:?}", q_new);
        if problem.is_feasible(&q_new) {
            let new_id = self.add_vertex(q_new.to_vec());
            self.add_edge(nearest_id, new_id);
            if squared_euclidean(&q_new, &q_target).sqrt() < extend_length {
                return ExtendStatus::Reached(new_id);
            }
            println!("target = {:?}", q_target);
            println!("advaneced to {:?}", q_target);
            return ExtendStatus::Advanced(new_id);
        }
        println!("traaaaaaaaaapped");
        ExtendStatus::Trapped
    }
    pub fn connect<P>(&mut self, q_target: &[f64], extend_length: f64, problem: &P) -> ExtendStatus
        where P: Problem + RandomSample
    {
        println!("start connecting...{:?}", q_target);
        loop {
            println!("connecting...{:?}", q_target);
            match self.extend(q_target, extend_length, problem) {
                ExtendStatus::Trapped => return ExtendStatus::Trapped,
                ExtendStatus::Reached(id) => return ExtendStatus::Reached(id),
                ExtendStatus::Advanced(id) => {
                    println!("advanced to {}", id);
                }
            };
        }
    }
    pub fn get_until_root(&self, id: usize) -> Vec<Vec<f64>> {
        let mut nodes = Vec::new();
        let mut cur_id = id;
        while let Some(parent_id) = self.vertices[cur_id].parent_id {
            cur_id = parent_id;
            nodes.push(self.vertices[cur_id].data.clone())
        }
        nodes
    }
}

pub trait Problem {
    fn is_feasible(&self, point: &[f64]) -> bool;
}

pub trait RandomSample {
    fn random_sample(&self) -> Vec<f64>;
}

pub fn dual_rrt_connect<P>(start: &[f64],
                           goal: &[f64],
                           extend_length: f64,
                           problem: &P,
                           num_max_try: usize)
                           -> Result<Vec<Vec<f64>>, String>
    where P: Problem + RandomSample
{
    assert!(start.len() == goal.len());
    let mut tree_a = Tree::new(start.len());
    let mut tree_b = Tree::new(start.len());
    tree_a.add_vertex(start.to_vec());
    tree_b.add_vertex(goal.to_vec());
    println!("start");
    for _ in 0..num_max_try {
        // TODO: set configuration range
        println!("tree_a = {:?}", tree_a.vertices.len());
        println!("tree_b = {:?}", tree_b.vertices.len());
        let q_rand = problem.random_sample();
        let extend_status = tree_a.extend(&q_rand, extend_length, problem);
        let new_id_opt = match extend_status {
            ExtendStatus::Trapped => None,
            ExtendStatus::Advanced(new_id) => {
                println!("advanced");
                Some(new_id)
            }
            ExtendStatus::Reached(new_id) => {
                println!("reached");
                Some(new_id)
            }
        };
        match new_id_opt {
            Some(new_id) => {
                let q_new = tree_a.vertices[new_id].data.clone();
                if let ExtendStatus::Reached(reach_id) =
                    tree_b.connect(&q_new, extend_length, problem) {
                    let mut a_all = tree_a.get_until_root(new_id);
                    let mut b_all = tree_b.get_until_root(reach_id);
                    println!("reeeeeeeeeeech");
                    println!("{:?}", a_all.len());
                    println!("{:?}", b_all.len());
                    a_all.reverse();
                    a_all.append(&mut b_all);
                    //a_all.reverse();
                    return Ok(a_all);
                }
            }
            None => println!("trapped"),
        }
        mem::swap(&mut tree_a, &mut tree_b);
    }
    return Err("failed".to_string());
}

#[test]
fn it_works() {
    extern crate env_logger;
    use rand::distributions::{IndependentSample, Range};
    pub struct BoxProblem {}

    impl Problem for BoxProblem {
        fn is_feasible(&self, point: &[f64]) -> bool {
            !(point[0].abs() < 1.0 && point[1].abs() < 1.0)
        }
    }
    impl RandomSample for BoxProblem {
        fn random_sample(&self) -> Vec<f64> {
            let between = Range::new(-2.0, 2.0);
            let mut rng = rand::thread_rng();
            vec![between.ind_sample(&mut rng), between.ind_sample(&mut rng)]
        }
    }

    let p = BoxProblem {};
    let start = [-1.2, 0.0];
    let goal = [1.2, 0.0];
    assert!(p.is_feasible(&start));
    assert!(p.is_feasible(&goal));
    println!("{:?}",
             dual_rrt_connect(&start, &goal, 0.5, &p, 1000).unwrap());
}
