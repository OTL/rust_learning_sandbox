extern crate rand;
#[macro_use]
extern crate log;
extern crate alga;
extern crate nalgebra as na;
extern crate num_traits;

use alga::general::Real;
use alga::linear::EuclideanSpace;
use num_traits::bounds::Bounded;
use std::fmt::Debug;
use std::mem;


pub enum ExtendStatus {
    Reached(usize),
    Advanced(usize),
    Trapped,
}

#[derive(Debug, Clone)]
pub struct Node<V: EuclideanSpace + Debug> {
    parent_id: Option<usize>,
    id: usize,
    data: V,
}

impl<V> Node<V>
    where V: EuclideanSpace + Debug
{
    pub fn new(data: V, id: usize) -> Self {
        Node {
            parent_id: None,
            id: id,
            data: data,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Tree<V: EuclideanSpace + Debug> {
    pub vertices: Vec<Node<V>>,
}

impl<V> Tree<V>
    where V: EuclideanSpace + Debug
{
    pub fn new() -> Self {
        Tree { vertices: Vec::new() }
    }
    pub fn add_vertex(&mut self, q: V) -> usize {
        let id = self.vertices.len();
        self.vertices.push(Node::new(q, id));
        id
    }
    pub fn add_edge(&mut self, q1_id: usize, q2_id: usize) {
        self.vertices[q2_id].parent_id = Some(q1_id);
    }
    pub fn get_nearest(&self, q: &V) -> usize {
        // TODO: faster search
        let mut min_distance = <V as EuclideanSpace>::Real::max_value();
        let mut min_index = 0;
        for node in &self.vertices {
            let dist = na::distance(&node.data, q);
            if dist < min_distance {
                min_distance = dist;
                min_index = node.id;
            }
        }
        min_index
    }
    pub fn extend<P>(&mut self,
                     q_target: &V,
                     extend_length: <V as EuclideanSpace>::Real,
                     problem: &P)
                     -> ExtendStatus
        where P: Problem<V>
    {
        let nearest_id = self.get_nearest(&q_target);
        let nearest_q = self.vertices[nearest_id].data.clone();
        let diff_target = q_target.clone() - nearest_q.clone();
        let diff_dist = na::distance(q_target, &nearest_q);
        let q_new = if diff_dist < extend_length {
            q_target.clone()
        } else {
            nearest_q + (diff_target / diff_dist) * extend_length
        };

        if problem.is_feasible(&q_new) {
            let new_id = self.add_vertex(q_target.clone());
            self.add_edge(nearest_id, new_id);
            if na::distance(&q_new, &q_target) < extend_length {
                return ExtendStatus::Reached(new_id);
            }
            info!("distance = {}, len = {}",
                  na::distance(&q_new, &q_target),
                  extend_length);

            info!("target = {:?}", q_target);
            info!("advaneced to {:?}", q_target);
            return ExtendStatus::Advanced(new_id);
        }
        ExtendStatus::Trapped
    }
    pub fn connect<P>(&mut self,
                      q_target: &V,
                      extend_length: <V as EuclideanSpace>::Real,
                      problem: &P)
                      -> ExtendStatus
        where P: Problem<V>
    {
        loop {
            match self.extend(q_target, extend_length, problem) {
                ExtendStatus::Trapped => return ExtendStatus::Trapped,
                ExtendStatus::Reached(id) => return ExtendStatus::Reached(id),
                ExtendStatus::Advanced(id) => {
                    println!("advanced to {}", id);
                }
            };
        }
    }
    pub fn get_until_root(&self, id: usize) -> Vec<V> {
        let mut nodes = Vec::new();
        let mut cur_id = id;
        while let Some(parent_id) = self.vertices[cur_id].parent_id {
            cur_id = parent_id;
            nodes.push(self.vertices[cur_id].data.clone())
        }
        nodes
    }
}

pub trait Problem<V> {
    fn is_feasible(&self, point: &V) -> bool;
    fn random_sample(&self) -> V;
}

pub struct BoxProblem {}

impl<V> Problem<V> for BoxProblem
    where V: EuclideanSpace + rand::Rand + Debug
{
    fn is_feasible(&self, point: &V) -> bool {
        let v = point.coordinates();
        println!("v= {:?}", point);
        (!((v[0] > na::convert(0.0) && v[0] < na::convert(1.0)) &&
           (v[1] > na::convert(0.0) && v[1] < na::convert(1.0)) &&
           (v[2] > na::convert(0.0) && v[2] < na::convert(1.0)))) &&
        (v[0].abs() < na::convert(2.0)) && (v[1].abs() < na::convert(2.0)) &&
        (v[2].abs() < na::convert(2.0))
    }
    fn random_sample(&self) -> V {
        let mut q_rand: V = rand::random();
        q_rand = q_rand.scale_by(na::convert(2.0));
        while !self.is_feasible(&q_rand) {
            q_rand = rand::random();
            q_rand = q_rand.scale_by(na::convert(2.0));
        }
        q_rand
    }
}


pub fn dual_rrt_connect<V, P>(start: V,
                              goal: V,
                              extend_length: <V as EuclideanSpace>::Real,
                              problem: &P)
                              -> Result<Vec<V>, String>
    where V: EuclideanSpace + Debug,
          P: Problem<V>
{
    let mut tree_a = Tree::new();
    let mut tree_b = Tree::new();
    tree_a.add_vertex(start);
    tree_b.add_vertex(goal);
    let max_try = 1000;
    println!("start");
    for _ in 0..max_try {
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
                    println!("{:?}", a_all.len());
                    println!("{:?}", b_all.len());
                    a_all.reverse();
                    a_all.append(&mut b_all);
                    //a_all.reverse();
                    return Ok(a_all);
                }
            }
            None => {}
        }
        mem::swap(&mut tree_a, &mut tree_b);
    }
    return Err("failed".to_string());
}

#[test]
fn it_works() {
    extern crate env_logger;
    let p = BoxProblem {};
    let start = na::Point3::new(-0.2, 0.5, 0.1);
    let goal = na::Point3::new(1.2, 0.5, 0.1);
    println!("{:?}", dual_rrt_connect(start, goal, 0.01, &p).unwrap());
}
