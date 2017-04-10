extern crate nalgebra as na;


use alga::general::Real;
use na::{Vector3, UnitQuaternion, Dim, Matrix};
use na::allocator::Allocator;

use na::storage::Storage;


pub fn to_euler_angles<T: Real>(q: &UnitQuaternion<T>) -> Vector3<T> {
    let x = q[0];
    let y = q[1];
    let z = q[2];
    let w = q[3];
    let ysqr = y * y;
    let _1: T = T::one();
    let _2: T = na::convert(2.0);

    // roll
    let t0 = _2 * (w * x + y * z);
    let t1 = _1 - _2 * (x * x + ysqr);
    let roll = t0.atan2(t1);

    // pitch
    let t2 = _2 * (w * y - z * x);
    let t2 = if t2 > _1 { _1 } else { t2 };
    let t2 = if t2 < -_1 { -_1 } else { t2 };
    let pitch = t2.asin();

    // yaw
    let t3 = _2 * (w * z + x * y);
    let t4 = _1 - _2 * (ysqr + z * z);
    let yaw = t3.atan2(t4);

    Vector3::new(roll, pitch, yaw)
}


pub fn try_pseudo_inverse<N, R, C, S>
    (matrix: &Matrix<N, R, C, S>)
     -> Option<Matrix<N, C, R, <S::Alloc as Allocator<N, C, R>>::Buffer>>
    where N: Real,
          R: Dim,
          C: Dim,
          S: Storage<N, R, C>,
          S::Alloc: Allocator<N, C, R> + Allocator<N, R, R>
{
    match (matrix * matrix.transpose()).try_inverse() {
        Some(mat) => Some(matrix.transpose() * mat),
        None => None,
    }
}

mod test {
    #[test]
    fn test_to_euler_angles() {
        use super::*;
        let q = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        let rpy = to_euler_angles(&q);
        assert!((rpy[0] - 0.1).abs() < 0.0001);
        assert!((rpy[1] - 0.2).abs() < 0.0001);
        assert!((rpy[2] - 0.3).abs() < 0.0001);
    }

    #[test]
    fn try_pseudo_inverse_identity() {
        use super::*;
        let m = na::Matrix3::<f64>::identity();
        assert_eq!(try_pseudo_inverse(&m).unwrap(), m);

        let m = na::Matrix6::<f32>::identity();
        assert_eq!(try_pseudo_inverse(&m).unwrap(), m);
    }

    #[test]
    fn try_pseudo_inverse() {
        use super::*;
        let m = na::Matrix3::new(1.0, 2.0, 3.0, 4.0, 2.0, 6.0, 7.0, 1.0, 1.0);
        let pseudo = try_pseudo_inverse(&m).unwrap();
        let pseudo_pseudo = try_pseudo_inverse(&pseudo).unwrap();
        let small = pseudo_pseudo - m;
        for val in small.iter() {
            assert!(val.abs() < 0.00001);
        }
        let small = m * pseudo * m - m;
        for val in small.iter() {
            assert!(val.abs() < 0.00001);
        }
    }

    #[test]
    fn try_pseudo_inverse_65fail() {
        use super::*;
        let m = na::Matrix6x5::new(1.0,
                                   2.0,
                                   3.0,
                                   4.0,
                                   5.0,
                                   6.0,
                                   1.0,
                                   2.0,
                                   3.0,
                                   4.0,
                                   5.0,
                                   6.0,
                                   1.0,
                                   2.0,
                                   3.0,
                                   4.0,
                                   5.0,
                                   6.0,
                                   1.0,
                                   2.0,
                                   3.0,
                                   4.0,
                                   5.0,
                                   6.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0);
        assert!(try_pseudo_inverse(&m).is_none());
    }
    #[test]
    fn try_pseudo_inverse_56() {
        use super::*;
        let m = na::Matrix5x6::new(1.0,
                                   1.0,
                                   1.0,
                                   1.0,
                                   1.0,
                                   2.0,
                                   2.0,
                                   2.0,
                                   2.0,
                                   2.0,
                                   3.0,
                                   3.0,
                                   3.0,
                                   3.0,
                                   3.0,
                                   4.0,
                                   4.0,
                                   4.0,
                                   4.0,
                                   4.0,
                                   5.0,
                                   5.0,
                                   5.0,
                                   5.0,
                                   5.0,
                                   6.0,
                                   6.0,
                                   6.0,
                                   6.0,
                                   6.0);
        let pseudo = try_pseudo_inverse(&m).unwrap();
        let small = m * pseudo * m - m;
        for val in small.iter() {
            assert!(val.abs() < 0.00001);
        }
    }

}
