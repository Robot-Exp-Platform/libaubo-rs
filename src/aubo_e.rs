use crate::{AuboRobot, AuboType};

pub trait AuboE {}

pub struct _AuboE3;
pub struct _AuboE5;
pub struct _AuboE10;

impl AuboE for _AuboE3 {}
impl AuboE for _AuboE5 {}
impl AuboE for _AuboE10 {}

impl AuboType for _AuboE3 {
    const N: usize = 6;
}
impl AuboType for _AuboE5 {
    const N: usize = 6;
}
impl AuboType for _AuboE10 {
    const N: usize = 6;
}

pub type AuboE3 = AuboRobot<_AuboE3, { _AuboE3::N }>;
pub type AuboE5 = AuboRobot<_AuboE5, { _AuboE5::N }>;
pub type AuboE10 = AuboRobot<_AuboE10, { _AuboE10::N }>;
