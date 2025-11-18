use crate::{AuboRobot, AuboType};

pub trait AuboI {}

pub struct _AuboI3;
pub struct _AuboI5;
pub struct _AuboI7;
pub struct _AuboI10;
pub struct _AuboI12;
pub struct _AuboI16;
pub struct _AuboI20;

impl AuboI for _AuboI3 {}
impl AuboI for _AuboI5 {}
impl AuboI for _AuboI7 {}
impl AuboI for _AuboI10 {}
impl AuboI for _AuboI12 {}
impl AuboI for _AuboI16 {}
impl AuboI for _AuboI20 {}

impl AuboType for _AuboI3 {
    const N: usize = 6;
}

impl AuboType for _AuboI5 {
    const N: usize = 6;
}

impl AuboType for _AuboI7 {
    const N: usize = 6;
}

impl AuboType for _AuboI10 {
    const N: usize = 6;
}

impl AuboType for _AuboI12 {
    const N: usize = 6;
}

impl AuboType for _AuboI16 {
    const N: usize = 6;
}

impl AuboType for _AuboI20 {
    const N: usize = 6;
}

pub type AuboI3 = AuboRobot<_AuboI3, { _AuboI3::N }>;
pub type AuboI5 = AuboRobot<_AuboI5, { _AuboI5::N }>;
pub type AuboI7 = AuboRobot<_AuboI7, { _AuboI7::N }>;
pub type AuboI10 = AuboRobot<_AuboI10, { _AuboI10::N }>;
pub type AuboI12 = AuboRobot<_AuboI12, { _AuboI12::N }>;
pub type AuboI16 = AuboRobot<_AuboI16, { _AuboI16::N }>;
pub type AuboI20 = AuboRobot<_AuboI20, { _AuboI20::N }>;
