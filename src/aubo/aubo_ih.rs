use crate::{AuboRobot, AuboType};

pub trait AuboIH {}

pub struct _AuboI3H;
pub struct _AuboI5H;
pub struct _AuboI7H;
pub struct _AuboI10H;
pub struct _AuboI12H;
pub struct _AuboI16H;

impl AuboIH for _AuboI3H {}
impl AuboIH for _AuboI5H {}
impl AuboIH for _AuboI7H {}
impl AuboIH for _AuboI10H {}
impl AuboIH for _AuboI12H {}
impl AuboIH for _AuboI16H {}

impl AuboType for _AuboI3H {
    const N: usize = 6;
}

impl AuboType for _AuboI5H {
    const N: usize = 6;
}

impl AuboType for _AuboI7H {
    const N: usize = 6;
}

impl AuboType for _AuboI10H {
    const N: usize = 6;
}

impl AuboType for _AuboI12H {
    const N: usize = 6;
}

impl AuboType for _AuboI16H {
    const N: usize = 6;
}

pub type AuboI3H = AuboRobot<_AuboI3H, { _AuboI3H::N }>;
pub type AuboI5H = AuboRobot<_AuboI5H, { _AuboI5H::N }>;
pub type AuboI7H = AuboRobot<_AuboI7H, { _AuboI7H::N }>;
pub type AuboI10H = AuboRobot<_AuboI10H, { _AuboI10H::N }>;
pub type AuboI12H = AuboRobot<_AuboI12H, { _AuboI12H::N }>;
pub type AuboI16H = AuboRobot<_AuboI16H, { _AuboI16H::N }>;
