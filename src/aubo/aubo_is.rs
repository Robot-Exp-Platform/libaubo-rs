use crate::{AuboRobot, AuboType};

pub trait AuboIS {}

pub struct _AuboIS7;
pub struct _AuboIS10;

pub struct _AuboIS20;
pub struct _AuboIS25;
pub struct _AuboIS35;

impl AuboIS for _AuboIS7 {}
impl AuboIS for _AuboIS10 {}
impl AuboIS for _AuboIS20 {}
impl AuboIS for _AuboIS25 {}
impl AuboIS for _AuboIS35 {}

impl AuboType for _AuboIS7 {
    const N: usize = 6;
}

impl AuboType for _AuboIS10 {
    const N: usize = 6;
}

impl AuboType for _AuboIS20 {
    const N: usize = 6;
}

impl AuboType for _AuboIS25 {
    const N: usize = 6;
}

impl AuboType for _AuboIS35 {
    const N: usize = 6;
}

pub type AuboIS7 = AuboRobot<_AuboIS7, { _AuboIS7::N }>;
pub type AuboIS10 = AuboRobot<_AuboIS10, { _AuboIS10::N }>;
pub type AuboIS20 = AuboRobot<_AuboIS20, { _AuboIS20::N }>;
pub type AuboIS25 = AuboRobot<_AuboIS25, { _AuboIS25::N }>;
pub type AuboIS35 = AuboRobot<_AuboIS35, { _AuboIS35::N }>;
