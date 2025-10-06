use crate::{AuboRobot, AuboType};

pub trait AuboC {}

pub struct _AuboC5 {}

impl AuboC for _AuboC5 {}
impl AuboType for _AuboC5 {
    const N: usize = 6;
}

pub type AuboC5 = AuboRobot<_AuboC5, { _AuboC5::N }>;
