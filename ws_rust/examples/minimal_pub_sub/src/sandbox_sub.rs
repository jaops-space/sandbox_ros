use std::env;

use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let mut node = rclrs::create_node(&context, "sandbox_subscriber")?;

    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<sensor_msgs::msg::JointState, _>(
        "/maxon/joint_states",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: sensor_msgs::msg::JointState| {
            num_messages += 1;
            println!("[/maxon/joint_states] [message {}] : name = '{:?}'", num_messages, msg.name);
            println!("[/maxon/joint_states] [message {}] : position = '{:?}'", num_messages, msg.position);
            println!("[/maxon/joint_states] [message {}] : velocity = '{:?}'", num_messages, msg.velocity);
        },
    )?;

    rclrs::spin(&node).map_err(|err| err.into())
}
