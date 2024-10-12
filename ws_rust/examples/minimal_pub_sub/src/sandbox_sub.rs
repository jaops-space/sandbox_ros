use std::env;

use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let mut node = rclrs::create_node(&context, "sandbox_subscriber")?;

    let mut num_messages: usize = 0;

    // example joint_state message type
    let _subscript = node.create_subscription::<sensor_msgs::msg::JointState, _>(
        "/maxon/joint_states",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: sensor_msgs::msg::JointState| {
            num_messages += 1;
            println!("[/maxon/joint_states] [message {}] : name = '{:?}'", num_messages, msg.name);
            println!("[/maxon/joint_states] [message {}] : position = '{:?}'", num_messages, msg.position);
            println!("[/maxon/joint_states] [message {}] : velocity = '{:?}'", num_messages, msg.velocity);
        },
    )?;

    // example Float64 message type
    let _subscript = node.create_subscription::<std_msgs::msg::Float64, _>(
        "/maxon/canopen_motor/base_link2_joint_velocity_controller/command",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: std_msgs::msg::Float64| {
            println!("[/maxon/canopen_motor/base_link2_joint_velocity_controller/command] {}", msg.data);
        },
    )?;
    
    // note: there are 6 joints in each 'leg'. the only difference is the ? in the topic name, e.g. /ang_leg3_joint?_set
    // TBD if we have to subscribe to them all but no worries either way.
    let _subscript = node.create_subscription::<std_msgs::msg::Float64, _>(
        "/ang_leg3_joint1_set",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: std_msgs::msg::Float64| {
            println!("[/ang_leg3_joint1_set] {}", msg.data);
        },
    )?;

    let _subscript = node.create_subscription::<std_msgs::msg::Float64, _>(
        "/read_leg3_joint1",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: std_msgs::msg::Float64| {
            println!("[/read_leg3_joint1] {}", msg.data);
        },
    )?;
    
    // example Log message type
    // note: DEBUG=10 INFO=20 WARN=30 ERROR=40 FATAL=50
    let _subscript = node.create_subscription::<rcl_interfaces::msg::Log, _>(
        "/rosout",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: rcl_interfaces::msg::Log| {
            println!("[/rosout] level = {} | name = {} | msg = {}", msg.level, msg.name, msg.msg);
        },
    )?;

    rclrs::spin(&node).map_err(|err| err.into())
}
