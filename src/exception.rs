use thiserror::Error;

#[derive(Error, Debug)]
pub enum FrankaException {
    /// ModelException is thrown if an error occurs when loading the model library.
    #[error("Model exception: {0}")]
    ModelException(String),

    /// NetworkException is thrown if a connection to the robot cannot be established, or when a timeout occurs.
    #[error("Control exception: {0}")]
    NetworkException(String),

    /// ProtocolException is thrown if the robot returns an incorrect message.
    #[error("Control exception: {0}")]
    ProtocolException(String),

    /// IncompatibleVersionException is thrown if the robot does not support this version of libfranka.
    #[error(
        "Incompatible version: server version {server_version}, client version {client_version}"
    )]
    IncompatibleVersionException {
        server_version: u64,
        client_version: u64,
    },

    /// ControlException is thrown if an error occurs during motion generation or torque control.
    /// The exception holds a vector with the last received robot states. The number of recorded
    /// states can be configured in the Robot constructor.
    #[error("Control exception: {0}")]
    ControlException(String),

    /// CommandException is thrown if an error occurs during command execution.
    #[error("Command exception: {0}")]
    CommandException(String),

    /// RealtimeException is thrown if realtime priority cannot be set.
    #[error("Realtime exception: {0}")]
    RealtimeException(String),

    /// InvalidOperationException is thrown if an operation cannot be performed.
    #[error("InvalidOperationException exception: {0}")]
    InvalidOperationException(String),
}

pub type FrankaResult<T> = Result<T, FrankaException>;
