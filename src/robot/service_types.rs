use std::marker::ConstParamTy;

#[derive(ConstParamTy, PartialEq, Eq)]
pub enum Command {
    Connect,
    Move,
    StopMove,
    GetCartesianLimit,
    SetCollisionBehavior,
    SetJointImpedance,
    SetCartesianImpedance,
    SetGuidingMode,
    SetEEToK,
    SetNEToEE,
    SetLoad,
    SetFilters,
    AutomaticErrorRecovery,
    LoadModelLibrary,
    GetRobotModel,
}

pub struct CommandHeader<const C: Command> {
    pub command_id: u32,
    pub size: u32,
}

pub struct Request<const C: Command, R> {
    pub header: CommandHeader<C>,
    pub request: R,
}

pub struct Response<const C: Command, S> {
    pub header: CommandHeader<C>,
    pub status: S,
}

pub type ConnectRequest = Request<{ Command::Connect }, ConnectData>;
pub type MoveRequest = Request<{ Command::Move }, MoveData>;
pub type StopMoveRequest = Request<{ Command::StopMove }, ()>;
pub type GetCartesianLimitRequest = Request<{ Command::GetCartesianLimit }, GetCartesianLimitData>;
pub type SetCollisionBehaviorRequest =
    Request<{ Command::SetCollisionBehavior }, SetCollisionBehaviorData>;
pub type SetJointImpedanceRequest = Request<{ Command::SetJointImpedance }, SetJointImpedanceData>;
pub type SetCartesianImpedanceRequest =
    Request<{ Command::SetCartesianImpedance }, SetCartesianImpedanceData>;
pub type SetGuidingModeRequest = Request<{ Command::SetGuidingMode }, SetGuidingModeData>;
pub type SetEEToKRequest = Request<{ Command::SetEEToK }, SetEEToKData>;
pub type SetNEToEERequest = Request<{ Command::SetNEToEE }, SetNEToEEData>;
pub type SetLoadRequest = Request<{ Command::SetLoad }, SetLoadData>;
pub type SetFiltersRequest = Request<{ Command::SetFilters }, SetFiltersData>;
pub type AutomaticErrorRecoveryRequest = Request<{ Command::AutomaticErrorRecovery }, ()>;
pub type LoadModelLibraryRequest = Request<{ Command::LoadModelLibrary }, LoadModelLibraryData>;
pub type GetRobotModelRequest = Request<{ Command::GetRobotModel }, ()>;

pub type ConnectResponse = Response<{ Command::Connect }, ConnectStatus>;
pub type MoveResponse = Response<{ Command::Move }, MoveStatus>;
pub type StopMoveResponse = Response<{ Command::StopMove }, StopMoveStatus>;
pub type GetCartesianLimitResponse =
    Response<{ Command::GetCartesianLimit }, GetCartesianLimitResponseData>;
pub type SetCollisionBehaviorResponse =
    Response<{ Command::SetCollisionBehavior }, GetterSetterStatus>;
pub type SetJointImpedanceResponse = Response<{ Command::SetJointImpedance }, GetterSetterStatus>;
pub type SetCartesianImpedanceResponse =
    Response<{ Command::SetCartesianImpedance }, GetterSetterStatus>;
pub type SetGuidingModeResponse = Response<{ Command::SetGuidingMode }, GetterSetterStatus>;
pub type SetEEToKResponse = Response<{ Command::SetEEToK }, GetterSetterStatus>;
pub type SetNEToEEResponse = Response<{ Command::SetNEToEE }, GetterSetterStatus>;
pub type SetLoadResponse = Response<{ Command::SetLoad }, GetterSetterStatus>;
pub type SetFiltersResponse = Response<{ Command::SetFilters }, GetterSetterStatus>;
pub type AutomaticErrorRecoveryResponse =
    Response<{ Command::AutomaticErrorRecovery }, AutomaticErrorRecoveryStatus>;
pub type LoadModelLibraryResponse = Response<{ Command::LoadModelLibrary }, LoadModelLibraryStatus>;
pub type GetRobotModelResponse = Response<{ Command::GetRobotModel }, DefaultStatus>;

pub enum DefaultStatus {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
}

pub enum GetterSetterStatus {
    Success,
    CommandNotPossibleRejected,
    InvalidArgumentRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
}

// ! Connect Command

pub struct ConnectData {
    pub version: u16,
    pub udp_port: u16,
}

pub enum ConnectStatus {
    Success,
    IncompatibleLibraryVersion,
}

// ! Move Command

pub enum MoveControllerMode {
    JointImpedance,
    CartesianImpedance,
    ExternalController,
}

pub enum MoveMotionGeneratorMode {
    JointPosition,
    JointVelocity,
    CartesianPosition,
    CartesianVelocity,
}

pub struct MoveDeviation {
    translation: f64,
    rotation: f64,
    elbow: f64,
}

pub struct MoveData {
    controller_mode: MoveControllerMode,
    motion_generator_mode: MoveMotionGeneratorMode,
    maximum_path_deviation: MoveDeviation,
    maximum_goal_deviation: MoveDeviation,
}

pub enum MoveStatus {
    Success,
    MotionStarted,
    Preempted,
    PreemptedDueToActivatedSafetyFunctions,
    CommandRejectedDueToActivatedSafetyFunctions,
    CommandNotPossibleRejected,
    StartAtSingularPoseRejected,
    InvalidArgumentRejected,
    ReflexAborted,
    EmergencyAborted,
    InputErrorAborted,
    Aborted,
}

// ! StopMove Command

pub enum StopMoveStatus {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
    EmergencyAborted,
    ReflexAborted,
    Aborted,
}

// ! GetCartesianLimit Command

pub struct GetCartesianLimitData {
    id: i32,
}

pub struct GetCartesianLimitResponseData {
    object_world_size: [f64; 3],
    object_frame: [f64; 16],
    object_activation: bool,
}

// ! SetCollisionBehavior Command

pub struct SetCollisionBehaviorData {
    lower_torque_thresholds_acceleration: [f64; 7],
    upper_torque_thresholds_acceleration: [f64; 7],
    lower_torque_thresholds_nominal: [f64; 7],
    upper_torque_thresholds_nominal: [f64; 7],
    lower_force_thresholds_acceleration: [f64; 6],
    upper_force_thresholds_acceleration: [f64; 6],
    lower_force_thresholds_nominal: [f64; 6],
    upper_force_thresholds_nominal: [f64; 6],
}

// ! SetJointImpedance Command

pub struct SetJointImpedanceData {
    k_theta: [f64; 7],
}

// ! SetCartesianImpedance Command

pub struct SetCartesianImpedanceData {
    k_x: [f64; 6],
}

// ! SetGuidingMode Command

pub struct SetGuidingModeData {
    guiding_mode: [bool; 6],
    nullspace: bool,
}

// ! SetEEToK Command

pub struct SetEEToKData {
    pose_ee_to_k: [f64; 16],
}

// ! SetNEToEE Command

pub struct SetNEToEEData {
    pose_ne_to_ee: [f64; 16],
}

// ! SetLoad Command

pub struct SetLoadData {
    m_load: f64,
    x_cload: [f64; 3],
    i_load: [f64; 9],
}

// ! SetFilters Command

pub struct SetFiltersData {
    joint_position_filter_frequency: f64,
    joint_velocity_filter_frequency: f64,
    cartesian_position_filter_frequency: f64,
    cartesian_velocity_filter_frequency: f64,
    controller_filter_frequency: f64,
}

// ! AutomaticErrorRecovery Command

pub enum AutomaticErrorRecoveryStatus {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
    ManualErrorRecoveryRequiredRejected,
    ReflexAborted,
    EmergencyAborted,
    Aborted,
}

// ! LoadModelLibrary Command

pub enum LoadModelLibraryArchitecture {
    X64,
    X86,
    Arm,
    Arm64,
}

pub enum LoadModelLibrarySystem {
    Linux,
    Windows,
}

pub struct LoadModelLibraryData {
    architecture: LoadModelLibraryArchitecture,
    system: LoadModelLibrarySystem,
}

pub enum LoadModelLibraryStatus {
    Success,
    Error,
}
