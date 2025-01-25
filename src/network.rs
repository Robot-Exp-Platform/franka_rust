use std::{
    collections::HashMap,
    io::{Read, Write},
    net::{SocketAddr, TcpStream, UdpSocket},
    sync::{Arc, Condvar, Mutex},
    thread,
    time::Duration,
};

use crate::exception::{FrankaException, FrankaResult /* 引入需要的错误类型 */};
use crate::robot::service_types::*; // 引入 service_types 里的定义

/// TCP keepalive 相关设置在 C++ 里是一个元组，这里我们用一个可选的结构体来封装。
pub struct TcpKeepalive {
    pub enabled: bool,
    pub keep_idle: u32,
    pub keep_count: u32,
    pub keep_interval: u32,
}

/// 用于存储尚未被“上层”提取的响应
struct ResponseSlot {
    /// 存储完整的字节数据。可根据你的协议进行反序列化后再存储。
    raw_data: Vec<u8>,
}

#[derive(Clone)]
pub struct Network {
    /// TCP 连接（阻塞模式）
    tcp_stream: Arc<Mutex<TcpStream>>,
    /// UDP socket（阻塞模式）
    udp_socket: Arc<Mutex<UdpSocket>>,
    /// 我们绑定的本地 UDP 端口，用于和机器人交换 UDP 通信
    udp_port: u16,

    /// command_id 计数器，用于给每个请求分配一个独立 ID
    command_id_counter: Arc<Mutex<u32>>,

    /// 存储已完成但尚未被用户读取的响应。Key 为 command_id。
    pending_responses: Arc<Mutex<HashMap<u32, ResponseSlot>>>,

    /// 当有新的响应放入 `pending_responses` 时，使用这个 Condvar 通知等待方。
    response_cv: Arc<Condvar>,

    /// 用于后台线程 stop 的标记，这里仅做演示
    stop_reader: Arc<Mutex<bool>>,
}

impl Network {
    /// 创建 Network 并完成 TCP/UDP 连接初始化
    pub fn new(
        franka_address: String,
        franka_port: u16,
        tcp_timeout: Duration,
        udp_timeout: Duration,
        keepalive: Option<TcpKeepalive>,
    ) -> FrankaResult<Self> {
        // 1. 连接 TCP
        let addr = format!("{}:{}", franka_address, franka_port);
        let tcp_stream = TcpStream::connect_timeout(
            &addr.parse().map_err(|_| {
                FrankaException::NetworkException("Invalid franka address".to_string())
            })?,
            tcp_timeout,
        )
        .map_err(|e| FrankaException::NetworkException(format!("TCP connect error: {e}")))?;

        // 设置超时（阻塞模式下 read/write 超时）
        tcp_stream
            .set_read_timeout(Some(tcp_timeout))
            .map_err(|e| FrankaException::NetworkException(format!("set_read_timeout: {e}")))?;
        tcp_stream
            .set_write_timeout(Some(tcp_timeout))
            .map_err(|e| FrankaException::NetworkException(format!("set_write_timeout: {e}")))?;

        // 如果要启用 keepalive，用 socket2 crate 来做更精细的设置
        if let Some(k) = keepalive {
            if k.enabled {
                // 这里只是演示，可以更深入地用 socket2
                // 在大多数系统上只有默认的 setsockopt TCP_KEEPIDLE/TCP_KEEPCNT/TCP_KEEPINTVL
                // ...
            }
        }

        // 2. 打开 UDP
        let udp_socket = UdpSocket::bind("0.0.0.0:0")
            .map_err(|e| FrankaException::NetworkException(format!("UDP bind error: {e}")))?;
        udp_socket
            .set_read_timeout(Some(udp_timeout))
            .map_err(|e| FrankaException::NetworkException(format!("UDP set_read_timeout: {e}")))?;

        let local_addr = udp_socket.local_addr().map_err(|e| {
            FrankaException::NetworkException(format!("Get local UDP addr error: {e}"))
        })?;
        let udp_port = local_addr.port();

        // 3. 构造并返回
        Ok(Network {
            tcp_stream: Arc::new(Mutex::new(tcp_stream)),
            udp_socket: Arc::new(Mutex::new(udp_socket)),
            udp_port,
            command_id_counter: Arc::new(Mutex::new(0)),
            pending_responses: Arc::new(Mutex::new(HashMap::new())),
            response_cv: Arc::new(Condvar::new()),
            stop_reader: Arc::new(Mutex::new(false)),
        })
    }

    /// 返回当前所使用的本地 UDP 端口
    pub fn udp_port(&self) -> u16 {
        self.udp_port
    }

    /// 启动后台线程负责循环读取 TCP 响应并存入 pending_responses。
    /// 一旦有新响应到达，就用 `Condvar` 唤醒阻塞等待的请求方。
    pub fn start_reader_thread(&self) -> thread::JoinHandle<()> {
        let tcp_stream = Arc::clone(&self.tcp_stream);
        let pending_responses = Arc::clone(&self.pending_responses);
        let response_cv = Arc::clone(&self.response_cv);
        let stop_flag = Arc::clone(&self.stop_reader);

        thread::spawn(move || {
            loop {
                // 如果已经需要停止，就退出后台线程
                if *stop_flag.lock().unwrap() {
                    break;
                }

                let mut stream = match tcp_stream.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        // 如果 mutex 中毒，就退出
                        break;
                    }
                };

                // 这里是一个简化处理，每次尝试读取一条完整消息。
                // 实际需求中，你也许需要先读固定头，再根据头里 size 来读剩余 payload。
                let mut header_buf = [0_u8; 8]; // 假设前 8 字节包含 command_id 和 size

                // 阻塞式读：先读 header
                if let Err(e) = stream.read_exact(&mut header_buf) {
                    // 如果对端断开等，也可以做更多处理
                    eprintln!("TCP read error: {}", e);
                    continue;
                }

                // 根据你自己的协议格式解析 command_id / total_size
                // 这里只做示例
                let command_id = u32::from_le_bytes(header_buf[0..4].try_into().unwrap());
                let total_size = u32::from_le_bytes(header_buf[4..8].try_into().unwrap());

                // 读取剩余 payload
                let mut payload = vec![0_u8; total_size as usize];
                if let Err(e) = stream.read_exact(&mut payload) {
                    eprintln!("TCP read payload error: {}", e);
                    continue;
                }

                // 存入 pending_responses
                {
                    let mut map = pending_responses.lock().unwrap();
                    map.insert(command_id, ResponseSlot { raw_data: payload });
                    // 唤醒等待此 command_id 的线程
                    response_cv.notify_all();
                }
            }
        })
    }

    /// 停止后台读取线程
    pub fn stop_reader_thread(&self) {
        let mut flag = self.stop_reader.lock().unwrap();
        *flag = true;
        // 后台线程会在下一次循环判断到此标志后退出
    }

    // ----------- UDP 部分示例 -----------

    /// 简易阻塞 UDP 接收，要求一次性收满 size_of::<T>() 字节
    pub fn udp_blocking_receive<T: Copy>(&self) -> FrankaResult<T> {
        use std::mem::size_of;
        let mut buf = vec![0u8; size_of::<T>()];
        let socket = self.udp_socket.lock().unwrap();

        let (recv_size, _from_addr) = socket
            .recv_from(&mut buf)
            .map_err(|e| FrankaException::NetworkException(format!("UDP recv error: {e}")))?;
        if recv_size != size_of::<T>() {
            return Err(FrankaException::ProtocolException(format!(
                "UDP expected {} bytes, got {recv_size}",
                size_of::<T>()
            )));
        }
        // unsafe 反序列化成 T
        let data = unsafe { std::ptr::read(buf.as_ptr() as *const T) };
        Ok(data)
    }

    /// 简易阻塞 UDP 发送
    pub fn udp_send<T: Copy>(&self, data: &T, dest: &SocketAddr) -> FrankaResult<()> {
        use std::mem::size_of;
        let socket = self.udp_socket.lock().unwrap();
        let bytes_sent = socket
            .send_to(
                unsafe {
                    std::slice::from_raw_parts((data as *const T) as *const u8, size_of::<T>())
                },
                dest,
            )
            .map_err(|e| FrankaException::NetworkException(format!("UDP send error: {e}")))?;
        if bytes_sent != size_of::<T>() {
            return Err(FrankaException::NetworkException(
                "UDP sent size mismatch".to_string(),
            ));
        }
        Ok(())
    }

    // ----------- TCP 发送与等待响应的核心逻辑示例 -----------

    /// 将请求发送到 TCP。返回分配的 command_id，用于后续匹配响应。
    fn send_tcp_request(&self, raw_bytes: &[u8]) -> FrankaResult<u32> {
        let mut guard = self.command_id_counter.lock().unwrap();
        let cmd_id = *guard;
        *guard += 1;
        drop(guard); // 提前释放锁

        let mut stream = self.tcp_stream.lock().unwrap();
        // 写出 raw_bytes，这里假设 raw_bytes 的前面部分就包含了 cmd_id、size 等信息
        stream
            .write_all(raw_bytes)
            .map_err(|e| FrankaException::NetworkException(format!("TCP write error: {e}")))?;
        Ok(cmd_id)
    }

    /// 等待后台线程读到指定 command_id 的响应后，将其取出并返回。
    fn wait_for_response(&self, command_id: u32) -> FrankaResult<Vec<u8>> {
        let mut map = self.pending_responses.lock().unwrap();

        // 如果没有读到，就等待 condvar 唤醒再看看
        loop {
            if let Some(slot) = map.remove(&command_id) {
                // 找到后直接返回
                return Ok(slot.raw_data);
            }
            map = self
                .response_cv
                .wait(map)
                .map_err(|_| FrankaException::NetworkException("Condvar poisoned".to_string()))?;
        }
    }

    // ----------- 基于 service_type 里的请求/响应做的示例 -----------

    /// 发送 Connect 请求，并阻塞等待 Connect 响应
    pub fn connect_robot(&self, version: u16) -> FrankaResult<u16> {
        // 1. 构造数据：ConnectRequest
        //   假设 ConnectRequest = { header, request: ConnectData { version, udp_port } }
        //   这里省略复杂序列化，演示一个简单做法：将头 + body 做个打包
        let request_data = ConnectData {
            version,
            udp_port: self.udp_port,
        };
        let request_bytes = self.serialize_connect_request(&request_data)?;
        let cmd_id = self.send_tcp_request(&request_bytes)?;

        // 2. 等待后台线程读取到对应 command_id 的响应
        let response_bytes = self.wait_for_response(cmd_id)?;

        // 3. 反序列化 ConnectResponse
        let response = self.deserialize_connect_response(&response_bytes)?;

        // 校验 status
        match response {
            ConnectResponse {
                header: _,
                status: ConnectStatus::Success,
            } => Ok(version),
            ConnectResponse {
                status: ConnectStatus::IncompatibleLibraryVersion,
                ..
            } => {
                // 这里演示怎么抛出 IncompatibleVersionException
                Err(FrankaException::IncompatibleVersionException {
                    server_version: 0, // 需根据实际协议取出版本
                    client_version: version as u64,
                })
            }
        }
    }

    /// 演示一个简易的“序列化 ConnectRequest”的方法
    fn serialize_connect_request(&self, data: &ConnectData) -> FrankaResult<Vec<u8>> {
        // 假设命令头 8 字节： [cmd_id:4, size:4]
        // 再跟 ConnectData { version: u16, udp_port: u16 } 一共 4 字节
        // 真实情况请根据 service_type 的定义来做
        let mut result = Vec::new();

        // header 的 cmd_id 先暂用 0，真正写的时候由 send_tcp_request 覆盖也可以
        // 这里为了简单我们只是演示
        let cmd_id_placeholder = 0_u32.to_le_bytes();
        let size = (4_u32).to_le_bytes(); // 这里仅演示 ConnectData 占 4 字节

        result.extend_from_slice(&cmd_id_placeholder);
        result.extend_from_slice(&size);

        // ConnectData: version(u16) + udp_port(u16)
        result.extend_from_slice(&data.version.to_le_bytes());
        result.extend_from_slice(&data.udp_port.to_le_bytes());

        Ok(result)
    }

    /// 演示一个简易的“反序列化 ConnectResponse”
    fn deserialize_connect_response(&self, raw: &[u8]) -> FrankaResult<ConnectResponse> {
        // 假设同样前 8 字节是头，后面 2 字节表示 status
        if raw.len() < 10 {
            return Err(FrankaException::ProtocolException(
                "ConnectResponse too small".to_string(),
            ));
        }
        let _cmd_id = u32::from_le_bytes(raw[0..4].try_into().unwrap());
        let _size = u32::from_le_bytes(raw[4..8].try_into().unwrap());

        // status
        let status_raw = u16::from_le_bytes(raw[8..10].try_into().unwrap());
        let status = match status_raw {
            0 => ConnectStatus::Success,
            _ => ConnectStatus::IncompatibleLibraryVersion,
        };

        let response = ConnectResponse {
            header: CommandHeader {
                command_id: _cmd_id,
                size: _size,
            },
            status,
        };
        Ok(response)
    }
}

// 在需要的地方，调用 `new` -> `start_reader_thread` -> `connect_robot` 即可。
// 结束时可以调用 `stop_reader_thread` 再 Join。
