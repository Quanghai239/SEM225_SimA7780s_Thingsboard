# 🌿 Smart Agriculture: Hệ Thống Giám Sát Đất 9-in-1 qua Mạng 4G LTE

[![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg)]()
[![Framework](https://img.shields.io/badge/Framework-Arduino%20%7C%20FreeRTOS-orange.svg)]()
[![Connectivity](https://img.shields.io/badge/Network-4G%20LTE%20(A7680S)-red.svg)]()
[![Cloud](https://img.shields.io/badge/Cloud-ThingsBoard-305680.svg)]()

Dự án IoT ứng dụng trong nông nghiệp thông minh, sử dụng vi điều khiển ESP32 kết hợp với hệ điều hành thời gian thực **FreeRTOS** để thu thập, xử lý và truyền tải dữ liệu môi trường đất. Hệ thống đọc 9 thông số từ cảm biến đất chuyên dụng qua chuẩn giao tiếp RS485 và truyền dữ liệu JSON lên nền tảng quản lý IoT **ThingsBoard** thông qua module SIM 4G LTE A7680S.

## 📑 Mục Lục
- [Tính Năng Nổi Bật](#-tính-năng-nổi-bật)
- [Cấu Trúc Phần Cứng](#-cấu-trúc-phần-cứng)
- [Kiến Trúc Phần Mềm (FreeRTOS)](#-kiến-trúc-phần-mềm-freertos)
- [Định Dạng Dữ Liệu (Telemetry)](#-định-dạng-dữ-liệu-telemetry)
- [Cấu Trúc Thư Mục](#-cấu-trúc-thư-mục)
- [Hướng Dẫn Cài Đặt & Cấu Hình](#-hướng-dẫn-cài-đặt--cấu-hình)
- [Tác Giả](#-tác-giả)

## ✨ Tính Năng Nổi Bật
- **Giám sát 9 thông số đất (9-in-1):** Độ ẩm (Moisture), Nhiệt độ (Temp), Độ dẫn điện (EC), độ pH, N-P-K (Nitơ, Phốt pho, Kali), Độ mặn (Salinity) và Tổng chất rắn hòa tan (TDS).
- **Truyền thông 4G/LTE tốc độ cao:** Sử dụng module SIM A7680S điều khiển qua tập lệnh AT (được đóng gói thành thư viện `A7680_ThingsBoard`).
- **Xử lý đa luồng (Multi-tasking):** Ứng dụng FreeRTOS với Semaphore để phân tách task đọc cảm biến và task gửi dữ liệu lên mạng, đảm bảo không bị nghẽn (blocking) hệ thống.
- **Dấu thời gian chuẩn xác (Timestamp):** Tích hợp mạch thời gian thực RTC DS3231 qua I2C để gắn thời gian thực vào mỗi gói tin telemetry.
- **Giao thức công nghiệp:** Đọc cảm biến bằng chuẩn công nghiệp RS485 / Modbus RTU và truyền qua MQTT.

## 🛠 Cấu Trúc Phần Cứng
Hệ thống xoay quanh ESP32. Sơ đồ chân (Pinout) được cấu hình trong mã nguồn như sau:

| Thiết bị / Module | Giao tiếp | Chân ESP32 (Pin) | Ghi chú |
| :--- | :--- | :--- | :--- |
| **Module SIM A7680S** | UART2 | `RX = 22`, `TX = 23` | Tốc độ Baud: 115200 |
| **Cảm biến đất 9-in-1** | UART1 (RS485) | `TX = 13`, `RX = 14`, `DE = 12`| Tốc độ Baud: 4800, Half-Duplex |
| **RTC DS3231** | I2C | `SDA = 19`, `SCL = 18` | Lấy dữ liệu thời gian thực |

## ⚙️ Kiến Trúc Phần Mềm (FreeRTOS)
Dự án được phân chia luồng xử lý trên **Core 1** (`APP_TASK_CORE`) của ESP32 nhằm tối ưu hóa hiệu suất:
1. **Task Sensor (`TASK_PRIO_SENSOR = 2`):** Định kỳ gửi lệnh Modbus RTU (có tính toán mã CRC) để đọc dữ liệu từ cảm biến RS485, sau đó giải mã chuỗi byte phản hồi thành các giá trị `float`.
2. **Task ThingsBoard (`TASK_PRIO_THINGSBOARD = 4`):** Có độ ưu tiên cao hơn, chịu trách nhiệm đóng gói chuỗi JSON và duy trì kết nối MQTT qua mạng Cellular để publish dữ liệu. Các luồng được đồng bộ hóa dữ liệu an toàn.

## 📦 Định Dạng Dữ Liệu (Telemetry)
Dữ liệu được vi điều khiển chuyển đổi sang chuẩn JSON trước khi gửi lên Topic MQTT của ThingsBoard, ví dụ:
```json
{
  "ThoiGian": "2026/05/27 10:15:30",
  "DoAm": 65.50,
  "NhietDo": 28.30,
  "DoDanDien": 1.20,
  "Ph": 6.50,
  "Nito": 45.00,
  "PhotPho": 20.00,
  "Kali": 30.00,
  "DoMan": 0.50,
  "TongChatRanHoaTan": 250.00
}
