// pybind_hesai_sdk.cpp

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h> // For py::buffer_info

#include <chrono>
#include <memory>
#include <sstream>
#include <thread>

// Include Hesai SDK headers
#include "hesai_lidar_sdk.hpp"
#include "lidar_types.h"
#include "driver_param.h"
#include "plat_utils.h" // For GetMicroTickCount
#include "ptc_client.h"

namespace py = pybind11;
using namespace hesai::lidar;

// Define our specific SDK and Frame types for clarity
using LidarPointT = LidarPointXYZICRT;
using HesaiLidarSdk_XYZICRT = HesaiLidarSdk<LidarPointT>;
using LidarDecodedFrame_XYZICRT = LidarDecodedFrame<LidarPointT>;

namespace {

void WaitUntilOpen(hesai::lidar::PtcClient& client, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_s);
    const int timeout_ms = static_cast<int>(timeout_s * 1000.0);
    if (timeout_ms > 0) {
        client.SetSocketTimeout(
            static_cast<uint32_t>(timeout_ms),
            static_cast<uint32_t>(timeout_ms));
    }
    while (!client.IsOpen()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error("PTC connection timeout");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

std::unique_ptr<hesai::lidar::PtcClient> ConnectPtc(
    const std::string& ip, uint16_t port, double timeout_s) {
    auto client = std::make_unique<hesai::lidar::PtcClient>(ip, port);
    WaitUntilOpen(*client, timeout_s);
    return client;
}

py::bytes U8ArrayToBytes(const hesai::lidar::u8Array_t& data) {
    return py::bytes(
        reinterpret_cast<const char*>(data.data()),
        static_cast<py::ssize_t>(data.size()));
}

}  // namespace

// PYBIND11_MODULE defines the entry point for the Python module.
// The first argument (pyhesai_wrapper_cpp) is the module name in Python (e.g., import pyhesai_wrapper_cpp).
PYBIND11_MODULE(pyhesai_wrapper_cpp, m) {
    m.doc() = "pybind11 wrapper for Hesai Lidar SDK 2.0";

    // --- Wrap Enums from driver_param.h ---

    py::enum_<SourceType>(m, "SourceType")
        .value("DATA_FROM_LIDAR", SourceType::DATA_FROM_LIDAR)
        .value("DATA_FROM_PCAP", SourceType::DATA_FROM_PCAP)
        .value("DATA_FROM_ROS_PACKET", SourceType::DATA_FROM_ROS_PACKET)
        .value("DATA_FROM_SERIAL", SourceType::DATA_FROM_SERIAL)
        .export_values();

    py::enum_<UseTimestampType>(m, "UseTimestampType")
        .value("point_cloud_timestamp", UseTimestampType::point_cloud_timestamp)
        .value("sdk_recv_timestamp", UseTimestampType::sdk_recv_timestamp)
        .export_values();

    // --- Wrap Point Structure from lidar_types.h ---

    py::class_<LidarPointT>(m, "LidarPointXYZICRT")
        .def(py::init<>())
        .def_readwrite("x", &LidarPointT::x)
        .def_readwrite("y", &LidarPointT::y)
        .def_readwrite("z", &LidarPointT::z)
        .def_readwrite("intensity", &LidarPointT::intensity)
        .def_readwrite("confidence", &LidarPointT::confidence)
        .def_readwrite("ring", &LidarPointT::ring)
        .def_readwrite("timestamp", &LidarPointT::timestamp)
        .def("__repr__",
            [](const LidarPointT &p) {
                return "<LidarPointXYZICRT: x=" + std::to_string(p.x) +
                       ", y=" + std::to_string(p.y) +
                       ", z=" + std::to_string(p.z) +
                       ", i=" + std::to_string(p.intensity) +
                       ", c=" + std::to_string(p.confidence) +
                       ", r=" + std::to_string(p.ring) +
                       ", t=" + std::to_string(p.timestamp) + ">";
            });
    
    // Register the struct format with NumPy
    PYBIND11_NUMPY_DTYPE(LidarPointT, x, y, z, intensity, confidence, ring, timestamp);

    // --- Wrap Parameter Structs from driver_param.h ---
    
    py::class_<TransformParam>(m, "TransformParam")
        .def(py::init<>())
        .def_readwrite("x", &TransformParam::x)
        .def_readwrite("y", &TransformParam::y)
        .def_readwrite("z", &TransformParam::z)
        .def_readwrite("roll", &TransformParam::roll)
        .def_readwrite("pitch", &TransformParam::pitch)
        .def_readwrite("yaw", &TransformParam::yaw);

    py::class_<DecoderParam>(m, "DecoderParam")
        .def(py::init<>())
        .def_readwrite("transform_param", &DecoderParam::transform_param)
        .def_readwrite("thread_num", &DecoderParam::thread_num)
        .def_readwrite("enable_udp_thread", &DecoderParam::enable_udp_thread)
        .def_readwrite("enable_parser_thread", &DecoderParam::enable_parser_thread)
        .def_readwrite("pcap_play_synchronization", &DecoderParam::pcap_play_synchronization)
        .def_readwrite("frame_start_azimuth", &DecoderParam::frame_start_azimuth)
        .def_readwrite("use_timestamp_type", &DecoderParam::use_timestamp_type)
        .def_readwrite("fov_start", &DecoderParam::fov_start)
        .def_readwrite("fov_end", &DecoderParam::fov_end)
        .def_readwrite("distance_correction_flag", &DecoderParam::distance_correction_flag)
        .def_readwrite("socket_buffer_size", &DecoderParam::socket_buffer_size);


    py::class_<InputParam>(m, "InputParam")
        .def(py::init<>())
        .def_readwrite("source_type", &InputParam::source_type)
        .def_readwrite("device_ip_address", &InputParam::device_ip_address)
        .def_readwrite("multicast_ip_address", &InputParam::multicast_ip_address)
        .def_readwrite("udp_port", &InputParam::udp_port)
        .def_readwrite("ptc_port", &InputParam::ptc_port)
        .def_readwrite("use_ptc_connected", &InputParam::use_ptc_connected)
        .def_readwrite("pcap_path", &InputParam::pcap_path)
        .def_readwrite("correction_file_path", &InputParam::correction_file_path)
        .def_readwrite("firetimes_path", &InputParam::firetimes_path)
        .def_readwrite("send_packet_ros", &InputParam::send_packet_ros)
        .def_readwrite("send_point_cloud_ros", &InputParam::send_point_cloud_ros)
        .def_readwrite("frame_id", &InputParam::frame_id)
        .def_readwrite("ros_send_packet_topic", &InputParam::ros_send_packet_topic)
        .def_readwrite("ros_send_point_topic", &InputParam::ros_send_point_topic)
        .def_readwrite("ros_send_correction_topic", &InputParam::ros_send_correction_topic)
        .def_readwrite("ros_send_imu_topic", &InputParam::ros_send_imu_topic)
        .def_readwrite("ros_recv_correction_topic", &InputParam::ros_recv_correction_topic)
        .def_readwrite("ros_recv_packet_topic", &InputParam::ros_recv_packet_topic);


    py::class_<DriverParam>(m, "DriverParam")
        .def(py::init<>())
        .def_readwrite("input_param", &DriverParam::input_param)
        .def_readwrite("decoder_param", &DriverParam::decoder_param)
        .def_readwrite("use_gpu", &DriverParam::use_gpu)
        .def_readwrite("frame_id", &DriverParam::frame_id)
        .def_readwrite("log_level", &DriverParam::log_level)
        .def_readwrite("log_Target", &DriverParam::log_Target)
        .def_readwrite("log_path", &DriverParam::log_path);

    // --- WRAP IMU SUPPORT ---
    
    py::class_<LidarImuData>(m, "LidarImuData")
        .def(py::init<>())
        .def_readwrite("timestamp", &LidarImuData::timestamp)
        .def_readwrite("imu_accel_x", &LidarImuData::imu_accel_x)
        .def_readwrite("imu_accel_y", &LidarImuData::imu_accel_y)
        .def_readwrite("imu_accel_z", &LidarImuData::imu_accel_z)
        .def_readwrite("imu_ang_vel_x", &LidarImuData::imu_ang_vel_x)
        .def_readwrite("imu_ang_vel_y", &LidarImuData::imu_ang_vel_y)
        .def_readwrite("imu_ang_vel_z", &LidarImuData::imu_ang_vel_z)
        .def("__repr__",
            [](const LidarImuData &imu) {
                return "<LidarImuData: t=" + std::to_string(imu.timestamp) +
                        ", accel_x=" + std::to_string(imu.imu_accel_x) +
                        ", ang_vel_x=" + std::to_string(imu.imu_ang_vel_x) + ">";
            });
    // --- END IMU SUPPORT ---


    // --- Wrap Decoded Frame from lidar_types.h ---

    py::class_<LidarDecodedFrame_XYZICRT>(m, "LidarDecodedFrame_XYZICRT")
        .def_readonly("frame_index", &LidarDecodedFrame_XYZICRT::frame_index)
        .def_readonly("points_num", &LidarDecodedFrame_XYZICRT::points_num)
        .def_readonly("packet_num", &LidarDecodedFrame_XYZICRT::packet_num)

        // Sweep timestamps, in seconds on the lidar's own clock (use_timestamp_type=0)
        // or the SDK receive clock (use_timestamp_type=1). frame_start_timestamp is
        // the value the ROS driver puts in header.stamp, and it matches the single
        // revolution exposed by 'points' below.
        .def_readonly("frame_start_timestamp", &LidarDecodedFrame_XYZICRT::frame_start_timestamp)
        .def_readonly("frame_end_timestamp", &LidarDecodedFrame_XYZICRT::frame_end_timestamp)

        // Expose 'points' as a zero-copy NumPy array
        .def_property_readonly("points", [](LidarDecodedFrame_XYZICRT &frame) {
            return py::array_t<LidarPointT>(
                { static_cast<py::ssize_t>(frame.points_num) },
                { static_cast<py::ssize_t>(sizeof(LidarPointT)) },
                frame.points,
                py::cast(frame)  // Keep frame alive while array is in use
            );
        });

        // --- ADDED FOR JT128 SUPPORT ---
        // Expose 'jt128_buffer' as a zero-copy NumPy buffer
        // NOTE: Commented out because jt128_buffer may not be available in all SDK versions
        /*
        .def_property_readonly("jt128_buffer", [](py::object &obj) {
            LidarDecodedFrame_XYZICRT &frame = obj.cast<LidarDecodedFrame_XYZICRT&>();

            // --- THIS IS THE FIX ---
            // Explicitly cast all shape/size arguments to py::ssize_t
            // to match the py::buffer_info constructor signature.
            py::ssize_t itemsize = static_cast<py::ssize_t>(sizeof(uint8_t));
            py::ssize_t ndim = 2;
            std::vector<py::ssize_t> shape = {
                static_cast<py::ssize_t>(frame.packet_num),
                static_cast<py::ssize_t>(1100)
            };
            std::vector<py::ssize_t> strides = {
                static_cast<py::ssize_t>(sizeof(JT128buffer)),
                static_cast<py::ssize_t>(sizeof(uint8_t))
            };

            return py::buffer_info(
                frame.jt128_buffer, // ptr
                itemsize,           // itemsize
                py::format_descriptor<uint8_t>::format(), // format
                ndim,               // ndim
                shape,              // shape
                strides             // strides
            );
            // --- END OF FIX ---
        });
        */
        // --- END JT128 SUPPORT ---

    // --- Wrap the main HesaiLidarSdk class ---

    py::class_<HesaiLidarSdk_XYZICRT>(m, "HesaiLidarSdk_XYZICRT")
        // The SDK constructor unconditionally prints a version banner to
        // std::cout (hesai_lidar_sdk.hpp). It does not go through libhesai's
        // Logger, so DriverParam::log_Target cannot suppress it; divert
        // std::cout for the duration of the construction instead. Construction
        // only ever happens on the calling thread while it holds the GIL.
        .def(py::init([](bool quiet) {
                std::ostringstream sink;
                std::streambuf* saved = quiet ? std::cout.rdbuf(sink.rdbuf()) : nullptr;
                auto sdk = std::unique_ptr<HesaiLidarSdk_XYZICRT>(new HesaiLidarSdk_XYZICRT());
                if (quiet) std::cout.rdbuf(saved);
                return sdk;
            }), py::arg("quiet") = false)
        .def("Init", &HesaiLidarSdk_XYZICRT::Init, "Initialize the Lidar SDK")
        .def("Start", &HesaiLidarSdk_XYZICRT::Start, "Start the SDK processing threads")
        .def("Stop", &HesaiLidarSdk_XYZICRT::Stop, py::call_guard<py::gil_scoped_release>(), "Stop the SDK and clean up")

        // Wrap the point cloud callback registration
        .def("RegRecvCallback",
            [](HesaiLidarSdk_XYZICRT &sdk, std::function<void(LidarDecodedFrame_XYZICRT*)> callback) { // 1. Changed to pointer
                sdk.RegRecvCallback(
                    [callback](const LidarDecodedFrame_XYZICRT& frame) {
                        py::gil_scoped_acquire gil;
                        // 2. Pass as a pointer. const_cast is needed so Python can access buffer properties.
                        callback(const_cast<LidarDecodedFrame_XYZICRT*>(&frame)); 
                    }
                );
            }, py::arg("callback"), "Register the point cloud callback")

        // --- IMU SUPPORT ---
        // Wrap the IMU data callback registration (overloaded method)
        .def("RegRecvCallback",
            [](HesaiLidarSdk_XYZICRT &sdk, std::function<void(const LidarImuData&)> callback) {
                sdk.RegRecvCallback(
                    [callback](const LidarImuData& imu_data) {
                        py::gil_scoped_acquire gil;
                        callback(imu_data);
                    }
                );
            }, py::arg("callback"), "Register the IMU data callback")
        // --- END IMU SUPPORT ---

        // Replicate the IsPlayEnded function from test.cc
        .def("IsPlayEnded", [](HesaiLidarSdk_XYZICRT &sdk) {
            if (sdk.lidar_ptr_) {
                return sdk.lidar_ptr_->IsPlayEnded();
            }
            return false;
        });

    // --- Wrap Utility Functions ---
    m.def("GetMicroTickCount", &GetMicroTickCount, "Get the current time in microseconds");

    // --- PTC Client ---
    py::class_<hesai::lidar::PtcClient>(m, "PtcClient")
        .def(py::init<const std::string&, uint16_t>(),
             py::arg("ip"), py::arg("port") = 9347)
        .def("wait_until_open",
             [](hesai::lidar::PtcClient& self, double timeout_s) {
                 WaitUntilOpen(self, timeout_s);
             },
             py::arg("timeout_s"))
        .def("is_open", &hesai::lidar::PtcClient::IsOpen)
        .def("get_return_mode",
             [](hesai::lidar::PtcClient& self) {
                 uint8_t mode = 0;
                 if (!self.GetReturnMode(mode)) {
                     throw std::runtime_error("GetReturnMode failed");
                 }
                 return static_cast<int>(mode);
             })
        .def("set_return_mode",
             [](hesai::lidar::PtcClient& self, uint8_t mode) {
                 if (!self.SetReturnMode(mode)) {
                     throw std::runtime_error("SetReturnMode failed");
                 }
             },
             py::arg("mode"))
        .def("get_ptp_lock_offset_us",
             [](hesai::lidar::PtcClient& self) {
                 uint16_t offset_us = 0;
                 if (self.GetPTPLockOffset(offset_us) != 0) {
                     throw std::runtime_error("GetPTPLockOffset failed");
                 }
                 return static_cast<int>(offset_us);
             })
        .def("set_ptp_lock_offset_us",
             [](hesai::lidar::PtcClient& self, uint16_t offset_us) {
                 if (!self.SetPTPLockOffset(offset_us)) {
                     throw std::runtime_error("SetPTPLockOffset failed");
                 }
             },
             py::arg("offset_us"))
        .def("get_spin_rate",
             [](hesai::lidar::PtcClient& self) {
                 uint16_t rpm = 0;
                 if (!self.GetSpinRate(rpm)) {
                     throw std::runtime_error("GetSpinRate failed");
                 }
                 return static_cast<int>(rpm);
             })
        .def("set_spin_speed",
             [](hesai::lidar::PtcClient& self, uint32_t rpm) {
                 if (!self.SetSpinSpeed(rpm)) {
                     throw std::runtime_error("SetSpinSpeed failed");
                 }
             },
             py::arg("rpm"))
        .def("get_point_cloud_config",
             [](hesai::lidar::PtcClient& self) {
                 uint8_t ultra_precise = 0;
                 uint8_t filter = 0;
                 if (!self.GetPointCloudConfig(ultra_precise, filter)) {
                     throw std::runtime_error("GetPointCloudConfig failed");
                 }
                 return py::make_tuple(static_cast<int>(ultra_precise),
                                       static_cast<int>(filter));
             })
        .def("set_point_cloud_config_selective",
             [](hesai::lidar::PtcClient& self, uint8_t ultra_precise,
                uint8_t filter) {
                 if (!self.SetPointCloudConfigSelective(ultra_precise, filter)) {
                     throw std::runtime_error(
                         "SetPointCloudConfigSelective failed");
                 }
             },
             py::arg("ultra_precise"), py::arg("filter"))
        .def("get_lidar_ptp_status",
             [](hesai::lidar::PtcClient& self) {
                 uint8_t status = 0;
                 if (!self.GetLidarPtpStatus(status)) {
                     throw std::runtime_error("GetLidarPtpStatus failed");
                 }
                 return static_cast<int>(status);
             })
        .def("get_ptp_diagnostics_raw",
             [](hesai::lidar::PtcClient& self, uint8_t query_type) {
                 hesai::lidar::u8Array_t dataOut;
                 if (self.GetPTPDiagnostics(dataOut, query_type) != 0) {
                     throw std::runtime_error("GetPTPDiagnostics failed");
                 }
                 return U8ArrayToBytes(dataOut);
             },
             py::arg("query_type") = 1)
        .def("get_config_info_raw",
             [](hesai::lidar::PtcClient& self) {
                 hesai::lidar::u8Array_t dataOut;
                 if (self.GetConfigInfoRaw(dataOut) != 0) {
                     throw std::runtime_error("GetConfigInfoRaw failed");
                 }
                 return U8ArrayToBytes(dataOut);
             })
        .def("get_inventory_info_raw",
             [](hesai::lidar::PtcClient& self) {
                 hesai::lidar::u8Array_t dataIn;
                 hesai::lidar::u8Array_t dataOut;
                 int ret = self.QueryCommand(dataIn, dataOut, 0x07);
                 if (ret != 0) {
                     throw std::runtime_error("GetInventoryInfo failed");
                 }
                 return U8ArrayToBytes(dataOut);
             })
        .def("query_command",
             [](hesai::lidar::PtcClient& self, uint8_t cmd, const std::string& data_in) {
                 hesai::lidar::u8Array_t in_bytes(data_in.begin(), data_in.end());
                 hesai::lidar::u8Array_t out_bytes;
                 int ret = self.QueryCommand(in_bytes, out_bytes, cmd);
                 if (ret != 0) {
                     throw std::runtime_error("QueryCommand failed");
                 }
                 return U8ArrayToBytes(out_bytes);
             },
             py::arg("cmd"), py::arg("data_in") = "")
        .def("set_upgrade_percent_callback",
             [](hesai::lidar::PtcClient& self, py::function callback) {
                 self.SetUpgradePercentCallback(
                     [callback](float percent) {
                         py::gil_scoped_acquire acquire;
                         callback(percent);
                     });
             },
             py::arg("callback"),
             "Register a callback invoked with upgrade progress percent (0-100)")
        .def("upgrade_lidar_patch",
             [](hesai::lidar::PtcClient& self, const std::string& file_path,
                uint32_t cmd_id, int is_extern) {
                 int ret;
                 {
                     py::gil_scoped_release release;
                     ret = self.UpgradeLidarPatch(file_path, cmd_id, is_extern);
                 }
                 if (ret != 0) {
                     throw std::runtime_error("UpgradeLidarPatch failed");
                 }
             },
             py::arg("file_path"),
             py::arg("cmd_id") = 0x83,
             py::arg("is_extern") = 0,
             "Upload firmware patch via PTC 0x83 (or extern subcmd)")
        .def("get_upgrade_progress",
             [](hesai::lidar::PtcClient& self) {
                 return py::make_tuple(
                     self.GetUpgradeCurrentPacket(),
                     self.GetUpgradeTotalPackets(),
                     self.GetUpgradeStatus());
             },
             "Return (current_packet, total_packets, status)");


    m.def("ptc_reachable",
          [](const std::string& ip, uint16_t port, double timeout_s) -> bool {
              try {
                  auto client = ConnectPtc(ip, port, timeout_s);
                  return client->IsOpen();
              } catch (const std::exception&) {
                  return false;
              }
          },
          py::arg("ip"),
          py::arg("port") = 9347,
          py::arg("timeout_s") = 2.0,
          "Return True if PTC TCP port accepts a connection");

    m.def("download_calibration_bytes", [](const std::string& ip_address, int ptc_port) -> py::bytes {
        hesai::lidar::PtcClient ptc_client(ip_address, ptc_port);
        
        // Wait for connection to establish
        int timeouts = 0;
        while(!ptc_client.IsOpen() && timeouts < 50) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            timeouts++;
        }
        
        if (!ptc_client.IsOpen()) {
            throw std::runtime_error("Failed to connect to PTC client at " + ip_address + ":" + std::to_string(ptc_port));
        }

        hesai::lidar::u8Array_t dataIn;
        hesai::lidar::u8Array_t dataOut;
        uint8_t ptc_cmd = 0x05; // 0x05 is GetLidarCalibration

        int ret = ptc_client.QueryCommand(dataIn, dataOut, ptc_cmd);
        if (ret != 0) {
            throw std::runtime_error("Failed to query calibration data. Return code: " + std::to_string(ret));
        }

        return py::bytes(reinterpret_cast<const char*>(dataOut.data()), dataOut.size());
    }, py::arg("ip_address"), py::arg("ptc_port") = 9347, "Download lidar calibration file via PTC client");

}