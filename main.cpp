#include <gflags/gflags.h>
#include <functional>
#include <iostream>
#include <fstream>
#include <random>
#include <memory>
#include <chrono>
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <atomic>

#ifndef _WIN32
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#endif

#include <inference_engine.hpp>
#include <samples/ocv_common.hpp>
#include <samples/slog.hpp>
#include <samples/args_helper.hpp>
#include <ie_iextension.h>
#include <ext_list.hpp>

#include "security_barrier_camera.hpp"

#include "multiwii.h"
#include "multiwii.hpp"
#include "timer.hpp"
#include "pid.h"

using namespace InferenceEngine;

bool ParseAndCheckCommandLine(int argc, char *argv[]) {
    // ---------------------------Parsing and validation of input args--------------------------------------

    gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
    if (FLAGS_h) {
        showUsage();
        return false;
    }

    slog::info << "Parsing input parameters" << slog::endl;

    if (FLAGS_i.empty()) {
        throw std::logic_error("Parameter -i is not set");
    }

    if (FLAGS_m.empty()) {
        throw std::logic_error("Parameter -m is not set");
    }

    if (FLAGS_display_resolution.find("x") == std::string::npos) {
        throw std::logic_error("Incorrect format of -displayresolution parameter. Correct format is  \"width x height\". For example \"1920x1080\"");
    }

    return true;
}

void frameToBlob(const cv::Mat& frame, InferRequest::Ptr& inferRequest, const std::string& inputName)
{
    if (FLAGS_auto_resize)
    {
        /* Just set input blob containing read image. Resize and layout conversion will be done automatically */
        inferRequest->SetBlob(inputName, wrapMat2Blob(frame));
    }
    else
    {
        /* Resize and copy data from the image to the input blob */
        Blob::Ptr frameBlob = inferRequest->GetBlob(inputName);
        matU8ToBlob<uint8_t>(frame, frameBlob);
    }
}

#if defined(_WIN32)
bool launchDebugger()
{
    // Get System directory, typically c:\windows\system32
    std::wstring systemDir(MAX_PATH + 1, '\0');
    UINT nChars = GetSystemDirectoryW(&systemDir[0], systemDir.length());
    if (nChars == 0) return false; // failed to get system directory
    systemDir.resize(nChars);

    // Get process ID and create the command line
    DWORD pid = GetCurrentProcessId();
    std::wostringstream s;
    s << systemDir << L"\\vsjitdebugger.exe -p " << pid;
    std::wstring cmdLine = s.str();

    // Start debugger process
    STARTUPINFOW si;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);

    PROCESS_INFORMATION pi;
    ZeroMemory(&pi, sizeof(pi));

    if (!CreateProcessW(NULL, &cmdLine[0], NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi)) return false;

    // Close debugger process handles to eliminate resource leak
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);

    // Wait for the debugger to attach
    while (!IsDebuggerPresent()) Sleep(100);

    // Stop execution so the debugger can take over
    DebugBreak();
    return true;
}
#endif

static std::vector<Color> colors = {
    {255, 255, 255},
    {155, 155, 155},
    {229, 255, 23},
    {255, 23, 23},
    {25, 195, 0},
    {8, 0, 255},
    {0, 0, 0}
};

struct Detection
{
    int label;
    float confidence;

    float xmin;
    float ymin;
    float xmax;
    float ymax;

    float size;
};

void msp_runner(std::string serial_port)
{
    ceSerial* serial = new ceSerial(serial_port, 115200, 8, 'N', 1);

    auto serial_code = serial->Open();
    if (serial_code != 0)
    {
        printf("Unable to open serial port: %li\n", serial_code);
        return;
    }

    uint16_t throttle = 0;

    bool in_rx_recovery = true;

    while (true)
    {
        Msp::MspStatusEx* rcv = Msp::receive_parameters<Msp::MspStatusEx>(serial, Msp::MspCommand::STATUS_EX);
        printf("\nflight_mode_flags: %u, average_system_load_percent: %u, arming_flags: %u \n", rcv->initial_flight_mode_flags, rcv->average_system_load_percent, rcv->arming_flags);

        uint32_t arming_flags = rcv->arming_flags;
        for (unsigned int i = 0; i < 22; i++)
        {
            if (arming_flags & (1u << i))
                slog::info << " " << (i + 1) << ": " << ((arming_flags & (1u << i)) == (1u << i));
        }

        if (in_rx_recovery)
        {
            Msp::MspReceiver reset_params = {
                .roll = 1500,
                .pitch = 1500,
                .throttle = 900,
                .yaw = 1500,
                .aux_1 = 1000,
                .aux_2 = 1000,
                .aux_3 = 1000
            };
            Msp::send_command<Msp::MspReceiver>(serial, Msp::MspCommand::SET_RAW_RC, &reset_params);
        }
        else
        {
            uint16_t new_throttle = throttle + 900;

            Msp::MspReceiver params = {
                .roll = 1500,
                .pitch = 1500,
                .throttle = new_throttle,
                .yaw = 1500,
                .aux_1 = 1000,
                .aux_2 = 1000,
                .aux_3 = 2000
            };
            Msp::send_command<Msp::MspReceiver>(serial, Msp::MspCommand::SET_RAW_RC, &params);

            throttle = (throttle + 1) % 200;
        }

        // If the drone loses RX and it's still armed then it will go into failsafe mode
        // To get out of failsafe, we need to reset the throttle and disable the arming switch
        // for 2 seconds before trying to do anything else

        in_rx_recovery =
            (arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_RX_FAILSAFE)
            || (arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_FAILSAFE);

#if defined(_WIN32)
        Sleep(200);
#else
        usleep(200);
#endif
    }
}

void detection_runner(int argc, char* argv[])
{
    try {
        /** This demo covers 3 certain topologies and cannot be generalized **/
        slog::info << "InferenceEngine: " << GetInferenceEngineVersion() << slog::endl;

        // ------------------------------ Parsing and validation of input args ---------------------------------
        if (!ParseAndCheckCommandLine(argc, argv))
        {
            return;
        }

        cv::VideoCapture cap;
        if (!((FLAGS_i == "cam") ? cap.open(0) : cap.open(FLAGS_i.c_str())))
        {
            throw std::logic_error("Cannot open input file or camera: " + FLAGS_i);
        }

        const size_t width = (size_t)cap.get(cv::CAP_PROP_FRAME_WIDTH);
        const size_t height = (size_t)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

        cv::Mat curr_frame;  cap >> curr_frame;
        cv::Mat next_frame;

        if (!cap.grab())
        {
            throw std::logic_error("This demo supports only video (or camera) inputs !!! "
                "Failed getting next frame from the " + FLAGS_i);
        }

        slog::info << "Loading Inference Engine" << slog::endl;
        Core ie;

        slog::info << "Device info: " << slog::endl;
        std::cout << ie.GetVersions(FLAGS_d);

        /** Loading default extensions **/
        if (FLAGS_d.find("CPU") != std::string::npos)
        {
            /**
             * cpu_extensions library is compiled from "extension" folder containing
             * custom MKLDNNPlugin layer implementations. These layers are not supported
             * by mkldnn, but they can be useful for inferring custom topologies.
            **/
            ie.AddExtension(std::make_shared<Extensions::Cpu::CpuExtensions>(), "CPU");
        }

        if (!FLAGS_l.empty())
        {
            // CPU(MKLDNN) extensions are loaded as a shared library and passed as a pointer to base extension
            IExtensionPtr extension_ptr = make_so_pointer<IExtension>(FLAGS_l.c_str());
            ie.AddExtension(extension_ptr, "CPU");
        }
        if (!FLAGS_c.empty())
        {
            // clDNN Extensions are loaded from an .xml description and OpenCL kernel files
            ie.SetConfig({ {PluginConfigParams::KEY_CONFIG_FILE, FLAGS_c} }, "GPU");
        }

        /** Per layer metrics **/
        if (FLAGS_pc) {
            ie.SetConfig({ { PluginConfigParams::KEY_PERF_COUNT, PluginConfigParams::YES } });
        }

        // --------------------------- 2. Read IR Generated by ModelOptimizer (.xml and .bin files) ------------

        slog::info << "Loading network files" << slog::endl;

        CNNNetReader netReader;
        std::string binFileName = fileNameNoExt(FLAGS_m) + ".bin";

        netReader.ReadNetwork(FLAGS_m);
        netReader.getNetwork().setBatchSize(1);
        netReader.ReadWeights(binFileName);

        /** SSD-based network should have one input and one output **/
        // --------------------------- 3. Configure input & output ---------------------------------------------
        // --------------------------- Prepare input blobs -----------------------------------------------------
        slog::info << "Checking that the inputs are as the demo expects" << slog::endl;

        InputsDataMap inputInfo(netReader.getNetwork().getInputsInfo());
        auto inputName = inputInfo.begin()->first;
        auto& input = inputInfo.begin()->second;

        input->setPrecision(Precision::U8);

        if (FLAGS_auto_resize)
        {
            input->getPreProcess().setResizeAlgorithm(ResizeAlgorithm::RESIZE_BILINEAR);
            input->getInputData()->setLayout(Layout::NHWC);
        }
        else
        {
            input->getInputData()->setLayout(Layout::NCHW);
        }

        OutputsDataMap outputInfo(netReader.getNetwork().getOutputsInfo());
        auto outputName = outputInfo.begin()->first;
        auto& output = outputInfo.begin()->second;

        output->setPrecision(Precision::FP32);
        output->setLayout(Layout::NCHW);

        const SizeVector outputDims = output->getTensorDesc().getDims();
        const int maxProposalCount = outputDims[2];
        const int objectSize = outputDims[3];

        assert(objectSize == 7);
        assert(outputDims.size() == 4);

        // --------------------------- 4. Loading model to the device ------------------------------------------
        slog::info << "Loading model to the device" << slog::endl;
        ExecutableNetwork network = ie.LoadNetwork(netReader.getNetwork(), FLAGS_d);

        // --------------------------- 5. Create infer request -------------------------------------------------
        InferRequest::Ptr async_infer_request_curr = network.CreateInferRequestPtr();
        InferRequest::Ptr async_infer_request_next = network.CreateInferRequestPtr();


        // --------------------------- 2. Read IR Generated by ModelOptimizer (.xml and .bin files) ------------

        slog::info << "Loading network files" << slog::endl;

        CNNNetReader attr_netReader;
        std::string attr_binFileName = "D:\\Git\\drone-c++\\Security\\object_attributes\\vehicle\\resnet10_update_1\\dldt\\FP16\\vehicle-attributes-recognition-barrier-0039.bin";

        attr_netReader.ReadNetwork("D:\\Git\\drone-c++\\Security\\object_attributes\\vehicle\\resnet10_update_1\\dldt\\FP16\\vehicle-attributes-recognition-barrier-0039.xml");
        attr_netReader.getNetwork().setBatchSize(1);
        attr_netReader.ReadWeights(attr_binFileName);

        /** SSD-based network should have one input and one output **/
        // --------------------------- 3. Configure input & output ---------------------------------------------
        // --------------------------- Prepare input blobs -----------------------------------------------------
        slog::info << "Checking that the inputs are as the demo expects" << slog::endl;

        InputsDataMap attr_inputInfo(attr_netReader.getNetwork().getInputsInfo());
        auto attr_inputName = attr_inputInfo.begin()->first;
        auto& attr_input = attr_inputInfo.begin()->second;

        attr_input->setPrecision(Precision::U8);

        if (FLAGS_auto_resize)
        {
            attr_input->getPreProcess().setResizeAlgorithm(ResizeAlgorithm::RESIZE_BILINEAR);
            attr_input->getInputData()->setLayout(Layout::NHWC);
        }
        else
        {
            attr_input->getInputData()->setLayout(Layout::NCHW);
        }

        OutputsDataMap attr_outputInfo(attr_netReader.getNetwork().getOutputsInfo());

        DataPtr& color_output = attr_outputInfo.at("color");
        color_output->setPrecision(Precision::FP32);
        color_output->setLayout(Layout::NCHW);

        DataPtr& type_output = attr_outputInfo.at("type");
        type_output->setPrecision(Precision::FP32);
        type_output->setLayout(Layout::NCHW);

        /*
        const SizeVector attr_outputDims = attr_output->getTensorDesc().getDims();
        const int attr_maxProposalCount = attr_outputDims[2];
        const int attr_objectSize = attr_outputDims[3];

        assert(attr_objectSize == 1);
        assert(attr_outputDims.size() == 4);*/

        // --------------------------- 4. Loading model to the device ------------------------------------------
        slog::info << "Loading model to the device" << slog::endl;
        ExecutableNetwork attr_network = ie.LoadNetwork(attr_netReader.getNetwork(), FLAGS_d);

        // --------------------------- 5. Create infer request -------------------------------------------------
        InferRequest::Ptr attr_infer_request = attr_network.CreateInferRequestPtr();



        //// --------------------------- 6. Do inference ---------------------------------------------------------
        slog::info << "Start inference " << slog::endl;

        bool isLastFrame = false;
        bool isAsyncMode = true;  // execution is always started using SYNC mode
        bool isModeChanged = false;  // set to TRUE when execution mode is changed (SYNC<->ASYNC)

        bool isTrackingCar = false;
        int iteration = -1;

        PID pid_x(-100, 100, 0.1, 0.01, 0.5);
        PID pid_y(-100, 100, 0.1, 0.01, 0.5);

        cv::Point2f prevCenter(-1, -1);

        typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
        auto total_t0 = std::chrono::high_resolution_clock::now();
        auto wallclock = std::chrono::high_resolution_clock::now();
        double ocv_decode_time = 0, ocv_render_time = 0;

        slog::info << "To close the application, press 'CTRL+C' here or switch to the output window and press ESC key" << slog::endl;
        slog::info << "To switch between sync/async modes, press TAB key in the output window" << slog::endl;
        while (true)
        {
            auto t0 = std::chrono::high_resolution_clock::now();
            // Here is the first asynchronous point:
            // in the async mode we capture frame to populate the NEXT infer request
            // in the regular mode we capture frame to the CURRENT infer request
            if (!cap.read(next_frame))
            {
                if (next_frame.empty())
                {
                    isLastFrame = true;  // end of video file
                }
                else
                {
                    throw std::logic_error("Failed to get frame from cv::VideoCapture");
                }
            }

            auto t1 = std::chrono::high_resolution_clock::now();
            ocv_decode_time = std::chrono::duration_cast<ms>(t1 - t0).count();

            t0 = std::chrono::high_resolution_clock::now();
            // Main sync point:
            // in the truly Async mode we start the NEXT infer request, while waiting for the CURRENT to complete
            // in the regular mode we start the CURRENT request and immediately wait for it's completion
            if (isAsyncMode)
            {
                if (isModeChanged)
                {
                    frameToBlob(curr_frame, async_infer_request_curr, inputName);
                    async_infer_request_curr->StartAsync();
                }
                if (!isLastFrame)
                {
                    frameToBlob(next_frame, async_infer_request_next, inputName);
                    async_infer_request_next->StartAsync();
                }
            }
            else if (!isModeChanged)
            {
                frameToBlob(curr_frame, async_infer_request_curr, inputName);
                async_infer_request_curr->StartAsync();
            }

            if (async_infer_request_curr->Wait(IInferRequest::WaitMode::RESULT_READY) == OK)
            {
                t1 = std::chrono::high_resolution_clock::now();
                ms detection_time = std::chrono::duration_cast<ms>(t1 - t0);

                t0 = std::chrono::high_resolution_clock::now();
                ms wall = std::chrono::duration_cast<ms>(t0 - wallclock);
                wallclock = t0;

                cv::rectangle(curr_frame, cv::Point2f(0, 0), cv::Point2f(480, 68), cv::Scalar(222, 133, 31), -1);

                t0 = std::chrono::high_resolution_clock::now();
                std::ostringstream out;
                out << "OpenCV cap/render time: " << std::fixed << std::setprecision(2)
                    << (ocv_decode_time + ocv_render_time) << " ms";
                cv::putText(curr_frame, out.str(), cv::Point2f(0, 25), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 255));
                out.str("");
                out << "Wallclock time " << (isAsyncMode ? "(TRUE ASYNC): " : "(SYNC, press Tab): ");
                out << std::fixed << std::setprecision(2) << wall.count() << " ms (" << 1000.f / wall.count() << " fps)";
                cv::putText(curr_frame, out.str(), cv::Point2f(0, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 255));
                if (!isAsyncMode)
                {  // In the true async mode, there is no way to measure detection time directly
                    out.str("");
                    out << "Detection time  : " << std::fixed << std::setprecision(2) << detection_time.count()
                        << " ms ("
                        << 1000.f / detection_time.count() << " fps)";
                    cv::putText(curr_frame, out.str(), cv::Point2f(0, 75), cv::FONT_HERSHEY_TRIPLEX, 0.5,
                        cv::Scalar(255, 255, 255));
                }

                // ---------------------------Process output blobs--------------------------------------------------
                // Processing results of the CURRENT request

                const float* detection_values = async_infer_request_curr->GetBlob(outputName)->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();

                std::vector<Detection> detections;

                for (int i = 0; i < maxProposalCount; i++)
                {
                    float image_id = detection_values[i * objectSize + 0];
                    if (image_id < 0) break;

                    auto confidence = detection_values[i * objectSize + 2];
                    if (confidence <= FLAGS_t) continue;

                    auto xmin = detection_values[i * objectSize + 3] * width;
                    auto ymin = detection_values[i * objectSize + 4] * height;
                    auto xmax = detection_values[i * objectSize + 5] * width;
                    auto ymax = detection_values[i * objectSize + 6] * height;

                    // TODO: Not sure how this can be negative? But sometimes it is...
                    if (xmin < 0 || ymin < 0) continue;

                    Detection detection =
                    {
                        .label = static_cast<int>(detection_values[i * objectSize + 1]),
                        .confidence = detection_values[i * objectSize + 2],

                        .xmin = xmin,
                        .ymin = ymin,
                        .xmax = xmax,
                        .ymax = ymax,

                        .size = (xmax - xmin) * (ymax - ymin)
                    };

                    detections.push_back(detection);

                    if (FLAGS_r)
                    {
                        std::cout << "[" << i << "," << detection.label << "] element, prob = " << detection.confidence <<
                            "    (" << detection.xmin << "," << detection.ymin << ")-(" << detection.xmax << "," << detection.ymax << ")"
                            << ((confidence > FLAGS_t) ? " WILL BE RENDERED!" : "") << std::endl;
                    }
                }

                if (detections.size() == 0)
                {
                    // LOST track of the car
                    slog::warn << "No vehicles found this frame, did we lose track of it?" << slog::endl;

                    continue;
                }

                auto biggest_vehicle_idx = std::max_element(detections.begin(), detections.end(),
                    [](Detection const& lhs, Detection const& rhs) { return lhs.size < rhs.size; }) - detections.begin();

                Detection detection = detections[biggest_vehicle_idx];

                auto xmin = detection.xmin;
                auto ymin = detection.ymin;
                auto xmax = detection.xmax;
                auto ymax = detection.ymax;

                iteration = (iteration + 1) % 10;

                // Only check the vehicle attributes every once in a while
                // Also re-check sooner if we aren't currently tracking a car
                if (iteration == 0 || !isTrackingCar)
                {
                    cv::Mat croppedImage = curr_frame(cv::Rect2f(xmin, ymin, xmax - xmin, ymax - ymin));
                    frameToBlob(croppedImage, attr_infer_request, "input");

                    attr_infer_request->StartAsync();

                    // Wait until the result is ready. This is blocking and will cause the thread to wait...
                    while (attr_infer_request->Wait(IInferRequest::WaitMode::RESULT_READY) != OK) {}

                    const float* color_detections = attr_infer_request->GetBlob("color")->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();
                    const float* type_detections = attr_infer_request->GetBlob("type")->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();

                    auto color_idx = std::max_element(color_detections, color_detections + 7) - color_detections;
                    auto type_idx = std::max_element(type_detections, type_detections + 4) - type_detections;

                    // COLOR: Red, TYPE: Car
                    isTrackingCar = (color_idx == 3 && type_idx == 0);
                }

                if (isTrackingCar)
                {
                    cv::Scalar outline_color(23, 23, 255);
                    cv::rectangle(curr_frame, cv::Point2f(xmin, ymin), cv::Point2f(xmax, ymax), outline_color);

                    cv::Point2f center((xmin + xmax) / 2, (ymin + ymax) / 2);
                    cv::Scalar center_color(0, 0, 0);

                    cv::rectangle(curr_frame, cv::Point2f(center.x - 1, center.y - 1), cv::Point2f(center.x + 1, center.y + 1), center_color, 2);

                    /*if (prevCenter.x != -1)
                    {
                        cv::Point2f dc = center - prevCenter;
                        cv::Point2f dv = dc / wall.count();

                        std::cout << "Wall time: " << wall.count() << ", dv: " << dv << std::endl;
                    }*/

                    //prevCenter = center;


                    auto adj_x = pid_x.calculate(wall.count(), width / 2, center.x);
                    auto adj_y = pid_y.calculate(wall.count(), height / 2, center.y);

                    std::cout << "Adj: " << cv::Point2f(adj_x, adj_y) << std::endl;
                }
            }
            cv::imshow("Detection results", curr_frame);

            t1 = std::chrono::high_resolution_clock::now();
            ocv_render_time = std::chrono::duration_cast<ms>(t1 - t0).count();

            if (isLastFrame)
            {
                break;
            }

            if (isModeChanged)
            {
                isModeChanged = false;
            }

            // Final point:
            // in the truly Async mode we swap the NEXT and CURRENT requests for the next iteration
            curr_frame = next_frame;
            next_frame = cv::Mat();
            if (isAsyncMode)
            {
                async_infer_request_curr.swap(async_infer_request_next);
            }

            const int key = cv::waitKey(1);
            if (27 == key)  // Esc
                break;
            if (9 == key)
            {  // Tab
                isAsyncMode ^= true;
                isModeChanged = true;
            }
        }
        // -----------------------------------------------------------------------------------------------------
        auto total_t1 = std::chrono::high_resolution_clock::now();
        ms total = std::chrono::duration_cast<ms>(total_t1 - total_t0);
        std::cout << "Total Inference time: " << total.count() << std::endl;

        /** Show performace results **/
        if (FLAGS_pc)
        {
            printPerformanceCounts(*async_infer_request_curr, std::cout, getFullDeviceName(ie, FLAGS_d));
        }
    }
    catch (const std::exception& error) {
        slog::err << error.what() << slog::endl;
        return;
    }
    catch (...) {
        slog::err << "Unknown/internal exception happened." << slog::endl;
        return;
    }

    slog::info << "Execution successful" << slog::endl;
}

int main(int argc, char *argv[])
{
    setbuf(stdout, NULL);

    std::thread msp_runner_thread(msp_runner, argv[1]);
    //std::thread detection_runner_thread(detection_runner, argc, argv);

    msp_runner_thread.join();
    //detection_runner_thread.join();

    return 0;
}
