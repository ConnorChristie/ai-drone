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
#include <regex>

#ifndef _WIN32
#include <signal.h>
#include <cstdlib>
#endif

#include <inference_engine.hpp>
#include <ie_iextension.h>

#ifdef HAS_CPU_EXTENSION
#include <ext_list.hpp>
#endif

#include "ocv_common.hpp"
#include "slog.hpp"

#include "drone.hpp"
#include "timer.hpp"
#include "utils.hpp"
#include "multiwii.h"
#include "pid.h"
#include "drone_controller.h"

#include "blockingconcurrentqueue.h"
#include "remote_server.hpp"

using namespace InferenceEngine;

bool ParseAndCheckCommandLine(int argc, char *argv[])
{
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

    if (FLAGS_ma.empty()) {
        throw std::logic_error("Parameter -ma is not set");
    }

    if (FLAGS_msp_port_name.empty()) {
        throw std::logic_error("Parameter -msp_port_name is not set");
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

cv::Mat capture_frame(cv::VideoCapture cap, const size_t width, const size_t height, bool* isLastFrame)
{
    cv::Mat temp_frame;

    if (!cap.read(temp_frame))
    {
        if (temp_frame.empty())
        {
            *isLastFrame = true;  // end of video file
        }
        else
        {
            throw std::logic_error("Failed to get frame from cv::VideoCapture");
        }
    }

    cv::Mat yuv(height, width, CV_8UC2, temp_frame.data);
    cv::Mat rgb(height, width, CV_8UC3);

    cv::cvtColor(yuv, rgb, cv::COLOR_YUV2BGRA_YUY2);
    yuv.release();

    return rgb;
}

void detection_runner(DroneController* drone_controller)
{
    const std::regex cam_regex("cam([0-9]+)");
    std::smatch cam_match;

    try
    {
        slog::info << "InferenceEngine: " << GetInferenceEngineVersion() << slog::endl;

        cv::VideoCapture cap;

        if (std::regex_match(FLAGS_i, cam_match, cam_regex))
        {
            auto cam_index = stoi(cam_match[1].str());
            cap.open(cam_index, cv::CAP_V4L2);

            cap.set(cv::CAP_PROP_MODE, 3);
        }
        else
        {
            cap.open(FLAGS_i.c_str());
        }

        const size_t width = (size_t)cap.get(cv::CAP_PROP_FRAME_WIDTH);
        const size_t height = (size_t)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

        bool isLastFrame = false;
        bool isAsyncMode = true;
        bool isModeChanged = false;  // set to TRUE when execution mode is changed (SYNC<->ASYNC)

        bool isTrackingCar = false;
        int relativeIteration = -1;

        cv::Mat curr_frame = capture_frame(cap, width, height, &isLastFrame);
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

        #ifdef HAS_CPU_EXTENSION
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
        #endif

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

        slog::info << "Loading vehicle network files" << slog::endl;

        CNNNetReader netReader;

        netReader.ReadNetwork(FLAGS_m);
        netReader.getNetwork().setBatchSize(1);
        netReader.ReadWeights(fileNameNoExt(FLAGS_m) + ".bin");

        // --------------------------- 3. Configure input & output ---------------------------------------------

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
        slog::info << "Loading vehicle detection model to the device" << slog::endl;
        ExecutableNetwork network = ie.LoadNetwork(netReader.getNetwork(), FLAGS_d);

        // --------------------------- 5. Create infer request -------------------------------------------------
        InferRequest::Ptr async_infer_request_curr = network.CreateInferRequestPtr();
        InferRequest::Ptr async_infer_request_next = network.CreateInferRequestPtr();

        // --------------------------- 2. Read IR Generated by ModelOptimizer (.xml and .bin files) ------------

        slog::info << "Loading vehicle attributes network files" << slog::endl;

        CNNNetReader attr_netReader;

        attr_netReader.ReadNetwork(FLAGS_ma);
        attr_netReader.getNetwork().setBatchSize(1);
        attr_netReader.ReadWeights(fileNameNoExt(FLAGS_ma) + ".bin");

        // --------------------------- 3. Configure input & output ---------------------------------------------
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

        // --------------------------- 4. Loading model to the device ------------------------------------------
        slog::info << "Loading vehicle attributes model to the device" << slog::endl;
        ExecutableNetwork attr_network = ie.LoadNetwork(attr_netReader.getNetwork(), FLAGS_d);

        // --------------------------- 5. Create infer request -------------------------------------------------
        InferRequest::Ptr attr_infer_request = attr_network.CreateInferRequestPtr();

        // --------------------------- 6. Do inference ---------------------------------------------------------
        slog::info << "Start inference " << slog::endl;

        typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
        auto total_t0 = std::chrono::high_resolution_clock::now();
        auto wallclock = std::chrono::high_resolution_clock::now();
        double ocv_decode_time = 0, ocv_render_time = 0;

        // drone_controller->init(width, height);

        slog::info << "To close the application, press 'CTRL+C' here or switch to the output window and press ESC key" << slog::endl;
        slog::info << "To switch between sync/async modes, press TAB key in the output window" << slog::endl;
        while (true)
        {
            auto t0 = std::chrono::high_resolution_clock::now();

            // Here is the first asynchronous point:
            // in the async mode we capture frame to populate the NEXT infer request
            // in the regular mode we capture frame to the CURRENT infer request
            next_frame = capture_frame(cap, width, height, &isLastFrame);

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
                out << "Wallclock time: ";
                out << std::fixed << std::setprecision(2) << wall.count() << " ms (" << 1000.f / wall.count() << " fps)";

                std::cout << out.str() << std::endl;
                cv::putText(curr_frame, out.str(), cv::Point2f(0, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 255));

                // In the true async mode, there is no way to measure detection time directly
                if (!isAsyncMode)
                {
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
                    // Lost track of the car
                    slog::warn << "No vehicles found this frame, did we lose track of it?" << slog::endl;
                }
                else
                {
                    auto biggest_vehicle_idx = std::max_element(detections.begin(), detections.end(),
                        [](Detection const& lhs, Detection const& rhs) { return lhs.size < rhs.size; }) - detections.begin();

                    Detection detection = detections[biggest_vehicle_idx];

                    auto xmin = detection.xmin;
                    auto ymin = detection.ymin;
                    auto xmax = detection.xmax;
                    auto ymax = detection.ymax;

                    relativeIteration = (relativeIteration + 1) % 10;

                    // Only check the vehicle attributes every once in a while
                    // Also re-check sooner if we aren't currently tracking a car
                    if (relativeIteration == 0 || !isTrackingCar)
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

                    auto dt = wall.count() / 1000;

                    if (isTrackingCar)
                    {
                        cv::Scalar outline_color(23, 23, 255);
                        cv::rectangle(curr_frame, cv::Point2f(xmin, ymin), cv::Point2f(xmax, ymax), outline_color);

                        cv::Point2f center((xmin + xmax) / 2, (ymin + ymax) / 2);
                        cv::Scalar center_color(0, 0, 0);

                        cv::rectangle(curr_frame, cv::Point2f(center.x - 1, center.y - 1), cv::Point2f(center.x + 1, center.y + 1), center_color, 2);

                        drone_controller->update_pid(dt, center.x, center.y, detection.size);
                    }
                    else
                    {
                        drone_controller->update_pid(dt, -1, -1, -1);
                    }
                }
            }

            cv::imshow("Detection results", curr_frame);
            frame_queue.enqueue(curr_frame);

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
            curr_frame.release();
            curr_frame = next_frame;
            next_frame = cv::Mat();
            if (isAsyncMode)
            {
                async_infer_request_curr.swap(async_infer_request_next);
            }

            const int key = cv::waitKey(1);
            if (27 == key) // Esc
                break;
            if (9 == key)  // Tab
            {
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
    try
    {
        setbuf(stdout, NULL);

        if (!ParseAndCheckCommandLine(argc, argv))
        {
            return -1;
        }

        DroneController drone_controller(FLAGS_msp_port_name);

        std::thread web_server(&run_web_server);
        std::thread drone_controller_thread(&DroneController::run, &drone_controller);
        std::thread detection_runner_thread(detection_runner, &drone_controller);

        web_server.join();
        drone_controller_thread.join();
        detection_runner_thread.join();
    }
    catch (const std::exception& error) {
        slog::err << error.what() << slog::endl;
        return -1;
    }
    catch (...) {
        slog::err << "Unknown/internal exception happened." << slog::endl;
        return -1;
    }

    return 0;
}
