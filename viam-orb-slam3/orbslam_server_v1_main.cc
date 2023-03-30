// This is an experimental integration of orbslam into RDK.
#include "orbslam_server_v1.h"
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>
#include <signal.h>

#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <chrono>

using namespace boost::filesystem;
using grpc::Server;
using grpc::ServerBuilder;

void exit_loop_handler(int s) {
    BOOST_LOG_TRIVIAL(info) << "Finishing session";
    viam::b_continue_session = false;
}

int main(int argc, char **argv) {
    struct sigaction sigHandler;

    sigHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigHandler.sa_mask);
    sigHandler.sa_flags = 0;

    sigaction(SIGTERM, &sigHandler, NULL);
    sigaction(SIGINT, &sigHandler, NULL);

    static_assert((sizeof(float) == 4) && (CHAR_BIT == 8) && (sizeof(int) == 4),
                  "32 bit float & 8 bit char & 32 bit int is assumed");

    viam::SLAMServiceImpl slamService;
    slamService.SetSlam(nullptr);

    try {
        const vector<string> args(argv + 1, argv + argc);
        viam::utils::ParseAndValidateArguments(args, slamService);
    } catch (const runtime_error &error) {
        BOOST_LOG_TRIVIAL(fatal) << error.what();
        return 1;
    }

    // setup the SLAM server
    ServerBuilder builder;

    std::unique_ptr<int> selected_port = std::make_unique<int>(0);
    builder.AddListeningPort(slamService.slam_port,
                             grpc::InsecureServerCredentials(),
                             selected_port.get());

    // Increasing the gRPC max message size from the default value of 4MB to
    // 32MB, to match the limit that is set in RDK. This is necessary for
    // transmitting large pointclouds.
    builder.SetMaxSendMessageSize(viam::maximumGRPCByteLimit);
    builder.RegisterService(&slamService);

    // Start the SLAM gRPC server
    std::unique_ptr<Server> server(builder.BuildAndStart());

    // This log line is needed by rdk to get the port.
    BOOST_LOG_TRIVIAL(info) << "Server listening on " << *selected_port;

    // Determine which settings file to use(.yaml)
    const path myPath(slamService.path_to_settings);
    path latest;
    std::time_t latest_tm = 0;
    for (auto &&entry :
         boost::make_iterator_range(directory_iterator(myPath), {})) {
        path p = entry.path();

        if (is_regular_file(p) && p.extension() == ".yaml") {
            std::time_t timestamp = last_write_time(p);
            if (timestamp > latest_tm) {
                if (!slamService.use_live_data ||
                    p.stem().string().find(slamService.camera_name) !=
                        string::npos) {
                    latest = p;
                    latest_tm = timestamp;
                }
            }
        }
    }
    if (latest.empty()) {
        BOOST_LOG_TRIVIAL(fatal)
            << "No correctly formatted .yaml file found, Expected:\n"
               "{sensor}_data_{dateformat}.yaml";
        return 1;
    }

    // report the current yaml file check if it matches our format
    const string myYAML = latest.stem().string();
    BOOST_LOG_TRIVIAL(debug) << "Our yaml file: " << myYAML;

    string full_path_to_settings =
        slamService.path_to_settings + "/" + latest.filename().string();
    if (!slamService.use_live_data) {
        if (myYAML.find("_data_") != string::npos)
            slamService.camera_name = myYAML.substr(0, myYAML.find("_data_"));
        else {
            BOOST_LOG_TRIVIAL(fatal)
                << "No correctly formatted .yaml file found, Expected:\n"
                   "{sensor}_data_{dateformat}.yaml\n"
                   "as most the recent config in directory";
            return 1;
        }
    }

    // Grab timestamp from yaml
    slamService.yamlTime = viam::utils::ReadTimeFromTimestamp(
        myYAML.substr(myYAML.find("_data_") + viam::filenamePrefixLength));
    BOOST_LOG_TRIVIAL(debug)
        << "The time from our config is: " << slamService.yamlTime
        << " seconds";

    // Find the vocabulary file.
    path pathToVocabFromConfig(slamService.path_to_vocab);
    if (exists(pathToVocabFromConfig)) {
        BOOST_LOG_TRIVIAL(debug) << "Using vocabulary file in config folder";
    } else {
        auto programLocation = boost::dll::program_location();
        auto relativePathToVocab = programLocation.parent_path().parent_path();
        relativePathToVocab.append("share/orbslam/Vocabulary/ORBvoc.txt");
        if (exists(relativePathToVocab)) {
            BOOST_LOG_TRIVIAL(debug)
                << "Using vocabulary file from relative path";
            slamService.path_to_vocab = relativePathToVocab.string();
        } else {
            BOOST_LOG_TRIVIAL(fatal)
                << "No vocabulary file found, looked in "
                << pathToVocabFromConfig << " and " << relativePathToVocab;
            return 1;
        }
    }

    // Start SLAM
    SlamPtr SLAM = nullptr;
    ORB_SLAM3::System::eSensor slam_mode;
    if (slamService.slam_mode == "rgbd") {
        BOOST_LOG_TRIVIAL(info) << "RGBD selected";
        slam_mode = ORB_SLAM3::System::RGBD;
    } else if (slamService.slam_mode == "mono") {
        BOOST_LOG_TRIVIAL(info) << "Mono selected";
        slam_mode = ORB_SLAM3::System::MONOCULAR;
    }
    // Create SLAM system. It initializes all system threads and gets ready
    // to process frames.
    SLAM = std::make_unique<ORB_SLAM3::System>(
        slamService.path_to_vocab, full_path_to_settings, slam_mode,
        slamService.local_viewer_flag, 0);

    if (slamService.pure_localization_mode) {
        BOOST_LOG_TRIVIAL(info) << "Setting SLAM to localization mode";
        SLAM->ActivateLocalizationMode();

        // Set current map for localization to the saved map with largest
        // number of points
        auto allMaps = SLAM->GetAtlas()->GetAllMaps();
        ORB_SLAM3::Map *largestMap;
        auto nPoints = 0;
        for (size_t i = 0; i < allMaps.size(); i++) {
            if (allMaps[i]->GetAllMapPoints().size() > nPoints) {
                largestMap = allMaps[i];
            }
        }
        SLAM->GetAtlas()->ChangeMap(largestMap);
    }

    slamService.SetSlam(SLAM.get());
    if (!slamService.use_live_data) {
        BOOST_LOG_TRIVIAL(info) << "Running in offline mode";
        slamService.StartSaveAtlasAsOsa(SLAM.get());
        slamService.ProcessDataOffline(SLAM.get());
        slamService.StopSaveAtlasAsOsa();
        // Continue to serve requests.
        while (viam::b_continue_session) {
            this_thread::sleep_for(chrono::microseconds(
                viam::checkForShutdownIntervalMicroseconds));
        }
    } else {
        BOOST_LOG_TRIVIAL(info) << "Running in online mode";
        slamService.StartSaveAtlasAsOsa(SLAM.get());
        slamService.ProcessDataOnline(SLAM.get());
        slamService.StopSaveAtlasAsOsa();
    }

    slamService.SetSlam(nullptr);
    SLAM->Shutdown();
    BOOST_LOG_TRIVIAL(info) << "System shutdown";

    return 0;
}
