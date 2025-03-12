//
// Created by ubuntu-slam on 22-6-23.
//

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <sys/wait.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs/legacy/constants_c.h>

#include<System.h>
#include "Config.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
string GetDatasetName(const string &strSequencePath);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_OpenLORIS path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true, false);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
//    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];
//        clahe->apply(imRGB,imRGB);
        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    SLAM.Spin();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    cout << "fps: " << nImages/totaltime << endl;

    string dataset_name = GetDatasetName(string(argv[3]));
    auto trajString = "evaluation/RGBD_TUM_"+dataset_name+"_KeyFrameTrajectory";
    auto trajStringCam = "evaluation/RGBD_TUM_"+dataset_name+"_CameraTrajectory";
    auto snapString = "evaluation/RGBD_TUM_"+dataset_name+".msgpack";
    auto gtJsonTrajString = "evaluation/RGBD_TUM_"+dataset_name+"_gtTraj.json";

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(trajStringCam+".txt");
    SLAM.SaveKeyFrameTrajectoryTUM(trajString+".txt");  // rpj only
    SLAM.SaveKeyFrameTrajectoryNGP(trajString+".json"); // rpj (+ pht if train extrinsics)
    SLAM.SaveSnapShot(snapString);

    int pid = fork();
    if (pid < 0)
    {
        cout << "fork failed" << endl;
    }
    else if( pid == 0 )
    {
        // For headless version, we do not need to spin the program.
        // But instead, terminate training process and execute evaluation script.
        auto gtString = string(argv[3]) + "/groundtruth.txt";
        auto trajPathString = trajString + ".txt";
        auto trajPathStringCam = trajStringCam + ".txt";
        auto plotString = trajString + "_rpj.png";
        char *gtPath = (char *)(gtString.c_str());
        char *trajPath = (char *)(trajPathString.c_str());
        char *trajPathCam = (char *)(trajPathStringCam.c_str());
        char *plotPath = (char *)(plotString.c_str());
        char *gtJsonTrajPath = (char *)(gtJsonTrajString.c_str());

        std::cout << "ATE w/ reprojection error:" << std::endl;
        char *execArgs[] = {"python3", "scripts/evaluate_ate.py", gtPath, trajPathCam, "--verbose", "--plot", plotPath, "--save_gt_json", gtJsonTrajPath, NULL};
        execvp("python3", execArgs);
    }
    wait(NULL);

    std::cout << std::endl;

    pid = fork();
    if (pid < 0)
    {
        cout << "fork failed" << endl;
    }
    else if( pid == 0 )
    {
        // For headless version, we do not need to spin the program.
        // But instead, terminate training process and execute evaluation script.
        auto gtString = string(argv[3]) + "/groundtruth.txt";
        auto trajPathString = trajString + ".json";
        auto plotString = trajString + "_rpj+pht.png";
        char *gtPath = (char *)(gtString.c_str());
        char *trajPath = (char *)(trajPathString.c_str());
        char *plotPath = (char *)(plotString.c_str());

        std::cout << "ATE w/ reprojection error (+ photometric error if optimize extrinsic == true):" << std::endl;
        char *execArgs[] = {"python3", "scripts/evaluate_ate.py", gtPath, trajPath, "--verbose", "--plot", plotPath, NULL};
        execvp("python3", execArgs);
    }
    wait(NULL);

#ifdef ORBEEZ_GUI
    SLAM.AddGroundTruthTraj(gtJsonTrajString);

    cout << "Press ctrl + c to exit the program " << endl;

    // Don't stop program, to see the Nerf training result
    volatile int keep_spinning = 1;
    while (keep_spinning) ; // spin
#endif 

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

string GetDatasetName(const string &strSequencePath) 
{
    string s(strSequencePath);
    std::string delimiter = "/";

    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        s.erase(0, pos + delimiter.length());
    }

    if (s.length() == 0)
        return token;
    else
        return s;
}


