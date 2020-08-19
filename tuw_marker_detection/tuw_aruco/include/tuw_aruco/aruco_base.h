/*
 * Copyright (c) 2016, Lukas Pfeifhofer <lukas.pfeifhofer@devlabs.pro>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUW_ARUCO_ARUCO_BASE_H
#define TUW_ARUCO_ARUCO_BASE_H

#include "aruco.h"
#include "tuw_aruco/aruco_parameters.h"
#include "tuw_aruco/aruco_marker_pose.h"
#include "opencv2/opencv.hpp"

class ArUcoBase {
public:
    ArUcoBase();

    ~ArUcoBase();

    void detectMarkers(vector<aruco::Marker> &markers, cv::Mat image);
    void estimatePose(vector<ArUcoMarkerPose> &markerPoses, vector<aruco::Marker> &markers, aruco::CameraParameters cameraParams);
    ArUcoParameters &getParameters();
    void refreshParameters();

private:
    ArUcoParameters params_;

    aruco::MarkerDetector detector_;
    std::map <uint32_t, aruco::MarkerPoseTracker> tracker_;

};


#endif //TUW_ARUCO_ARUCO_BASE_H
