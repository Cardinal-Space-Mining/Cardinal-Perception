/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#pragma once

// --- PRINTING ENABLE/DISABLE ------------------------------------------------
#ifndef PERCEPTION_PRINT_STARTUP_CONFIGS
    #define PERCEPTION_PRINT_STARTUP_CONFIGS 1
#endif

#ifndef TRANSFORM_SYNC_PRINT_DEBUG
    #define TRANSFORM_SYNC_PRINT_DEBUG 0
#endif

// --- PUBLISHERS ENABLE/DISABLE ----------------------------------------------
#ifndef PERCEPTION_PUBLISH_GRAV_ESTIMATION
    #define PERCEPTION_PUBLISH_GRAV_ESTIMATION 1
#endif

#ifndef PERCEPTION_PUBLISH_LIO_DEBUG
    #define PERCEPTION_PUBLISH_LIO_DEBUG 1
#endif

#ifndef PERCEPTION_PUBLISH_LFD_DEBUG
    #define PERCEPTION_PUBLISH_LFD_DEBUG 1
#endif

#ifndef PERCEPTION_PUBLISH_TRJF_DEBUG
    #define PERCEPTION_PUBLISH_TRJF_DEBUG 1
#endif

#ifndef PERCEPTION_PUBLISH_TRAV_DEBUG
    #define PERCEPTION_PUBLISH_TRAV_DEBUG 1
#endif

#ifndef PERCEPTION_PUBLISH_FULL_MAP
    #define PERCEPTION_PUBLISH_FULL_MAP 0
#endif

// --- FEATURES ENABLE/DISABLE ------------------------------------------------
#ifndef PERCEPTION_USE_SCAN_DESKEW
    #define PERCEPTION_USE_SCAN_DESKEW 1
#endif

#ifndef PERCEPTION_USE_NULL_RAY_DELETION
    #define PERCEPTION_USE_NULL_RAY_DELETION 0
#endif

// --- PIPELINE STAGES ENABLE/DISABLE -----------------------------------------
#ifndef PERCEPTION_ENABLE_MAPPING
    #define PERCEPTION_ENABLE_MAPPING 1
#endif

#ifndef PERCEPTION_ENABLE_TRAVERSIBILITY
    #define PERCEPTION_ENABLE_TRAVERSIBILITY (PERCEPTION_ENABLE_MAPPING)
#endif

#ifndef PERCEPTION_ENABLE_PATH_PLANNING
    #define PERCEPTION_ENABLE_PATH_PLANNING (PERCEPTION_ENABLE_TRAVERSIBILITY)
#endif

#ifndef PERCEPTION_USE_TAG_DETECTION_PIPELINE
    #define PERCEPTION_USE_TAG_DETECTION_PIPELINE 0
#endif
#ifndef PERCEPTION_USE_LFD_PIPELINE
    #define PERCEPTION_USE_LFD_PIPELINE (!PERCEPTION_USE_TAG_DETECTION_PIPELINE)
#endif

// --- ROS TOPIC CONFIGURATION ------------------------------------------------
#ifndef PERCEPTION_TOPIC_PREFIX
    #define PERCEPTION_TOPIC_PREFIX "/cardinal_perception"
#endif
#ifndef PERCEPTION_PUBSUB_QOS
    #define PERCEPTION_PUBSUB_QOS rclcpp::SensorDataQoS()
#endif

// --- VERSION ----------------------------------------------------------------
#ifndef PERCEPTION_VERSION_MAJOR
    #define PERCEPTION_VERSION_MAJOR 0
#endif
#ifndef PERCEPTION_VERSION_MINOR
    #define PERCEPTION_VERSION_MINOR 0
#endif
#ifndef PERCEPTION_VERSION_PATCH
    #define PERCEPTION_VERSION_PATCH 0
#endif


// --- AUTOMATIC DEDUCTIONS/ASSERTIONS ----------------------------------------
#if ((PERCEPTION_USE_TAG_DETECTION_PIPELINE) && (PERCEPTION_USE_LFD_PIPELINE))
static_assert(
    false,
    "Tag detection and lidar fiducial pipelines are mutually exclusive."
    "You may only enable one at a time.");
#endif
#if ((PERCEPTION_ENABLE_TRAVERSIBILITY) && !(PERCEPTION_ENABLE_MAPPING))
    #undef PERCEPTION_ENABLE_TRAVERSIBILITY
    #define PERCEPTION_ENABLE_TRAVERSIBILITY 0
#endif


// --- HELPER MACROS ----------------------------------------------------------
#if PERCEPTION_ENABLE_MAPPING > 0
    #define IF_MAPPING_ENABLED(...) __VA_ARGS__
    #define MAPPING_ENABLED         1
#else
    #define IF_MAPPING_ENABLED(...)
    #define MAPPING_ENABLED 0
#endif
#if PERCEPTION_ENABLE_TRAVERSIBILITY > 0
    #define IF_TRAVERSABILITY_ENABLED(...) __VA_ARGS__
    #define TRAVERSABILITY_ENABLED         1
#else
    #define IF_TRAVERSABILITY_ENABLED(...)
    #define TRAVERSABILITY_ENABLED 0
#endif
#if PERCEPTION_ENABLE_PATH_PLANNING > 0
    #define IF_PATH_PLANNING_ENABLED(...) __VA_ARGS__
    #define PATH_PLANNING_ENABLED         1
#else
    #define IF_PATH_PLANNING_ENABLED(...)
    #define PATH_PLANNING_ENABLED 0
#endif

#if PERCEPTION_USE_TAG_DETECTION_PIPELINE > 0
    #define IF_TAG_DETECTION_ENABLED(...) __VA_ARGS__
    #define TAG_DETECTION_ENABLED         1
#else
    #define IF_TAG_DETECTION_ENABLED(...)
    #define TAG_DETECTION_ENABLED 0
#endif
#if PERCEPTION_USE_LFD_PIPELINE > 0
    #define IF_LFD_ENABLED(...) __VA_ARGS__
    #define LFD_ENABLED         1
#else
    #define IF_LFD_ENABLED(...)
    #define LFD_ENABLED 0
#endif

#define PERCEPTION_TOPIC(subtopic) PERCEPTION_TOPIC_PREFIX "/" subtopic

#define TEXTIFY(X) #X
#define TEXTIFY_EVAL(X) TEXTIFY(X)
#define FORMAT_VERSION_STR(MAJOR, MINOR, PATCH)              \
    "V" TEXTIFY(MAJOR) "." TEXTIFY(MINOR) "." TEXTIFY(PATCH)

#define PERCEPTION_VERSION_STR    \
    FORMAT_VERSION_STR(           \
        PERCEPTION_VERSION_MAJOR, \
        PERCEPTION_VERSION_MINOR, \
        PERCEPTION_VERSION_PATCH)
