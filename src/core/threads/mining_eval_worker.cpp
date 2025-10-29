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

#include "mining_eval_worker.hpp"

#include <csm_metrics/profiling.hpp>


namespace csm
{
namespace perception
{

MiningEvalWorker::MiningEvalWorker(
    RclNode& node,
    const Tf2Buffer& tf_buffer) :
    node{node},
    tf_buffer{tf_buffer},
    pub_map{&node, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS}
{
}

MiningEvalWorker::~MiningEvalWorker() { this->stopThreads(); }

void MiningEvalWorker::configure(const std::string& odom_frame)
{
    this->odom_frame = odom_frame;
}

void MiningEvalWorker::accept(
    const UpdateMiningEvalSrv::Request::SharedPtr& req,
    const UpdateMiningEvalSrv::Response::SharedPtr& resp)
{
    // TODO
    (void)req;
    (void)resp;
}

ResourcePipeline<MiningEvalResources>& MiningEvalWorker::getInput()
{
    return this->mining_eval_resources;
}

void MiningEvalWorker::startThreads()
{
    if (!this->threads_running)
    {
        this->threads_running = true;
        this->mining_eval_thread =
            std::thread{&MiningEvalWorker::mining_eval_thread_worker, this};
    }
}

void MiningEvalWorker::stopThreads()
{
    this->threads_running = false;

    this->query_notifier.notifyExit();
    this->mining_eval_resources.notifyExit();
    if (this->mining_eval_thread.joinable())
    {
        this->mining_eval_thread.join();
    }
}

void MiningEvalWorker::mining_eval_thread_worker()
{
    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Mining evaluation thread started successfully.");

    do
    {
        if (this->srv_enable_state.load())
        {
            auto& buff = this->mining_eval_resources.waitNewestResource();
            if (!this->threads_running.load())
            {
                return;
            }

            buff.query = this->query_notifier.aquireNewestOutput();

            PROFILING_SYNC();
            PROFILING_NOTIFY_ALWAYS(mining_eval);
            this->mining_eval_callback(buff);
            PROFILING_NOTIFY_ALWAYS(mining_eval);
        }
        else
        {
            this->query_notifier.waitNewestResource();
        }
    }  //
    while (this->threads_running.load());

    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Mining evaluation thread exited cleanly.");
}


void MiningEvalWorker::mining_eval_callback(MiningEvalResources& buff)
{
    // TODO
    (void)buff;
}

};  // namespace perception
};  // namespace csm
