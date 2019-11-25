/* Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "toy_mbes_receptor/toy_mbes_receptor.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "lolo_mbes_node");

    ros::NodeHandle nh_sss_r;
    ros::NodeHandle nh_sss_l;
    ros::CallbackQueue sss_r_queue;
    ros::CallbackQueue sss_l_queue;
    nh_sss_r.setCallbackQueue(&sss_r_queue);
    nh_sss_l.setCallbackQueue(&sss_l_queue);

    MBESReceptor* sss_scanner = new MBESReceptor(ros::this_node::getName(), nh_sss_r, nh_sss_l);

    ros::AsyncSpinner spinner_sss_right(1, &sss_r_queue);
    ros::AsyncSpinner spinner_sss_left(1, &sss_l_queue);
    spinner_sss_right.start();
    spinner_sss_left.start();

    ros::waitForShutdown();

    if(!ros::ok()){
        delete sss_scanner;
    }

    return 0;
}
