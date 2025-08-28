//
// Created by Gastone Pietro Rosati Papini on 10/08/22.
//

#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <cstring> 

extern "C" {
#include "screen_print_c.h"
}
#include "screen_print.h"
#include "server_lib.h"
#include "logvars.h"

//MATLAB
#include "primitives.h"

#define DEFAULT_SERVER_IP  "127.0.0.1"
#define SERVER_PORT             30000  // Server port
#define DT 0.05

// Handler for CTRL-C
#include <signal.h>
static uint32_t server_run = 1;
void intHandler(int signal) {
    server_run = 0;
}

int main(int argc, const char * argv[]) {
    logger.enable(true);

    // Messages variables
    scenario_msg_t scenario_msg;
    manoeuvre_msg_t manoeuvre_msg;
    size_t scenario_msg_size = sizeof(scenario_msg.data_buffer);
    size_t manoeuvre_msg_size = sizeof(manoeuvre_msg.data_buffer);
    uint32_t message_id = 0;

#ifndef _MSC_VER
    // More portable way of supporting signals on UNIX
    struct sigaction act;
    act.sa_handler = intHandler;
    sigaction(SIGINT, &act, NULL);
#else
    signal(SIGINT, intHandler);
#endif

    server_agent_init(DEFAULT_SERVER_IP, SERVER_PORT);

    // Start server
    printLine();
    printTable("Waiting for scenario message...", 0);
    printLine();
    while (server_run == 1) {

        // Clean the buffer
        memset(scenario_msg.data_buffer, '\0', scenario_msg_size);
        if (server_receive_from_client(&server_run, &message_id, &scenario_msg.data_struct) == 0) {
            printLogTitle(message_id, "received message");

            input_data_str *in = &scenario_msg.data_struct;
            manoeuvre_msg.data_struct.CycleNumber = scenario_msg.data_struct.CycleNumber;
            manoeuvre_msg.data_struct.Status = scenario_msg.data_struct.Status;

            double minAcc = -10;
            double maxAcc = 5;
            double time = in->ECUupTime;
            double vel = in->VLgtFild;
            double acc = fmin(fmax(in->ALgtFild, minAcc),maxAcc);
            static double init_dist = in->TrfLightDist;
            double dist = init_dist - in->TrfLightDist;

// ----------------------------------------------------
//             TEST 2
//             Test acc and brake
             double req_acc = a_opt(DT,vel,acc,in->TrfLightDist,25,0,10 - time);
             double req_vel = v_opt(DT,vel,acc,in->TrfLightDist,25,0,10 - time);
             if( time > 5){
                 req_acc = a_opt(DT,vel,acc,in->TrfLightDist,0,0,15 - time);
                 req_vel = v_opt(DT,vel,acc,in->TrfLightDist,0,0,15 - time);
             }
// ----------------------------------------------------

            // PID control
            static double integral = 0;
            double P_gain = 0.02;
            double I_gain = 1;
            double error = req_acc - acc;
            integral = integral + error * DT;
            double req_ped = P_gain * error + I_gain * integral;

            // Log on file
            logger.log_var("logout", "time", in->ECUupTime);
            logger.log_var("logout", "dist", dist);
            logger.log_var("logout", "req_acc", req_acc);
            logger.log_var("logout", "acc", acc);
            logger.log_var("logout", "req_vel", req_vel);
            logger.log_var("logout", "vel", vel);

            // Print
            printLogVar(message_id, "CycleNumber", scenario_msg.data_struct.CycleNumber);
            printLogVar(message_id, "time", in->ECUupTime);
            printLogVar(message_id, "dist [m]", dist);
            printLogVar(message_id, "vel [m/s]", vel);
            printLogVar(message_id, "acc [m/s^2]", acc);
            printLogVar(message_id, "req_acc [m/s^2]", req_acc);
            printLogVar(message_id, "req_ped [-]", req_ped);

            logger.write_line("logout");
            manoeuvre_msg.data_struct.RequestedAcc = req_ped;

            if (server_send_to_client(server_run, message_id, &manoeuvre_msg.data_struct) == -1) {
                perror("error send_message()");
                exit(EXIT_FAILURE);
            } else {
                printLogTitle(message_id, "sent message");
            }
        }
    }
    server_agent_close();
    return 0;
}