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
            // TEST 3
           int type = 0;

           // Return coeff
           double coeffsT2[6];
           double v2;
           double T2;
           double coeffsT1[6];
           double v1;
           double T1;

           // Best coeff
           double *bestCoeff;
           double bestv;
           double bests;
           double bestT;

           double lookhead = 100;
           if(dist < init_dist - 100) {
               double Vr = 25;
               double maxTime = 0;
               double minTime = 0;
               pass_primitive(vel, acc, lookhead, Vr, Vr,
                              minTime, maxTime, coeffsT2, &v2, &T2, coeffsT1, &v1, &T1);
               type = 1;
               bestCoeff = coeffsT2;
               bests = lookhead;
               bestv = v2;
               bestT = T2;
           }else{
               static double pos_init_brake = dist;
               double init_stop_dist = 100;
               double stop_dist = pos_init_brake + init_stop_dist - dist;
               stop_primitive(vel, acc, stop_dist, bestCoeff, &bests, &bestT);
               type = -1;
               bestv = 0;
           }

           // Primitive information
           static double old_req_acc = 0;
           double longGain = 1;
           double j0 = bestCoeff[3];
           double s0 = bestCoeff[4];
           double cr0 = bestCoeff[5];
           double tOffs = 0;
           double jT0 = j0 + tOffs*s0 + 0.5*tOffs*tOffs*cr0;
           double jT1 = j0 + (DT+tOffs)*s0 + 0.5*(DT+tOffs)*(DT+tOffs)*cr0;
           double req_acc = fmin(fmax((old_req_acc + longGain * (DT * (jT1 + jT0) * 0.5)), minAcc),maxAcc);
           old_req_acc = req_acc;
           double req_vel = v_opt(DT,vel,acc,bests,bestv,0,bestT);
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

            logger.log_var("logout", "type", type);
            logger.log_var("logout", "sf", bests);
            logger.log_var("logout", "vf", bestv);
            logger.log_var("logout", "C0", bestCoeff[0]);
            logger.log_var("logout", "C1", bestCoeff[1]);
            logger.log_var("logout", "C2", bestCoeff[2]);
            logger.log_var("logout", "C3", bestCoeff[3]);
            logger.log_var("logout", "C4", bestCoeff[4]);
            logger.log_var("logout", "C5", bestCoeff[5]);
            logger.log_var("logout", "T", bestT);

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