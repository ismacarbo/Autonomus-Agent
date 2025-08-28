//
// Created by Gastone Pietro Rosati Papini on 10/08/22.
//

#include <stdio.h>
#include <math.h>
#include <vector>
#include <string.h>
#include <algorithm>

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

#ifndef WIN32
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

            // Velocity Limit
            double VMIN = 10; // Minimum velocity
            double VMAX = 25; // Maximum velocity
            double TMIN = 1;
            double TMAX = 100;
            // Time Limit
//            double VMIN = 0; // Minimum velocity
//            double VMAX = 30; // Maximum velocity
//            double TMIN = 10; // Minimum time
//            double TMAX = 30; // Maximum time

            // Vehicle state
            double minAcc = -10;
            double maxAcc = 5;
            double vmin, vmax, tmin, tmax;
            double acc = fmin(fmax(in->ALgtFild, minAcc), maxAcc);
            double vel = in->VLgtFild;
            static double init_dist = in->TrfLightDist;

            double Vr = in->RequestedCruisingSpeed;
            double sf = in->TrfLightDist;
            double minTime, maxTime;
            if (init_dist - in->TrfLightDist > 80){
                static bool set = true;
                if(set == true) {
                    minTime = in->ECUupTime;
                    maxTime = in->ECUupTime;
                    set = false;
                }
                vmin = VMIN;
                vmax = VMAX;
                tmin = minTime+TMIN-in->ECUupTime;
                tmax = maxTime+TMAX-in->ECUupTime;
            }else{
                minTime = 10;
                maxTime = 30;
                vmin = Vr;
                vmax = Vr;
                tmin = minTime-in->ECUupTime;
                tmax = maxTime-in->ECUupTime;

            }

            pass_primitive(vel, acc, sf, vmin, vmax, tmin, tmax,
                           coeffsT2, &v2, &T2, coeffsT1, &v1, &T1);

            // Vmax
            bestCoeff = coeffsT1;
            bestv = v1;
            bests = sf;
            bestT = T1;
            // Vmin
//            bestCoeff = coeffsT2;
//            bestv = v2;
//            bests = sf;
//            bestT = T2;

            static double ECUtimeOLD = 0.;
            ECUtimeOLD = in->ECUupTime;

            static double old_req_acc = 0;
            double longGain = 20;
            double j0 = bestCoeff[3];
            double s0 = bestCoeff[4];
            double cr0 = bestCoeff[5];
            double tOffs = 0;
            double jT0 = j0 + tOffs*s0 + 0.5*tOffs*tOffs*cr0;
            double jT1 = j0 + (DT+tOffs)*s0 + 0.5*(DT+tOffs)*(DT+tOffs)*cr0;
            double req_acc = fmin(fmax((old_req_acc + longGain * (DT * (jT1 + jT0) * 0.5)), minAcc),maxAcc);
            old_req_acc = req_acc;
            double req_vel = v_opt(DT,vel,acc,bests,bestv,0,bestT);

            // PID definition
            double P_gain = 0.02;
            double I_gain = 1;
            double error = req_acc - acc;
            static double integral = 0;
            integral = integral + error * DT;
            double req_ped = P_gain * error + I_gain * integral;

            // Log on file
            logger.log_var("logout", "time", in->ECUupTime);
            logger.log_var("logout", "dist", init_dist - in->TrfLightDist);
            logger.log_var("logout", "vel", vel);
            logger.log_var("logout", "acc", acc);
            logger.log_var("logout", "sf", sf);
            logger.log_var("logout", "vmin", vmin);
            logger.log_var("logout", "vmax", vmax);
            logger.log_var("logout", "tmin", tmin);
            logger.log_var("logout", "tmax", tmax);
            logger.log_var("logout", "req_acc", req_acc);
            logger.log_var("logout", "req_vel", req_vel);
            logger.log_var("logout", "C0", bestCoeff[0]);
            logger.log_var("logout", "C1", bestCoeff[1]);
            logger.log_var("logout", "C2", bestCoeff[2]);
            logger.log_var("logout", "C3", bestCoeff[3]);
            logger.log_var("logout", "C4", bestCoeff[4]);
            logger.log_var("logout", "C5", bestCoeff[5]);
            logger.log_var("logout", "T", bestT);
            logger.write_line("logout");

            // Print
            printLogVar(message_id, "CycleNumber", scenario_msg.data_struct.CycleNumber);
            printLogVar(message_id, "time", in->ECUupTime);
            printLogVar(message_id, "dist [m]", init_dist - in->TrfLightDist);
            printLogVar(message_id, "vel [m/s]", vel);
            printLogVar(message_id, "req_vel [m/s^2]", req_vel);
            printLogVar(message_id, "req_ped [-]", req_ped);

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