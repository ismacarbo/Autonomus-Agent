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
            //static auto start = std::chrono::system_clock::now();
            //auto time = std::chrono::system_clock::now()-start;
            //double num_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(time).count()/1000.0;

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
            int type = 0;
            double *bestCoeff;
            double bestv;
            double bests;
            double bestT;

            // Vehicle state
            double minAcc = -10;
            double maxAcc = 5;
            //double vmin, vmax, tmin, tmax;
            double acc = fmin(fmax(in->ALgtFild, minAcc), maxAcc);
            double vel = in->VLgtFild;
            static double init_dist = in->TrfLightDist;

            // Agent
            double lookhead = fmax(50,in->VLgtFild*5);        // Lookhaed distance
            double Vmax = 15;                                               // Max velocity close to traffic light
            double Vmin = 3;                                                // Min velocity 3 m/s
            double Vr = in->RequestedCruisingSpeed;                         // Requested velocity

            // Safety time and space
            double Ss = 5;                                                  // Safety space (the car pass if the traffic light is green Ss meter before)
            double Ts = Ss / Vmin;                                          // The maximum time to travel the safety space
            double Sin = 10;                                                // Dimentions of the intersection
            double Tin = Sin / Vmin;                                        // The maximum time to travel the intersection

            // Define pass dist and stop dist
            double traffic_light_dist_pass = 0;
            double traffic_light_dist_stop = 0;
            if (in->NrTrfLights != 0) {
                traffic_light_dist_pass = in->TrfLightDist;
                traffic_light_dist_stop = in->TrfLightDist - Ss/2;  // The car stop half safety space distance
            }


            printLogVar(message_id, "Lookhaed", lookhead);

            // Freeflow motion
            double minTime = 0, maxTime = 0;
            if (in->NrTrfLights == 0 || traffic_light_dist_pass >= lookhead) {
                pass_primitive(vel, acc, lookhead, Vr, Vr,
                               minTime, maxTime, coeffsT2, &v2, &T2, coeffsT1, &v1, &T1);
                printLogVar(message_id, "Primitive", "NO TRAFFIC LIGHT (FAR)");
                type = 1;
                bestCoeff = coeffsT2;
                bestv = v2;
                bests = lookhead;
                bestT = T2;
            } else {
                printLogVar(message_id, "TrafficLight Dist", in->TrfLightDist);
                int phase = 0;
                double time_to_green, time_to_red;
                // The traffic light is green
                if (in->TrfLightCurrState == 1 &&
                    in->TrfLightFirstNextState == 2) {
                    printLogVar(message_id, "TrafficLight Green -> Yellow", 1);
                    time_to_green = 0;
                    time_to_red = in->TrfLightFirstTimeToChange - Tin;
                    phase = 1;
                }
                // The traffic light is yellow
                if (phase == 0 && in->TrfLightCurrState == 2 &&
                    in->TrfLightFirstNextState == 3) {
                    printLogVar(message_id, "TrafficLight Yellow -> Red", 1);
                    time_to_green = in->TrfLightSecondTimeToChange + Ts;
                    time_to_red = in->TrfLightThirdTimeToChange - Tin;
                    phase = 2;
                }
                // The traffic light is red
                if (phase == 0 && in->TrfLightCurrState == 3 &&
                    in->TrfLightFirstNextState == 1) {
                    printLogVar(message_id, "TrafficLight Red -> Green", 1);
                    time_to_green = in->TrfLightFirstTimeToChange + Ts;
                    time_to_red = in->TrfLightSecondTimeToChange - Tin;
                    phase = 3;
                }
                printLogVar(message_id, "time to green", time_to_green);
                printLogVar(message_id, "time to red", time_to_red);

                minTime = time_to_green;
                maxTime = time_to_red;

                printLogVar(message_id, "Tmin", minTime);
                printLogVar(message_id, "Tmax", maxTime);

                if(traffic_light_dist_pass < Ss && phase == 1){
                    minTime = 0;
                    maxTime = 0;
                    pass_primitive(vel, acc, lookhead, Vr, Vr,
                                   minTime, maxTime, coeffsT2, &v2, &T2, coeffsT1, &v1, &T1);
                    printLogVar(message_id, "Primitive", "PASS CLOSE TO TRAFFIC LIGHT");
                    type = 4;
                    bestCoeff = coeffsT1;
                    bestv = v1;
                    bests = lookhead;
                    bestT = T1;
                }else{
                    // If the vehicle is far from the traffic light
                    pass_primitive(vel, acc, traffic_light_dist_pass, Vmin, Vmax,
                                   minTime, maxTime, coeffsT2, &v2, &T2, coeffsT1, &v1, &T1);
                    if (v1 == 0) {
                        // There are no possibilty to pass
                        if(traffic_light_dist_stop > 0){
                            stop_primitive(vel, acc, traffic_light_dist_stop, coeffsT2, &bests, &bestT);
                            type = -1;
                            printLogVar(message_id, "Primitive", "STOP TO TRAFFIC LIGHT");
//                        if ((coeffsT2[3] > 0) && (vel > 2)) {
//                            stop_primitive_j0(vel, acc, coeffsT2, &bests, &bestT);
//                            type = -2;
//                            printLogVar(message_id, "Primitive", "STOP TO TRAFFIC LIGHT J0");
//                        } else {
//                            printLogVar(message_id, "Primitive", "STOP TO TRAFFIC LIGHT");
//                        }
                            bestCoeff = coeffsT2;
                            bestv = 0;
                        }else{
                            type = -2;
                            printLogVar(message_id, "Primitive", "No Primitive");
                            bestCoeff = coeffsT2;
                            bestv = 0;
                        }

                    } else {
                        if ((coeffsT2[3] > 0 && coeffsT1[3] < 0) || (coeffsT1[3] > 0 && coeffsT2[3] < 0)) {
                            pass_primitive_j0(vel, acc, traffic_light_dist_pass, Vmin, Vmax, coeffsT1, &v1, &T1);
                            type = 3;
                            printLogVar(message_id, "Primitive", "PASS TO TRAFFIC LIGHT J0");
                            bestCoeff = coeffsT1;
                            bestv = v1;
                            bests = traffic_light_dist_pass;
                            bestT = T1;
                        } else {
                            if(fabs(coeffsT2[3]) > fabs(coeffsT1[3])){
                                bestCoeff = coeffsT1;
                                bestv = v1;
                                bests = traffic_light_dist_pass;
                                bestT = T1;
                            }else{
                                bestCoeff = coeffsT2;
                                bestv = v2;
                                bests = traffic_light_dist_pass;
                                bestT = T2;
                            }
                            type = 2;
                            printLogVar(message_id, "Primitive", "PASS TO TRAFFIC LIGHT");
                        }
                    }
                }
            }
            printLogVar(message_id, "Time of primitive", bestT);
            printLogVar(message_id, "Final velocity", bestv);
            printLogVar(message_id, "Final position", bests);

            static double ECUtimeOLD = 0.;
            double TS_agent = in->ECUupTime - ECUtimeOLD;
            ECUtimeOLD = in->ECUupTime;


            static double old_req_acc = 0;
            double longGain = 1;
            double j0 = bestCoeff[3];
            double s0 = bestCoeff[4];
            double cr0 = bestCoeff[5];
            double tOffs = 0;
            double jT0 = j0 + tOffs*s0 + 0.5*tOffs*tOffs*cr0;
            double jT1 = j0 + (DT+tOffs)*s0 + 0.5*(DT+tOffs)*(DT+tOffs)*cr0;
            double jint = (DT * (jT1 + jT0) * 0.5);
            double req_acc = fmin(fmax((old_req_acc + longGain * (DT * (jT1 + jT0) * 0.5)), minAcc),maxAcc);
            old_req_acc = req_acc;
            double req_vel = v_opt(DT,vel,acc,bests,bestv,0,bestT);

            // PID control
            static double integral = 0;
            double P_gain = 0.02;
            double I_gain = 1;
            double error = req_acc - acc;
            integral = integral + error * DT;
            double req_ped = P_gain * error + I_gain * integral;

            // Reset memory
            if(vel < 0.1 && old_req_acc < 0 && jint > 0){
                old_req_acc = 0;
                integral = 0;
            }

            // Log on file
            logger.log_var("logout", "time", in->ECUupTime);
            logger.log_var("logout", "dist", init_dist - in->TrfLightDist);
            logger.log_var("logout", "trdist", in->TrfLightDist);
            logger.log_var("logout", "phase", scenario_msg.data_struct.TrfLightCurrState);
            logger.log_var("logout", "tmin", minTime);
            logger.log_var("logout", "tmax", maxTime);
            // Primitive information
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
            // Low Level Control
            logger.log_var("logout", "req_acc", req_acc);
            logger.log_var("logout", "acc", acc);
            logger.log_var("logout", "req_vel", req_vel);
            logger.log_var("logout", "vel", vel);
            logger.write_line("logout");

            for(int i = 0;i < in->NrObjs; i++){
                logger.log_var("obstacles", "ObjID", in->ObjID[i]);
                logger.log_var("obstacles", "ObjX", in->ObjX[i]);
                logger.log_var("obstacles", "ObjY", in->ObjY[i]);
                logger.log_var("obstacles", "ObjLen", in->ObjLen[i]);
                logger.log_var("obstacles", "ObjWidth", in->ObjWidth[i]);
                logger.log_var("obstacles", "ObjVel", in->ObjVel[i]);
                logger.log_var("obstacles", "ObjCourse", in->ObjCourse[i]);
                logger.write_line("obstacles");
            }

            // Print
//            printLogVar(message_id, "time", in->ECUupTime);
//            printLogVar(message_id, "dist [m]", init_dist - in->TrfLightDist);
//            printLogVar(message_id, "vel [m/s]", vel);
//            printLogVar(message_id, "req_acc [m/s^2]", req_acc);
//            printLogVar(message_id, "req_ped [-]", req_ped);
//            printLogVar(message_id, "V0", vel);
//            printLogVar(message_id, "A0", acc);
//            printLogVar(message_id, "J0", bestCoeff[3]);
//            printLogVar(message_id, "Sn0", bestCoeff[4]);
//            printLogVar(message_id, "Cr0", bestCoeff[5]);
//            printLogVar(message_id, "CycleNumber", scenario_msg.data_struct.CycleNumber);

            manoeuvre_msg.data_struct.RequestedAcc = req_ped;
            // Simple lateral control
            //double pos_error = in->LaneWidth/4-in->LatOffsLineL;
            //double angle_error = -in->LaneHeading;
            //manoeuvre_msg.data_struct.RequestedSteerWhlAg = 0.05 * pos_error + 1 * angle_error;

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