# File to do stat runs

import matplotlib.pyplot as plt

from simulator import simulator
from coordinator import coordinator
from planner import planner
import time
import numpy as np

def statrunsNumSteps():

    num_runs = 100

    output_file_0 = "../results/final_greedy.txt"
    output_file_1 = "../results/final_auction.txt"
    output_file_2 = "../results/final_replan.txt"
    output_file_3 = "../results/final_greedy_full.txt"

    file_0 = open(output_file_0,'w')
    file_1 = open(output_file_1,'w')
    file_2 = open(output_file_2,'w')
    file_3 = open(output_file_3,'w')

    num_rows = 15
    row_len = 8
    num_bins = 500
    num_workers = [4,8,12,16]
    num_bots = [2,4,6,8,10,12,14,16,18,20]

    # 0 = greedy, 1 = auction based, 2= replanning
    coord_method = [0,1,2,3]
    #coord_method = [3]

    num_timestep = 300

    file_0.write("num_workers, num_bots, percent_picked_mean, percent_picked_sd, time_wasted_mean, time_wasted_sd\n")
    file_1.write("num_workers, num_bots, percent_picked_mean, percent_picked_sd, time_wasted_mean, time_wasted_sd\n")
    file_2.write("num_workers, num_bots, percent_picked_mean, percent_picked_sd, time_wasted_mean, time_wasted_sd\n")
    file_3.write("num_workers, num_bots, percent_picked_mean, percent_picked_sd, time_wasted_mean, time_wasted_sd\n")

    for nw in num_workers:
        for nb in num_bots:
            for cm in coord_method:
                print "Num workers: ", nw
                print "Num bots: ", nb
                print "Coord Method: ", cm
                print "----------------------------------------------"

                percent_picked = []
                time_wasted = []

                for i in range(num_runs):
                    #print i
                    sim = simulator(num_rows, row_len, nb, num_bins, nw)
                    coord = coordinator(cm)
                    plnr = planner(sim)

                    

                    for timestep in range(num_timestep):
                        plan = coord.cordStep(sim)

                        plan = plnr.getPlan(plan, sim)
    
                        sim.step()

                    percent_picked.append(sim.apples_picked / sim.total_apples * 100)
                    time_wasted.append(sim.wasted_time)

                if cm == 0:
                    writeToFile(nw, nb, percent_picked, time_wasted, file_0)
                elif cm == 1:
                    writeToFile(nw, nb, percent_picked, time_wasted, file_1)
                elif cm == 2:
                    writeToFile(nw, nb, percent_picked, time_wasted, file_2)
                elif cm == 3:
                    writeToFile(nw, nb, percent_picked, time_wasted, file_3)

    # writeHeader(file, num_rows, row_len, num_bins, num_workers, num_bots, coord_method, num_timestep)

    # file.write("percentage picked, ")
    # file.write("percentage picked, ")
    # file.write("time wasted\n")

    

    # for i in range(num_runs):

    #     sim = simulator(num_rows, row_len, num_bots, num_bins, num_workers)
    #     coord = coordinator(coord_method)
    #     plnr = planner(sim)

    #     for timestep in range(num_timestep):
    #         plan = coord.cordStep(sim)

    #         plan = plnr.getPlan(plan, sim)
    
    #         sim.step()

    #     percent_picked.append(sim.apples_picked / sim.total_apples * 100)
    #     file.write(str(sim.apples_picked / sim.total_apples * 100))
    #     file.write(", ")
    #     file.write(str(sim.wasted_time))
    #     file.write("\n")
    #     print "done running iteration: ", i
    # print "mean percent picked: ", np.mean(percent_picked)
    # print "std percent picked: ", np.std(percent_picked)

def statrunsTimeDone():

    num_runs = 20

    output_file = "../results/tLimited_auction.txt"

    file = open(output_file,'w')

    num_rows = 10
    row_len = 8
    num_bins = 100
    num_workers = 15
    num_bots = 8

    # 0 = greedy, 1 = auction based, 2= replanning
    coord_method = 2

    writeHeader(file, num_rows, row_len, num_bins, num_workers, num_bots, coord_method, '0')

    file.write("percentage picked, ")
    file.write("time wasted\n")

    time_done = []

    for i in range(num_runs):

        sim = simulator(num_rows, row_len, num_bots, num_bins, num_workers)
        coord = coordinator(coord_method)
        plnr = planner(sim)
        time_taken = 0

        while (sim.applesLeft() != 0 and time_taken < 10000):
            plan = coord.cordStep(sim)

            plan = plnr.getPlan(plan, sim)
    
            sim.step()

            #sim.drawSimulator()

            time_taken += 1

        time_done.append(time_taken)

        print "done running iteration: ", i
    print "mean percent picked: ", np.mean(time_done)
    print "std percent picked: ", np.std(time_done)

def writeHeader(file, num_rows, row_len, num_bins, num_workers, num_bots, coord_method, num_timestep):
    file.write("parameters\n")
    
    file.write("------------------------------------------------------------------\n")

    file.write("num_rows: ")
    file.write(str(num_rows))
    file.write("\n")

    file.write("row_len: ")
    file.write(str(row_len))
    file.write("\n")

    file.write("num_bins: ")
    file.write(str(num_bins))
    file.write("\n")

    file.write("num_workers: ")
    file.write(str(num_workers))
    file.write("\n")

    file.write("num_bots: ")
    file.write(str(num_bots))
    file.write("\n")

    file.write("coord method: ")
    file.write(str(coord_method))
    file.write("\n")

    file.write("num_timestep: ")
    file.write(str(num_timestep))
    file.write("\n")

    file.write("------------------------------------------------------------------\n")

def writeToFile(nw, nb, percent_picked, time_wasted, file):
    file.write(str(nw))
    file.write(", ")
    file.write(str(nb))
    file.write(", ")
    file.write(str(np.mean(percent_picked)))
    #print np.mean(percent_picked)
    file.write(", ")
    file.write(str(np.std(percent_picked)))
    #print np.std(percent_picked)
    file.write(", ")
    file.write(str(np.mean(time_wasted)))
    #print np.mean(time_wasted)
    file.write(", ")
    file.write(str(np.std(time_wasted)))
    #print np.std(time_wasted)
    file.write("\n")

if __name__ == '__main__':
    statrunsNumSteps()
    #statrunsTimeDone()

