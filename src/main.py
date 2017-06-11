import matplotlib.pyplot as plt

from simulator import simulator
from coordinator import coordinator
from planner import planner
import time

def main():
  num_rows = 4
  row_len = 8
  num_bins = 500
  num_workers = 4
  num_bots = 2

  # 0 = greedy, 1 = auction based, 2 = replanning, 3 = yawei greedy
  coord_method = 0

  num_timestep = 300


  sim = simulator(num_rows, row_len, num_bots, num_bins, num_workers)
  coord = coordinator(coord_method)
  plnr = planner(sim)
  sim.drawSimulator()

  for timestep in range(num_timestep):
    plan = coord.cordStep(sim)
    # print sim.getIdleBots()
    # for item in plan:
    #   print "Robot: ", item.robot_id
    #   print "Goals: ", item.locations
    # print "here"

    plan = plnr.getPlan(plan, sim)
    # print "Planner Plan", plan
    sim.drawSimulator()
    # print "apples picked: ",sim.apples_picked


    # for key in sim.bots.keys():
    #   print key, sim.bots[key].plan
    #raw_input()
    print timestep
    
    sim.step()
  print "done running"
  print "total num apples: ", sim.total_apples
  print "num apples picked: ", sim.apples_picked
  print "percentage picked: ", sim.apples_picked / sim.total_apples * 100
  print "wasted time: ", sim.wasted_time
  #plt.show()
  return 0

if __name__ == '__main__':
	main()