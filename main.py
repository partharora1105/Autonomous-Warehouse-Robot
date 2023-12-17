from __future__ import absolute_import
import sys
import threading
import time

from grid import Grid
from visualization import GUIWindow
from robot import Robot
from robot import exploration_state_machine
from robot import planning
from helpers import *


# control loop for assignment
class RobotEnv:

    def __init__(self, robbie, grid, program_state='exploration', testing=False):
        self.robbie = robbie
        self.grid = grid
        self.program_state = program_state
        self.testing = testing
        self.robbie.markers_found_or_picked = robbie.read_marker_around(grid)


    def update(self):

        if self.program_state == 'exploration':
            self.robbie = exploration_state_machine(robbie=self.robbie, grid=self.grid)
        
        elif self.program_state == 'tasks':
            self.robbie = planning(self.robbie,self.grid)
            add_noise(self.robbie)
            add_offset_noise(self.robbie)

        self.update_motion() # update where the robot is on the grid and what it can see


    def update_motion(self):

        self.robbie.dt = self.robbie.TIMESTEP
        self.robbie.move_diff_drive(self.grid, self.robbie.vl, self.robbie.vr, self.robbie.dt)

        if self.program_state == 'exploration':
            # read markers around
            marker_list = self.robbie.read_marker_around(self.grid)

            #update markers found        
            if len(self.robbie.markers_found_or_picked) != len(set(self.robbie.markers_found_or_picked).union(set(marker_list))):
                self.robbie.markers_found_or_picked = list(set(self.robbie.markers_found_or_picked).union(set(marker_list)))
                fname = self.grid.fname
                print(f'{fname} found {len(self.robbie.markers_found_or_picked)}/{self.grid.LANDMARKS_TOTAL} markers')
        
        if self.program_state == 'tasks':
            # read markers around
            self.robbie.pickup_marker(self.grid)                

# thread to run robot environment when GUI is on
class RobotEnvThread(threading.Thread):

    def __init__(self, robot_env, gui=None, testing=False, time_limit=None):
        threading.Thread.__init__(self, daemon=True)
        self.robot_env = robot_env
        self.gui = gui
        self.testing = testing
        self.time_limit = time_limit if time_limit else math.inf
        self.robbie = robot_env.robbie
        self.grid = self.robot_env.grid
        self.map_name = self.robot_env.grid.fname

    def run(self):
        print(f'Running on map: {self.map_name}')
        start_time = time.time()
        if self.testing:
            while True:
                try:
                    self.robot_env.update()
                    elapsed_time = time.time() - start_time
                except Exception as e:
                    print(f'Exception in {self.map_name}: {e}! Thread terminated.')
                    break
                if elapsed_time > self.time_limit:
                    print(f'Run time exceeds {self.time_limit:.2f} seconds')
                    break
                if len(self.robbie.markers_found_or_picked) == self.grid.LANDMARKS_TOTAL:
                    print(f'{self.map_name} done in {elapsed_time:.2f} seconds')
                    break
        else:
            while True:
                self.robot_env.update()
                self.gui.show_robot(self.robot_env.robbie)
                self.gui.updated.set()

                time.sleep(robot_pause_time)
                if len(self.robbie.markers_found_or_picked) == self.grid.LANDMARKS_TOTAL:
                    elapsed_time = time.time() - start_time
                    print(f'{self.map_name} done in {elapsed_time:.2f} seconds')
                    time.sleep(3)
                    stopevent.set()
                    break
        
        return self.robbie.markers_found_or_picked, self.grid.LANDMARKS_TOTAL


if __name__ == "__main__":
    global stopevent
    map_filename = "warehouse_map.json"
    state = 'exploration'
    if len(sys.argv) > 1:
        state = 'exploration' if sys.argv[1] == "1" else 'tasks'
    robot_pause_time = 0.04
    grid = Grid(map_filename)
    robot_init_pose = grid.start

    robbie = Robot(robot_init_pose[0], robot_init_pose[1], robot_init_pose[2])
    robot_env = RobotEnv(robbie, grid)
    robot_env.program_state = state
    
    stopevent = threading.Event()
    gui = GUIWindow(grid,robot_env.program_state, stopevent)
    robot_thread = RobotEnvThread(robot_env, gui)
    robot_thread.start()
    gui.start()


            