from multiprocessing import Pipe, Queue, Process
from pipeline import ColorPipeline, EmotionPipeline, LocationPipeline
from simulator import Simulator
from robot import Robot
from swarm import Swarm
import select

import numpy as np

from utils import Color, DensityFunction, Emotion
import time


def proc_simulator(conn):
  robot_1 = Robot( 
    robot_pose=[0, -5, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value],
    K=1
  )
  robot_2 = Robot( 
    robot_pose=[0, 5, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value],
    K=1
  )
  robot_3 = Robot( 
    robot_pose=[-5, -6, 0.0],
    equiped_color=[Color.CYAN.value]
  )
  robot_4 = Robot( 
    robot_pose=[5, -5, 0.0],
    equiped_color=[Color.CYAN.value]
  )
  robot_5 = Robot(
    robot_pose=[0, 0, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value]
  )
  robots = [robot_1, robot_2, robot_3, robot_4, robot_5]

  env = np.array([
    [10, 10],
    [10, -10],
    [-10, -10],
    [-10, 10],
    [10, 10]
  ])

  swarm = Swarm(robots, env)
  sim = Simulator(swarm, env)

  density_functions = []

  sim.plot_environment(env)
  sim.plot_swarm(swarm)
  sim.plot()

  while True:
    readable, _, _ = select.select([conn], [], [], 0.1)  # 0.1 is the timeout in seconds
    if conn in readable:
      message = conn.recv()
      if message == 'exit':
          conn.close()
          break
      elif message is not None:
        print('simulator receive:', message)

        density_functions = [
          DensityFunction(
            type='gaussian',
            color=color.value,
            center=location,
            variance=[3, 3]
          ) for color, location in message
        ]
        sim.plot_density_functions(density_functions)
  
    

    vor_robots, vor_prime = swarm.color_coverage_control(density_functions)


    for j in range(len(robots)):
      robot = robots[j]
      vor_robot = vor_robots[j]

      if vor_robot is None:
        continue
      
      # vw = robot.coverage_control(vor_robot, L=1, delta=10)
      vw = robot.coverage_control(vor_robot, delta=10)
      
      color = robot.mix_color(vor_robot,
        swarm.cyan_density_functions,
        swarm.magenta_density_functions,
        swarm.yellow_density_functions
      )





    sim.plot_swarm(swarm)
    sim.update_plot()


def proc_pipeline(input_conn, output_conn):
  emotion_pipeline = EmotionPipeline()
  color_pipeline = ColorPipeline()
  location_pipeline = LocationPipeline()

  while True:
    message = input_conn.recv()
    if message == 'exit':
      output_conn.send('exit')
      input_conn.close()
      output_conn.close()
      break
    elif message:
      print("pipeline receive:", message)
      emotion_pipeline.receive_chords(message)
      emotions = emotion_pipeline.predict_emotions()

      color_pipeline.receive_emotions(emotions)
      color = color_pipeline.predict_colors()

      location_pipeline.receive_emotions(emotions)
      location = location_pipeline.predict_locations()

      # zip color and location
      output = list(zip(color, location))
      print("pipeline send:", output)
      output_conn.send(output)


if __name__ == '__main__':
    sim_input, pipe_output = Pipe()
    pipe_input, sys_send = Pipe()

    sim_p = Process(target=proc_simulator, args=(sim_input,))
    pipe_p = Process(target=proc_pipeline, args=(pipe_input, pipe_output))
    sim_p.start()
    pipe_p.start()


  # listen to user input
  # listen to music
  # send to pipeline

    messages = ['C', 'Cm', 'D', 'Dm', 'E', 'Em', 'F', 'Fm', 'G', 'Gm', 'A', 'Am', 'B', 'Bm']
    while len(messages) > 0:
      message = messages.pop(0)
      print('main send:', message)
      sys_send.send([message])
      time.sleep(5)

    sys_send.send('exit')

    # sim_p.join()
    pipe_p.join()
    print('done')
