from multiprocessing import Queue, Process
from music import get_audio_chords,  get_audio_tempo
from pipeline import ColorPipeline, EmotionPipeline, LocationPipeline
from simulator import Simulator
from robot import Robot
from swarm import Swarm
import select

import numpy as np

from utils import Color, DensityFunction, Emotion
import time


def proc_simulator(queue):
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

  prev_cursor_pos = None

  while True:
    readable, _, _ = select.select([queue], [], [], 0.1)  # 0.1 is the timeout in seconds
    if queue in readable:
      message = queue.recv()
      if message == 'exit':
          queue.close()
          break
      elif message is not None:
        print('simulator receive:', message)

        density_functions = [
          DensityFunction(
            type='gaussian',
            color=color.value,
            center= sim.cursor_pos if sim.cursor_pos is not None else location,
            variance=[3, 3]
          ) for color, location in message
        ]
        sim.plot_density_functions(density_functions)


    
    prev_cursor_pos = sim.cursor_pos
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


def proc_pipeline(file_path, queue):
  chords = get_audio_chords(file_path)
  tempos = get_audio_tempo(file_path)

  emotion_pipeline = EmotionPipeline()
  color_pipeline = ColorPipeline()
  location_pipeline = LocationPipeline()

  for i in range(min(len(chords), len(tempos))):
    chord = chords[i]
    emotion = emotion_pipeline.predict_emotion(chord)

    color = color_pipeline.predict_color(emotion)
    location = location_pipeline.predict_location(chord)

    # zip color and location
    output = list(zip(color, location))
    # print("pipeline send:", output)
    # queue.send(output)
    time.sleep(1)


if __name__ == '__main__':
    queue = Queue()
    audio_file = 'c.mp3'
    # sim_p = Process(target=proc_simulator, args=(queue,))
    pipe_p = Process(target=proc_pipeline, args=(audio_file, queue))
    # sim_p.start()
    pipe_p.start()


  # listen to user input
  # listen to music
  # send to pipeline

    # sim_p.join()
    pipe_p.join()
    queue.close()
    print('done')
