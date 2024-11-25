from multiprocessing import Queue, Process
from queue import Empty
from consts import RHO, VARANCE_X, VARANCE_Y
from music import get_audio_chords,  get_audio_tempo
from pipeline import ColorPipeline, EmotionPipeline, CenterPipeline, cart2pol
from simulator import Simulator
from robot import Robot
from swarm import Swarm
import matplotlib.pyplot as plt
import math
from argparse import ArgumentParser


import numpy as np

from utils import Color, DensityFunction
import time


def proc_simulator(queue, robots, output_file):

  env = np.array([
    [10, 10],
    [10, -10],
    [-10, -10],
    [-10, 10],
    [10, 10]
  ])

  swarm = Swarm(robots, env)
  sim = Simulator(swarm)

  colors = None
  center = None
  tempo = None
  chord = None
  prev_chord = None

  phi = 0

  sim.plot()
  sim.plot_environment()
  sim.plot_swarm()


  while True:
    try:
      item = queue.get_nowait()
      if item is None:
        plt.savefig(output_file)
        break
      else:
        i_chord, i_colors, i_center, i_tempo = item
        if i_chord != chord:
          center = i_center
          colors = i_colors
        tempo = i_tempo
        prev_chord = chord
        chord = i_chord

    except Empty:
      pass

    if center is None or colors is None:
      continue

    if sim.cursor_pos is not None and prev_chord == chord:
      center = sim.get_cursor_pos()



    step = 2 * np.pi / len(colors)
    density_functions = [
      DensityFunction(
        type='gaussian',
        color=color.value,
        center= [
          center[0] + np.cos(i*step) * RHO + np.cos(math.radians(phi)) * RHO, 
          center[1] + np.sin(i*step) * RHO + np.sin(math.radians(phi)) * RHO
          ],
        variance=[VARANCE_X, VARANCE_Y]
      ) for i, color in enumerate(colors)
    ]

    vor_robots, _ = swarm.color_coverage_control(density_functions)

    for j in range(len(robots)):
      robot = robots[j]
      vor_robot = vor_robots[j]

      if vor_robot is None:
        continue
      
      robot.coverage_control(vor_robot, delta=10)
      
      robot.coverage_control_color(vor_robot,
        swarm.cyan_density_functions,
        swarm.magenta_density_functions,
        swarm.yellow_density_functions
      )

      robot.tempo_control_L(tempo)

    phi = (phi + 1) % 360

    # sim.plot_density_functions(density_functions)
    sim.plot_swarm()
    sim.update_plot()


def proc_pipeline(file_path, queue):
  chords = get_audio_chords(file_path)
  tempos = get_audio_tempo(file_path)

  chords += [chords[-1]] * 10
  tempos += [tempos[-1]] * 10

  emotion_pipeline = EmotionPipeline()
  color_pipeline = ColorPipeline()
  center_pipeline = CenterPipeline()

  for i in range(min(len(chords), len(tempos))):
    chord = chords[i]
    tempo = tempos[i]
    emotions = emotion_pipeline.predict_emotion(chord)
    colors = color_pipeline.predict_colors(emotions)
    center = center_pipeline.predict_center(chord)



    # zip color and location
    output = (chord, colors, center, tempo)
    queue.put(output)
    time.sleep(1)

  queue.put(None)






if __name__ == '__main__':

  # get parameters from args
  parser = ArgumentParser()
  parser.add_argument('audio', type=str, help='audio file path')
  parser.add_argument('--output', type=str, help='output file path', default='output.png')
  args = parser.parse_args()
  audio_file = args.audio
  output_file = args.output


  full_color = [Color.CYAN.value, Color.MAGENTA.value, Color.YELLOW.value]
  poses = [[i,0,0] for i in range(-3,4)]
  colors = [full_color for _ in range(len(poses))]
  robots = [Robot(
    robot_pose=pose, 
    equiped_color=color
  ) for pose, color in zip(poses, colors)]


  queue = Queue()
  sim_p = Process(target=proc_simulator, args=(queue, robots, output_file))
  pipe_p = Process(target=proc_pipeline, args=(audio_file, queue))
  sim_p.start()
  pipe_p.start()


# listen to user input
# listen to music
# send to pipeline

  sim_p.join()
  pipe_p.join()
  queue.close()
  print('done')
