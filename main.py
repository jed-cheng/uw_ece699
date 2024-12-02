from multiprocessing import Process, Pipe
from consts import RHO, TRAIL_WIDTH, VARANCE_X, VARANCE_Y
from music import get_audio_chords,  get_audio_tempo
from pipeline import ColorPipeline, EmotionPipeline, CenterPipeline, cart2pol
from simulator import Simulator
from robot import Robot
from swarm import Swarm
import matplotlib.pyplot as plt
import math
from argparse import ArgumentParser
import select
import numpy as np
from utils import Color, DensityFunction
import time


def proc_simulator(conn, robots, output_file, val_trail, val_L):

  env = np.array([
    [10, 10],
    [10, -10],
    [-10, -10],
    [-10, 10],
    [10, 10]
  ])

  swarm = Swarm(robots, env)
  sim = Simulator(swarm, val_trail, val_L)

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
    readable, _, _ = select.select([conn], [], [], 0.01)  # 0.1 is the timeout in seconds
    if conn in readable:
      item = conn.recv()
      if item is None:
          plt.savefig(output_file)
          conn.close()
          break
      elif item is not None:
        # print('simulator receive:', item)

        i_chord, i_colors, i_center, i_tempo = item
        if i_chord != chord:
          center = i_center
          colors = i_colors
        tempo = i_tempo
        prev_chord = chord
        chord = i_chord



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
          center[0] + np.cos(i*step+math.radians(phi)) * RHO + np.cos(math.radians(phi)) * RHO, 
          center[1] + np.sin(i*step+math.radians(phi)) * RHO + np.sin(math.radians(phi)) * RHO
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
      # print(robot.get_L())

    phi = (phi + 1) % 360

    # sim.plot_density_functions(density_functions)
    sim.plot_swarm()
    sim.update_plot()


def init_robots(num_robot, num_color,val_L,val_trail):
  full_color = [Color.CYAN.value, Color.MAGENTA.value, Color.YELLOW.value]
  # place robots evenly on x-axis from -8 to 8
  poses  = [[i,0,0] for i in np.linspace(-8, 8, num_robot)]
  colors = [
      [full_color[(i + j) % len(full_color)] for j in range(num_color)]
      for i in range(len(poses))
  ]
  robots = [Robot(
    robot_pose=pose, 
    equiped_color=color,
    L = val_L,
    trail_width=val_trail
  ) for pose, color in zip(poses, colors)]
  return robots





if __name__ == '__main__':

  # get parameters from args
  parser = ArgumentParser()
  parser.add_argument('audio', type=str, help='audio file path')
  parser.add_argument('--output', type=str, help='output file path', default='output.png')
  
  parser.add_argument('--l', type=float, help='l', default=1.0)
  parser.add_argument('--trail', type=int, help='trail width', default=TRAIL_WIDTH)
  parser.add_argument('--robot', type=int, help='number of robot', default=6)
  parser.add_argument('--color', type=int, help='number of equipped color for each robot', default=3)

  args = parser.parse_args()
  audio_file = args.audio
  output_file = args.output

  num_robot = args.robot
  num_color = args.color
  val_trail = args.trail
  val_L = args.l


  # get chords and tempos from audio file
  chords = get_audio_chords(audio_file)
  tempos = get_audio_tempo(audio_file)

  chords += [chords[-1]] * 5
  tempos += [tempos[-1]] * 5

  emotion_pipeline = EmotionPipeline()
  color_pipeline = ColorPipeline()
  center_pipeline = CenterPipeline()


  # full_color = [Color.CYAN.value, Color.MAGENTA.value, Color.YELLOW.value]
  # poses  = [[i,0,0] for i in np.linspace(-8, 8, num_robot)]
  # colors = [
  #   [Color.CYAN.value],
  #   [Color.MAGENTA.value],
  #   [Color.YELLOW.value, Color.CYAN.value],
  #   [Color.MAGENTA.value, Color.YELLOW.value],
  #   full_color,
  #   full_color
  # ]
  # colors = [
  #   [Color.CYAN.value],
  #   [Color.MAGENTA.value],
  #   [Color.YELLOW.value],
  #   [Color.YELLOW.value, Color.CYAN.value],
  #   [Color.MAGENTA.value, Color.YELLOW.value],
  #   [Color.CYAN.value, Color.MAGENTA.value]
  # ]
  # colors = [
  #   [Color.YELLOW.value, Color.CYAN.value],
  #   [Color.MAGENTA.value, Color.YELLOW.value],
  #   [Color.CYAN.value, Color.MAGENTA.value],
  #   full_color,
  #   full_color,
  #   full_color
  # ]
  # colors = [
  #   [Color.CYAN.value],
  #   [Color.MAGENTA.value],
  #   [Color.YELLOW.value],
  #   full_color,
  #   full_color,
  #   full_color
  # ]
  robots = init_robots(num_robot, num_color, val_L, val_trail)


  in_conn, out_conn = Pipe()
  sim_p = Process(target=proc_simulator, args=(out_conn, robots, output_file, val_trail, val_L))
  sim_p.start()

  for i in range(min(len(chords), len(tempos))):
    # print(f'Processing {i+1}/{len(chords)}')
    chord = chords[i]
    tempo = tempos[i]
    emotions = emotion_pipeline.predict_emotion(chord)
    colors = color_pipeline.predict_colors(emotions)
    center = center_pipeline.predict_center(chord)

    output = (chord, colors, center, tempo)
    in_conn.send(output)
    time.sleep(1.0)
  
  in_conn.send(None)


  sim_p.join()
  in_conn.close()
  out_conn.close()
  print('done')
