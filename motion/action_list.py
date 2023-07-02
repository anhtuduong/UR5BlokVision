
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import
from motion.command import Command

# Constants
model_1 = 'X1-Y1-Z2 1'
model_2 = 'X1-Y1-Z2 2'
model_3 = 'X1-Y1-Z2 3'
table = 'tavolo'

ACTION_LIST = [

    Command.spawn_model_static('Spawn model 1 static', model_name=model_1, is_static=True),
    Command.delete_model('Delete model 1', model_name=model_1),

    Command.move_joints('Move to middle', joints=[-1.24, -1.12, -2.64, -0.98, -1.59, -0.27]),
    Command.move_to(f'Move above {model_2}', pose=[0.1850, 0.7, 1.045, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to(f'Move down to {model_2}', pose=[0.1850, 0.7, 0.945, 0.0, 1.0, 0.0, 0.0]),
    Command.move_gripper(f'Grasp {model_2}', diameter=30),
    Command.move_to(f'Pick up {model_2}', pose=[0.1850, 0.7, 1.045, 0.0, 1.0, 0.0, 0.0]),
    Command.move_joints('Move to middle', joints=[-1.24, -1.12, -2.64, -0.98, -1.59, -0.27]),
    Command.move_to('Move above the place position', pose=[0.72, 0.5570, 1.045, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to('Move down to the place position', pose=[0.72, 0.5570, 0.983, 0.0, 1.0, 0.0, 0.0]),
    Command.attach_models('Attach models', model_name_1=model_2, model_name_2=model_1+' static'),
    Command.move_gripper(f'Release {model_2}', diameter=70),
    Command.move_to(f'Move above {model_2}', pose=[0.72, 0.5570, 1.1, 0.0, 1.0, 0.0, 0.0]),
    Command.spawn_model_static('Spawn model 2 static', model_name=model_2, is_static=True),
    Command.detach_models('Detach models', model_name_1=model_2, model_name_2=model_1+' static'),
    Command.delete_model('Delete model 2', model_name=model_2),

    Command.move_joints('Move to middle', joints=[-1.24, -1.12, -2.64, -0.98, -1.59, -0.27]),
    Command.move_to(f'Move above {model_3}', pose=[0.185, 0.6, 1.045, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to(f'Move down to {model_3}', pose=[0.185, 0.6, 0.945, 0.0, 1.0, 0.0, 0.0]),
    Command.move_gripper(f'Grasp {model_3}', diameter=30),
    Command.move_to(f'Pick up {model_3}', pose=[0.185, 0.6, 1.045, 0.0, 1.0, 0.0, 0.0]),
    Command.move_joints('Move to middle', joints=[-1.24, -1.12, -2.64, -0.98, -1.59, -0.27]),
    Command.move_to('Move above the place position', pose=[0.72, 0.5570, 1.055, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to('Move down to the place position', pose=[0.72, 0.5570, 1.02, 0.0, 1.0, 0.0, 0.0]),
    Command.attach_models('Attach models', model_name_1=model_3, model_name_2=model_2+' static'),
    Command.move_gripper(f'Release {model_3}', diameter=70),
    Command.move_to(f'Move above {model_3}', pose=[0.72, 0.5570, 1.1, 0.0, 1.0, 0.0, 0.0]),
    Command.spawn_model_static('Spawn model 3 static', model_name=model_3, is_static=True),
    Command.detach_models('Detach models', model_name_1=model_3, model_name_2=model_2+' static'),
    Command.delete_model('Delete model 3', model_name=model_3),

]