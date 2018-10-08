from sys import argv

from engine import RocketEngine
from params import THRUST_SCALE


def help(sim, eng=False):
    return \
           """Rocket Simulator for use in TARC 2019.
           Usage: python {} {}
           """.format(sim, 'eng_file' if eng else '')


def validate_engine_file(argv, sim):
    assert len(argv) > 1, 'Incorrect arguments.\n\n{}'.format(help(sim, True))
    filename = argv[1]

    assert filename.endswith('.eng'), \
        '{} is not an supported NAR Thrust file.\n\n{}'.format(filename, help(sim, True))
    return filename

def get_eng_file_from_argv(sim):
    with open(validate_engine_file(argv, sim)) as f:
        return RocketEngine(f, THRUST_SCALE)
