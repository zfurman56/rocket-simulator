import compat


def help():
    return \
           """Rocket Simulator for use in TARC 2018.
           Usage: python rocket_sim.py eng_file
           """


def validate_engine_file(argv):
    assert len(argv) > 1, 'Incorrect arguments.\n\n{}'.format(help())
    filename = argv[1]

    assert filename.endswith('.eng'), \
        '{} is not an supported NAR Thrust file.\n\n{}'.format(filename, help())
    return filename
