from apogee_ctrl import ApogeeSimulator
from descent_ctrl import DescentSimulator
from utils import get_eng_file_from_argv

class CombinedControl():
    print("====== APOGEE CTRL ======")
    sim = ApogeeSimulator(get_eng_file_from_argv(__file__))
    sim.simulate()
    sim.print_final_values()
    sim.plot('Apogee Control Simulaton Results').show(block=False)

    print("====== DESCENT CTRL ======")
    sim = DescentSimulator(sim.state)
    sim.simulate()
    sim.print_final_values()
    sim.plot('Combined Control Simulation Results').show()

if __name__ == '__main__':
    CombinedControl()