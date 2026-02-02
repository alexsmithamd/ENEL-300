import math as m
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
from datetime import datetime
import os

import component_functions as cf

def save_results(name_list, number_list):
    now = datetime.now()
    time = now.strftime("%Y_%m_%d,%H_%M_%S")

    name = f"{time} calculation_results.csv"
    path = os.getcwd() + "/calculations"

    df = pd.DataFrame({'name' : name_list, 'values': number_list})
    df.to_csv(f"{path}/{name}", index = False)

    
def buck_converter_calcs():
    soft_start_cap = 10e-9
    bottom_resistor = 22.1e3
    indcutor = 3.3e-6
    VOUT = 5
    max_input_voltage = 14


    soft_start_time = cf.buck_eq_1_soft_start_cap(soft_start_cap)
    top_resistor = cf.buck_eq_2_feedback_resistors(bottom_resistor, VOUT)
    value_array = np.array(cf.buck_eq_4_5_6_inductor_current(indcutor, VOUT, max_input_voltage))
    cap_rms_current = cf.buck_eq_7_rms_out_cap_current_rating(indcutor, VOUT, max_input_voltage)

    arr = np.array([soft_start_time, top_resistor, cap_rms_current])
    arr = np.ndarray.flatten(arr)
    arr = np.append(arr, value_array)
    names = ["soft start T", "top resistor R", "cap rms I", "L pk-pk I", "L max pk I", "L rms I"]
    save_results(names, arr)


def main():
    buck_converter_calcs()


if __name__ == "__main__":
    main()