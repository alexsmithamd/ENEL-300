import math as m
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import component_functions as cf
from datetime import datetime
import os

FB_REF_V = 0.765
FSW = 700e3
MAX_CONT_OUT_CURRENT = 3
CURRENT_SS = 2 # in microamps

# Equation numbers corispond to numbering in the datasheet
# All functions for all components are herer but in some sense of order
# Assume units for everything are in base SI units except if there is an explicit comment


def buck_eq_1_soft_start_cap(soft_start_capacitor):
    soft_start_time = ((soft_start_capacitor / 1e-9) * FB_REF_V * 1.1) / CURRENT_SS
    return soft_start_time

def buck_eq_2_feedback_resistors(bottom_reistor, output_voltage):
    top_resistor = (output_voltage - FB_REF_V) * (bottom_reistor / FB_REF_V)
    return top_resistor

def buck_eq_4_5_6_inductor_current(inductor, output_voltage, maximum_input_voltage):
    inductor_pk_to_pk_current = (output_voltage * (maximum_input_voltage - output_voltage)) / (maximum_input_voltage * inductor * FSW)
    indcutor_peak_current = MAX_CONT_OUT_CURRENT + inductor_pk_to_pk_current / 2
    inductor_rms_current = m.sqrt((MAX_CONT_OUT_CURRENT ** 2) + (1 / 12) * (inductor_pk_to_pk_current ** 2))
    return [inductor_pk_to_pk_current, indcutor_peak_current, inductor_rms_current]

def buck_eq_7_rms_out_cap_current_rating(inductor, output_voltage, maximum_input_voltage):
    capacitor_current_rating = (output_voltage * (maximum_input_voltage - output_voltage)) / (m.sqrt(12) * maximum_input_voltage * inductor * FSW)
    return capacitor_current_rating


def main():
    print(buck_eq_2_feedback_resistors(1e4, 5))


if __name__ == "__main__":
    main()