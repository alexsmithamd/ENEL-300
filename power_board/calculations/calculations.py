import math as m
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import component_functions as cf
from datetime import datetime
import os

def save_results(name_list, number_list):
    now = datetime.now()
    time = now.strftime("%Y_%m_%d,%H_%M_%S")

    name = f"{time} calculation_results.csv"
    path = os.getcwd() + "/calculations"

    df = pd.DataFrame({'name' : name_list, 'values': number_list})
    df.to_csv(f"{path}/{name}", index = False)

def function():
    x = np.arange(100)
    y = np.atan(x)
    
def test():
    function()
    arr = np.array([1,2,3,4])
    names = ["a", 'b', 'c', 'd']
    save_results(names, arr)


def main():
    test()


if __name__ == "__main__":
    main()
    print(2 + 4 + 7 + 1 + 1 + 1 + 9 + 4 + 5 + 1 + 3 + 1 + 8 + 7 + 8 + 2 + 1)