import math as m
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import component_functions as cf

def save_results(name_list, number_list):
    df = pd.DataFrame({'name' : name_list, 'values': number_list})
    df.to_csv('calculation_results.csv', index = False)


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