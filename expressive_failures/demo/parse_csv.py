import csv 
import numpy as np 
from numpy import genfromtxt

def row_to_column(filename):
	data = genfromtxt(filename, delimiter=',')
	return data.T[1:]

