from scipy.interpolate import interp1d
import numpy as np 


class LinearInterpolation:
    def __init__(self):
        pass 

    def interpolate(self, key, value, query):
        """
        Linear interpolation
        """
        # throw exception for invalid arguments

        # calculate linear interpolation

        f = interp1d(key, value, kind='linear', fill_value='extrapolate', assume_sorted=True)
        value_query = f(query)
        
        return value_query
