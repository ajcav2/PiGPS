import numpy as np
from scipy import interpolate

def interpolation(lst1,lst2,time):

    '''
    Parameters:
    -----------
    lst1, lst2 : list[float]
        List of floats corresponding to measurements
    time : float
        The duration of the data collection

    Returns:
    --------
    d1, d2 : list[float]
        Extrapolated list for the shorter vector (may
        require trimming longer vector). d1 is the longer
        list.
    t1 : list[float]
        Vector giving measurement times
    '''

    # Determine which list is longer
    if (len(lst1) > len(lst2)):
        d1 = lst1
        d2 = lst2
    else:
        d1 = lst2
        d2 = lst1

    # Calculate time between measurements
    dt1 = time/len(d1)
    dt2 = time/len(d2)

    # Generate vector of measurement times
    t1 = np.arange(0,time,dt1)
    t2 = np.arange(0,time,dt2)

    # Ensure vectors are same length
    short = min(len(t2),len(d2))
    t2 = t2[:short]
    d2 = d2[:short]

    # Create interpolation function
    f = interpolate.interp1d(t2,d2)

    # Use interpolation function to generate new values
    xnew = np.arange(0,time-dt2,dt1)
    d2 = f(xnew)

    # Trim longer vector to match size
    d1 = d1[:len(d2)]
    t1 = t1[:len(xnew)]

    return d1,d2,t1

    
