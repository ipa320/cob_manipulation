#! /usr/bin/env python

import sys
import os
import glob
import matplotlib.pyplot as plt
from ast import literal_eval

def main():

    path = sys.argv[1]
    print "%d files found"%len(glob.glob(os.path.join(sys.path[0], "..", "data", path, "*", "*", "data_set")))
    for data_file in glob.glob(os.path.join(sys.path[0], "..", "data", path, "*", "*", "data_set")):
        data_file_abspath = os.path.abspath(data_file)
        with open(data_file_abspath, 'r') as f_in:
            data_set = literal_eval(f_in.read())
        
        time = data_set['time']
        for name, data in data_set.iteritems():
            if name != 'time':
                plt.plot(time, data[0], label=name)
        plt.title('Measurement data')
        plt.legend(bbox_to_anchor=(.5, -.12), loc=8, ncol=4, borderaxespad=0.)
        plt.savefig(os.path.join(os.path.dirname(data_file_abspath), "all_measurement_data.pdf"), format='pdf')
        plt.clf()


if __name__ == '__main__':
    main()
