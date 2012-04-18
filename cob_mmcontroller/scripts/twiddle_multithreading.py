#! /usr/bin/env python

import os
import sys
import glob
import time
import threading
import matplotlib.pyplot as plt
from ast import literal_eval #to convert string to dict

params = {'Kp': [1.0, 0.1], 'Ki': [1.0, 0.1], 'Kd': [1.0, 0.1], 'T1': [1.0, 0.1], 'T2': [1.0, 0.1]}
model_dict = {#'P': "params['Kp'][0]*u", 
              #y = Kp*u
              #'PT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + T1/dt*y_m1)
              #'I': "params['Ki'][0]*u_sum", 
              ##y = Ki*u_sum
              'PI': "params['Kp'][0]*u + params['Ki'][0]*u_sum", 
              ##y = Kp*u + Ki*u_sum
              'PIT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Ki'][0]*u_sum + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Ki*u_sum + T1/dt*y_m1)
              'PIT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Ki'][0]*u_sum + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)", 
              ##y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Ki*u_sum + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              #'PD': "params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt", 
              ##y = Kp*u + Kd*(u - u_m1)/dt
              'PDT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Kd*(u - u_m1)/dt + T1/dt*y_m1)
              'PDT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)", 
              ##y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Kd*(u - u_m1)/dt + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              'PID': "params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt", 
              ##y = Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt
              'PIDT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt + T1/dt*y_m1)
              'PIDT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)" 
              #y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              }


def main():
    threadID = 1
    path = sys.argv[1]
    data_file_dict = {}

    print "%d files found"%len(glob.glob(os.path.join(sys.path[0], "..", "data", path, "*", "data_set")))
    for data_file in glob.glob(os.path.join(sys.path[0], "..", "data", path, "*", "data_set")):
        data_file_abspath = os.path.abspath(data_file)
        with open(data_file_abspath, 'r') as f_in:
            data_set = literal_eval(f_in.read())

        interesting_data_list = []
        for name, data in data_set.iteritems():
            if name != 'time':
                if abs(sum(data[2])) > 0.001:
                    interesting_data_list.append(name)

        data_file_dict[data_file_abspath] = interesting_data_list


    for data_file_path, data_list in data_file_dict.iteritems():
        with open(data_file_path, 'r') as f_in:
            data_set = literal_eval(f_in.read())
        time = data_set['time']
        for data_name in data_list:
            data = data_set[data_name]
            for model_name, model in model_dict.iteritems():
                while threading.activeCount > 7:
                    time.sleep(0.01)
                thread = fittingThread(threadID, "Thread-"+str(threadID))
                thread.run(os.path.dirname(data_file_path), data, data_name, model, model_name, time)
                threadID += 1

    print "All files processed"

    for result_dir in glob.glob(os.path.join(sys.path[0], "..", "data", path, "*")):
        result_dir_abs = os.path.abspath(result_dir)
        if os.path.isdir(result_dir_abs):
            result_list = []
            for result_file in glob.glob(os.path.join(result_dir_abs, "result_*")):
                with open(result_file, 'r') as f_res_in:
                    result_list.append(literal_eval(f_res_in.read()))

            error_list = []
            for result in result_list:
                error_list.append(result[3])
            best_result_index = result_list.index(min(error_list))
            with open(os.path.join(result_dir_abs, "best_model_%s"%result_list[best_result_index][2]), 'w') as f_bes_res:
                f_best_res.write(str(result_list[best_result_index]))


    
class fittingThread(threading.Thread):
    def __init__(self, threadID, threadName):
        self.threadName = threadName
        self.threadID = threadID
        print "%s: started"%threadName
        threading.Thread.__init__(self)

    def run(self, dir_path, data, data_name, model, model_name, time):
        print "%s: fitting model %s for %s of twist %s"%(self.threadName, model_name, data_name, os.path.basename(dir_path))
        (best_error, params) = twiddle(model, data, time)
        plt.plot(time, data[0])
        model_output = run_model(params, model, data, time)
        plt.plot(time[:len(model_output[1])], model_output[1])
        plt.suptitle(os.path.basename(dir_path)+" "+model_name, fontsize=10)
        plt.title(" ".join([(k + ': ' + "%.3f" %v[0]) for k,y in params.iteritems()]), fontsize=8)
        plt.savefig(os.path.join(dir_path, data_name+model_name+".pdf"), format="pdf")
        plt.clf()
        print "%s: plot saved into %s.pdf"%(self.threadName, model_name)

        with open(os.path.join(dir_path, "result_%s_%s"%(data_name, model_name)), 'w') as f_res:
            f_res.write(str([os.path.basename(dir_path), data_name, model_name, best_error, params]))

        print "%s: finshed"%self.threadName
        return

                
            
            
    def twiddle(self, model, data, time, tol = 0.0001):
        global params
        params = {'Kp': [1.0, 0.1], 'Ki': [1.0, 0.1], 'Kd': [1.0, 0.1], 'T1': [1.0, 0.1], 'T2': [1.0, 0.1]}

        n = 0
        best_error = run_model(params, model, data, time)[0]
        while sum([x[1] for x in params.itervalues()]) > tol: 
            if n > 10e9 and sum([x[1] for x in params.itervalues()]) < 10*tol:
                print "Too many steps"
                break
            for name in params.iterkeys():
                params[name][0] += params[name][1]
                err = run_model(params, model, data, time)[0]
                if err < best_error:
                    best_error = err
                    params[name][1] *= 1.1
                else:
                    params[name][0] -= 2*params[name][1]
                    err = run_model(params, model, data, time)[0]
                    if  err < best_error:
                        best_error = err
                        params[name][1] *= 1.1
                    else:
                        params[name][0] += params[name][1]
                        params[name][1] *= 0.9
                print "\r", sum([x[1] for x in params.itervalues()]), best_error, [x[0] for x in params.itervalues()], n, "\t\t\t",
            n += 1
        print 
        return best_error, params



                    
    def run_model(self, params, model, data, time):
        y_list = []

        error = 0.0 
        error_sum = 0.0
        
        u_sum = 0.0
        u_m1 = 0.0

        y_m1 = 0.0
        y_m2 = data[0][0]

        dt_m1 = 0.0

        for i in range(len(data[0])):
            u = data[2][i]
            u_sum += u

            dt = time[i] - dt_m1
            if dt == 0.0: continue

            y = eval(model)
            y_list.append(y)
            error = y - data[0][i]
            
            y_m2 = y_m1
            y_m1 = y
            u_m1 = u
            dt_m = time[i]

            error_sum += abs(error)

        return error_sum, y_list


if __name__ == '__main__':
    main()
