#! /usr/bin/env python

import os
import sys
import glob
import matplotlib.pyplot as plt
from ast import literal_eval #to convert string to dict

current_path = ''
overall_result = []
model_counter = {}
params = {'Kp': [1.0, 0.1], 'Ki': [1.0, 0.1], 'Kd': [1.0, 0.1], 'T1': [1.0, 0.1], 'T2': [1.0, 0.1]}
model_dict = {#'P': "params['Kp'][0]*u", 
              #y = Kp*u
              #'PT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + T1/dt*y_m1)
              #'I': "params['Ki'][0]*u_sum", 
              ##y = Ki*u_sum
              #'PI': "params['Kp'][0]*u + params['Ki'][0]*u_sum", 
              ##y = Kp*u + Ki*u_sum
              #'PIT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Ki'][0]*u_sum + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Ki*u_sum + T1/dt*y_m1)
              'PIT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Ki'][0]*u_sum + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)", 
              ##y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Ki*u_sum + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              #'PD': "params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt", 
              ##y = Kp*u + Kd*(u - u_m1)/dt
              #'PDT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Kd*(u - u_m1)/dt + T1/dt*y_m1)
              #'PDT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)", 
              ##y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Kd*(u - u_m1)/dt + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              #'PID': "params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt", 
              ##y = Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt
              #'PIDT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt + T1/dt*y_m1)
              'PIDT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)" 
              #y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              }


def main():
    global current_path
    path = sys.argv[1]

    print "%d files found"%len(glob.glob(os.path.join(sys.path[0], "..", "data", path, "*", "data_set")))
    for data_file in glob.glob(os.path.join(sys.path[0], "..", "data", path, "*", "data_set")):
        current_path = os.path.abspath(os.path.dirname(data_file))

        with open(data_file, 'r') as f_in:
            data_set = literal_eval(f_in.read())
        
        result = parse_data_set(data_set)
        with open(os.path.join(current_path, 'result'), 'w') as f_out:
            f_out.write(str(result))

    for entry in overall_result:
        print entry
    with open(os.path.join(os.path.dirname(current_path), "overall_result"), 'w') as f_all:
        f_all.write(str(overall_result))

    print model_counter
    with open(os.path.join(os.path.dirname(current_path), "model_counter"), 'w') as f_count:
        f_count.write(str(model_counter))

    



def parse_data_set(data_set):
    global overall_result
    global model_counter
    time = data_set['time']
    twist = {}
    for name, data in data_set.iteritems():
        if name != 'time':
            if abs(sum(data[2])) > 0.001:
                twist[name] = []
                for model_name, model in model_dict.iteritems():
                    print os.path.basename(current_path), name, model_name
                    (best_error, params) = twiddle(model, data, time)
                    twist[name].append([best_error, model_name, params])
                    plt.plot(time, data[0])
                    plt.plot(time[:-1], run_model(params, model, data, time)[1])
                    plt.text(1., .1, os.path.basename(current_path)+" "+model_name, fontsize=10)
                    plt.text(1., -.1, " ".join([("%.3f" %x[0]) for x in params.itervalues()]), fontsize=8)
                    plt.savefig(os.path.join(current_path, model_name+".pdf"), format="pdf")
                    #print "\nPlot of model %s saved"%model_name
                    plt.clf()
                twist[name].sort()
                overall_result.append({name: twist[name][0]})
                if twist[name][0][1] not in model_counter:
                    model_counter[twist[name][0][1]] = 0
                model_counter[twist[name][0][1]] += 1

                #plot and save best
                plt.plot(time, data[0])
                plt.plot(time[:-1], run_model(twist[name][0][2], model_dict[twist[name][0][1]], data, time)[1])
                plt.text(1., .1, os.path.basename(current_path)+" "+twist[name][0][1], fontsize=10)
                plt.text(1., -.1, " ".join([("%.3f" %x[0]) for x in twist[name][0][2].itervalues()]), fontsize=8)
                plt.savefig(os.path.join(current_path, "best_model_"+twist[name][0][1]+".pdf"), format="pdf")
                plt.clf()

    return twist

            
        
        
def twiddle(model, data, time, tol = 0.001):
    global params

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



                
def run_model(params, model, data, time):
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
